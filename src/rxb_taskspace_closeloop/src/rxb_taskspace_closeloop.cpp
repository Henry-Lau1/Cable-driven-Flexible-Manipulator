#include <rxb_taskspace_closeloop/rxb_taskspace_closeloop.h>

namespace RXB_TaskSpace_CloseLoop {
/// ------------------------------------------------------------------------------------------------
/// 函数功能：获得角度编码器数值，计算并更新当前时刻末端齐次变换矩阵
/// 返回类型：
///     void
/// 函数参数：
///     const rxb_msgs::sensorRead::ConstPtr& data          -- [输入]接收到的角度编码器数据（注意，其中没有拉力传感器数据），单位：deg
/// 成员变量：
///     vector<double> jointAngleMeasured                   -- [输出]保存的角度编码器数据，单位：deg
void SingleSegController::EncoderData_SubFunc(const rxb_msgs::sensorRead::ConstPtr& data){
    std::vector<double> jointAngleSum(2, 0);
    for(int i = 0; i < encoder_num_all; i++){
        if(i%8==0 || i%8==3 || i%8==4 || i%8==7){
            jointAngleMeasured[static_cast<long>(i)] = data->joint_angle[static_cast<unsigned long>(i)];
            jointAngleSum[0] += jointAngleMeasured[static_cast<long>(i)];
        }else{
            jointAngleMeasured[static_cast<long>(i)] = -data->joint_angle[static_cast<unsigned long>(i)];
            jointAngleSum[1] += jointAngleMeasured[static_cast<long>(i)];
        }
    }
    jointAngleActualAverage[0] = jointAngleSum[0]/4;
    jointAngleActualAverage[1] = jointAngleSum[1]/4;

    Eigen::MatrixXd segT;
    segT = transformM * RXB_T_Segment(RXB_l_mm, jointAngleMeasured, 1) * transformM;

    std_msgs::Float32MultiArray segT_msg;
    for(long i = 0; i < 3; i++){
        for(long j = 0; j < 4; j++){
            segT_msg.data.push_back(static_cast<float>(segT(i, j)));
        }
    }
    actualT_pub.publish(segT_msg);
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：计算目标齐次变换矩阵
/// 返回类型：
///     void
/// 函数参数：
///     const std_msgs::Float64MultiArray& data          -- [输入]接收到的角度编码器数据（注意，其中没有拉力传感器数据），单位：deg
/// 成员变量：
///     vector<double> jointAngleMeasured                   -- [输出]保存的角度编码器数据，单位：deg
void SingleSegController::FKCal_SubFunc(const std_msgs::Float64MultiArray& data){
    Eigen::VectorXd targetJoint(encoder_num_all);
    targetJoint << data.data[1], -data.data[0], -data.data[0], data.data[1], data.data[1], -data.data[0], -data.data[0], data.data[1];

    Eigen::MatrixXd segT;
    segT = transformM * RXB_T_Segment(RXB_l_mm, targetJoint, 1) * transformM;

    std_msgs::Float32MultiArray segT_msg;
    std::cout << "The targetT is: " << std::endl;
    for(long i = 0; i < 3; i++){
        for(long j = 0; j < 4; j++){
            segT_msg.data.push_back(static_cast<float>(segT(i, j)));
            std::cout << segT(i, j) << "\t";
        }
        std::cout << std::endl;
    }
    targetT_pub.publish(segT_msg);
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：接收ui界面的move指令，调用规划函数
/// 返回类型：
///     void
/// 函数参数：
///     const rxb_msgs::jointControl::ConstPtr& data - mode          -- [输入]singleSeg为1，multiSeg为2
///                                                  - data          -- [输入]期望关节角，顺序为q2、q1、q4、q3、q6、q5，单位：deg
/// 成员变量：
///     vector<double> jointAngleMeasured                   -- [输出]保存的角度编码器数据，单位：deg
bool SingleSegController::TaskSpaceCtrl_SingleSeg_ServerFunc(rxb_msgs::taskSpaceCtrl_singleSeg::Request& req, rxb_msgs::taskSpaceCtrl_singleSeg::Response& res){
    std::cout << "The desiredJoint is: ";
    for(int i = 0; i < dof; i++){
        std::cout << req.desiredJoint[static_cast<unsigned long>(i)] << "\t";
    }
    std::cout << std::endl;
    std_msgs::Float32MultiArray desiredJoint_msg;
    for(int i = 0; i < dof; i++){
        desiredJoint_msg.data.push_back(req.desiredJoint[static_cast<unsigned long>(i)]);
    }
    desiredJoint_pub.publish(desiredJoint_msg);

    std::cout << "The planJoint is: ";
    for(int i = 0; i < dof; i++){
        std::cout << req.planJoint[static_cast<unsigned long>(i)] << "\t";
    }
    std::cout << std::endl;

    Eigen::Vector2d desiredJoint, planJoint;
    for(int i = 0; i < dof; i++){
        if(i%2 == 0){
            desiredJoint[i] = static_cast<double>(req.desiredJoint[static_cast<unsigned long>(i+1)]);
            planJoint[i] = static_cast<double>(req.planJoint[static_cast<unsigned long>(i+1)]);
        }else{
            desiredJoint[i] = -static_cast<double>(req.desiredJoint[static_cast<unsigned long>(i-1)]);
            planJoint[i] = -static_cast<double>(req.planJoint[static_cast<unsigned long>(i-1)]);
        }
    }

    Eigen::Vector2d d_compensateJoint, compensateJoint;
    RXB_SegmentalCloseLoopControl(desiredJoint, jointAngleMeasured, d_compensateJoint);
    for(int i = 0; i < dof; i++){
        compensateJoint[i] = planJoint[i] + dt * d_compensateJoint[i];
    }

    res.success = true;
    for(int i = 0; i < dof; i++){
        if(i%2 == 0){
            res.compensateJoint.push_back(-static_cast<float>(compensateJoint[static_cast<long>(i)+1]));
        }else{
            res.compensateJoint.push_back(static_cast<float>(compensateJoint[static_cast<long>(i)-1]));
        }
    }

    std_msgs::Float32MultiArray compensateJoint_msg;
    for(int i = 0; i < dof; i++){
        compensateJoint_msg.data.push_back(res.compensateJoint[static_cast<unsigned long>(i)]);
    }
    planJoint_pub.publish(compensateJoint_msg);
    std::cout << "The compensateJoint is: ";
    for(int i = 0; i < dof; i++){
        std::cout << res.compensateJoint[static_cast<unsigned long>(i)] << "\t";
    }
    std::cout << std::endl;

//    Eigen::VectorXd compensateJointCal(encoder_num_all);
//    compensateJointCal << compensateJoint[0], compensateJoint[1], compensateJoint[1], compensateJoint[0], compensateJoint[0], compensateJoint[1], compensateJoint[1], compensateJoint[0];
//    Eigen::MatrixXd compensateT;
//    compensateT = transformM * RXB_T_Segment(RXB_l_mm, compensateJointCal, 1) * transformM;
//    std_msgs::Float32MultiArray compensateT_msg;
//    for(long i = 0; i < 3; i++){
//        for(long j = 0; j < 4; j++){
//            compensateT_msg.data.push_back(static_cast<float>(compensateT(i, j)));
//        }
//    }
//    desiredT_pub.publish(compensateT_msg);

    std::cout << std::endl;

    return true;
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：初始化SingleSegController类
SingleSegController::SingleSegController(ros::NodeHandle nh_):spinner(4), nh(nh_) {
    jointAngleMeasured = Eigen::VectorXd::Zero(encoder_num_all);
    jointAngleActualAverage = Eigen::VectorXd::Zero(dof);

    targetT_pub = nh.advertise<std_msgs::Float32MultiArray>("taskSpaceCtrl/targetT", 1);
    actualT_pub = nh.advertise<std_msgs::Float32MultiArray>("taskSpaceCtrl/actualT", 1);
    desiredJoint_pub = nh.advertise<std_msgs::Float32MultiArray>("taskSpaceCtrl/desiredJoint", 1);
    desiredT_pub = nh.advertise<std_msgs::Float32MultiArray>("taskSpaceCtrl/desiredT", 1);
    planJoint_pub = nh.advertise<std_msgs::Float32MultiArray>("taskSpaceCtrl/planJoint", 1);
    errorTwist_pub = nh.advertise<std_msgs::Float32MultiArray>("taskSpaceCtrl/errorTwist", 1);

    encoderData_sub = nh.subscribe<rxb_msgs::sensorRead>("serialport/Sensor_Encoder_data", 1, &SingleSegController::EncoderData_SubFunc, this);
    FKCal_sub = nh.subscribe("taskSpaceCtrl/FKCal", 1, &SingleSegController::FKCal_SubFunc, this);

    taskSpaceCtrl_singleSeg_server = nh.advertiseService("taskSpaceCtrl/singleSeg", &SingleSegController::TaskSpaceCtrl_SingleSeg_ServerFunc, this);

    spinner.start();
}


/// ------------------------------------------------------------------------------------------------
/// 函数功能：计算DH表一行的齐次变换矩阵【已验证】
/// 返回类型：
///		MatrixXd T			-- [输出]DH表一行的齐次变换矩阵
/// 函数参数：
///		double ai			-- [输入]杆长，单位：毫米（mm）
///		double alpha		-- [输入]扭角，单位：角度（deg）
///		double di			-- [输入]偏距，单位：毫米（mm）
///		double theta		-- [输入]关节变量，单位：角度（deg）
Eigen::MatrixXd SingleSegController::TransformationMatrix(double ai, double alpha, double di, double theta)
{
    alpha = alpha * dr;
    theta = theta * dr;

    double ci = std::cos(theta);
    double si = std::sin(theta);
    double li = std::cos(alpha);
    double ui = std::sin(alpha);

    Eigen::MatrixXd T(4, 4);
    T << ci, -li*si,  ui*si, ai*ci,
         si,  li*ci, -ui*ci, ai*si,
          0,     ui,     li,    di,
          0,      0,      0,     1;

    return T;
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：计算一个臂段的齐次变化矩阵（可以指定是否为包含基座的臂段）【已验证】
/// 返回类型：
///		Matrix4d T_seg		-- 一个臂段的齐次变化矩阵
/// 函数参数：
///		double l			-- 一个臂段的柔性臂杆长
///		VectorXd theta		-- 一个臂段的编码器角度（根据输入的角度个数来确定臂段的编码器个数，进而确定关节个数）
///		int flag_RXB_base	-- 臂段是否包含基座，不包含则为0（默认）
Eigen::Matrix4d SingleSegController::RXB_T_Segment(const double& l, const Eigen::VectorXd& theta, int flag_RXB_base) // 函数原型给默认值，函数定义处不用给默认值
{
    // 参数定义
    int encoder_num_ttl = static_cast<int>(theta.size()); // 编码器总数
    double ai = l;
    double alpha = 0; // 扭角的单位是角度！！
    int temp_neg = 1; // 用于给alpha加上负号，初值为1
    Eigen::Matrix4d T_temp;
    T_temp.setZero();
    Eigen::Matrix4d T_seg;
    T_seg.setIdentity(); // 初始化为单位矩阵

    // 计算每个编码器对应的齐次变换矩阵
    for (int i = 0; i < encoder_num_ttl; i++)
    {
        if (i % 2 == 0) // 若i为偶数（i = 0, 2, 4, …），对应第（i + 1）个编码器
        {
            temp_neg = temp_neg * (-1); // DH表修改
            alpha = 90 * temp_neg; // 给90°加上符号，原因见DH参数表
            T_temp = TransformationMatrix(0, alpha, 0, theta(i)); // 计算第(i + 1)个编码器对应的齐次变换矩阵
        }
        else // 若i为奇数（i = 1, 3, 5, …），对应第（i + 1）个编码器
        {
            T_temp = TransformationMatrix(ai, 0, 0, theta(i)); // 计算第(i + 1)个编码器对应的齐次变换矩阵
        }
        // std::cout << T_temp << std::endl;
        T_seg = T_seg * T_temp;
        // std::cout << T_seg << std::endl << std::endl;
    }

    // 若包含基座，则再加上基座的距离
    if (flag_RXB_base != 0)
    {
        T_seg = TransformationMatrix(l, 0, 0, 0) * T_seg;
    }

    return T_seg;
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：将实际运动末端位置投影到理想运动工作空间上，采用几何方法（Geometric Method 2，割线法）
///		【！！该函数需要重载！！】
/// 【重载1：从文件中直接读取拟合系数和中心缩放系数】【已验证】
/// 返回类型：
///		void						-- 无返回值
/// 函数参数：
///		int file_input_flag			-- [输入]判断是否从文件中读取参数
///		Vector3d p_actual			-- [输入]实际末端位置
///		Vector3d p_result			-- [输出]投影到理想工作空间的位置
///		double vertical_distance	-- [输出]实际点与投影点的距离
void SingleSegController::RXB_Project2IdealPlane_GM2(int file_input_flag,
                                const Eigen::Vector3d& p_actual,
                                Eigen::Vector3d& p_result,
                                double& vertical_distance)
{
    // 读取文件中的拟合系数和中心缩放参数
    //		读取p_ws_fit
    std::ifstream file_input_p_ws_fit("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_taskspace_closeloop/RXB_IdealPlaneWSFitting_Segment_p_ws_fit.txt"); // 打开文件操作【LJH视情况修改】
    if (!file_input_p_ws_fit.is_open()) // 如果没有成功打开
    {
        std::cerr << "Error: Unable to open file: RXB_IdealPlaneWSFitting_Segment_p_ws_fit.txt" << std::endl;
    }
    Eigen::VectorXd p_ws_fit(poly_num_ws_fit + 1);
    for (int i = 0; i <= poly_num_ws_fit; ++i)
    {
        if (!(file_input_p_ws_fit >> p_ws_fit[i]))
        {
            std::cerr << "Error: Failed to read data from file: RXB_IdealPlaneWSFitting_Segment_p_ws_fit.txt" << std::endl;
        }
    }
    file_input_p_ws_fit.close();

    //  读取sc_parameter
    std::ifstream file_input_sc_parameter("/home/rxb/rxb_norCor_forcePercept_ljh_240607/src/rxb_taskspace_closeloop/RXB_IdealPlaneWSFitting_Segment_sc_parameter.txt"); // 打开文件操作【LJH视情况修改】
    if (!file_input_sc_parameter.is_open()) // 如果没有成功打开
    {
        std::cerr << "Error: Unable to open file: RXB_IdealPlaneWSFitting_Segment_sc_parameter.txt" << std::endl;
    }
    Eigen::MatrixXd sc_parameter(2, 2);
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            if (!(file_input_sc_parameter >> sc_parameter(i, j)))
            {
                std::cerr << "Error: Failed to read data from file: RXB_IdealPlaneWSFitting_Segment_sc_parameter.txt" << std::endl;
            }
        }
    }
    file_input_sc_parameter.close();

    // 参数处理
    double u_num_mean = sc_parameter(0, 0);
    double u_num_std = sc_parameter(0, 1);
    double x_num_mean = sc_parameter(1, 0);
    double x_num_std = sc_parameter(1, 1);

    // 将立体空间上的点转化到理想平面上
    double phi_IdealPlane = std::atan2(p_actual(2), p_actual(1)); // 理想平面夹角
    double ua = std::sqrt(p_actual(1) * p_actual(1) + p_actual(2) * p_actual(2)); // 臂段末端点在u轴上的真实值
    double ua_sc = (ua - u_num_mean) / u_num_std;
    double xa = p_actual(0);

    // 采用几何方法（割线法）求投影点
    double x_ua_sc = 0; // 迭代初始值
    double dxa_sc = 0;
    for (int i = 0; i <= poly_num_ws_fit; ++i) // 【问题已解决！i需要循环到poly_num_ws_fit！】
    {
        if (std::abs(p_ws_fit(i)) > 1e-8)
        {
            x_ua_sc += p_ws_fit(i) * std::pow(ua_sc, poly_num_ws_fit - i); // 【问题已解决！cpp的数组从零开始，与matlab不同！】
            dxa_sc += (poly_num_ws_fit  - i) * p_ws_fit(i) * std::pow(ua_sc, poly_num_ws_fit - 1 - i);
        }
    }
    double x_ua = x_ua_sc * x_num_std + x_num_mean;
    double dxa = (x_num_std / u_num_std) * dxa_sc;

    // 计算投影点位置
    double u_GM2_result = ua + dxa * (xa - x_ua) / (1 + dxa * dxa);
    double x_GM2_result = (xa * dxa * dxa + x_ua) / (1 + dxa * dxa);

    // 转换为笛卡尔坐标系下的投影点
    double x_proj = x_GM2_result;
    double u_proj = u_GM2_result * std::cos(phi_IdealPlane);
    double w_proj = u_GM2_result * std::sin(phi_IdealPlane);
    std::cout << "The Project Point is: " << w_proj << "\t" << -u_proj << "\t" << x_proj << std::endl;

    // 计算理论最小距离
    vertical_distance = std::sqrt((xa - x_GM2_result) * (xa - x_GM2_result) + (ua - u_GM2_result) * (ua - u_GM2_result));

    // 输出结果
    p_result = { x_proj, u_proj, w_proj }; // p_result其实是列向量，见函数参数定义
}

///【重载2：输入拟合系数和中心缩放系数】【已验证】
/// 返回类型：
///		void						-- 无返回值
/// 函数参数：
///		VectorXd p_ws_fit			-- [输入]拟合系数
///		MatrixXd sc_parameter		-- [输入]中心缩放参数
///		Vector3d p_actual			-- [输入]实际末端位置
///		Vector3d p_result			-- [输出]投影到理想工作空间的位置
///		double vertical_distance	-- [输出]实际点与投影点的距离
void SingleSegController::RXB_Project2IdealPlane_GM2(const Eigen::VectorXd& p_ws_fit,
                                const Eigen::MatrixXd& sc_parameter,
                                const Eigen::Vector3d& p_actual,
                                Eigen::Vector3d& p_result,
                                double& vertical_distance)
{
    // 参数处理
    double u_num_mean = sc_parameter(0, 0);
    double u_num_std = sc_parameter(0, 1);
    double x_num_mean = sc_parameter(1, 0);
    double x_num_std = sc_parameter(1, 1);

    // 将立体空间上的点转化到理想平面上
    double phi_idealplane = std::atan2(p_actual(2), p_actual(1)); // 理想平面夹角【已验证】
    double ua = std::sqrt(p_actual(1) * p_actual(1) + p_actual(2) * p_actual(2)); // 臂段末端点在u轴上的真实值【已验证】
    double ua_sc = (ua - u_num_mean) / u_num_std; // 【已验证】
    double xa = p_actual(0);

    // 采用几何方法（割线法）求投影点
    double x_ua_sc = 0; // 迭代初始值
    double dxa_sc = 0;
    for (int i = 0; i <= poly_num_ws_fit; ++i) { // 【问题已解决！i需要循环到poly_num_ws_fit！】
        if (std::abs(p_ws_fit(i)) > 1e-8)
        {

            //std::cout << std::fixed << std::setprecision(6) << "p_ws_fit(i):\t" << p_ws_fit(i) << std::endl;
            //std::cout << std::fixed << std::setprecision(6) << "poly_num_ws_fit + 0 - i:\t" << poly_num_ws_fit + 0 - i << std::endl;

            x_ua_sc += p_ws_fit(i) * std::pow(ua_sc, poly_num_ws_fit - i); // 【问题已解决！cpp的数组从零开始，与matlab不同！】
            dxa_sc += (poly_num_ws_fit - i) * p_ws_fit(i) * std::pow(ua_sc, poly_num_ws_fit - 1 - i);
        }
    }
    double x_ua = x_ua_sc * x_num_std + x_num_mean;
    double dxa = (x_num_std / u_num_std) * dxa_sc;

    // 计算投影点位置
    double u_GM2_result = ua + dxa * (xa - x_ua) / (1 + dxa * dxa);
    double x_GM2_result = (xa * dxa * dxa + x_ua) / (1 + dxa * dxa);

    // 转换为笛卡尔坐标系下的投影点
    double x_proj = x_GM2_result;
    double u_proj = u_GM2_result * std::cos(phi_idealplane);
    double w_proj = u_GM2_result * std::sin(phi_idealplane);

    // 计算理论最小距离
    vertical_distance = std::sqrt((xa - x_GM2_result) * (xa - x_GM2_result) + (ua - u_GM2_result) * (ua - u_GM2_result));

    // 输出结果
    p_result = { x_proj, u_proj, w_proj }; // p_result其实是列向量，见函数参数定义
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：定义牛顿迭代法待求根的函数u【已验证】
/// 返回类型：
///		double u	-- 牛顿迭代法的原函数（的一部分）
/// 函数参数：
///		double t	-- 通过三角变换得到的辅助量
double SingleSegController::u(double t)
{
    return - (4 * t) * (- 5 + 15 * t * t - 11 * std::pow(t, 4) + std::pow(t, 6)) / std::pow((1 + t * t), 4);
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：定义牛顿迭代法待求根的函数的导数dudt【已验证】
/// 返回类型：
///		double dudt -- 牛顿迭代法的原函数（的一部分）的导数
/// 函数参数：
///		double t	-- 通过三角变换得到的辅助量
double SingleSegController::dudt(double t)
{
    return 4 * (5 - 80 * t * t + 130 * std::pow(t, 4) - 40 * std::pow(t, 6) + std::pow(t, 8)) / std::pow((1 + t * t), 5);
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：由末端位置求得关节运动角度(牛顿迭代法) - 仅限一段4关节的情况！！（因为理想工作空间是按一段4个关节拟合的）【已验证，还需引入杆长变量】
/// 返回类型：
///		double x_result							-- 函数零点
/// 函数参数：
///		Eigen::Vector3d& p_actual_projected		-- 投影到理想工作平面上的位置
///		double x0								-- 迭代初值
///		double e								-- 迭代允许误差
///		int N_max								-- 最大迭代次数
double SingleSegController::RXB_SegCL_P2Theta_NM(const Eigen::Vector3d& p_actual_projected,
                            const double& x0,
                            const double& e,
                            const int& N_max)
{
    // 参数预定义
    int count = 0; // 循环次数
    double x1 = x0;
    double x2 = 0;
    double e_result = 0; // 每次迭代误差
    double Loxy = std::sqrt(p_actual_projected(1) * p_actual_projected(1) + p_actual_projected(2) * p_actual_projected(2)) / RXB_l_mm; // 125为臂杆长度，后续需要替换为参数

    while (count <= N_max) // 当没有达到最大循环次数时
    {
        count++;

        // 计算曲线上x1处的切线与x轴的交点
        x2 = x1 - (u(x1) - Loxy) / dudt(x1);
        e_result = std::abs(u(x2) - Loxy);

        // std::cout << std::fixed << std::setprecision(8) << "x2: " << x2 << std::endl;
        // std::cout << "e_result: " << e_result << std::endl;

        // 判断交点是否为函数零点
        if (e_result <= e) // 若小于允许误差
        {
            break; // x2为零点
        }
        else if (count > N_max) // 若超过最大迭代次数
            {
                std::cout << "Function returns no solution!\n";
            }

        x1 = x2; // 将算出来的点作为下次循环带入的值
    }

    // 由辅助变量计算theta值
    double theta_cal = 2 * std::atan(x2); // 函数的零点是由万能公式代换过的

    // 返回函数零点
    return theta_cal;
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：计算JR1【已验证】
/// 返回类型：
///		Matrix2d JR1	-- dφ、dθ到dPy、dPz的系数矩阵
/// 函数参数：
///		double l		-- 柔性臂杆长
///		double phi		-- 运动平面夹角（y轴绕x轴转动角度）
///		double theta	-- 两个臂杆之间等效运动角度
Eigen::Matrix2d SingleSegController::RXB_SegCL_JR1_Calculate(const double& l, const double& phi, const double& theta)
{
    double u = l * ((1 - std::cos(4 * theta)) / (2 * std::tan(theta / 2)) + std::sin(4 * theta) / 2);
    double A = l * (	  std::cos(    theta)
                    + 2 * std::cos(2 * theta)
                    + 3 * std::cos(3 * theta)
                    + 4 * std::cos(4 * theta));

    Eigen::Matrix2d JR1;
    JR1 << - u * std::sin(phi), A * std::cos(phi),
             u * std::cos(phi), A * std::sin(phi);

    return JR1;
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：由φ、θ解算q1、q2【已验证】
/// 返回类型：
///		void			-- 无返回值
/// 函数参数：
///		double phi		-- [输入]等效φ角
///		double theta	-- [输入]等效θ角
///		double qz1		-- [输出]计算出的等效qz1角
///		double qy2		-- [输出]计算出的等效qy2角
void SingleSegController::RXB_SegCL_phitheta2PY(const double& phi, const double& theta, double& qz1, double& qy2)
{
    qz1 = std::atan2(std::tan(theta) * std::cos(phi), 1);
    qy2 = -std::atan2(std::tan(phi) * std::sin(qz1), 1); // 表达式不唯一，但其他表达式可能会出现相差180°的问题
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：计算JR2【已验证】
/// 返回类型：
///		Matrix2d JR2	-- dq1、dq2到dφ、dθ的系数矩阵
/// 函数参数：
///		double q1		-- 单个关节绕z轴运动角度
///		double q2		-- 单个关节绕y轴运动角度
Eigen::Matrix2d SingleSegController::RXB_SegCL_JR2_Calculate(const double& q1, const double& q2)
{
    Eigen::Matrix2d JR2;
    if (abs(q2) <= 1e-4) // q2为零时需要单独处理
    {
        JR2 <<									   0, - 1 / std::sin(q1),
               std::abs(std::tan(q1)) / std::tan(q1),				   0;
    }
    else
    {
        double den1 = 1 + (std::sin(q1) * std::sin(q1)) / (std::tan(q2) * std::tan(q2));
        double den2 = std::sqrt(- 1 + 1 / (std::cos(q1) * std::cos(q1) * std::cos(q2) * std::cos(q2)));

        JR2 << cos(q1) / (tan(q2) * den1), - sin(q1) / (sin(q2) * sin(q2) * den1),
                           tan(q1) / den2,						   tan(q2) / den2;
    }

    return JR2;
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：段末端闭环补偿，比较段末端理想位置和段末端实际位置计算得到补偿运动量【已验证】
/// 返回类型：
///		void						-- 无返回值
/// 函数参数：
///		Vector3d p_actual_projected	-- [输入]段末端实际位置投影到理想工作平面上的位置
///		Vector3d p_desire			-- [输入]段末端期望位置
///		Vector2d q_equivalent		-- [输出]等效理想运动角度，数据格式：{qz1_cal, qy2_cal}，单位：角度
///		Vector2d dq_compensate		-- [输出]需要补偿的角度，数据格式：{dqz1_cal, dqy2_cal}，单位：角度
void SingleSegController::RXB_SegCL_Compensate(Eigen::Vector3d& p_actual_projected,
                          Eigen::Vector3d& p_desire,
                          Eigen::Vector2d& q_equivalent,
                          Eigen::Vector2d& dq_compensate)
{
    // 计算理想工作平面上的期望位置与投影位置的距离差值
    Eigen::Vector3d dP = p_desire - p_actual_projected;

    // 计算JR
    //		计算JR1
    //			由投影位置计算运动平面夹角φ
    double phi_cal = atan2(p_actual_projected(2), p_actual_projected(1));
    //			利用牛顿迭代法计算得到关节等效运动角度θ
    double x0 = 0; // 迭代初始点
    double e = 1e-8; // 迭代允许误差
    int N_max = 10; // 最大迭代次数
    double theta_cal = RXB_SegCL_P2Theta_NM(p_actual_projected, x0, e, N_max); // 利用牛顿法计算θ
    //			由微分表达式计算JR1
    Eigen::Matrix2d JR1_cal = RXB_SegCL_JR1_Calculate(RXB_l_mm, phi_cal, theta_cal);

    //		计算JR2
    //			由φ、θ计算得到qz1、qy2
    double qz1_cal = 0;
    double qy2_cal = 0;
    RXB_SegCL_phitheta2PY(phi_cal, theta_cal, qz1_cal, qy2_cal);
    q_equivalent << qz1_cal * rd, qy2_cal* rd; // 等效运动角度q_equivalen在这里被赋值！
    //			由微分表达式计算JR2
    Eigen::Matrix2d JR2_cal = RXB_SegCL_JR2_Calculate(qz1_cal, qy2_cal);


    // 计算dqz1、dqy2
    //		计算dφ、dθ
    Eigen::Vector2d dP2_dP3 = {dP(1), dP(2)};
    Eigen::Vector2d dphi_dtheta_cal = JR1_cal.inverse() * dP2_dP3;
    //		计算dqz1、dqy2
    dq_compensate = JR2_cal.inverse() * dphi_dtheta_cal * rd; // 补偿角度dq_compensate在这里被赋值！
}

/// ------------------------------------------------------------------------------------------------
/// 函数功能：段闭环的接口程序（将段闭环功能打包），输入实际关节角，输出补偿的关节角【待完成】
/// 返回类型：
///		void					-- 无返回值
/// 函数参数：
///		Vector2d theta_ideal	-- 【输入】期望关节角度，需要从ROS中程序得到
///		VectorXd theta_actual	-- 【输入】实际关节角度，需要根据不同的RXB调整具体大小
///		Vector2d dq_compensate	-- 【输出】计算出的PY方向待补偿的角度
void SingleSegController::RXB_SegmentalCloseLoopControl(Eigen::Vector2d& theta_ideal_temp,
                                   Eigen::VectorXd& theta_actual,
                                   Eigen::Vector2d& dq_compensate)
{
    // 参数设置
    int local_encoder_num_ttl = encoder_num_all; // 一个臂段内的传感器总数
    int flag_RXB_base = 1; // 用于判断是否为基座的标志

    // 计算理想情况下段末端的位置
    Eigen::VectorXd theta_ideal(local_encoder_num_ttl);
    theta_ideal << theta_ideal_temp(0), theta_ideal_temp(1), theta_ideal_temp(1), theta_ideal_temp(0),
                   theta_ideal_temp(0), theta_ideal_temp(1), theta_ideal_temp(1), theta_ideal_temp(0); // 按照PYYPPYYP的格式装入theta_ideal中
    //		计算齐次变换矩阵
    Eigen::Matrix4d T_seg_d = RXB_T_Segment(RXB_l_mm, theta_ideal, flag_RXB_base);
    //		整理得到理想段末端位置
    Eigen::Vector3d p_desire;
    p_desire = T_seg_d.col(3).segment(0, 3); // 将T中第（3 + 1）列，从第（0 + 1）个元素开始的三个元素提取出来，作为p
    std::cout << "The Desired Point is: " << p_desire[2] << "\t" << -p_desire[1] << "\t" << p_desire[0] << std::endl;

    Eigen::MatrixXd desiredT;
    desiredT = transformM * T_seg_d * transformM;
    std_msgs::Float32MultiArray desiredT_msg;
    for(long i = 0; i < 3; i++){
        for(long j = 0; j < 4; j++){
            desiredT_msg.data.push_back(static_cast<float>(desiredT(i, j)));
        }
    }
    desiredT_pub.publish(desiredT_msg);

    // 计算实际情况下段末端的位置
    //		计算齐次变换矩阵
    Eigen::Matrix4d T_seg_a = RXB_T_Segment(RXB_l_mm, theta_actual, flag_RXB_base);
    //		整理得到实际段末端位置
    Eigen::Vector3d p_actual;
    p_actual = T_seg_a.col(3).segment(0, 3); // 将T中第（3 + 1）列，从第（0 + 1）个元素开始的三个元素提取出来，作为p

    // 计算误差速度旋量
    Eigen::Matrix4d errorT = T_seg_a.inverse() * T_seg_d;
    Eigen::VectorXd errorTwist = mr::se3ToVec(mr::MatrixLog6(errorT));
    std_msgs::Float32MultiArray errorTwist_msg;
    errorTwist_msg.data.push_back(static_cast<float>(errorTwist[2]));
    errorTwist_msg.data.push_back(static_cast<float>(-errorTwist[1]));
    errorTwist_msg.data.push_back(static_cast<float>(errorTwist[0]));
    errorTwist_msg.data.push_back(static_cast<float>(errorTwist[5]));
    errorTwist_msg.data.push_back(static_cast<float>(-errorTwist[4]));
    errorTwist_msg.data.push_back(static_cast<float>(errorTwist[3]));
    for(unsigned long i = 0; i < 3; i++){
        errorTwist_msg.data[i] *= static_cast<float>(rd);
    }
    errorTwist_pub.publish(errorTwist_msg);

    // 段末端补偿计算
    //		将实际段末端位置投影到理想工作平面上
    int file_input_flag = 1; // 用于判断是否从文件中读取数据
    Eigen::Vector3d p_actual_projected; // 用于存放投影结果
    p_actual_projected.setZero();
    double vertical_distance = 0; // 投影点与实际点的距离
    RXB_Project2IdealPlane_GM2(file_input_flag, p_actual, p_actual_projected, vertical_distance);
    //		计算单段需要补偿的角度值
    Eigen::Vector2d q_equivalent; // 存放等效角度
    q_equivalent.setZero();
    RXB_SegCL_Compensate(p_actual_projected, p_desire, q_equivalent, dq_compensate);
}

}
