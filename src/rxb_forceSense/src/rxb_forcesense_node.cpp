//头文件保存位置
#include "rxb_forcesense_node.h"

namespace rxb_forcesense{
  // 全局变量
  bool rxb_forcesense_node_running = false;

  // 函数
  double deg2rad(double degrees) {
      return degrees * M_PI / 180.0;
  }
  vector<double>& deg2rad(vector<double>&& degrees){
    for(unsigned long i = 0; i < degrees.size(); i++){
      degrees[i] = deg2rad(degrees[i]);
    }
    return degrees;
  }
  double rad2deg(double radians){
    return radians * 180.0 / M_PI;
  }

  //类Database相关成员变量及成员函数
  Database::Database(const ros::NodeHandle& nh_) : nh(nh_), spinner(1){
    mass.resize(segNum*jointNum);
    inertia.resize(segNum*jointNum);
    GIM.resize(segNum*jointNum);
    SetMassInertia();

    jointAngleMeasured.resize(segNum*jointNum, 0);
    actCableForceMeasured.resize(3*segNum, 0);
    corCableForceMeasured.resize(4*segNum, 0);

    // 对应发布者，用于参考
    // serial_encoder_pub = nh_.advertise<rxb_msgs::sensorRead>("serialport/Sensor_Encoder_data", 1);
    // serial_pub = nh_.advertise<rxb_msgs::sensorRead>("serialport/sensor_data", 1);
    //创建ros消息发布订阅者
    jointAngleMeasured_sub = nh.subscribe<rxb_msgs::sensorRead>("serialport/Sensor_Encoder_data", 1, &Database::JointAngleMeasured_ReceiveFunc, this);
    cableForceMeasured_sub = nh.subscribe<rxb_msgs::sensorRead>("serialport/sensor_data", 1, &Database::CableForceMeasured_ReceiveFunc, this);

    spinner.start();
  }
  void Database::JointAngleMeasured_ReceiveFunc(const rxb_msgs::sensorRead::ConstPtr& jointAngle){
    for(unsigned long i = 0; i < segNum*jointNum; i++){
      jointAngleMeasured[i] = deg2rad(jointAngle->joint_angle[i]);
    }
  }
  void Database::CableForceMeasured_ReceiveFunc(const rxb_msgs::sensorRead::ConstPtr& cableForce){
    // 此处根据硬件系统设定，须随硬件系统进行改动
    //驱动绳顺序：30度， 150度， 270度
    actCableForceMeasured[0] = cableForce->cable_force[18];
    actCableForceMeasured[1] = cableForce->cable_force[19];
    actCableForceMeasured[2] = cableForce->cable_force[20];

    //联动绳
    corCableForceMeasured[0] = cableForce->cable_force[16]; // 小8正方向
    corCableForceMeasured[1] = cableForce->cable_force[17]; // 小8负方向
    corCableForceMeasured[2] = cableForce->cable_force[14]; // 大8正方向
    corCableForceMeasured[3] = cableForce->cable_force[15]; // 小8负方向
  }
  void Database::Disp(void) const {
    cout << fixed << setprecision(6); //统一输出格式，全局有效
    cout << "Class Database: jointAngle = ";
    for(auto& jAM : jointAngleMeasured){
      cout << jAM << " ";
    }
    cout << endl;

    cout << "Class Database: actCableForce = ";
    for(auto& aCFM : actCableForceMeasured){
      cout << aCFM << " ";
    }
    cout << endl;

    cout << "Class Database: corCableForce = ";
    for(auto& cCFM : corCableForceMeasured){
      cout << cCFM << " ";
    }
    cout << endl;;
  }
  void Database::SetMassInertia(void){
    const double mass_crossA = 0.0841;  // 十字轴质量，单位：千克
    const double mass_rod = 0.2472; // 中间连杆质量，单位：千克
    const double mass_ex = 0.4809;  // 末端六维力传感器装配，进行补偿后的质量，单位：千克

    // 相对于SW模型质量估计中的坐标系，需要转变到运动学建模坐标系
    Matrix<double, 3, 3> inertia_crossA; // 十字轴惯性矩阵，量纲：千克，米
    Matrix<double, 3, 3> inertia_rod; // 中间连杆惯性矩阵，量纲：千克，米
    Matrix<double, 3, 3> inertia_ex;  // 末端六维力传感器装配，进行补偿后的惯性矩阵，量纲：千克，米
    inertia_crossA << 34.7230, -0.1390, -0.0907, -0.1390, 34.2914, -0.0238, -0.0901, -0.0226, 56.9060;
    inertia_crossA /= 1000000;
    inertia_rod << 378.7743, 0, 0, 0, 396.4197, -5.3302, 0, -5.3302, 293.1281;
    inertia_rod /= 1000000;
    inertia_ex.setZero();
    inertia_ex.diagonal() << 1470.7686, 1413.7622, 579.1939;
    inertia_ex /= 1000000;

    vector<Matrix<double, 3, 3>> R(segNum*jointNum);
    R[0] << 0, -1, 0, -1, 0, 0, 0, 0, -1;
    R[1] << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    R[2] << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    R[3] << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    R[4] << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    R[5] << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    R[6] << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    R[7] << 0, 1, 0, -1, 0, 0, 0, 0, 1;

    for(unsigned long i = 0; i < segNum*jointNum; i++){
      if(i == segNum*jointNum-1){  // 末端
        mass[i] = mass_ex;
        inertia[i] = inertia_ex;
      }else if(i%2 == 0){ // 十字轴
        mass[i] = mass_crossA;
        inertia[i] = R[i] * inertia_crossA * R[i].transpose();
      }else{  // 中间连杆
        mass[i] = mass_rod;
        inertia[i] = R[i] * inertia_rod * R[i].transpose();
      }
      GIM[i].block<3, 3>(0, 0) = inertia[i];
      GIM[i].block<3, 3>(0, 3).setZero();
      GIM[i].block<3, 3>(3, 0).setZero();
      GIM[i].block<3, 3>(3, 3) = mass[i] * Matrix3d::Identity();
    }
  }

  // 类SlidingWindowSmooth相关成员变量及成员函数
  SlidingWindowSmooth::SlidingWindowSmooth(const string& name_, unsigned long dim_): name(name_), dim(dim_){
    sum.resize(dim, 0);
    result.resize(dim, 0);
  }
  void SlidingWindowSmooth::AddData(const vector<double>& data){
    // 错误检测
    if(data.size() != dim){
      cerr << "Class SlidingWindowSmooth Function AddData wrong." << endl;
      return;
    }

    // 加入新的数据
    dq.push_back(data);
    for(unsigned long i = 0; i < dim; i++){
      sum[i] += data[i];
    }

    // 处理滑动窗口
    unsigned long dqSize = dq.size();
    if(dqSize > windowSize){
      for(unsigned long i = 0; i < dim; i++){
        sum[i] -= dq[0][i];
      }
      dq.pop_front();
      dqSize--;
    }

    // 更新结果
    for(unsigned long i = 0; i < dim; i++){
      result[i] = sum[i] / dqSize;
    }

    return;
  }
  void SlidingWindowSmooth::Disp(void) const{
    cout << name << " result: ";
    for(unsigned long i = 0; i < dim; i++){
      cout << result[i] << "  ";
    }
    cout << endl;
  }

  // 类KalmanFilter相关成员变量及成员函数
  KalmanFilter::KalmanFilter(const string& name_, const unsigned long dim_, const Database& db, const vector<double>& startValue, const vector<double>& Q_)
    : name(name_), dim(dim_){
    if(startValue.size() != dim_){
      cerr << "Class KalmanFilter Function KalmanFilter wrong." << endl;
      return;
    }

    A << 1, db.dt, 0.5*db.dt*db.dt,
         0, 1, db.dt,
         0, 0, 1;
    H << 1, 0, 0;
    Q.setZero();
    Q.diagonal() << Q_[0], Q_[1], Q_[2];
    R << 0.1;

    x.resize(dim);
    for(unsigned long i = 0; i < dim; i++){
      x[i] << startValue[i], 0, 0;
    }
    P.resize(dim);
    for(unsigned long i = 0; i < dim; i++){
      P[i].setZero();
      P[i].diagonal() << 1, 1, 1;
    }

    result.resize(dim);
  }
  void KalmanFilter::Predict(void){
    for(unsigned long i = 0; i < dim; i++){
      x[i] = A * x[i];
      P[i] = A * P[i] * A.transpose() + Q;
    }
  }
  void KalmanFilter::Update(const vector<double>& z_){
    for(unsigned long i = 0; i < dim; i++){
      Matrix<double, 1, 1> z;
      z << z_[i];

      MatrixXd K = P[i]*H.transpose() * (H*P[i]*H.transpose() + R).inverse();
      x[i] = x[i] + K*(z-H*x[i]);
      P[i] = (MatrixXd::Identity(3, 3) - K*H) * P[i];
      result[i] = x[i];
    }
  }
  void KalmanFilter::AddData(const SlidingWindowSmooth& sws){
    if(sws.dim != dim){
      cerr << "Class KalmanFilter Funciton AddData wrong." << endl;
      return;
    }

    Predict();
    Update(sws.result);
  }
  void KalmanFilter::Disp(void) const{
    cout << name << "----------------------------------------" << endl;
    for(unsigned long i = 0; i < dim; i++){
      cout << "ID " << i << ":";   
      cout << result[i].transpose();
      cout << endl;
    }
  }

  // 类BodyState相关成员变量及成员函数
  BodyState::BodyState(const Database& db): dim(db.segNum*db.jointNum){
    M.resize(dim);
    vel.resize(dim);
    acc.resize(dim);
    axis.resize(dim);
    T.resize(dim);
    GF.resize(dim);
    GIM.resize(dim);

    for(unsigned long i = 0; i < dim; i++){
      M[i] = Matrix<double, 4, 4>::Identity();
      if(i == 0){
        M[i](2, 3) = -db.L_base;
      }else if(i == dim-1){
        M[i](2, 3) = -(db.L_ex);
      }else{
        M[i](2, 3) = -db.L/2;
      }
    }
    vel_0 << 0, 0, 0, 0, 0, 0;
    acc_0 << 0, 0, 0, 0, 0, 9.81;
    axis[0] << 1, 0, 0, 0, 0, 0;
    axis[1] << 0, 1, 0, db.L/2, 0, 0;
    axis[2] << 0, 1, 0, 0, 0, 0;
    axis[3] << 1, 0, 0, 0, -db.L/2, 0;
    axis[4] << 1, 0, 0, 0, 0, 0;
    axis[5] << 0, 1, 0, db.L/2, 0, 0;
    axis[6] << 0, 1, 0, 0, 0, 0;
    axis[7] << 1, 0, 0, 0, -db.L_ex, 0;
    for(unsigned long i = 0; i < dim; i++){
      GIM[i] = db.GIM[i];
    }
  }
  void BodyState::CalT(const KalmanFilter &thetaKF){
    for(unsigned long i = 0; i < dim; i++){
      T[i] = mr::MatrixExp6(-mr::VecTose3(axis[i]) * thetaKF.result[i](0)) * M[i];
    }
  }
  void BodyState::CalVel(const KalmanFilter &thetaKF){
    for(unsigned long i = 0; i < dim; i++){
      if(i == 0)
        vel[i] = axis[i]*thetaKF.result[i](1) + mr::Adjoint(T[i])*vel_0;
      else
        vel[i] = axis[i]*thetaKF.result[i](1) + mr::Adjoint(T[i])*vel[i-1];
    }
  }
  void BodyState::CalAcc(const KalmanFilter &thetaKF){
    for(unsigned long i = 0; i < dim; i++){
      if (i==0)
        acc[i] = axis[i]*thetaKF.result[i](2) + mr::ad(vel[i])*axis[i]*thetaKF.result[i](1) + mr::Adjoint(T[i])*acc_0;
      else
        acc[i] = axis[i]*thetaKF.result[i](2) + mr::ad(vel[i])*axis[i]*thetaKF.result[i](1) + mr::Adjoint(T[i])*acc[i-1];
    }
  }
  void BodyState::CalGF(void){
    for(unsigned long i = 0; i < dim; i++){
      GF[i] = GIM[i] * acc[i] - mr::ad(vel[i]).transpose() * GIM[i] * vel[i];
    }
  }
  void BodyState::AddData(const KalmanFilter &thetaKF){
    CalT(thetaKF);
    CalVel(thetaKF);
    CalAcc(thetaKF);
    CalGF();
  }
  void BodyState::Disp() const{
    cout << "Class BodyState: " << "---------------------------------" << endl;
    for(unsigned long i = 0; i < dim; i++){
      cout << "ID " << i << "----" << endl;
      cout << M[i] << endl;
      cout << T[i] << endl;
      cout << vel[i].transpose() << endl;
      cout << acc[i].transpose() << endl;
      cout << GF[i].transpose() << endl;
      cout << endl;
    }
  }

  // 类ActCable相关成员变量及成员函数
  ActCable::ActCable(const Database& db, const KalmanFilter& thetaKF, const vector<double>& holeAngle_)
    : dim(3*db.segNum), segNum(db.segNum), jointNum(db.jointNum), L_disk(db.L_disk), r_hole(db.r_hole), dt(db.dt), holeAngle(holeAngle_){
    M << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 2*L_disk, 0, 0, 0, 1;
    segControl.resize(dim);
    S_Y.resize(dim);
    S_P.resize(dim);
    for(unsigned long i = 0; i < dim; i++){
      segControl[i] = i/3 + 1;
      S_Y[i] << 1, 0, 0, 0, L_disk, r_hole*sin(holeAngle[i]);
      S_P[i] << 0, 1, 0, -L_disk, 0, -r_hole*cos(holeAngle[i]);
    }

    dis2End.resize(dim);
    dis2End_last.resize(dim);
    speed2End.resize(dim);
    cableAngle.resize(dim);
    cableDir.resize(dim);
    cableMeasured.resize(dim);
    cableTension.resize(dim);
    cableFriction.resize(dim);
    for(unsigned long i = 0; i < dim; i++){
      dis2End[i].resize(segControl[i]*jointNum);
      dis2End_last[i].resize(segControl[i]*jointNum);
      speed2End[i].resize(segControl[i]*jointNum);
      cableAngle[i].resize(segControl[i]*jointNum);
      cableDir[i].resize(segControl[i]*jointNum);
      cableTension[i].resize(segControl[i]*jointNum);
      cableFriction[i].resize(segControl[i]*jointNum);
    }

    CalCableDis(thetaKF);
  }
  vector<double> ActCable::CalCableDis_YP(const double theta_P, const double theta_Y, const unsigned long ca_num){
    vector<double> dis(dim);
    for(unsigned long i = 0; i < dim; i++){
      Matrix<double, 4, 4> T;
      T = mr::MatrixExp6(mr::VecTose3(S_Y[i])*theta_Y) * mr::MatrixExp6(mr::VecTose3(S_P[i])*theta_P) * M;

      Vector3d z;
      z << 0, 0, 1;

      Vector3d p = T.block<3, 1>(0, 3);
      Matrix<double, 3, 3> R = T.block<3, 3>(0, 0);
      dis[i] = p.norm();
      cableDir[i][2*ca_num] = p/dis[i];
      cableAngle[i][2*ca_num] = acos(cableDir[i][2*ca_num].dot(z));
      p = R.transpose() * (-p);
      cableDir[i][2*ca_num+1] = p/dis[i];
      cableAngle[i][2*ca_num+1] = M_PI - acos(cableDir[i][2*ca_num+1].dot(z));
    }
    return dis;
  }
  vector<double> ActCable::CalCableDis_PY(const double theta_P, const double theta_Y, const unsigned long ca_num){
    vector<double> dis(dim);
    for(unsigned long i = 0; i < dim; i++){
      if(ca_num >= segControl[i]*jointNum/2)
        continue;

      Matrix<double, 4, 4> T;
      T = mr::MatrixExp6(mr::VecTose3(S_P[i])*theta_P) * mr::MatrixExp6(mr::VecTose3(S_Y[i])*theta_Y) * M;

      Vector3d z;
      z << 0, 0, 1;

      Vector3d p = T.block<3, 1>(0, 3);
      Matrix<double, 3, 3> R = T.block<3, 3>(0, 0);
      dis[i] = p.norm();
      cableDir[i][2*ca_num] = p/dis[i];
      cableAngle[i][2*ca_num] = acos(cableDir[i][2*ca_num].dot(z));
      p = R.transpose() * (-p);
      cableDir[i][2*ca_num+1] = p/dis[i];
      cableAngle[i][2*ca_num+1] = M_PI - acos(cableDir[i][2*ca_num+1].dot(z));
    }
    return dis;
  }
  void ActCable::CalCableDis(const KalmanFilter& thetaKF){
    dis2End_last = dis2End;
    for(auto& row : dis2End){
      for(auto& val : row){
        val = 0.0;
      }
    }

    vector<double> add;
    for(unsigned long i = 0; i < segNum*jointNum/2; i++){ // 对于每一个十字轴
      if(i%2 == 0){  // YP
        add = CalCableDis_YP(thetaKF.result[2*i+1](0), thetaKF.result[2*i](0), i);
        for(unsigned long j = 0; j < dim; j++){ // 对于每根驱动绳
          if(i >= segControl[j]*jointNum/2){
            continue;
          }
          for(unsigned long z = 0; z <= i; z++){
            if(z != 0)
              dis2End[j][2*z-1] += add[j];
            dis2End[j][2*z] += add[j];
          }
        }
      }else{  // PY
        add = CalCableDis_PY(thetaKF.result[2*i](0), thetaKF.result[2*i+1](0), i);
        for(unsigned long j = 0; j < dim; j++){ // 对于每根驱动绳
          if(i >= segControl[j]*jointNum/2){
            continue;
          }
          for(unsigned long z = 0; z <= i; z++){
            if(z != 0)
              dis2End[j][2*z-1] += add[j];
            dis2End[j][2*z] += add[j];
          }
        }
      }
    }
    for(unsigned long i = 0; i < dim; i++){
      for(unsigned long j = 0; j < segControl[i]*jointNum; j++){
        speed2End[i][j] = (dis2End[i][j] - dis2End_last[i][j]) / dt;
      }
    }
  }
  void ActCable::CalCableTen(const KalmanFilter& forceKF){
    for(unsigned long cableID = 0; cableID < dim; cableID++){
      cableMeasured[cableID] = forceKF.result[cableID](0);
      for(unsigned long i = 0; i < segControl[cableID]*jointNum; i++){
        if(speed2End[cableID][i] > 0){ // 松绳
          double mu = -7.23e-8*pow(rad2deg(cableAngle[cableID][i]), 5) + 3.37e-6*pow(rad2deg(cableAngle[cableID][i]), 4) - 3.53e-5*pow(rad2deg(cableAngle[cableID][i]), 3) + 1.77e-4*pow(rad2deg(cableAngle[cableID][i]), 2) + 7.59e-3*rad2deg(cableAngle[cableID][i]) + 0.010;
          mu /= 4;
          if(speed2End[cableID][i] >= 0.25e-3){  // 库伦+粘滞模型
            if(i == 0){
              cableTension[cableID][i] = (mu+1)*cableMeasured[cableID] + K_v*rad2deg(cableAngle[cableID][i])*speed2End[cableID][i];
              cableFriction[cableID][i] = cableTension[cableID][i] - cableMeasured[cableID];
            }else{
              cableTension[cableID][i] = (mu+1)*cableTension[cableID][i-1] + K_v*rad2deg(cableAngle[cableID][i])*speed2End[cableID][i];
              cableFriction[cableID][i] = cableTension[cableID][i] - cableTension[cableID][i-1];
            }
          }else{  // Dahl模型
            double F_coulomb;
            if(i == 0){
              F_coulomb = mu*cableMeasured[cableID];
            }else{
              F_coulomb = mu*cableTension[cableID][i-1];
            }

            if(cableFriction[cableID][i] > F_coulomb){
              cableFriction[cableID][i] = F_coulomb;
            }else{
              cableFriction[cableID][i] = cableFriction[cableID][i] + sigma*(1-cableFriction[cableID][i]/F_coulomb)*speed2End[cableID][i]*dt;
              if(cableFriction[cableID][i] > F_coulomb){
                cableFriction[cableID][i] = F_coulomb;
              }
            }

            if(i == 0){
              cableTension[cableID][i] = cableMeasured[cableID] + cableFriction[cableID][i];
            }else{
              cableTension[cableID][i] = cableTension[cableID][i-1] + cableFriction[cableID][i];
            }
          }
        }else if(speed2End[cableID][i] < 0){ // 拉绳
          double mu = -(3.48e-7*pow(rad2deg(cableAngle[cableID][i]), 5) - 2.29e-5*pow(rad2deg(cableAngle[cableID][i]), 4) + 4.64e-4*pow(rad2deg(cableAngle[cableID][i]), 3) - 3.52e-3*pow(rad2deg(cableAngle[cableID][i]), 2) - 3.33e-3*rad2deg(cableAngle[cableID][i]) - 0.004);
          mu /= 4;
          if(speed2End[cableID][i] <= -0.25e-3){  // 库伦+粘滞模型
            if(i == 0){
              cableTension[cableID][i] = (cableMeasured[cableID]+K_v*rad2deg(cableAngle[cableID][i])*speed2End[cableID][i]) / (mu+1);
              cableFriction[cableID][i] = cableTension[cableID][i] - cableMeasured[cableID];
            }else{
              cableTension[cableID][i] = (cableTension[cableID][i-1]+K_v*rad2deg(cableAngle[cableID][i])*speed2End[cableID][i]) / (mu+1);
              cableFriction[cableID][i] = cableTension[cableID][i] - cableTension[cableID][i-1];
            }
          }else{  // Dahl模型
            double F_coulomb;
            if(i == 0){
              F_coulomb = mu/(mu+1) * cableMeasured[cableID];
            }else{
              F_coulomb = mu/(mu+1) * cableTension[cableID][i-1];
            }

            if(cableFriction[cableID][i] < -F_coulomb){
              cableFriction[cableID][i] = -F_coulomb;
            }else{
              cableFriction[cableID][i] = cableFriction[cableID][i] + sigma*(1+cableFriction[cableID][i]/F_coulomb)*speed2End[cableID][i]*dt;
              if(cableFriction[cableID][i] < -F_coulomb){
                cableFriction[cableID][i] = -F_coulomb;
              }
            }

            if(i == 0){
              cableTension[cableID][i] = cableMeasured[cableID] + cableFriction[cableID][i];
            }else{
              cableTension[cableID][i] = cableTension[cableID][i-1] + cableFriction[cableID][i];
            }
          }
        }else{  // 绳索位置没有发生变化
          if(i == 0){
            cableTension[cableID][i] = cableMeasured[cableID] + cableFriction[cableID][i];
          }else{
            cableTension[cableID][i] = cableTension[cableID][i-1] + cableFriction[cableID][i];
          }
        }
      }
    }
  }
  void ActCable::Disp(void){
    for(unsigned long i = 0; i < dim; i++){
      cout << "Class ActCable: ID " << i << "--------------------" << endl;

      cout << "dis2End = ";
      for(unsigned long j = 0; j < segControl[i]*jointNum; j++){
        cout << dis2End[i][j] << "  ";
      }
      cout << endl;

      cout << "speed2End = ";
      for(unsigned long j = 0; j < segControl[i]*jointNum; j++){
        cout << speed2End[i][j] << "  ";
      }
      cout << endl;

      cout << "cableTension = ";
      for(unsigned long j = 0; j < segControl[i]*jointNum; j++){
        cout << cableTension[i][j] << "  ";
      }
      cout << endl;

      cout << "cableFriction = ";
      for(unsigned long j = 0; j < segControl[i]*jointNum; j++){
        cout << cableFriction[i][j] << "  ";
      }
      cout << endl;
    }
  }

  // 类BCorCable相关成员变量及成员函数
  BCorCable::BCorCable(const Database& db, const KalmanFilter& thetaKF, const KalmanFilter& forceKF)
    : dim(db.segNum*2), jointNum(db.jointNum), dt(db.dt){
    wrapAngle.resize(dim);
    wrapAngle_last.resize(dim);
    cableSpeed.resize(dim);
    cableMeasured.resize(dim);
    cableTension.resize(dim);
    cableFriction.resize(dim);

    UpdateInf(thetaKF, forceKF);
  }
  void BCorCable::UpdateInf(const KalmanFilter &thetaKF, const KalmanFilter &forceKF){
    wrapAngle_last = wrapAngle;
    for(unsigned long i = 0; i < dim/2; i++){ // 对于每一段
      wrapAngle[2*i] = wrapAngle_0 - thetaKF.result[jointNum*i+jointNum-1](0);
      wrapAngle[2*i+1] = wrapAngle_0 + thetaKF.result[jointNum*i+jointNum-1](0);
      cableMeasured[2*i] = forceKF.result[4*i+2](0);
      cableMeasured[2*i+1] = forceKF.result[4*i+3](0);
    }
  }
  void BCorCable::CalCableTen(const KalmanFilter& thetaKF, const KalmanFilter& forceKF){
    UpdateInf(thetaKF, forceKF);
    for(unsigned long i = 0; i < dim; i++){
      cableSpeed[i] = radius * (wrapAngle[i] - wrapAngle_last[i]) / dt;
      if(abs(cableSpeed[i]) >= 0.25e-3){
        cableTension[i] = (cableMeasured[i] + yita*radius*cableSpeed[i]/mu*(signbit(cableSpeed[i])?-1:1)) * exp(-mu*wrapAngle[i]*(signbit(cableSpeed[i])?-1:1)) - yita*radius*cableSpeed[i]/mu*(signbit(cableSpeed[i])?-1:1);
        cableFriction[i] = cableTension[i] - cableMeasured[i];
      }else if(abs(cableSpeed[i]) < 0.25e-3 && cableSpeed[i] != 0.0){
        double F_coulomb = abs(cableMeasured[i] * (exp(-mu*wrapAngle[i]*(signbit(cableSpeed[i])?-1:1)) - 1));
        if(cableSpeed[i] > 0 && cableFriction[i] < -F_coulomb){
          cableFriction[i] = -F_coulomb;
        }else if(cableSpeed[i] < 0 && cableFriction[i] > F_coulomb){
          cableFriction[i] = F_coulomb;
        }else{
          cableFriction[i] = cableFriction[i] + sigma*(1-cableFriction[i]/F_coulomb*(signbit(cableSpeed[i])?1:-1))*(-cableSpeed[i])*dt;
          if(cableSpeed[i] > 0 && cableFriction[i] < -F_coulomb){
            cableFriction[i] = -F_coulomb;
          }else if(cableSpeed[i] < 0 && cableFriction[i] > F_coulomb){
            cableFriction[i] = F_coulomb;
          }
        }

        cableTension[i] = cableMeasured[i] + cableFriction[i];
      }else{
        cableTension[i] = cableMeasured[i] + cableFriction[i];
      }
    }
  }
  void BCorCable::Disp(void){
    cout << "Class BCorCable" <<  "------------------------------" << endl;
    cout << "wrapAngle =";
    for(unsigned long i = 0; i < dim; i++){
      cout << "  " << wrapAngle[i];
    }
    cout << endl;

    cout << "cableSpeed =";
    for(unsigned long i = 0; i < dim; i++){
      cout << "  " << cableSpeed[i];
    }
    cout << endl;

    cout << "cableMeasured =";
    for(unsigned long i = 0; i < dim; i++){
      cout << "  " << cableMeasured[i];
    }
    cout << endl;

    cout << "cableTension =";
    for(unsigned long i = 0; i < dim; i++){
      cout << "  " << cableTension[i];
    }
    cout << endl;

    cout << "cableFriction =";
    for(unsigned long i = 0; i < dim; i++){
      cout << "  " << cableFriction[i];
    }
    cout << endl;
  }

  // 类DynamicEquation相关成员变量及成员函数，只适用于当前特定结构
  DynamicEquation::DynamicEquation(const ros::NodeHandle& nh_, const Database& db, const ActCable& ac, unsigned long segID_)
    : nh(nh_), segNum(db.segNum), jointNum(db.jointNum), segID(segID_), holeRadius(db.r_hole){
    forceSenseResult_pub = nh.advertise<rxb_msgs::forceSenseResult>("force_sense/result", 1);
    forceSenseZero_sub = nh.subscribe<std_msgs::UInt32>("force_sense/zero", 1, &DynamicEquation::ForceSenseZero_ReceiveFunc, this);
    holeAngle.resize(3*(segNum-segID));
    actTension.resize(3*(segNum-segID));
    actCableDir.resize(3*(segNum-segID));
    rActCable.resize(3*(segNum-segID));
    for(unsigned long i = 0; i < 3*(segNum-segID); i++){
      holeAngle[i] = ac.holeAngle[3*segID + i];
      rActCable[i] << holeRadius*cos(holeAngle[i]), holeRadius*sin(holeAngle[i]), -cableDiskDis-mcDisComp;
    }
    result_0[0] = 0.0;
    result_0[1] = 0.0;
  }
  void DynamicEquation::UpdateInf(const KalmanFilter& thetaKF, const KalmanFilter& forceKF, const BCorCable& bcc, const ActCable& ac, const BodyState& bs){
    theta_Y = thetaKF.result[jointNum*segID + jointNum - 1](0);
    theta_P = thetaKF.result[jointNum*segID + jointNum - 2](0);
    sColForce[0] = forceKF.result[4*segID+0](0);
    sColForce[1] = forceKF.result[4*segID+1](0);
    bColForce[0] = forceKF.result[4*segID+2](0);
    bColForce[1] = forceKF.result[4*segID+3](0);
    bColTension[0] = bcc.cableTension[2*segID];
    bColTension[1] = bcc.cableTension[2*segID +1];
    resGF[0] = bs.GF[jointNum*segID+jointNum-2];
    resGF[1] = bs.GF[jointNum*segID+jointNum-1];
    actGF.setZero();
    for(unsigned long i = 0; i < 3*(segNum-segID); i++){
      actTension[i] = ac.cableTension[3*segID+i][jointNum*segID+jointNum-2];
      actCableDir[i] = ac.cableDir[3*segID+i][jointNum*segID+jointNum-1];
      actGF.block<3, 1>(3, 0) += actTension[i] * actCableDir[i];
      actGF.block<3, 1>(0, 0) += rActCable[i].cross(actTension[i]*actCableDir[i]);
    }
  }
  void DynamicEquation::CalA(void){
    A <<
    1, 0,               0,               0, 0, 0, 0,                     0,                    0,               0,                                      0,                                       0,
    0, 0,   -cos(theta_Y),    sin(theta_Y), 0, 0, 0,                     0,                    0,               0,                                      0,                                       0,
    0, 1,   -sin(theta_Y),   -cos(theta_Y), 0, 0, 0,                     0,                    0,               0,                                      0,                                       0,
    0, 0,               0,               0, 0, 0, 0,                     0, linkLength+mcDisComp,               0,                                      0, linkLength-mcDisComp+forcePointDisComp,
    0, 0,               1,               0, 0, 0, 0, -linkLength-mcDisComp,                    0,               0, mcDisComp-linkLength-forcePointDisComp,                                       0,
    0, 0,               0,               1, 0, 0, 0,                     0,                    0,               0,                                      0,                                       0,
    0, 0,               0,               0, 1, 0, 0,                    -1,                    0,               0,                                      0,                                       0,
    0, 0,               0,               0, 0, 1, 0,                     0,        -cos(theta_Y),    sin(theta_Y),                                      0,                                       0,
    0, 0,               0,               0, 0, 0, 1,                     0,        -sin(theta_Y),   -cos(theta_Y),                                      0,                                       0,
    0, 0,               0,               0, 0, 0, 0,                     1,                    0,               0,                                     -1,                                       0,
    0, 0,               0,               0, 0, 0, 0,                     0,                    1,               0,                                      0,                                      -1,
    0, 0,               0,               0, 0, 0, 0,                     0,                    0,               1,                                      0,                                       0;
  }
  void DynamicEquation::Calb(void){
    b <<
    resGF[0](0) - bColForce[1]*bColRadius*pow(cos(bColOutAngle_0 + theta_Y),2) - sColDis*sColForce[0]*sin(sColOutAngle_0 - theta_P) - bColForce[1]*bColRadius*pow(sin(bColOutAngle_0 + theta_Y), 2) - bColRadius*bColTension[0]*pow(cos(bColInAngle_0), 2) + bColRadius*bColTension[1]*pow(cos(bColInAngle_0), 2) - bColRadius*bColTension[0]*pow(sin(bColInAngle_0), 2) + bColRadius*bColTension[1]*pow(sin(bColInAngle_0), 2) + bColForce[0]*bColRadius*pow(cos(bColOutAngle_0 - theta_Y), 2) + bColForce[0]*bColRadius*pow(sin(bColOutAngle_0 - theta_Y), 2) + sColDis*sColForce[1]*sin(sColOutAngle_0 + theta_P),
    - sColForce[0]*sColRadius*pow(cos(sColOutAngle_0 - theta_P), 2) - sColForce[0]*sColRadius*pow(sin(sColOutAngle_0 - theta_P), 2) + sColForce[1]*sColRadius*pow(cos(sColOutAngle_0 + theta_P), 2) + sColForce[1]*sColRadius*pow(sin(sColOutAngle_0 + theta_P), 2) + resGF[0](1),
    resGF[0](2) - sColDis*sColForce[0]*cos(sColOutAngle_0 - theta_P) - sColDis*sColForce[1]*cos(sColOutAngle_0 + theta_P),
    resGF[1](0) - actGF(0) + bColForce[0]*sin(bColOutAngle_0)*(bColDis + mcDisComp) - bColForce[1]*sin(bColOutAngle_0)*(bColDis + mcDisComp),
    resGF[1](1) - actGF(1),
    resGF[1](2) - actGF(2),
    resGF[0](3) + sColForce[1]*cos(sColOutAngle_0 + theta_P) - sColForce[0]*cos(sColOutAngle_0 - theta_P),
    resGF[0](4) + bColForce[1]*sin(bColOutAngle_0 + theta_Y) + bColTension[0]*sin(bColInAngle_0) - bColTension[1]*sin(bColInAngle_0) - bColForce[0]*sin(bColOutAngle_0 - theta_Y),
    resGF[0](5) - bColForce[1]*cos(bColOutAngle_0 + theta_Y) + sColForce[1]*sin(sColOutAngle_0 + theta_P) + bColTension[0]*cos(bColInAngle_0) + bColTension[1]*cos(bColInAngle_0) - bColForce[0]*cos(bColOutAngle_0 - theta_Y) + sColForce[0]*sin(sColOutAngle_0 - theta_P),
    resGF[1](3) - actGF(3),
    resGF[1](4) - actGF(4) + bColForce[0]*sin(bColOutAngle_0) - bColForce[1]*sin(bColOutAngle_0),
    resGF[1](5) - actGF(5) + bColForce[0]*cos(bColOutAngle_0) + bColForce[1]*cos(bColOutAngle_0);
  }
  void DynamicEquation::SolveEq(void){
    BDCSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
    Matrix<double, 12, 12> U = svd.matrixU();
    Matrix<double, 12, 12> V = svd.matrixV();
    Matrix<double, 12, 1> S_diag = svd.singularValues();
    double S_diag_max = S_diag.maxCoeff();
    Matrix<double, 12, 1> S_diag_pInv = S_diag.unaryExpr([&S_diag_max](double s){
      if(abs(S_diag_max/s) >= 1000.0)
        return 0.0;
      else
        return 1.0/s;
    });
    Matrix<double, 12, 12> S_pInv = MatrixXd::Zero(A.rows(), A.cols());
    S_pInv.diagonal() = S_diag_pInv;
    solution = V*S_pInv*U.transpose() * b;
    result[0] = solution(10)-result_0[0];
    result[1] = solution(11)-result_0[1];
  }
  void DynamicEquation::Advertise(void){
    rxb_msgs::forceSenseResult fSR;
    fSR.ID = static_cast<unsigned char>(segID);
    fSR.force.resize(2*segNum);
    fSR.force[0] = result[0];
    fSR.force[1] = result[1];
    forceSenseResult_pub.publish(fSR);
  }
  void DynamicEquation::ForceSense(const KalmanFilter &thetaKF, const KalmanFilter &forceKF, const BodyState &bs, const ActCable &ac, const BCorCable &bcc){
    UpdateInf(thetaKF, forceKF, bcc, ac, bs);
    CalA();
    Calb();
    SolveEq();
    Advertise();
  }
  void DynamicEquation::Disp(void){
    cout << "Class DynamicEquation------------------------------------" << endl;
    cout << A << endl;
    cout << b.transpose() << endl;
    cout << solution.transpose() << endl;
    cout << "result =  " << result[0] << "  " << result[1] << endl;
  }
  void DynamicEquation::ForceSenseZero_ReceiveFunc(const std_msgs::UInt32::ConstPtr& data){
    if(data->data){
      result_0[0] = solution(10);
      result_0[1] = solution(11);
    }
  }

  // 与matlab对比，验证算法的正确性
  void Test(ros::NodeHandle nh){
    Database db(nh);
    SlidingWindowSmooth jointAngleSmooth("jointAngleSmooth", db.segNum*db.jointNum);
    SlidingWindowSmooth actCableForceSmooth("actCableForceSmooth", 3 * db.segNum);
    SlidingWindowSmooth corCableForceSmooth("corCableForceSmooth", 4 * db.segNum);
    KalmanFilter jointAngleKalman("jointAngleKalman", db.segNum*db.jointNum, db, db.jointAngleMeasured, {1, 5, 10});
    KalmanFilter actCableForceKalman("actCableForceKalman", 3*db.segNum, db, db.actCableForceMeasured, {1, 5, 10});
    KalmanFilter corCableForceKalman("corCableForceKalman", 4*db.segNum, db, db.corCableForceMeasured, {1, 5, 10});
    BodyState bs(db);
    ActCable ac(db, jointAngleKalman, deg2rad({30, 150, 270}));
    BCorCable bcc(db, jointAngleKalman, corCableForceKalman);
    DynamicEquation de(nh, db, ac, 0);

    vector<double> jointData(db.segNum*db.jointNum, 0);
    for(unsigned long i = 0; i < 10; i++){
        for(unsigned long j = 0; j < jointData.size(); j++){
            jointData[j] = deg2rad(i*0.1+j*1);
        }
        for(unsigned long j = 0; j < actCableForceKalman.result.size(); j++){
            actCableForceKalman.result[j][0] = i*1+j*10;
        }
        for(unsigned long j = 0; j < corCableForceKalman.result.size(); j++){
            corCableForceKalman.result[j][0] = i*10;
        }

        jointAngleSmooth.AddData(jointData);
        jointAngleSmooth.Disp();    // 已验证

        jointAngleKalman.AddData(jointAngleSmooth);
        jointAngleKalman.Disp();    // 已验证

        bs.AddData(jointAngleKalman);
        bs.Disp();  // 已验证

        ac.CalCableDis(jointAngleKalman);
        ac.CalCableTen(actCableForceKalman);
        ac.Disp();  // 已验证

        bcc.CalCableTen(jointAngleKalman, corCableForceKalman);
        bcc.Disp(); // 已验证

        de.ForceSense(jointAngleKalman, corCableForceKalman, bs, ac, bcc);
        de.Disp();  // 已验证
    }
  }
}


using namespace rxb_forcesense;

int main(int argc, char **argv){
  std::cout << "C++ Standard: " << __cplusplus << std::endl;

  //创建ros节点及ros句柄
  ros::init(argc, argv, "rxb_forceSense_node");
  ros::NodeHandle nh;

  // 激活节点运行标志
  rxb_forcesense_node_running = true;

  // 与matlab对比测试代码正确性
  // Test(nh);

  Database db(nh);
  SlidingWindowSmooth jointAngleSmooth("jointAngleSmooth", db.segNum*db.jointNum);
  SlidingWindowSmooth actCableForceSmooth("actCableForceSmooth", 3 * db.segNum);
  SlidingWindowSmooth corCableForceSmooth("corCableForceSmooth", 4 * db.segNum);
  KalmanFilter jointAngleKalman("jointAngleKalman", db.segNum*db.jointNum, db, db.jointAngleMeasured, {1, 5, 10});
  KalmanFilter actCableForceKalman("actCableForceKalman", 3*db.segNum, db, db.actCableForceMeasured, {1, 5, 10});
  KalmanFilter corCableForceKalman("corCableForceKalman", 4*db.segNum, db, db.corCableForceMeasured, {1, 5, 10});
  BodyState bs(db);
  ActCable ac(db, jointAngleKalman, deg2rad({30, 150, 270}));
  BCorCable bcc(db, jointAngleKalman, corCableForceKalman);
  DynamicEquation de(nh, db, ac, 0);

  // 等待回调函数执行完成，避免Database一开始的数据为0
  using namespace std::literals;
  this_thread::sleep_for(1s);

  unsigned long count = 0;  //计数器
  while(rxb_forcesense_node_running){
    jointAngleSmooth.AddData(db.jointAngleMeasured);
    actCableForceSmooth.AddData(db.actCableForceMeasured);
    corCableForceSmooth.AddData(db.corCableForceMeasured);

    jointAngleKalman.AddData(jointAngleSmooth);
    actCableForceKalman.AddData(actCableForceSmooth);
    corCableForceKalman.AddData(corCableForceSmooth);

    bs.AddData(jointAngleKalman);

    ac.CalCableDis(jointAngleKalman);
    ac.CalCableTen(actCableForceKalman);

    bcc.CalCableTen(jointAngleKalman, corCableForceKalman);

    de.ForceSense(jointAngleKalman, corCableForceKalman, bs, ac, bcc);

    if(count++ % 50 == 0){
      db.Disp();
//      jointAngleSmooth.Disp();
//      actCableForceSmooth.Disp();
//      corCableForceSmooth.Disp();
      jointAngleKalman.Disp();
      actCableForceKalman.Disp();
      corCableForceKalman.Disp();
      bs.Disp();
      ac.Disp();
      bcc.Disp();
      de.Disp();
      cout << endl;
    }

    this_thread::sleep_for(20ms);
  }

  return 0;
}
