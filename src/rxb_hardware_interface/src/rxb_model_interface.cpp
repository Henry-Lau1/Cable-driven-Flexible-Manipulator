#include "math.h"
#include <list>
#include <vector>
#include <iostream>
#include "rxb_model_interface.h"
#include "modern_robotics.h"

namespace RxbHardwareInterface {

    void jointPlan2CablePlan(std::list<jointState> &RXB_joint_plan, std::list<cableState> &RXB_Cable_Plan) {
        cableState cs;
        jointState js;
        std::list<jointState>::iterator itea;
        for(itea = RXB_joint_plan.begin(); itea != RXB_joint_plan.end(); itea++) {
            js = *itea;
            RXB_JntAngle_To_Cable_Length_new(js.j, cs.len);
            cs.t = js.t;
            cs.duration = js.duration;
            RXB_Cable_Plan.push_back(cs);
        }
        for(int i=0;i<Motor_num_all;++i){
          std::cout<<"绳索长度" << i << ": " << cs.len[i] << "\t"<<std::endl;
        }
        std::cout<<std::endl;
    }

    //  const float F_big=1,F_small=1.00;//绳索长度变化 大于零的乘以F_big，小于零的F_small；
    void RXB_JntAngle_To_Cable_Length_new(double *JointAngle, double *CableLengthCalcted ) {
        double Cable_Position[3];//储存绳孔方位角
        double r = 0;//绳孔分布半径
        int i = 0, k = 0;//迭代计数
        int Segment_Count;//关节段计数
        int Sg_num = g_RXBSegment_num;//关节段的总数
        double TemCableLength[3];
        double TemJntAngle[2];

        for (i = 0; i < Motor_num_all; i++) { //所有的元素置零
            CableLengthCalcted[i] = 0;
        }

        for (Segment_Count = 1; Segment_Count <= Sg_num; Segment_Count++) {//对每段关节段
            r = 0.5 * g_Cable_Distribute_Diameter[Segment_Count - 1]; //绳孔分布的半径
            TemJntAngle[0] = JointAngle[2 * (Segment_Count - 1)] * Deg2Rad; //Pitch关节角，单位rad
            TemJntAngle[1] = JointAngle[2 * (Segment_Count - 1)+1] * Deg2Rad; //Yaw关节角，单位rad

            for (i = Segment_Count; i <= Sg_num; i++) {//第Segment_Count段分别对经过其的后段所有驱动绳索的影响(理解的关键)
                //计算绳孔的位置,单位rad
                Cable_Position[0] = (g_CableAngleOnDisk[i - 1]) * Deg2Rad;
                Cable_Position[1] = Cable_Position[0] + 2*PI/3;
                Cable_Position[2] = Cable_Position[1] + 2*PI/3;

                RXB_CableLenOfSingleModuJoint(1, Cable_Position, r, TemJntAngle, TemCableLength); //PY类关节
                for(k = 0; k < 3; k++) { //对应绳长，速度，加速度叠加
                    CableLengthCalcted[3 * i - 3 + k] = CableLengthCalcted[3 * i - 3 + k] + g_RXBSegment_PY_num[Segment_Count-1] * TemCableLength[k];
                }

                RXB_CableLenOfSingleModuJoint(2, Cable_Position, r, TemJntAngle, TemCableLength); //YP类关节
                for(k = 0; k < 3; k++) { //对应绳长，速度，加速度叠加
                    CableLengthCalcted[3 * i - 3 + k] = CableLengthCalcted[3 * i - 3 + k] + g_RXBSegment_YP_num[Segment_Count-1] * TemCableLength[k];
                }
                //std::cout <<  CableLengthCalcted[(Segment_Count-1)*3+i] << std::endl;
            }
        }

//        for (i = 0; i < 3 * Sg_num; i++){
//            std::cout << i << " : " << CableLengthCalcted[i]<< std::endl;
//        }
    }

    /*void RXB_JntAngle_To_Cable_Length_new(double *JointAngle, double *CableLengthCalcted ) {
        double Cable_Position[3];//储存绳孔方位角
        double r = 0;//绳孔分布半径
        int i = 0, k = 0;//迭代计数
        int Segment_num;//关节段计数
        int YP_Joint_num, PY_Joint_num;//YP关节总数和PY关节总数
        int Sg_num = g_RXBSegment_num;//关节段的总数
        double TemCableLength[3];
        double TemJntAngle[2];

        if((g_RXBSegment_Joint_num/2)%2 == 1){
            YP_Joint_num = (g_RXBSegment_Joint_num/2)/2 + 1;
            PY_Joint_num = g_RXBSegment_Joint_num/2 - YP_Joint_num;
        }else {
            PY_Joint_num = YP_Joint_num = g_RXBSegment_Joint_num/4;
        }

        for (i = 0; i < Motor_num_all; i++) { //所有的元素置零
            CableLengthCalcted[i] = 0;
        }


        for (Segment_num = 1; Segment_num <= Sg_num; Segment_num++) {
            r = 0.5 * g_Cable_Distribute_Diameter[Segment_num - 1]; //绳孔分布的半径
            for (i = Segment_num; i <= Sg_num; i++) {//第segment_num段对经过其的所有绳索的影响(理解的关键)
                //第一组控制绳索长度计算
                //计算绳孔的位置
                Cable_Position[0] = g_CableAngleOnDisk_FirstGroup[i - 1];
                Cable_Position[1] = Cable_Position[0] + 120;
                Cable_Position[2] = Cable_Position[1] + 120;
                //Y-P configuration 对应基座的那种关节类型
                TemJntAngle[0] = JointAngle[2 * Segment_num - 2]; //填充关节角
                TemJntAngle[1] = JointAngle[2 * Segment_num - 1];
                RXB_CableLenOfSingleModuJoint(1, Cable_Position, r, TemJntAngle, TemCableLength); //1类关节

                for(k = 0; k < 3; k++) { //对应绳长，速度，加速度叠加
                    CableLengthCalcted[6 * i - 6 + k] = CableLengthCalcted[6 * i - 6 + k] + YP_Joint_num * TemCableLength[k];
                }

                //P-Y configuration，对应1类关节
                TemJntAngle[0] = JointAngle[2 * Segment_num - 1];
                TemJntAngle[1] = JointAngle[2 * Segment_num - 2];
                RXB_CableLenOfSingleModuJoint(2, Cable_Position, r, TemJntAngle, TemCableLength); //2类关节

                for(k = 0; k < 3; k++) { //对应绳长，速度，加速度叠加
                    CableLengthCalcted[6 * i - 6 + k] = CableLengthCalcted[6 * i - 6 + k] + PY_Joint_num * TemCableLength[k];
                }
                //std::cout <<  CableLengthCalcted[(Segment_num-1)*3+i] << std::endl;

                //第二组控制绳索长度计算
                Cable_Position[0] = g_CableAngleOnDisk_SecondGroup[i - 1];
                Cable_Position[1] = Cable_Position[0] + 120;
                Cable_Position[2] = Cable_Position[1] + 120;

                if(i == Segment_num){
                    TemJntAngle[0] = JointAngle[2 * Segment_num - 2]; //填充关节角
                    TemJntAngle[1] = JointAngle[2 * Segment_num - 1];
                    RXB_CableLenOfSingleModuJoint(1, Cable_Position, r, TemJntAngle, TemCableLength); //1类关节
                    for(k = 0; k < 3; k++) { //对应绳长，速度，加速度叠加
                        CableLengthCalcted[6 * i - 3 + k] = CableLengthCalcted[6 * i - 3 + k] + TemCableLength[k];
                    }
                }else {
                    //Y-P configuration 对应基座的那种关节类型
                    TemJntAngle[0] = JointAngle[2 * Segment_num - 2]; //填充关节角
                    TemJntAngle[1] = JointAngle[2 * Segment_num - 1];
                    RXB_CableLenOfSingleModuJoint(1, Cable_Position, r, TemJntAngle, TemCableLength); //1类关节
                    for(k = 0; k < 3; k++) { //对应绳长，速度，加速度叠加
                        CableLengthCalcted[6 * i - 3 + k] = CableLengthCalcted[6 * i - 3 + k] + YP_Joint_num * TemCableLength[k];
                    }
                    //P-Y configuration，对应1类关节
                    TemJntAngle[0] = JointAngle[2 * Segment_num - 1];
                    TemJntAngle[1] = JointAngle[2 * Segment_num - 2];
                    RXB_CableLenOfSingleModuJoint(2, Cable_Position, r, TemJntAngle, TemCableLength); //2类关节

                    for(k = 0; k < 3; k++) { //对应绳长，速度，加速度叠加
                        CableLengthCalcted[6 * i - 3 + k] = CableLengthCalcted[6 * i - 3 + k] + PY_Joint_num * TemCableLength[k];
                    }
                }
            }
        }
    }*/



    /* 计算关节段的绳子长度 */
    //参考《modern robotics》中的旋量运动学
    void RXB_CableLenOfSingleModuJoint(const int ModularJointNum, const double Cable_Position[3], const double r, const double JointAngle[2], double *CableLengthCalcted) {
        const int PYJoint_flag = 1;//PY关节标志
        const int YPJoint_flag = 2;//YP关节标志
        const double d = g_Disk_Distance;//圆盘间距，不计圆盘厚度
        int i;//计数

        Eigen::Matrix4d M;//零位齐次变换矩阵
        M <<    1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, d,
                0, 0, 0, 1;

        Eigen::MatrixXd Slist(6, 2);//广义旋转轴组成的矩阵
        Eigen::Vector2d thetaList;//旋转角度向量
        Eigen::Matrix4d T;//旋转过后对应绳孔位置坐标系之间的齐次变换矩阵
        Eigen::Vector3d p;//两绳孔之间的位移

        for (i = 0; i < 3; i++){
            if (YPJoint_flag == ModularJointNum){//YP
                Slist <<    1, 0,
                            0, 1,
                            0, 0,
                            0, -0.5*d,
                            0.5*d, 0,
                            r*sin(Cable_Position[i]), -r*cos(Cable_Position[i]);

                thetaList <<    JointAngle[1], JointAngle[0];
            }else if(PYJoint_flag == ModularJointNum){//PY
                Slist <<    0, 1,
                            1, 0,
                            0, 0,
                            -0.5*d,0,
                            0, 0.5*d,
                            -r*cos(Cable_Position[i]), r*sin(Cable_Position[i]);

                thetaList <<    JointAngle[0], JointAngle[1];
            }
            T = mr::FKinSpace(M, Slist, thetaList);
            p = T.block<3,1>(0,3);
            CableLengthCalcted[i] = sqrt(p.dot(p)) - d;
        }
    }

    /*void RXB_CableLenOfSingleModuJoint(int ModularJointNum, double Cable_Position[3], double r, double *JointAngle, double *CableLengthCalcted) {
        float d = 0;
        char k, i, j;
        float Cable_Pos, beta;
        float fai, arfa, d_fai, d_arfa, d2_arfa, d2_fai;
        float P1, P2, P3, TemCableLen;
        float darfa_P1, dfai_P1, darfa_P2, dfai_P2, darfa_P3, dfai_P3;
        float d2arfa_P1, d2fai_P1, d2arfa_P2, d2fai_P2, d2arfa_P3, d2fai_P3;
        float CableVelo_arfa, CableVelo_fai, CableAcc_arfa, CableAcc_fai;
        int Coh_arfa, Coh_fai;
        d = g_Disk_Distance * 0.5; //圆盘距离

        if ((ModularJointNum % 2) == 1) { //Y-P configuration
            arfa = JointAngle[0] / 180 * PI; //%rafa
            fai = -JointAngle[1] / 180 * PI; //%fai
            Coh_arfa = 1;
            Coh_fai = -1;
        } else { //P-Y configuration
            arfa = JointAngle[0] / 180 * PI;
            fai = JointAngle[1] / 180 * PI;
            Coh_arfa = 1;
            Coh_fai = 1;
        }



        for (k = 0; k < 3; k++) {
            Cable_Pos = Cable_Position[k];
            beta = Cable_Pos / 180.0 * PI;
            if ((ModularJointNum % 2) == 1) {
                beta = (Cable_Pos / 180.0 - 0.5) * PI;
            }
            P1 = r * cos(beta) * sin(arfa) * sin(fai) + r * cos(arfa) * sin(beta) - d * sin(arfa) * cos(fai) - r * sin(beta);
            P2 = d * sin(fai) + r * cos(fai) * cos(beta) - r * cos(beta);
            P3 = d * cos(fai) * cos(arfa) + d + r * sin(beta) * sin(arfa) - r * cos(beta) * cos(arfa) * sin(fai);
            TemCableLen = pow(P1, 2) + pow(P2, 2) + pow(P3, 2);
            ///绳长计算//////
            CableLengthCalcted[k] = pow(TemCableLen, (float)(0.5)) - 2 * d; //绳长计算



            darfa_P1 = (r * cos(beta) * cos(arfa) * sin(fai) - r * sin(arfa) * sin(beta) - d * cos(arfa) * cos(fai)) * d_arfa; //对arfa进行求导
            dfai_P1 = (r * cos(beta) * sin(arfa) * cos(fai) + d * sin(arfa) * sin(fai)) * d_fai; //对fai进行求导
            darfa_P2 = 0; //对arfa进行求导
            dfai_P2 = (d * cos(fai) - r * sin(fai) * cos(beta)) * d_fai; //对fai进行求导
            darfa_P3 = (-d * cos(fai) * sin(arfa) + r * sin(beta) * cos(arfa) + r * cos(beta) * sin(arfa) * sin(fai)) * d_arfa; //对arfa进行求导
            dfai_P3 = (-d * sin(fai) * cos(arfa) - r * cos(beta) * cos(arfa) * cos(fai)) * d_fai; //对fai进行求导
            CableVelo_arfa = 2 * P1 * darfa_P1 + 2 * P2 * darfa_P2 + 2 * P3 * darfa_P3;
            CableVelo_fai = 2 * P1 * dfai_P1 + 2 * P2 * dfai_P2 + 2 * P3 * dfai_P3;
            ///绳索速度计算//////
            //CableVeloCalcted[k] = 0.5 * (pow((float)TemCableLen, (float) -0.5)) * (CableVelo_arfa + CableVelo_fai); //绳索速度的计算



            d2arfa_P1 = (-r * cos(beta) * sin(arfa) * sin(fai) - r * cos(arfa) * sin(beta) + d * sin(arfa) * cos(fai)) * d2_arfa; //对arfa进行求2次导
            d2fai_P1 = (-r * cos(beta) * sin(arfa) * sin(fai) + d * sin(arfa) * cos(fai)) * d2_fai; //对fai进行求2次导
            d2arfa_P2 = 0; //对arfa进行求2次导
            d2fai_P2 = (-d * sin(fai) - r * cos(fai) * cos(beta)) * d2_fai; //对fai进行求2次导
            d2arfa_P3 = (-d * cos(fai) * cos(arfa) - r * sin(beta) * sin(arfa) + r * cos(beta) * cos(arfa) * sin(fai)) * d2_arfa; //对arfa进行求2次导
            d2fai_P3 = (-d * cos(fai) * cos(arfa) + r * cos(beta) * cos(arfa) * sin(fai)) * d2_fai; //对fai进行求2次导

            CableAcc_arfa = 2 * darfa_P1 * darfa_P1 + 2 * P1 * d2arfa_P1 + 2 * darfa_P2 * darfa_P2 + 2 * P2 * d2arfa_P2 + 2 * darfa_P3 * darfa_P3 + 2 * P3 * d2arfa_P3;
            CableAcc_fai = 2 * dfai_P1 * dfai_P1 + 2 * P1 * d2fai_P1 + 2 * dfai_P2 * dfai_P2 + 2 * P2 * d2fai_P2 + 2 * dfai_P3 * dfai_P3 + 2 * P3 * d2fai_P3;
            ///绳索加速度计算//////
//            CableVccCalcted[k] = 0.5 * (pow((float)TemCableLen, (float) -1.5)) * (pow(CableVelo_arfa, 2) + pow(CableVelo_fai, 2)) +
//                                 0.5 * (pow((float)TemCableLen, (float) -0.5)) * (CableAcc_arfa + CableAcc_fai); //绳索加速度的计算

        }//结束


    }*/

    //已知当前关节角，通过关节角速度计算绳索角速度
    void RXB_JntState_To_CableVel(double Jnt_Angle[], double dJnt_Angle[], double Cable_v[]) {
        cableState cs1, cs2;
        RXB_JntAngle_To_Cable_Length_new(Jnt_Angle, cs1.len);
        double Jnt_Angle2[Controllable_DOF_num_all];
        for(int i = 0; i < Controllable_DOF_num_all; ++i) {
            Jnt_Angle2[i] = Jnt_Angle[i] + dJnt_Angle[i] * global_T_Interval_s;
        }
        RXB_JntAngle_To_Cable_Length_new(Jnt_Angle2, cs2.len);

        for(int i = 0; i < Motor_num_all; ++i) {
            Cable_v[i] = (cs2.len[i] - cs1.len[i]) / global_T_Interval_s;
        }

        //std::cout << dJnt_Angle[8] << "\t" << dJnt_Angle[9] << "\t" << Cable_v[13]*CableLen2MotorPos << "\t" << Cable_v[14]*CableLen2MotorPos << "\t";

    }
}
