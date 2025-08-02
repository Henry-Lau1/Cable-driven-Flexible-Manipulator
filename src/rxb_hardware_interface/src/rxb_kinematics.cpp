#include "rxb_kinematics.h"


namespace RxbHardwareInterface {
//旋转矩阵到四元数
    geometry_msgs::Pose RotMat2Quatern(Eigen::Matrix4d T) {
        geometry_msgs::Pose pose;
        Eigen::Matrix3d R = T.topLeftCorner(3, 3);
        Eigen::Quaterniond q = Eigen::Quaterniond(R);
        q.normalize();
        pose.position.x = T(0, 3);
        pose.position.y = T(1, 3);
        pose.position.z = T(2, 3);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    }

//四元数到旋转矩阵
    Eigen::Matrix4d Quatern2RotMat(geometry_msgs::Pose pose) {
        Eigen::Matrix4d T = T.Zero();
        Eigen::Quaterniond q ;
        q.x() = pose.orientation.x ;
        q.y() = pose.orientation.y ;
        q.z() = pose.orientation.z ;
        q.w() = pose.orientation.w ;
        Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        T.Zero();
        T.topLeftCorner(3, 3) = R;
        T.rightCols(1) << pose.position.x, pose.position.y, pose.position.z, 1;

        return T;
    }

    void DH_table(double theta, int link_num, double link_len, Eigen::Vector4d &DH_Paras) {
        Eigen::Matrix<double, 4, 4> Tem_DH_Table;
        Eigen::Vector4d DH_Table;
        int type;
        //len , alpha, offset, theta
        Tem_DH_Table << 0, -90, 0, theta,
                     link_len, 0, 0, theta,
                     0, 90, 0, theta,
                     link_len, 0, 0, theta;
        type = link_num % 4;// 0 1 2 3
        DH_Paras << Tem_DH_Table(type, 0), Tem_DH_Table(type, 1), Tem_DH_Table(type, 2), Tem_DH_Table(type, 3);

    }

    void DH_Mat(double len, double alpha, double offset, double theta, Eigen::Matrix4d &tmp_Ti) {
        Eigen::Matrix4d Ti;
        double theta_t = (theta / 180) * M_PI;
        double alpha_t = (alpha / 180) * M_PI;
        double lambda = cos(alpha_t);
        double mu = sin(alpha_t);
        tmp_Ti << cos(theta_t), -lambda *sin(theta_t), mu *sin(theta_t), len *cos(theta_t),
               sin(theta_t), lambda *cos(theta_t), -mu *cos(theta_t), len *sin(theta_t),
               0, mu, lambda, offset,
               0, 0, 0, 1;

    }

    Eigen::Matrix<double, 6, Controllable_DOF_num_all>  Jacob_Mat(std::vector<Eigen::Matrix4d> &Ti) {
        int m = Ti.size();
        Eigen::Matrix<double, 3, Controllable_DOF_num_all> crs_zr, z, r;
        Eigen::Matrix<double, 6, Controllable_DOF_num_all> J;
        for (int i = 0; i < Controllable_DOF_num_all; i++) {
            if (i == 0) {//列
                z.block<3, 1>(0, i) << 0, 0, 1;
                r.block<3, 1>(0, i) = Ti[m - 1].block<3, 1>(0, 3);
            } else {
                z.block<3, 1>(0, i) = Ti[i - 1].block<3, 1>(0, 2);
                r.block<3, 1>(0, i) = Ti[m - 1].block<3, 1>(0, 3) - Ti[i - 1].block<3, 1>(0, 3);
            }
            crs_zr.block<3, 1>(0, i) = z.block<3, 1>(0, i).cross(r.block<3, 1>(0, i));
        }
        J.block<3, Controllable_DOF_num_all>(0, 0) = crs_zr;//Jv
        J.block<3, Controllable_DOF_num_all>(3, 0) = z;//Jw
        return J;
    }
    /*
    Eigen::Matrix<double, 6, Controllable_DOF_num_all> Jacob_Mat_Seg(std::vector<Eigen::Matrix4d> &Ti){

        int m = Ti.size();
        Eigen::Matrix<double, 3, ModuJnt_num_all> w1, v1, w2, v2;
        Eigen::Matrix<double, 6, ModuJnt_num_all> J;
        Eigen::Matrix<double, 3, 1> Pe = Ti[m - 1].block<1, 3>(0,3);
        for(int i = 0; i < ModuJnt_num_all; i++){
            if(i == 0){
                w1.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i + 0].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 3].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 4].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 7].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 8].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 11].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 12].block<1, 3>(0,2);
                    v1.block<3, 1>(0, i) = Pe + Ti[RXBSegment_Joint_num*i + 3].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 3].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 4].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 4].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 7].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 7].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 8].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 8].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 11].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 11].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 12].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 12].block<1, 3>(0,3));
            }else{
                w1.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i + 0].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 3].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 4].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 7].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 8].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 11].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 12].block<1, 3>(0,2);
                v1.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i + 0].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 0].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 3].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 3].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 4].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 4].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 7].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 7].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 8].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 8].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 11].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 11].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 12].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 12].block<1, 3>(0,3));
            }
            w2.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i + 1].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 2].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 5].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 6].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 9].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 10].block<1, 3>(0,2) + Ti[RXBSegment_Joint_num*i + 13].block<1, 3>(0,2);
            v2.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i + 1].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 1].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 2].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 2].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 5].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 5].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 6].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 6].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 9].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 9].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 10].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 10].block<1, 3>(0,3)) + Ti[RXBSegment_Joint_num*i + 13].block<1, 3>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 13].block<1, 3>(0,3));
            J.block<3, 1>(0, 2*i) = v1;
            J.block<3, 1>(3, 2*i) = w1;
            J.block<3, 1>(0, 2*i + 1) = v2;
            J.block<3, 1>(3, 2*i + 1) = w2;
        }
        return J;

    }
    */
    Eigen::Matrix<double, 6, Controllable_DOF_num_all> Jacob_Mat_Seg(std::vector<Eigen::Matrix4d> &Ti){

        int m = Ti.size();
        Eigen::Matrix<double, 3, ModuJnt_num_all> w1, v1, w2, v2;
        Eigen::Matrix<double, 6, Controllable_DOF_num_all> J;
        Eigen::Matrix<double, 3, 1> Pe = Ti[m - 1].block<3, 1>(0,3);
        Eigen::Matrix<double, 3, 1> zero; zero.block<3, 1>(0, 0) << 0, 0, 1;

        for(int i = 0; i < ModuJnt_num_all; i++){
            if(i == 0){
                w1.block<3, 1>(0, i) = zero + Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0,2);
                v1.block<3, 1>(0, i) = Pe + Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0,3));

            }else{
                w1.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i - 1].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0,2);
                v1.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i - 1].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i - 1].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0,3));
            }
            w2.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i + 0].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 1].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 4].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 5].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 8].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 9].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 12].block<3, 1>(0,2);
            v2.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i + 0].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 0].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 1].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 1].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 4].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 4].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 5].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 5].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 8].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 8].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 9].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 9].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 12].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 12].block<3, 1>(0,3));
            J.block<3, 1>(0, 2*i) = v1.block<3, 1>(0, i);
            J.block<3, 1>(3, 2*i) = w1.block<3, 1>(0, i);
            J.block<3, 1>(0, 2*i + 1) = v2.block<3, 1>(0, i);
            J.block<3, 1>(3, 2*i + 1) = w2.block<3, 1>(0, i);
        }

        return J;
    }
    Eigen::Matrix<double, 6, 2> Jacob_Mat_Seg_oneSeg(std::vector<Eigen::Matrix4d> &Ti){

        int m = Ti.size();
        Eigen::Matrix<double, 3, 1> w1, v1, w2, v2;
        Eigen::Matrix<double, 6, 2> J;
        Eigen::Matrix<double, 3, 1> Pe = Ti[m - 1].block<3, 1>(0,3);
        Eigen::Matrix<double, 3, 1> zero; zero.block<3, 1>(0, 0) << 0, 0, 1;

        for(int i = 0; i < 1; i++){
            if(i == 0){
                w1.block<3, 1>(0, i) = zero + Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0,2);
                v1.block<3, 1>(0, i) = Pe + Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0,3));

            }else{
                w1.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i - 1].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0,2);
                v1.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i - 1].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i - 1].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0,3));
            }
            w2.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i + 0].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 1].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 4].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 5].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 8].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 9].block<3, 1>(0,2) + Ti[RXBSegment_Joint_num*i + 12].block<3, 1>(0,2);
            v2.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i + 0].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 0].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 1].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 1].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 4].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 4].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 5].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 5].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 8].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 8].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 9].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 9].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 12].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 12].block<3, 1>(0,3));
            J.block<3, 1>(0, 2*i) = v1.block<3, 1>(0, i);
            J.block<3, 1>(3, 2*i) = w1.block<3, 1>(0, i);
            J.block<3, 1>(0, 2*i + 1) = v2.block<3, 1>(0, i);
            J.block<3, 1>(3, 2*i + 1) = w2.block<3, 1>(0, i);
        }

        return J;
    }


    Eigen::Matrix<double, 3, Controllable_DOF_num_all> Jacob_Mat_Seg_OnlyPos(std::vector<Eigen::Matrix4d> &Ti){

        int m = Ti.size();
        Eigen::Matrix<double, 3, ModuJnt_num_all> v1, v2;
        Eigen::Matrix<double, 3, Controllable_DOF_num_all> J;
        Eigen::Matrix<double, 3, 1> Pe = Ti[m - 1].block<3, 1>(0,3);
        Eigen::Matrix<double, 3, 1> zero; zero.block<3, 1>(0, 0) << 0, 0, 1;
        for(int i = 0; i < ModuJnt_num_all; i++){
            if(i == 0){
                v1.block<3, 1>(0, i) = Pe + Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0,3));

            }else{
                v1.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i - 1].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i - 1].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 2].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 3].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 6].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 7].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 10].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 11].block<3, 1>(0,3));
            }
            v2.block<3, 1>(0, i) = Ti[RXBSegment_Joint_num*i + 0].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 0].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 1].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 1].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 4].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 4].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 5].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 5].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 8].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 8].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 9].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 9].block<3, 1>(0,3)) + Ti[RXBSegment_Joint_num*i + 12].block<3, 1>(0, 2).cross(Pe - Ti[RXBSegment_Joint_num*i + 12].block<3, 1>(0,3));
            J.block<3, 1>(0, 2*i) = v1.block<3, 1>(0, i);
            J.block<3, 1>(0, 2*i + 1) = v2.block<3, 1>(0, i);
        }
        return J;
    }


    Eigen::MatrixXd pinv(Eigen::MatrixXd  A) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
        double  pinvtoler = 1.e-8; //tolerance
        int row = A.rows();
        int col = A.cols();
        int k = (row > col) ? col : row;
        Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
        Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
        Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
        for (long i = 0; i < k; ++i) {
            if (singularValues_inv(i) > pinvtoler) {
                singularValues_inv(i) = 1.0 / singularValues_inv(i);
            } else {
                singularValues_inv(i) = 0;
            }
        }
        for (long i = 0; i < k; ++i) {
            singularValues_inv_mat(i, i) = singularValues_inv(i);
        }
        X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose()); //X=VS+U*

        return X;
    }

    void fkd_kinematics(double *theta, std::vector<Eigen::Matrix4d> &Ti) {
        int link_num;
        int theta_num;
        Eigen::Vector4d DH_Para;
        DH_Para << 0, 0, 0, 0;
        Eigen::Matrix4d tmp_Ti, pre_Ti;
        std::vector<Eigen::Matrix4d>::iterator itea;
        tmp_Ti << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        Ti.clear();
        for (int joint_i = 0; joint_i < ModuJnt_num_all; joint_i++) {//4
            for (int i = 0; i < g_RXBSegment_Joint_num; i++) {//4
                link_num = joint_i * 8 + i ;//0
                if (((link_num + 1) % 4) > 1) { //23zu
                    theta_num = 2 * joint_i + 1;
                } else {
                    theta_num = 2 * joint_i ;//14zu
                }
                DH_table(theta[theta_num], link_num, link_len, DH_Para);//deg
                DH_Mat(DH_Para(0), DH_Para(1), DH_Para(2), DH_Para(3), tmp_Ti);
                if (link_num > 0) {
                    tmp_Ti = pre_Ti * tmp_Ti;
                    pre_Ti = tmp_Ti;
                    Ti.push_back(tmp_Ti);
                } else {
                    Ti.push_back(tmp_Ti);
                    pre_Ti = tmp_Ti;
                }
            }
        }
    }
    void fkd_kinematics_oneSeg(double *theta, std::vector<Eigen::Matrix4d> &Ti) {
        int link_num;
        int theta_num;
        Eigen::Vector4d DH_Para;
        DH_Para << 0, 0, 0, 0;
        Eigen::Matrix4d tmp_Ti, pre_Ti;
        std::vector<Eigen::Matrix4d>::iterator itea;
        tmp_Ti << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        Ti.clear();
        for (int joint_i = 0; joint_i < 1; joint_i++) {//4
            for (int i = 0; i < g_RXBSegment_Joint_num; i++) {//4
                link_num = joint_i * 8 + i ;//0
                if (((link_num + 1) % 4) > 1) { //23zu
                    theta_num = 2 * joint_i + 1;
                } else {
                    theta_num = 2 * joint_i ;//14zu
                }
                DH_table(theta[theta_num], link_num, link_len, DH_Para);//deg
                DH_Mat(DH_Para(0), DH_Para(1), DH_Para(2), DH_Para(3), tmp_Ti);
                if (link_num > 0) {
                    tmp_Ti = pre_Ti * tmp_Ti;
                    pre_Ti = tmp_Ti;
                    Ti.push_back(tmp_Ti);
                } else {
                    Ti.push_back(tmp_Ti);
                    pre_Ti = tmp_Ti;
                }
            }
        }
    }
    void fkd_actualKinematics(double *theta, std::vector<Eigen::Matrix4d> &Ti){
        int link_num;
        int theta_num;
        Eigen::Vector4d DH_Para;
        DH_Para << 0, 0, 0, 0;
        Eigen::Matrix4d tmp_Ti, pre_Ti;
        std::vector<Eigen::Matrix4d>::iterator itea;
        tmp_Ti << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        Ti.clear();
        for (int joint_i = 0; joint_i < ModuJnt_num_all; joint_i++) {//4
            for (int i = 0; i < g_RXBSegment_Joint_num; i++) {//4
                link_num = joint_i * 8 + i ;//0
                DH_table(theta[joint_i*g_RXBSegment_Joint_num + i], link_num, link_len, DH_Para);//deg
                DH_Mat(DH_Para(0), DH_Para(1), DH_Para(2), DH_Para(3), tmp_Ti);
                if (link_num > 0) {
                    tmp_Ti = pre_Ti * tmp_Ti;
                    pre_Ti = tmp_Ti;
                    Ti.push_back(tmp_Ti);
                } else {
                    Ti.push_back(tmp_Ti);
                    pre_Ti = tmp_Ti;
                }
            }
        }

    }
    void fkd_actualKinematics_oneSeg(double *theta, std::vector<Eigen::Matrix4d> &Ti){
        int link_num;
        int theta_num;
        Eigen::Vector4d DH_Para;
        DH_Para << 0, 0, 0, 0;
        Eigen::Matrix4d tmp_Ti, pre_Ti;
        std::vector<Eigen::Matrix4d>::iterator itea;
        tmp_Ti << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        Ti.clear();
        for (int joint_i = 0; joint_i < 1; joint_i++) {//4
            for (int i = 0; i < g_RXBSegment_Joint_num; i++) {//4
                link_num = joint_i * 8 + i ;//0
                DH_table(theta[joint_i*g_RXBSegment_Joint_num + i], link_num, link_len, DH_Para);//deg
                DH_Mat(DH_Para(0), DH_Para(1), DH_Para(2), DH_Para(3), tmp_Ti);
                if (link_num > 0) {
                    tmp_Ti = pre_Ti * tmp_Ti;
                    pre_Ti = tmp_Ti;
                    Ti.push_back(tmp_Ti);
                } else {
                    Ti.push_back(tmp_Ti);
                    pre_Ti = tmp_Ti;
                }
            }
        }

    }

    int inverse_kinematics(Eigen::Matrix4d &T_end, double *theta, double *ang_lim) {
        int m, N;
        bool All_Over_Joint_Limit = true;
        double d_p = 0, d_a = 0;
        Eigen::Vector3d dPos, dPos_old, dA, dA_old;
        Eigen::Matrix<double, 6, 1> dPos_A;
        Eigen::Matrix<double, 6, Controllable_DOF_num_all> J;
        Eigen::Matrix<double, Controllable_DOF_num_all, 1> Tem_dAngle;
        int count = 1, flag = 1;
        Eigen::Matrix<double, Controllable_DOF_num_all, 1> thetaV;
        N = ModuJnt_num_all;
        std::vector<Eigen::Matrix4d> Ti;
        fkd_kinematics(theta, Ti);
        m = Ti.size();
        for (int i = 0; i < Controllable_DOF_num_all; i++) {
            thetaV(i, 0) = theta[i];
        }
        dPos_old << 0, 0, 0;
        dA_old << 0, 0, 0;
        while (1) {
            fkd_kinematics(theta, Ti);
            dPos = T_end.block<3, 1>(0, 3) - Ti[m - 1].block<3, 1>(0, 3);
            dA = 0.5 * (Ti[m - 1].block<3, 1>(0, 0).cross(T_end.block<3, 1>(0, 0)) +
                        Ti[m - 1].block<3, 1>(0, 1).cross(T_end.block<3, 1>(0, 1)) +
                        Ti[m - 1].block<3, 1>(0, 2).cross(T_end.block<3, 1>(0, 2)));
            dPos_A << dPos(0), dPos(1), dPos(2), dA(0), dA(1), dA(2);
            J = Jacob_Mat(Ti);
            Tem_dAngle = pinv(J) * dPos_A;// a vector
            for (int i = 0; i < Controllable_DOF_num_all; i++) {
                thetaV(i, 0) = theta[i];
            }
            thetaV = thetaV + Tem_dAngle;
            for (int i = 0; i < Controllable_DOF_num_all; i++) {
                theta[i] = thetaV(i, 0);
            }
            All_Over_Joint_Limit = true;
            for (int theta_i = 0; theta_i < Controllable_DOF_num_all; theta_i++) {
                if (abs(theta[theta_i]) > ang_lim[theta_i]) {
                    if (abs(theta[theta_i]) > 0) {
                        theta[theta_i] = ang_lim[theta_i];
                    } else {
                        theta[theta_i] = -ang_lim[theta_i];
                    }
                } else {
                    All_Over_Joint_Limit = false;
                }
            }

            if (All_Over_Joint_Limit == true) {
                std::cerr << "all joint_angles larger than the limit!!" << std::endl;
                flag = 0;
                break;
            }

            d_p = dPos.norm();
            d_a = dA.norm();
            if (d_p < 0.00001 && d_a < 0.001) {
                flag = 1;
                //  std::cout << "the pos_error is:  " << d_p << std::endl;
                //  std::cout << "the joint_error is:  " << asin(d_a) / M_PI * 180 << std::endl;
                std::cout << "inverse_kinematics program done successfully  " << d_p << std::endl;
                break;
            }

            if ((abs(dPos.norm() - dPos_old.norm()) < 0.000000001 && abs(dA.norm() - dA_old.norm())) || count == 2000) {
                //  std::cout << "the pos_error is:  " << d_p << std::endl;
                //  std::cout << "the joint_error is:  " << asin(d_a) / M_PI * 180 << std::endl;
                if (d_p < 1 && d_a < 0.0175) {
                    flag = 2;
                } else {
                    flag = 0;
                    std::cerr << "inverse_kinematics program error!!" << std::endl;
                }
                std::cout << "inverse_kinematics program failed...... "  << std::endl;
                break;

            }
            dPos_old = dPos;
            dA_old = dA;
            count++;
        }
        return flag;//0 larger than limit or programmer error  1 success  2 calculate failed

    }
}
