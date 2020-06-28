//
// Created by hyj on 17-6-22.
//

#include "param.h"

Param::Param()
{
    Eigen::Matrix3d R;   // 把body坐标系朝向旋转一下,得到相机坐标系，好让它看到landmark,  相机坐标系的轴在body坐标系中的表示
    // 相机朝着轨迹里面看， 特征点在轨迹外部， 这里我们采用这个
    R << 0, 0, -1,
            -1, 0, 0,
            0, 1, 0;
    R_bc = R;
    t_bc = Eigen::Vector3d(0.05,0.04,0.03);

    R_bc_right = R;
    t_bc_right = Eigen::Vector3d(0.05,-0.04,0.03);

    // 相机外参数
    Eigen::Vector3d tcc = R_bc.transpose() * t_bc;
    Eigen::Vector3d tcc_right = R_bc_right.transpose() * t_bc_right;
    T_extri = tcc - tcc_right;

    cam_intrinsics = Eigen::Vector4d(1,1,1,1);

}