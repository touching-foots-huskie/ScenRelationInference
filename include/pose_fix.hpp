#ifndef POSE_FIX_H
#define POSE_FIX_H

#include <Eigen/Eigen>

#include <iostream>
/*
Return if the two poses are similar in symmetric & give transform
 */
bool SymmetricFix(const Eigen::Ref<Eigen::MatrixXd>& pose_1,
                  const Eigen::Ref<Eigen::MatrixXd>& pose_2,
                  Eigen::Ref<Eigen::MatrixXd> fix_transform,
                  int symmetric_code) {
    Eigen::Vector3d x_axis;
    x_axis << 1,
              0,
              0;
    Eigen::Vector3d y_axis;
    y_axis << 0,
              1,
              0;
    Eigen::Vector3d z_axis;
    z_axis << 0,
              0,
              1;
    // x, y, z in 1&2 
    Eigen::Vector3d x_axis_in_1 = pose_1 * x_axis;
    Eigen::Vector3d x_axis_in_2 = pose_2 * x_axis;  
    Eigen::Vector3d y_axis_in_1 = pose_1 * y_axis;
    Eigen::Vector3d y_axis_in_2 = pose_2 * y_axis;
    Eigen::Vector3d z_axis_in_1 = pose_1 * z_axis;
    Eigen::Vector3d z_axis_in_2 = pose_2 * z_axis; 
    // symmetric status

    double x_dot = x_axis_in_1.dot(x_axis_in_2);
    double y_dot = y_axis_in_1.dot(y_axis_in_2);
    double z_dot = z_axis_in_1.dot(z_axis_in_2);    
    // Tree transform | right multiply
    Eigen::MatrixXd z_180(4, 4);
    z_180 << -1, 0, 0, 0,
              0, -1,0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    Eigen::MatrixXd y_180(4, 4);
    y_180 << -1, 0,  0, 0,
              0, 1,  0, 0,
              0, 0, -1, 0,
              0, 0,  0, 1;
    Eigen::MatrixXd x_180(4, 4);
    x_180 << 1, 0,  0, 0,
             0, -1, 0, 0,
             0, 0, -1, 0,
             0, 0,  0, 1;
    switch (symmetric_code)
    {
    case 0 : {
        return false;
        break;
    }
    case 221 : {
        // cylinder
        if(z_dot >= 0) {
            fix_transform = Eigen::MatrixXd::Identity(4, 4);
            return true;
        }
        else {
            fix_transform = x_180;
            return true;
        }
        break;
    }
    case 222 : {
        // box
        if((x_dot > 0) && (y_dot > 0) && (z_dot > 0)) {
            fix_transform = Eigen::MatrixXd::Identity(4, 4);
            return true;
        }
        else if((x_dot > 0) && (y_dot < 0) && (z_dot < 0)) {
            // rotate from x_axis    
            fix_transform = x_180;
            return true;
        }
        else if((x_dot < 0) && (y_dot > 0) && (z_dot < 0)) {
            // rotate from y_axis    
            fix_transform = y_180;
            return true;
        }
        else if((x_dot < 0) && (y_dot < 0) && (z_dot > 0)) {
            // rotate from z_axis    
            fix_transform = z_180;
            return true;
        }
        else {
            // no suitable transform
            return false;
        }
        break;
    }
    case 2 : {
        // box bottle
        if((x_dot < 0) && (y_dot < 0) && (z_dot > 0)) {
            // rotate from z_axis    
            fix_transform = z_180;
            return true;
        }
        else {
            // no suitable transform
            return false;
        }
        break;
    }
    default: {
        std::cout << "Symmetric Mode didn't implemented" << std::endl;
        break; 
    }
        
}
};

#endif