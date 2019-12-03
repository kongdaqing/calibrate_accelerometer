#include "gravityNormFactor.h"

GravityNormFactor::GravityNormFactor(const double g,const Eigen::Vector3d& acc_msg)
{
    G_NORM = g;
    ax = acc_msg.x();
    ay = acc_msg.y();
    az = acc_msg.z();
}

bool GravityNormFactor::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const
{

    double theta,phi,psi;
    double kx,ky,kz;
    double bx,by,bz;

    theta = parameters[0][0];
    phi = parameters[0][1];
    psi = parameters[0][2];

    kx = parameters[1][0];
    ky = parameters[1][1];
    kz = parameters[1][2];

    bx = parameters[2][0];
    by = parameters[2][1];
    bz = parameters[2][2];

    double ax_c =    kx * (ax + bx) + psi * (ay + by) - theta * (az + bz);
    double ay_c =  -psi * (ax + bx) +  ky * (ay + by) +   phi * (az + bz);
    double az_c = theta * (ax + bx) - phi * (ay + by) +    kz * (az + bz);
    double acc_norm = sqrt(ax_c*ax_c + ay_c*ay_c + az_c*az_c);
    double delta_norm = acc_norm - G_NORM;
    residuals[0] = delta_norm*delta_norm;

    Eigen::Matrix<double, 1, 3> res2acc(1,3);
    res2acc << ax_c/acc_norm, ay_c/acc_norm, az_c/acc_norm;
    res2acc *= 2*delta_norm;


    if(jacobians)
    {
        if(jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double,1, 3>> jacob_angle(jacobians[0]);
            Eigen::Matrix3d acc2angle;
            acc2angle << -(az + bz), 0, ax + bx,
                    0, az + bz, -(ay + by),
                    ay + by, -(ax + bx), 0;
            jacob_angle = res2acc * acc2angle;
        }

        if(jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double,1, 3>> jacob_k(jacobians[1]);
            Eigen::Matrix3d acc2k;
            acc2k <<    ax + bx, 0, 0,
                    0, ay + by, 0,
                    0, 0, az + bz;
            jacob_k = res2acc * acc2k;
        }

        if(jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double,1, 3>> jacob_bias(jacobians[2]);
            Eigen::Matrix3d acc2bias;
            acc2bias <<  kx, -psi, theta,
                    psi,  ky, -phi,
                    -theta, phi,   kz;
            jacob_bias = res2acc * acc2bias;
        }
    }
    return true;
}
