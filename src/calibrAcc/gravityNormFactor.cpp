#include "gravityNormFactor.h"

GravityNormFactor::GravityNormFactor(const double g,const double w,const Eigen::Vector3d& acc_msg)
{
    G_NORM = g;
    weight = w;
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

    double ax_c =         kx * (ax + bx)   +   ky * psi * (ay + by)  -  kz * theta * (az + bz);
    double ay_c =  -kx * psi * (ax + bx)   +         ky * (ay + by)  +    kz * phi * (az + bz);
    double az_c = kx * theta * (ax + bx)   -   ky * phi * (ay + by)  +          kz * (az + bz);
    double acc_norm = sqrt(ax_c*ax_c + ay_c*ay_c + az_c*az_c);
    double delta_norm = acc_norm - G_NORM;
    residuals[0] = delta_norm*delta_norm + weight*(theta*theta + phi*phi + psi*psi);

    Eigen::Matrix<double, 1, 3> res2acc(1,3);
    res2acc << ax_c/acc_norm, ay_c/acc_norm, az_c/acc_norm;
    res2acc = 2 * delta_norm * res2acc;


    if(jacobians)
    {
        if(jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double,1, 3>> jacob_angle(jacobians[0]);
            Eigen::Matrix3d acc2angle;
            acc2angle << -kz * (az + bz),              0,   ky * (ay + by),
                                       0,  kz * (az + bz), -kx * (ax + bx),
                          kx * (ax + bx), -ky * (ay + by),               0;
            Eigen::Matrix<double,1, 3> constr2angle;
            constr2angle << 2*weight*theta, 2*weight*phi, 2*weight*psi;
            jacob_angle = res2acc * acc2angle + constr2angle;
        }

        if(jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double,1, 3>> jacob_k(jacobians[1]);
            Eigen::Matrix3d acc2k;
            acc2k <<           ax + bx,  psi * (ay + by),   -theta * (az + bz),
                      -psi * (ax + bx),          ay + by,      phi * (az + bz),
                     theta * (ax + bx), -phi * (ay + by),              az + bz;
            jacob_k = res2acc * acc2k;
        }

        if(jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double,1, 3>> jacob_bias(jacobians[2]);
            Eigen::Matrix3d acc2bias;
            acc2bias <<      kx,  ky * psi,  -kz * theta,
                      -ky * psi,        ky,     kz * phi,
                     kx * theta, -ky * phi,           kz;
            jacob_bias = res2acc * acc2bias;
        }
    }
    return true;
}
