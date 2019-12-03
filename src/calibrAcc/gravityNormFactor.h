#ifndef GRAVITYNORMFACTOR_H
#define GRAVITYNORMFACTOR_H
#include <ceres/ceres.h>
#include <Eigen/Dense>
class GravityNormFactor: public ceres::SizedCostFunction<1, 3, 3, 3>
{
public:
  GravityNormFactor(const double g,const Eigen::Vector3d& acc_msg);
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
  void check(double **parameters);

private:
  double G_NORM;
  double ax,ay,az;
};

#endif // GRAVITYNORMFACTOR_H
