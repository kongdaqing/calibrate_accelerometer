#include "calibrAcc.hpp"
CalibrAcc::CalibrAcc(unsigned int singleCount,bool setAngleConstant)
    : singleNum(singleCount),
      angleConstantFlg(setAngleConstant)
{
    pickOverFlg = false;
    for (int i = 0; i < 3; i++) {
        k[i] = 1;
        bias[i] = 0;
        angle[i] = 0;
        accDistrib[i] = 0;
        accDistrib[i+3] = 0;
    }
    accDistrib[6] = 0;
    cout << "[CalibrAcc]:Please rotate accelerometer in x axis smoothly!" << endl;
}



void CalibrAcc::pickAccMeasurement(Eigen::Vector3d& accMsg,Eigen::Vector3d& gyroMsg)
{
    static Eigen::Vector3d sumAcc{Eigen::Vector3d(0,0,0)};
    static int count = 0;
    count++;
    sumAcc[0] += accMsg.x();
    sumAcc[1] += accMsg.y();
    sumAcc[2] += accMsg.z();
    Eigen::Vector3d averAcc;
    if(count == ACCDIV)
    {
        count = 0;
        averAcc[0] = sumAcc.x()/ACCDIV;
        averAcc[1] = sumAcc.y()/ACCDIV;
        averAcc[2] = sumAcc.z()/ACCDIV;
        sumAcc.setZero();
    }else {
        return;
    }

    if(gyroMsg.norm()*RAD2DEG > MINRATEDEG || averAcc.norm() > G*1.2 || averAcc.norm() < G*0.8)
    {
        printf("[CalibrAcc]:Fast Rotation!Please Slow down!\n");
        return;
    }

    if(gyroMsg.norm()*RAD2DEG < MINRATEDEG*0.5f && averAcc.norm() < G*1.1 && averAcc.norm() > G*0.9 && averAcc.size() < CHECKSIZE)
        checkAcc.push_back(averAcc);


    bool pickFinishFlg = true;
    for (int i = 0;i < 7; i++) {
        pickFinishFlg = pickFinishFlg && (accDistrib[i] >= singleNum);
    }
    if(pickFinishFlg == true)
    {
        pickOverFlg = true;
        printf("[CalibrAcc]:Finish all rotation!\n");
        return;
    }


    if(averAcc.z() > (G * 0.95) && accDistrib[0] < singleNum)
    {
        accDistrib[0]++;
        vecAcc.push_back(accMsg);
        printf("[CalibrAcc]: Buffer %d acc datas in z-up direction!\n", accDistrib[0]);
        return;
    }
    if(averAcc.y() >  (G * 0.95) && accDistrib[1] < singleNum && accDistrib[0] >= singleNum)
    {
        accDistrib[1]++;
        vecAcc.push_back(accMsg);
        printf("[CalibrAcc]: Buffer %d acc datas in y-down direction!\n", accDistrib[1]);
        return;
    }

    if(averAcc.y() < (-G * 0.95) && accDistrib[2] < singleNum && accDistrib[1] >= singleNum)
    {
        accDistrib[2]++;
        vecAcc.push_back(accMsg);
        printf("[CalibrAcc]: Buffer %d acc datas in y-up direction!\n", accDistrib[2]);
        return;
    }


    if(averAcc.x() > (G * 0.95) && accDistrib[3] < singleNum && accDistrib[2] >= singleNum)
    {
        accDistrib[3]++;
        vecAcc.push_back(accMsg);
        printf("[CalibrAcc]: Buffer %d acc datas in x-up direction!\n", accDistrib[3]);
        return;
    }
    if(averAcc.x() < (-G * 0.95) && accDistrib[4] < singleNum && accDistrib[3] >= singleNum)
    {
        accDistrib[4]++;
        vecAcc.push_back(accMsg);
        printf("[CalibrAcc]: Buffer %d acc datas in x-down direction!\n", accDistrib[4]);
        return;
    }
    if(averAcc.z() <  (-G * 0.95) && accDistrib[5] < singleNum && accDistrib[4] >= singleNum)
    {
        accDistrib[5]++;
        vecAcc.push_back(accMsg);
        printf("[CalibrAcc]: Buffer %d acc datas in z-down direction!\n", accDistrib[5]);
        return;
    }


    if(fabs(averAcc.x()) > G*0.25 && fabs(averAcc.y()) > G*0.25 && fabs(averAcc.z()) > G*0.25 && accDistrib[6] < 2*singleNum && accDistrib[5] >= singleNum)
    {
        accDistrib[6]++;
        vecAcc.push_back(accMsg);
        printf("[CalibrAcc]: Buffer %d acc datas in other direction!\n", accDistrib[6]);
        return;
    }


}

void CalibrAcc::accOptimization()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);

    problem.AddParameterBlock(angle,3);
    problem.AddParameterBlock(k,3);
    problem.AddParameterBlock(bias,3);
    if(angleConstantFlg)
        problem.SetParameterBlockConstant(angle);

    for (unsigned int i = 0; i < vecAcc.size(); i++) {
        Eigen::Vector3d acc = vecAcc[i];
        //printf("[CalibrAcc]: acc %f,%f,%f\n",acc.x(),acc.y(),acc.z());
        GravityNormFactor* gnormFactor = new  GravityNormFactor(G,WEIGHT,acc);
        problem.AddResidualBlock(gnormFactor,loss_function,angle,k,bias);
    }



    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    cout << summary.BriefReport() << "\n";
    cout << "Parameters angle: " << angle[0] << " " << angle[1] << " " << angle[2] << endl;
    cout << "Parameters k: " << k[0] << " " << k[1] << " " << k[2] << endl;\
    cout << "Parameters bias: " << bias[0] << " " << bias[1] << " " << bias[2] << endl;

    checkCalibrParameters();

}

void CalibrAcc::calibrAccMeasurement(Eigen::Vector3d& rawAcc,Eigen::Vector3d& resAcc)
{
    Eigen::Matrix3d R;
    R <<            k[0],  k[1]*angle[2], -k[2]*angle[0],
          -k[0]*angle[2],           k[1],  k[2]*angle[1],
           k[0]*angle[0], -k[1]*angle[1],           k[2];
    Eigen::Vector3d b{Eigen::Vector3d(bias[0],bias[1],bias[2])};
   // cout <<"R: " <<  R << endl << "b : " << b << endl;
    resAcc = R * (rawAcc + b);
}
void CalibrAcc::checkCalibrParameters()
{

    double  AccNorm = 0,rawAccNorm = 0;

    for (int i = 0; i < vecAcc.size(); i++) {
        rawAccNorm += vecAcc[i].norm();
        Eigen::Vector3d tmpAcc;
        calibrAccMeasurement(vecAcc[i],tmpAcc);
        AccNorm += tmpAcc.norm();
    }
    rawAccNorm /= vecAcc.size();
    AccNorm /= vecAcc.size();
    printf("[CalibrAcc]: Raw average norm = %f, Calibr average norm = %f \n",rawAccNorm,AccNorm);

    double sumRSE = 0,rawSumRSE = 0;
    for (int i = 0; i < vecAcc.size(); i++) {
        double rawErr,calErr;
        rawErr = vecAcc[i].norm() - rawAccNorm;
        rawSumRSE += rawErr*rawErr;
        Eigen::Vector3d tmpAcc;
        calibrAccMeasurement(vecAcc[i],tmpAcc);
        calErr = tmpAcc.norm() - AccNorm;
        sumRSE += calErr*calErr;
    }
    double calibrRSE,rawRSE;
    calibrRSE = sumRSE/vecAcc.size();
    rawRSE = rawSumRSE/vecAcc.size();
    printf("[CalibrACC]:Raw acc rse = %f ,Calibrated acc rse = %f\n",rawRSE,calibrRSE);
}
