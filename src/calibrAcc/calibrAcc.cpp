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
    }
    accDistrib[3] = 0;
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

    bool pickFinishFlg = true;
    for (int i = 0;i < 4; i++) {
       pickFinishFlg = pickFinishFlg && (accDistrib[i] > singleNum);
    }
    if(pickFinishFlg == true)
    {
        pickOverFlg = true;
        printf("[CalibrAcc]:Finish all rotation!\n");
        return;
    }


    if(fabs(averAcc.x()) < G*0.1 && fabs(averAcc.y()) > G*0.1 && accDistrib[0] < 2*singleNum)
    {
        accDistrib[0]++;
        if(accDistrib[0] < 2*singleNum)
            vecAcc.push_back(accMsg);
        if(accDistrib[0] == singleNum)
            printf("[CalibrAcc]:Please rotation in y axis!\n");
        printf("[CalibrAcc]: Buffer %d acc datas in x axis!\n", accDistrib[0]);
        return;
    }
    if(fabs(averAcc.y()) < G*0.1 && fabs(averAcc.x()) > G*0.1 && accDistrib[1] < 2*singleNum && accDistrib[0] > singleNum)
    {
        accDistrib[1]++;
        if(accDistrib[1] < 2*singleNum)
            vecAcc.push_back(accMsg);
        if(accDistrib[1] == singleNum)
            printf("[CalibrAcc]:Please rotation in z axis!\n");
        printf("[CalibrAcc]: Buffer %d acc datas in y axis!\n", accDistrib[1]);
        return;
    }

    if(fabs(averAcc.z()) < G*0.1 && fabs(averAcc.y()) > G*0.1 &&  accDistrib[2] < 2*singleNum && accDistrib[1] > singleNum)
    {
        accDistrib[2]++;
        if(accDistrib[2] < 2*singleNum)
            vecAcc.push_back(accMsg);
        if(accDistrib[2] == singleNum)

        printf("[CalibrAcc]:Please rotation in other axis!\n");
        printf("[CalibrAcc]: Buffer %d acc datas in z axis!\n", accDistrib[2]);
        return;
    }

    if(fabs(averAcc.z()) > G*0.2 && fabs(averAcc.y()) > G*0.2 && fabs(averAcc.z()) > G*0.2 && accDistrib[3] < 2*singleNum && accDistrib[2] > singleNum)
    {
        accDistrib[3]++;
        if(accDistrib[3] < 2*singleNum)
            vecAcc.push_back(accMsg);
        printf("[CalibrAcc]: Buffer %d acc datas in other axis!\n", accDistrib[3]);
        return;
    }

}

void CalibrAcc::accOptimization()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    //loss_function = new ceres::HuberLoss(1.0);

    problem.AddParameterBlock(angle,3);
    problem.AddParameterBlock(k,3);
    problem.AddParameterBlock(bias,3);
    if(angleConstantFlg)
        problem.SetParameterBlockConstant(angle);

    for (unsigned int i = 0; i < vecAcc.size(); i++) {
        Eigen::Vector3d acc = vecAcc[i];
        //printf("[CalibrAcc]: acc %f,%f,%f\n",acc.x(),acc.y(),acc.z());
        GravityNormFactor* gnormFactor = new  GravityNormFactor(G,acc);
        problem.AddResidualBlock(gnormFactor,NULL,angle,k,bias);
    }



    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 30;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    cout << summary.BriefReport() << "\n";
    cout << "Parameters angle: " << angle[0] << " " << angle[1] << " " << angle[2] << endl;
    cout << "Parameters k: " << k[0] << " " << k[1] << " " << k[2] << endl;\
    cout << "Parameters bias: " << bias[0] << " " << bias[1] << " " << bias[2] << endl;
}
