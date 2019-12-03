#ifndef CALIBRACC_H
#define CALIBRACC_H
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <ceres/ceres.h>
#include "gravityNormFactor.h"

#define G 9.7936
#define RAD2DEG 57.3
#define MINRATEDEG 10.0
#define ACCDIV 1
#define CHECKSIZE 100
using namespace std;

enum ACC_STATE
{
    AX_UP,
    AX_DOWN,
    AY_UP,
    AY_DOWN,
    AZ_UP,
    AZ_DOWN
};

class CalibrAcc
{
public:
    CalibrAcc(unsigned int singleCount,bool setAngleConstant);
    ~CalibrAcc(){};
    void pickAccMeasurement(Eigen::Vector3d& accMsg,Eigen::Vector3d& gyroMsg);
    void accOptimization();
    void checkCalibrParameters();
    void calibrAccMeasurement(Eigen::Vector3d& rawAcc,Eigen::Vector3d& resAcc);
    bool pickOverFlg;
private:
vector<Eigen::Vector3d> vecAcc;
vector<Eigen::Vector3d> checkAcc;
bool angleConstantFlg;
unsigned int singleNum;
unsigned int accDistrib[7];
double k[3];
double angle[3];
double bias[3];

};



#endif
