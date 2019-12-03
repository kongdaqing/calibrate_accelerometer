#include <iostream>
#include "anker/readAnkerDataFile.h"
#include "calibrAcc/calibrAcc.hpp"
using namespace std;
int main(int argc,char** argv)
{
  cout << "Hello calibr ACC!" << endl;
  if(argc != 2)
  {
    printf("Please input imu file path!\n");
    return -1;
  }
  string file_path = argv[1];
  std::string imu_file = file_path +  "imu_file.cvs";
  std::string odo_file = file_path + "odometer_file.cvs";
  std::string opt_file = file_path + "optical_flow_file.cvs";

  readAnkerDataFile ankerSensor(imu_file,odo_file,opt_file);
  CalibrAcc calibr_acc(2000,true);

  for (int i = 0;ankerSensor.AnkerDataSet.size(); i++) {
       if(calibr_acc.pickOverFlg == false)
       {
             AnkerData anker = ankerSensor.AnkerDataSet.front();
             ankerSensor.AnkerDataSet.pop();
             Eigen::Vector3d accMsg(Eigen::Vector3d(anker.ax,anker.ay,anker.az));
             Eigen::Vector3d gyrMsg(Eigen::Vector3d(anker.gx,anker.gy,anker.gz));
             calibr_acc.pickAccMeasurement(accMsg,gyrMsg);
       }else {
          break;
       }
  }
  if(calibr_acc.pickOverFlg)
      calibr_acc.accOptimization();

}
