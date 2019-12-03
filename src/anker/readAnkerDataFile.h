#ifndef READANKERDATAFILE_H
#define READANKERDATAFILE_H
#include "ankerDataType.h"
#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include <math.h>
#include <assert.h>
using namespace std;

class readAnkerDataFile
{
public:
    readAnkerDataFile(const string& imu_file,const string& odometry_file,const string& optical_file);
    queue<AnkerData> AnkerDataSet;
private:
    bool type_is_int;
};

#endif // READANKERDATAFILE_H
