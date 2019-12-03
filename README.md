# calibrate_accelerometer
algrithem of calibrating accelerometer with least sqaured optimization
___________________________________________________
Model:
                1   psi  -theta    kx  0   0          bx
acc_calibr = [-psi   1     phi  ][  0  ky  0](acc + [ by ])
              theta -phi   1        0  0  kz          bz
Residual:
F(x) = (acc_calibr.norm - G_Norm)^2 + weight*(theta*theta + phi*phi + psi*psi);
后面一项是惩罚因子，用于限制安装误差角不要太大。
需要采集加速度数据和陀螺仪数据，数据要求：
1、平放、左面向上、右面向上、前面向上、后面线上、翻放每次达15s左右，然后不同角度的斜放30s
