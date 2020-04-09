# calibrate_accelerometer
## algrithm of calibrating accelerometer with least sqaured optimization

## model: <br>

                1   psi  -theta    kx  0   0          bx    <br>
acc_calibr = [-psi   1     phi  ][  0  ky  0](acc + [ by ]) <br>
              theta -phi   1        0  0  kz          bz    <br>
## Residual:
F(x) = (acc_calibr.norm - G_Norm)^2 + weight*(theta*theta + phi*phi + psi*psi) <br>
Note:后面一项是惩罚因子，用于限制安装误差角不要太大 <br>

## 数据采集要求：
1、平放、左面向上、右面向上、前面向上、后面向上、翻放每次达15s左右；<br>
2、不同角度的斜放30s左右； <br>
