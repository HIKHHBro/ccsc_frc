## 低盘

* [ ] 测试手动
* [ ] 测试里程计  下地正方向跑动  和世界坐标里程计
* [ ] 自动pid  不带电机，查看值正不正确
* [ ]

## 运动模型
https://www.sohu.com/a/193593445_715708

Welcome to the 2020-Infinite_Recharge wiki!

Software We Track
National Instruments
https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html#333285 Driver station, roboRio…

WPILib (with VS Code and extensions)
https://github.com/wpilibsuite/allwpilib/releases

You may want git Bash too: https://gitforwindows.org/

### Navx
https://www.kauailabs.com/navx-mxp/

### Rev Robotics Spark Max
https://www.revrobotics.com/sparkmax-software/

### CTRE Talon SRX / Phoenix Tuner for Pneumatics and PDP
http://www.ctr-electronics.com/talon-srx.html#product_tabs_technical_resources

### CTRE Falcon FX
Same installer as above, firmware updates here: http://www.ctr-electronics.com/control-system/motor-control/talon-fx.html#product_tabs_technical_resources

### Limelight
https://limelightvision.io/pages/downloads

### FRC Radio Configuration
https://docs.wpilib.org/en/latest/docs/getting-started/getting-started-frc-control-system/radio-programming.html (2020 uses same firmware as 2019, but good to have link in case we need to reflash)


## 电机上位机
https://github.com/CrossTheRoadElec/Phoenix-Releases/releases
CTRE.Phoenix.Framework.Windows.v5.19.1.0.zip

## 注意精度问题

## 传感器采样
Velocity Measurement Filter
The Talon SRX measures the velocity of all supported sensor types as well as the current position. Every 1ms a velocity sample is measured and inserted into a rolling average.

The velocity sample is measured as the change in position at the time-of-sample versus the position sampled 100ms-prior-to-time-of-sample. The rolling average is sized for 64 samples. Though these settings can be modified, the (100ms, 64 samples) parameters are default.
默认100ms 滑动平均64  默认传感器速率为1ms



## 抬升机构

If we measure a motor output of 7% to keep position, then our java code for Arbitrary Feed Forward with Motion Magic would look like this:

double feedforward = 0.07;
_motorcontroller.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFee

使用操持输出模式

# 物资和文档    
## 物资网站
1. 猎鹰电机:https://www.vexrobotics.com/ 
2. neo电机:https://www.revrobotics.com/
3. 摄像头:https://limelightvision.io/pages/downloads    
4. 颜色传感器:  http://revrobotics.com/
5. 气阀控制模块: http://www.ctr-electronics.com/pcm.html
## 库链接
1. 颜色传感器 http://revrobotics.com/content/sw/color-sensor-v3/sdk/REVColorSensorV3.json
2. navx(imu) https://www.kauailabs.com/dist/frc/2020/navx_frc.json  
3. Phoenix http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json
4. sparx(neo) http://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json

## 文档
1. neo  https://www.revrobotics.com/neo-brushless-motor-locked-rotor-testing/
## API  
1. 颜色传感器  https://www.revrobotics.com/content/sw/color-sensor-v3/sdk/docs/cpp/index.html
2. SPARX MAX电调(neo): https://www.revrobotics.com/content/sw/max/sw-docs/cpp/index.html    
3.  Phoenix http://www.ctr-electronics.com/downloads/api/cpp/html/index.html
## 例程
1. 颜色传感器  https://github.com/REVrobotics/Color-Sensor-v3-Examples
2. Phoenix https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages
3. neo电机 sparx电调 https://docs.revrobotics.com/sparkmax/software-resources/spark-max-code-examples

# 使用注意
## neo 
### pwm 
1. 查看状态灯，常亮白色为有信号，如果闪烁，断电5s重启
2. 不要使用pwm同时接usb

## 猎鹰
1. 扭矩过小，注意加减速比  50:14->42:22两级减速感觉还行

## navx 
1. yaw轴漂移严重
