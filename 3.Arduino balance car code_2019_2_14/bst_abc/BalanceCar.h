/*
BalanceCar.h BalanceCar.cpp - Library for BST-Balance car code.
Created by SKY ZHU&ROOMS LUO, OCTOBER 2, 2016.
JUST FOR THE Company of Technology of yahboom.
In order to  avoid Infringement Act,this core is not for the commerce except being authorized by the writer.
*/



#ifndef BalanceCar_h
#define BalanceCar_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


class BalanceCar
{
public:
  double speedpiout(double kps,double kis,double kds,int f,int b,double p0);
  float turnspin(int turnleftflag,int turnrightflag,int spinleftflag,int spinrightflag,double kpturn,double kdturn,float Gyroz);
  void pwma(double speedoutput,float rotationoutput,float angle,float angle6,int turnleftflag,int turnrightflag,int spinleftflag,int spinrightflag,
						int f,int b,float accelz,int Pin1,int Pin2,int Pin3,int Pin4,int PinPWMA,int PinPWMB);
	int pulseright = 0;
	int pulseleft = 0;
	int posture=0;
	int stopl = 0;
	int stopr = 0;
	double angleoutput=0,pwm1 = 0, pwm2 = 0;
private:
	float speeds_filterold;//速度滤波
	float positions;//位置
	int turnmax = 0;                                    //旋转输出幅值
	int turnmin = 0;                                  //旋转输出幅值
	float turnout = 0;
	int flag1 = 0;
	int flag2 = 0;
	int flag3 = 0;
	int flag4 = 0;
};
#endif
//
// END OF FILE
//