#include "zf_common_headfile.h"
Coor wheel;
int  servoLeftRear;
int  servoLeftFront;
int  servoRightRear;
int  servoRightFront;
void Servo_Init()
{
    wheel.RB_Roll=0;
    wheel.kp=0.001f;
    wheel.TargetLenth = 3.0;
    //wheel.XLeft=2;//车一
    //wheel.XRight=2;
    wheel.XLeft=2;
    wheel.XRight=2;
    wheel.YLeft = wheel.TargetLenth;
    wheel.YRight = wheel.TargetLenth;
    pwm_init(SERVO_1,SERVO_FREQ,0);
    pwm_init(SERVO_2,SERVO_FREQ,0);
    pwm_init(SERVO_4,SERVO_FREQ,0);
    pwm_init(SERVO_3,SERVO_FREQ,0);
}
float kp = 29.f,OUTPUT;
void Single_sided_bridge()
{

   static float s_roll;
   s_roll = fabs(eulerAngle.roll) < 2.0 ? 0 : eulerAngle.roll;
   OUTPUT  = kp  * s_roll;
   PWM_Limitf(&OUTPUT,1200);
    /*static float Err_v[3];
    Err_v[0] = eulerAngle.roll - 0;
    //OUTPUT+ = kp * Err_v[0]*/
}
void Mid_Set()
{
    servoLeftRear=0;
    servoLeftFront=0;
    servoRightRear=0;
    servoRightFront=0;
}

void AngleCal(float XL,float XR ,float YL, float YR)
{
    float alpha1,alpha2,beta1,beta2;
    int alphaLeftToAngle,betaLeftToAngle;
    float aLeft=2*XL*L1;
    float bLeft=2*YL*L1;
    float cLeft=XL*XL+YL*YL+L1*L1-L2*L2;
    float dLeft = 2 * L4 * (XL - L5);
    float eLeft = 2 * L4 * YL;
    float fLeft = ((XL - L5) * (XL- L5) + L4 * L4 +YL * YL - L3 * L3);

    float alpha3,alpha4,beta3,beta4;
    int32 alphaRightToAngle,betaRightToAngle;
    float aRight=2*XR*L1;
    float bRight=2*YR*L1;
    float cRight=XR*XR+YR*YR+L1*L1-L2*L2;
    float dRight = 2 * L4 * (XR - L5);
    float eRight = 2 * L4 * YR;
    float fRight = ((XR - L5) * (XR - L5) + L4 * L4 + YR * YR - L3 * L3);

    alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
    beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
    alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

    alpha3 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    alpha4 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    beta3 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
    beta4 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
    alpha3 = (alpha3 >= 0)?alpha3:(alpha3 + 2 * PI);
    alpha4 = (alpha4 >= 0)?alpha4:(alpha4 + 2 * PI);

    if(alpha1 >= PI/4) wheel.alphaLeft = alpha1;
    else wheel.alphaLeft = alpha2;
    if(beta1 >= 0 && beta1 <= PI/4) wheel.betaLeft = beta1;
    else wheel.betaLeft = beta2;
    alphaLeftToAngle = (int)((wheel.alphaLeft / 6.28) * 360);//弧度转角度
    betaLeftToAngle = (int)((wheel.betaLeft / 6.28) * 360);

    if(alpha3 >= PI/4) wheel.alphaRight = alpha3;
    else wheel.alphaRight = alpha4;
    if(beta3 >= 0 && beta3 <= PI/4) wheel.betaRight = beta3;
    else wheel.betaRight = beta4;
    alphaRightToAngle = (int)((wheel.alphaRight / 6.28) * 360);//弧度转角度
    betaRightToAngle = (int)((wheel.betaRight / 6.28) * 360);

    servoLeftFront = ((180-alphaLeftToAngle)*3000/90);
    servoLeftRear = ((-betaLeftToAngle)*3050/90);
    servoRightFront = ((alphaRightToAngle-180)*3000/90);
    servoRightRear = ((betaRightToAngle)*2950/90);

    if(SERVO1_MID + servoLeftFront>=7490  || SERVO1_MID + servoLeftFront<=1510)  Mid_Set();
    if(SERVO2_MID + servoLeftRear>=7490  || SERVO2_MID + servoLeftRear<=1510)    Mid_Set();
    if(SERVO3_MID + servoRightFront>=7490  || SERVO3_MID + servoRightFront<=1510)Mid_Set();
    if(SERVO4_MID + servoRightRear>=7490  || SERVO4_MID + servoRightRear<=1510)  Mid_Set();
    if(SELECTED_OBJECT == 1)
    {
        pwm_set_duty(SERVO_1,SERVO1_MID  + servoLeftFront  + att_pid.Velocity_out  - OUTPUT);//左前
        pwm_set_duty(SERVO_2,SERVO2_MID  + servoLeftRear   + att_pid.Velocity_out   + OUTPUT);//左后
        pwm_set_duty(SERVO_3,SERVO3_MID  + servoRightFront - att_pid.Velocity_out   - OUTPUT);//右q前ian
        pwm_set_duty(SERVO_4,SERVO4_MID  + servoRightRear  - att_pid.Velocity_out     + OUTPUT);//右后
    }

//车二
    else if(SELECTED_OBJECT == 2)
    {
        pwm_set_duty(SERVO_1,SERVO1_MID + servoLeftFront  - att_pid.Velocity_out  - OUTPUT);//左前
        pwm_set_duty(SERVO_2,SERVO2_MID + servoLeftRear   - att_pid.Velocity_out  + OUTPUT);//左后
        pwm_set_duty(SERVO_4,SERVO3_MID + servoRightFront + att_pid.Velocity_out  - OUTPUT);//右q前ian
        pwm_set_duty(SERVO_3,SERVO4_MID + servoRightRear  + att_pid.Velocity_out  + OUTPUT);//右后
    }
}
