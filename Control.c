#include "zf_common_headfile.h"
#define  PWMMAX 35

int8 error = 0;
int Single_sided_bridge_flag = 0;
float turn_left,turn_right;
void pid_init()
{
    if(SELECTED_OBJECT == 2)
    {
        att_pid.Targetspeed = 300.0;
        att_pid.Med_Angle   = -1.0;
        //角速度环
        an_pid.angle_rate_kp = -8.05f;
        an_pid.angle_rate_ki = -0.0005*0.8f;
        //角度环
        an_pid.angle_kp = 1.3f;
        an_pid.angle_kd = 0.003f;

        att_pid.Velocity_kp = 3.6f;
        att_pid.Velocity_ki = 0.08f;
        att_pid.Velocity_kd = 2.0f;
    }
    if(SELECTED_OBJECT == 1)
    {
        att_pid.Med_Angle   = 2.0;
        att_pid.Targetspeed = -600.0;
               //角速度环
        an_pid.angle_rate_kp = -5.8f;//电量11.5左右变回-5.8
        an_pid.angle_rate_ki = -0.0008f;
        //角度环
        an_pid.angle_kp = 1.5f;//电量11.5左右变回1.5
        an_pid.angle_kd = 0.01f;

        att_pid.Velocity_kp = 2.5f;
        att_pid.Velocity_ki = 0.0008f;
        att_pid.Velocity_kd = 2.f;
    }
    //转向环
    att_pid.Turn_kp1    = -1.2f;
    att_pid.Turn_kp2    = -0.00f;
    att_pid.Turn_kd     = -0.0f;
    att_pid.Gyro_GKD    = -7.50f;//
    att_pid.Gyro_GKD_1  = 0.0f;
    SPEEDMAX              = 1500;
    Yaw_Flag              = 0;
    wheel.TargetLenth =3.5;
}
/*  -400    -1.0f      0       0   -7.5f
 *  -500    -0.95f  -0.005f  -0.6f  -7.5f
 *  -600    -1.2f  -0.03f   -0.65f  -7.5f
 *  -700    -1.4f   -0.01f   -1.0f
 */


void PWM_Limiti(int32 *pwm,int32 limit)
{
    if(*pwm>limit) *pwm= limit;
    if(*pwm<-limit) *pwm= -limit ;
}
void PWM_Limitf(float *pwm,float limit)
{
    if(*pwm>limit) *pwm= limit;
    if(*pwm<-limit) *pwm= -limit ;
}

int32 Jump_Delay = 0,Jump_Flag = 0;
void Jump_Roadblock()
{
    att_pid.Velocity_kp=1.0f;////速度环
    att_pid.Velocity_ki=0.001f;//
    att_pid.Velocity_kd=1.0f;
    Jump_Delay++;

  switch (Jump_Flag)
  {
    case 0:
      if (Ele.status_flag == TIME_TO_JUMP){
          Jump_Flag = 1;
          erorr = 0;
      }
      break;
    case 1:
      AngleCal(1.1,1.1,3.0,3.0);
      if (Jump_Delay >= 80) Jump_Flag = 2;
      erorr = 0;
      break;
    case 2:
      AngleCal(1.1, 1.1, 12.0, 12.0);
      if (Jump_Delay >= 200) Jump_Flag = 3;
      erorr = 0;
      break;
    case 3:
      AngleCal(1.1,1.1,3.0,3.0);
      erorr = mid_err;
      if (Jump_Delay >= 320) Jump_Flag = 4;
      break;
    case 4:
      AngleCal(1.1, 1.1, 6.0, 6.0);
      if (Jump_Delay >= 360 && Jump_Flag == 4)
       {
          erorr = mid_err;
          Jump_Flag = 5;
          Jump_Delay = 0;
       }
      break;
  }
}
int32 s_count = 0;
int16 init_flag = 0;
void select_Function_Params()
{
    if(Ele.status_flag != WAITING) init_flag = 1;
    switch(Ele.status_flag)
    {
        case WAITING:           //正常状态
        {
            if(init_flag){
                pid_init();
                init_flag = 0;
            }
            erorr = mid_err;
            AngleCal(2.f,2.f,wheel.TargetLenth ,wheel.TargetLenth);
        }break;
        case TIME_TO_JUMP:      //跳跃
        {
            Jump_Roadblock();
        }
        break;
        case IS_ZEBRA:          //斑马线
        {
            att_pid.Targetspeed=0.0;
        }
        break;
        case IS_LCIRQUE://这个时候应该进行找下一个状态的条件
            /*
             Vel.Targetspeed=-450.0;//-300.0
             Con.Turn_kp1=-0.85f;//转向环-0.85
             Con.Turn_kp2=-0.000f;//0.0f;//
             Con.Turn_kd=-0.08f;//0.0f;//
             //Con.Gyro_GKD=-7.5f;//-7.5
             */
           // att_pid.Targetspeed=-400.0;
            att_pid.Turn_kp1=-0.80f;//转向环-0.85
            att_pid.Turn_kp2=-0.000f;//0.0f;//
            att_pid.Turn_kd=-0.85f;//0.0f;//
            att_pid.Gyro_GKD=-7.5f;//-7.5
             wheel.TargetLenth = 3.0;
             SPEEDMAX = 1200;

        break;
        case IN_LCIRQUE_BUXIAN:  //找入环的V字角点并补线,找到就补线
            //att_pid.Targetspeed=-350.0;
            att_pid.Turn_kp1=-1.2f;//转向环-0.85
            att_pid.Turn_kp2=-0.000f;//0.0f;//
            att_pid.Turn_kd=-0.90f;//0.0f;//
           // Con.Gyro_GKD=-7.0f;//-7.5

        break;
        case IN_LCIRQUE:  //在环内，找第一个不丢线点给他连上
           // att_pid.Targetspeed=-350.0;
            att_pid.Turn_kp1=-1.2f;//转向环-0.85
            att_pid.Turn_kp2=-0.00f;//0.0f;//
            att_pid.Turn_kd=-0.65f;//0.0f;//
          //Con.Gyro_GKD=-7.5f;//-7.5

        break;
        case OUT_LBUXIAN1:

            //att_pid.Targetspeed=-320.0;
            att_pid.Turn_kp1=-1.20f;//转向环-0.85
            att_pid.Turn_kp2=-0.000f;//0.0f;//
            att_pid.Turn_kd=-0.60f;//0.0f;//
            //Con.Gyro_GKD=-7.5f;//-7.5
        break;
        case OUT_LBUXIAN2:

           // att_pid.Targetspeed=-350.0;
            att_pid.Turn_kp1=-1.2f;//转向环-0.85
            att_pid.Turn_kp2=-0.000f;//0.0f;//
            att_pid.Turn_kd=-0.6f;//0.0f;//
          //Con.Gyro_GKD=-7.5f;//-7.5
        break;
        case OUT_LCIRQUE:

            //att_pid.Targetspeed=-400.0;
            att_pid.Turn_kp1=-1.2f;//转向环-0.85
            att_pid.Turn_kp2=-0.00f;//0.0f;//
            att_pid.Turn_kd=-0.f;//0.0f;//
            //Con.Gyro_GKD=-7.5f;//-7.5
        break;

        case IS_RCIRQUE://这个时候应该进行找下一个状态的条件
            /*
             Vel.Targetspeed=-450.0;//-300.0
             Con.Turn_kp1=-0.85f;//转向环-0.85
             Con.Turn_kp2=-0.000f;//0.0f;//
             Con.Turn_kd=-0.08f;//0.0f;//
             //Con.Gyro_GKD=-7.5f;//-7.5
             */
            att_pid.Targetspeed=-550.0;
            att_pid.Turn_kp1=-0.80f;//转向环-0.85
            att_pid.Turn_kp2=-0.006f;//0.0f;//
            att_pid.Turn_kd=-0.85f;//0.0f;//
            att_pid.Gyro_GKD=-7.5f;//-7.5
             wheel.TargetLenth = 3.0;
             SPEEDMAX = 1200;

        break;
        case IN_RCIRQUE_BUXIAN:  //找入环的V字角点并补线,找到就补线
            att_pid.Targetspeed=-570.0;
            att_pid.Turn_kp1=-0.11f;//转向环-0.85
            att_pid.Turn_kp2=-0.006f;//0.0f;//
            att_pid.Turn_kd=-0.90f;//0.0f;//
           // Con.Gyro_GKD=-7.0f;//-7.5
        break;
        case IN_RCIRQUE:  //在环内，找第一个不丢线点给他连上
            att_pid.Targetspeed=-570.0;
            att_pid.Turn_kp1=-0.8f;//转向环-0.85
            att_pid.Turn_kp2=-0.0035f;//0.0f;//
            att_pid.Turn_kd=-0.65f;//0.0f;//
          //Con.Gyro_GKD=-7.5f;//-7.5

        break;
        case OUT_RBUXIAN1:

            att_pid.Targetspeed=-600.0;
            att_pid.Turn_kp1=-0.70f;//转向环-0.85
            att_pid.Turn_kp2=-0.005f;//0.0f;//
            att_pid.Turn_kd=-0.60f;//0.0f;//
            //Con.Gyro_GKD=-7.5f;//-7.5
        break;
        case OUT_RBUXIAN2:

            att_pid.Targetspeed=-600.0;
            att_pid.Turn_kp1=-0.85f;//转向环-0.85
            att_pid.Turn_kp2=-0.0045f;//0.0f;//
            att_pid.Turn_kd=-0.6f;//0.0f;//
          //Con.Gyro_GKD=-7.5f;//-7.5
        break;
        case OUT_RCIRQUE:

            att_pid.Targetspeed=-570.0;
            att_pid.Turn_kp1=-0.95f;//转向环-0.85
            att_pid.Turn_kp2=-0.004f;//0.0f;//
            att_pid.Turn_kd=-0.7f;//0.0f;//
            //Con.Gyro_GKD=-7.5f;//-7.5
        break;
    }
    switch(Ele.Element_flag)
    {
       case IS_OUT_EDGE:
       {
           att_pid.Targetspeed = 0.0;
           att_pid.Turn_kp1    = 0.0;
           att_pid.Turn_kd     = 0.0;
           att_pid.Gyro_GKD    = 0.0f;//-7.5
       }break;
       case IS_Single_sided_bridge:
       {
           s_count++;
           SPEEDMAX              = 600;
           Yaw_Flag              = 1;//开启YAW积分Z_Yaw
           att_pid.Med_Angle   = 4.0;
           an_pid.angle_rate_kp = -5.8f;
           an_pid.angle_rate_ki = -0.0008f;
           //角度环
           an_pid.angle_kp = 1.8f;
           an_pid.angle_kd = 0.01f;
           wheel.TargetLenth = 5.5f;
           AngleCal(1.5f,1.5f,wheel.TargetLenth ,wheel.TargetLenth);
           //if(s_count > 50)
            //{
               //hybrid_control_flag = 1;
               att_pid.Targetspeed = -400.0;
               Single_sided_bridge();
               //if(s_count >= 3000) hybrid_control_flag = 0;
            //}
           att_pid.Gyro_GKD_1 = 1.1f;
           float roll =0;
           roll = My_ABS_F(eulerAngle.roll)*10;
           if( roll >= 9 )Single_sided_bridge_flag = 1;
           else Single_sided_bridge_flag = 0;
        }break;
   }
}
/*float calculate_turn_Bendangle(){
    static float bd_factor = 0.21f;
    static float filtter_acc_y,OUTPUT;
    filtter_acc_y =  0.1*imu660ra_acc_y + 0.9 * filtter_acc_y;
    OUTPUT = bd_factor * (filtter_acc_y - (-110));
    float filtter_output =  0.1*OUTPUT + 0.9 * filtter_output;

    if(abs(filtter_output) >3.0) filtter_output =3.0;
    if(abs(filtter_output) <0.5) filtter_output =0.0;
    //wheel.TargetLenth = filtter_output + 3.0;
    return (int)(filtter_acc_y/100);
}*/
int32 Time_count=0;
float Angel_Get;
float erorr;
Vertical_kp = -8;//
Vertical_kd = 8.0;//
int32 Vertical(float Mid,float Pitch,float gyro_y)//直立环
{
    static int32 PWM_OUT = 0;
    PWM_OUT = Vertical_kp*(Pitch-Mid) + Vertical_kd*gyro_y;
    return PWM_OUT;

}
void Con_loop()
{
    Time_count++;
    IMU_getEulerianAngles();
    select_Function_Params();
    if(Time_count % 10 == 0)
      {
        int16 speed_left  = (motor_value.receive_left_speed_data) ;
        int16 speed_right = (motor_value.receive_right_speed_data) ;
        att_pid.Velocity_out  = Velocity_in(att_pid.Targetspeed,speed_left,speed_right);
        //att_pid.Velocity_out  = hybrid_control(&ctrl,att_pid.Targetspeed,speed_left,speed_right);
        //Time_count        = 0;
      }
    if(Time_count % 3 == 0)
      {
        att_pid.Turn_out = Turn(erorr,imu_data.gyro_z);//mid_err
      }
     if(Time_count % 5 == 0)
      {
        Angel_Get   = up_right(0,att_pid.Med_Angle,eulerAngle.pitch);
      }
    float Gyro_Get  = Angle_Rate_Loop(Angel_Get,imu_data.gyro_y);
    //float Gyro_Get = Vertical(att_pid.Med_Angle,eulerAngle.pitch,imu_data.gyro_y);
    /*if(Time_count  >= 3000)
      {
        att_pid.Targetspeed = -500.0;
       }
    if(Time_count  >= 6000)  att_pid.Targetspeed = -800.0;*/
    //AngleCal(1.3, 1.3, 10.0, 10.5);
    //small_driver_set_duty(8 * (PWM_DUTY_MAX / 100), 8 * (PWM_DUTY_MAX / 100));
     if(SELECTED_OBJECT == 1)
      {
         int32 LPWM_OUT  = (int32)(Gyro_Get + att_pid.Turn_out);//Gyro_Get  Con.Vertical_out
         int32 RPWM_OUT  = (int32)(Gyro_Get - att_pid.Turn_out);//

         PWM_Limiti(&LPWM_OUT,PWMMAX);
         PWM_Limiti(&RPWM_OUT,PWMMAX);

         small_driver_set_duty(LPWM_OUT * (PWM_DUTY_MAX / 100), -RPWM_OUT * (PWM_DUTY_MAX / 100));
      }
     if(SELECTED_OBJECT == 2)
      {
         int32 LPWM_OUT  = (int32)(Gyro_Get -1.0*att_pid.Turn_out);//Gyro_Get  Con.Vertical_out
         int32 RPWM_OUT  = (int32)(Gyro_Get +1.0*att_pid.Turn_out);//

         PWM_Limiti(&LPWM_OUT,PWMMAX);
         PWM_Limiti(&RPWM_OUT,PWMMAX);

         small_driver_set_duty(-LPWM_OUT * (PWM_DUTY_MAX / 100),RPWM_OUT * (PWM_DUTY_MAX / 100));
      }
}









