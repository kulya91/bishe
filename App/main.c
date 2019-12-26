/*******************************************************************************/
#include "common.h"
#include "include.h"
#include "math.h"
#include "MK60_FTM.h"
#include <stdio.h>      /* printf */
#include <math.h>       /* atan */

uint8 img[CAMERA_H][CAMERA_W];                       //LCD显示  0xff黑   0x00白
uint8 imgbuff[CAMERA_SIZE];                          //定义存储接收图像的数组

uint8 straig_Lline[60]={                              //左边直线储存数组
  0 ,30 ,29 ,28 ,27 ,26,25,24,24,23,
  23,22,22,21,20,19,19,18,18,17,
  16,16,15,15 ,14 ,13 ,13 ,12 ,12 ,12 ,
  11 ,11 ,10 ,9 ,9 ,8 ,8 ,7 ,7 ,6 ,
  6 ,5 ,5 ,4 ,4 ,4 ,3 ,3 ,2 ,2 ,
  2 ,1 ,1 ,1 ,0 ,0 ,0 ,0 ,0 ,0
};

uint8 straig_Rline[60]={                             //右边直线储存数组
  79,42,43,43,44,45,46,47,48,49,
  50,51,52,53,54,55,56,57,58,59,
  60,61,61,62,63,64,64,65,66,67,
  67,68,67,67,68,69,69,70,71,71,
  71,72,72,73,74,74,75,75,76,76,
  77,77,77,78,78,79,79,79,79,79
};

uint8 straig_Mline[60]={                                //中间直线储存数组
  40,40,40,40,40,40,40,40,40,40,
  40,40,40,40,40,40,40,40,40,40,
  40,40,40,40,40,40,40,40,40,40,
  40,40,40,40,40,40,40,40,40,40,
  40,40,40,40,40,40,40,40,40,40,
  40,40,40,40,40,40,40,40,40,40
};

uint8 xz[60]={0,0,0,0,0,0,0,0,0,0,
0,1,1,2,2,3,4,5,6,8,
9,10,11,12,13,14,15,16,17,18,
19,20,21,22,23,24,25,26,27,28,
29,30,30,31,32,33,33,34,35,36,
37,37,38,39,40,40,0,0,0,0};           //矫正数组

uint8 xz1[60]={0,11,12,13,14,15,16,17,18,19,
20,21,21,23,24,24,25,26,27,28,
28,28,29,30,30,31,32,33,33,34,
35,35,36,36,37,38,38,39,39,39,
40,40,40,40,40,40,40,40,40,40,
40,40,40,40,40,40,40,40,40,40          //矫正数组
};

//1维的卡尔曼滤波
typedef struct {
  float x;  // 系统的状态量 
  float A;  // x(n)=A*x(n-1)+u(n),u(n)~N(0,q) 
  float H;  // z(n)=H*x(n)+w(n),w(n)~N(0,r)   
  float q;  // 预测过程噪声协方差
  float r;  // 测量过程噪声协方差 
  float p;  // 估计误差协方差
  float gain;//卡尔曼增益
}kalman_struct_left;

//1维的卡尔曼滤波
typedef struct {
  float x;  // 系统的状态量 
  float A;  // x(n)=A*x(n-1)+u(n),u(n)~N(0,q) 
  float H;  // z(n)=H*x(n)+w(n),w(n)~N(0,r)   
  float q;  // 预测过程噪声协方差
  float r;  // 测量过程噪声协方差 
  float p;  // 估计误差协方差
  float gain;//卡尔曼增益
}kalman_struct_right;

kalman_struct_left kalman_left;
kalman_struct_right kalman_right;
/******************************************电机舵机模块宏定义***********************************************************/
#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH2          //A5
#define MOTOR2_PWM  FTM_CH3          //A6
#define MOTOR3_PWM  FTM_CH4          //A7
#define MOTOR4_PWM  FTM_CH5          //D5

/********************************************************舵机变量模块*******************************************/
//  下面是  s30110 舵机  频率  50Hz  时间控制
#define dj_mid              695                    //舵机居中                680.0
#define dj_left_max         625                    //左极限                  650.0
#define dj_right_max        765                    //右极限                  820.0

uint16 duoji_duty;

/********************************************************电机PID*******************************************/
#define MOTOR_HZ    (20*1000)                        //电机频率

float  SpeedKP_left;
float  SpeedKI_left;
float  SpeedKD_left;

float  SpeedKP_right;
float  SpeedKI_right;
float  SpeedKD_right;

float left_E[3]={0.0,0.0,0.0};                            //左偏差
float right_E[3]={0.0,0.0,0.0};                           //右偏差

double right_speed=0.0,left_speed=0.0;                    //左轮输出PWM 右轮输出PWM
int dianji_left_speed=0,dianji_right_speed=0;             // 左轮理论速度  右轮理论速度 (求出来的值)

/********************************************************赛道分析变量模块*******************************************/
#define i_max 40                                                                    //检测行最下方
#define i_min 10                                                                    //检测行最上方 

int mid[60]={39},mid1[60];                                                               //中线存储数组    赛道一半宽储存数组
int mid_L_line[60]={0},mid_R_line[60]={0};                                          //左右边缘存储数组
int flag_L[60]={0},flag_R[60]={0};                                                  //1 是找到点  0 是未找到点

uint8  mid_j=30;                                                                    //中线偏移

uint8 left_currve_flag,right_currve_flag;                                           //  左弯道   右弯道
uint8 currve_flag ;                                                                 //弯道标志
uint8 zhidao_flag ;                                                                 //直道标志

/********************************************************蓝牙模块*******************************************/
float var[5];
int16 val_left,val_right;
int16 val_left2,val_right2;
int var2[60];
int var3[60];

/********************************************避障模块*******************************************************/
uint8 BZ=0;;
double time=0.0;

/********TOF*********/
unsigned char rxbuf[16], waitflag;
char ch_2019[16];
int rxflag = 0 ,rxcnt = 0 ;
int length_val[3] = {0} ;
int i_2019;

/*******陀螺仪*******/
unsigned char ucRxBuffer[12];
unsigned char  ucRxCnt = 0;
short Angle[3];
float angle_x,angle_y,angle_z;
short Acc[3];
float acc_x,acc_y,acc_z;
float gy_distance;
float gy_speed;

/********************************************************函数声明模块*******************************************/
void PORTA_IRQHandler();                                                           //中断
void DMA0_IRQHandler();
void PIT0_IRQHandler();                                                            //定时器中断
void uart0_test_handler(void);                                                     //串口接受中断
void uart1_test_handler(void);                                                     //串口接受中断

void  initall();                                                                    //初始化
void TOF_1020();                                                                   //TOF数据处理

/**********舵机************/
void duoji();                                                                   
void disu_duoji();
/*********电机*************/
void dianji_baodi();                                                            
void dianji_Right_baodi();
void dianji_Left_baodi();
void dianjihuang();
/***********图像处理**************/
void  processImage();                                                             //图像处理主函数
void getMidLine();                                                                //绘制中线
void analyzeRoad();                                                               //道路提前分析
void findFlag();                                                                 //弯直道判断
int getObstacleDistance();                                                     //获取障碍物距离

void lcd_camera_init();                                                            //LCD初始化
void lcd_display();                                                                //LCD显示
void drawingMidLine();                                                             //画中线

/***********tools*****************/ 
float getGyroDate(float x1,float x2);                                            //获取陀螺仪两次差值
float getGyroDistance(float a);                                                  //获取陀螺仪横向位移
void bizhang_time();
int GYH(int AD_max,int AD_min,int value);                                        //归一化
float m_sqrt(unsigned int x);                                                   //开根号函数
float Qulv(int x1,int x2,int x3,int y1,int y2,int y3);                         //曲率
int Atan (float Bz_K);                                                           //反正切弧度转化为角度

void kalman_init_left(kalman_struct_left *kalman_lcw, float init_x, float init_p);
float kalman_filter_left(kalman_struct_left *kalman_lcw, float measure);

void kalman_init_right(kalman_struct_right *kalman_lcw, float init_x, float init_p);
float kalman_filter_right(kalman_struct_right *kalman_lcw, float measure);

/**
*kalman_init - 卡尔曼滤波器初始化
*@kalman_lcw：卡尔曼滤波器结构体
*@init_x：待测量的初始值
*@init_p：后验状态估计值误差的方差的初始值
*/
void kalman_init_left(kalman_struct_left *kalman_lcw, float init_x, float init_p)
{
  kalman_lcw->x = init_x;//待测量的初始值，如有中值一般设成中值
  kalman_lcw->p = init_p;//后验状态估计值误差的方差的初始值
  kalman_lcw->A = 1;
  kalman_lcw->H = 1;
  kalman_lcw->q = 10;//预测（过程）噪声方差 实验发现修改这个值会影响收敛速率
  kalman_lcw->r =30;//测量（观测）噪声方差
  //这里两个参数是最关键的
}

/**
*kalman_filter - 卡尔曼滤波器
*@kalman_lcw:卡尔曼结构体
*@measure；测量值
*返回滤波后的值
*/
float kalman_filter_left(kalman_struct_left *kalman_lcw, float measure)
{
  /* Predict */
  kalman_lcw->x = kalman_lcw->A * kalman_lcw->x;
  kalman_lcw->p = kalman_lcw->A * kalman_lcw->A * kalman_lcw->p + kalman_lcw->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */
  
  /* Measurement */
  kalman_lcw->gain = kalman_lcw->p * kalman_lcw->H / (kalman_lcw->p * kalman_lcw->H * kalman_lcw->H + kalman_lcw->r);
  kalman_lcw->x = kalman_lcw->x + kalman_lcw->gain * (measure - kalman_lcw->H * kalman_lcw->x);
  kalman_lcw->p = (1 - kalman_lcw->gain * kalman_lcw->H) * kalman_lcw->p;
  
  return kalman_lcw->x;
}
/**
*kalman_init - 卡尔曼滤波器初始化
*@kalman_lcw：卡尔曼滤波器结构体
*@init_x：待测量的初始值
*@init_p：后验状态估计值误差的方差的初始值
*/
void kalman_init_right(kalman_struct_right *kalman_lcw, float init_x, float init_p)
{
  kalman_lcw->x = init_x;//待测量的初始值，如有中值一般设成中值
  kalman_lcw->p = init_p;//后验状态估计值误差的方差的初始值
  kalman_lcw->A = 1;
  kalman_lcw->H = 1;
  kalman_lcw->q = 10;//预测（过程）噪声方差 实验发现修改这个值会影响收敛速率
  kalman_lcw->r = 30;//测量（观测）噪声方差
  //这里两个参数是最关键的
}

/**
*kalman_filter - 卡尔曼滤波器
*@kalman_lcw:卡尔曼结构体
*@measure；测量值
*返回滤波后的值
*/
float kalman_filter_right(kalman_struct_right *kalman_lcw, float measure)
{
  /* Predict */
  kalman_lcw->x = kalman_lcw->A * kalman_lcw->x;
  kalman_lcw->p = kalman_lcw->A * kalman_lcw->A * kalman_lcw->p + kalman_lcw->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */
  
  /* Measurement */
  kalman_lcw->gain = kalman_lcw->p * kalman_lcw->H / (kalman_lcw->p * kalman_lcw->H * kalman_lcw->H + kalman_lcw->r);
  kalman_lcw->x = kalman_lcw->x + kalman_lcw->gain * (measure - kalman_lcw->H * kalman_lcw->x);
  kalman_lcw->p = (1 - kalman_lcw->gain * kalman_lcw->H) * kalman_lcw->p;
  
  return kalman_lcw->x;
}

/**********************液晶显示**************************************************/
void lcd_display()
{
  Site_t site     = {0, 0};                             //显示图像左上角位置
  Size_t imgsize  = {CAMERA_W, CAMERA_H};               //图像大小
  Size_t size;                                          //显示区域图像大小
  size.H = 60;
  size.W = 80;
  
  LCD_Img_Binary_Z(site, size, imgbuff, imgsize);
  
  site.x =60;
  site.y =110;
  LCD_num(site,abs(val_left),FCOLOUR,BCOLOUR);
  site.y =95;
  LCD_num(site,abs(val_right),FCOLOUR,BCOLOUR);
  site.y =70;
  LCD_num_C(site,length_val[0],FCOLOUR,BCOLOUR);
  
  site.x =0;
  site.y =60;
  
  LCD_num_C(site,angle_x,FCOLOUR,BCOLOUR);
  site.y=80;
  LCD_num_C(site,angle_y,FCOLOUR,BCOLOUR);
  site.y=100;
  LCD_num_C(site,abs(angle_y),FCOLOUR,BCOLOUR);
  
  site.x =100;
  site.y =0;
  LCD_num_C(site,BZ,FCOLOUR,BCOLOUR);
  site.y=20;
  LCD_num_C(site,zhidao_flag,FCOLOUR,BCOLOUR);
  site.y=40;
  LCD_num_C(site,(int)(time*100),FCOLOUR,BCOLOUR);
}

/***************************dianjihuang电机驱动**********************************/
void dianjihuang()
{
  //right
  FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=0;
  FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=0;
  //left
  FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=0;
  FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=0;
}
/*********************************主函数*****************************************************/
void  main(void)
{
  
  initall();
  kalman_init_left(&kalman_left,0,10000);
  kalman_init_right(&kalman_right,0,10000);
  while(1)
  {
    processImage();
    lcd_display();      //lcd显示
    drawingMidLine();
    var[0] =val_right;
    var[1] =val_left;
    //var[2] =val_right2;
   // var[3] =val_left2;
    var[2]=dianji_right_speed;
    var[3]=-dianji_left_speed;
    var[4]=zhidao_flag*100;
    
    vcan_sendware((float *)var, sizeof(var));
    
    if(BZ==1)
    { DisableInterrupts;
    dianjihuang();
    return;
    //DELAY_MS(50);
    }
  }
}

/*************************************舵机;由平均值计算出占空比**************************************/
void duoji()
{
  float x = 0.0,sum=0.0,ave=0.0; ;
  //舵机占空比
  float KP;                                             //PD算法常量
  float KD;
  
  static float duoji_last_error=0.0,duoji_error=0.0;             //中线平均偏差值
  
  if( zhidao_flag == 1&&BZ==0)
  {
    KP = 2.0 ;
    KD = 4.0 ;
  }
  else if(BZ==1)
  {
    KP = 4.6 ;
    KD = 5.2 ;
  }
  else
  {
    KP = 5.2 ;
    KD = 6.9 ;
  }
  
  for(int  s = i_max ; s >= i_min ; s-- )                   //计算均值
  {
    sum+=mid[s];
    x++;
  }
  
  ave=sum/x ;
  sum=0 ;
  
  duoji_error=ave-mid_j ;                               //计算偏差
  
  duoji_duty=(uint16)(dj_mid+KP*duoji_error+KD*(duoji_error-duoji_last_error) ) ;   //dj;
  duoji_last_error=duoji_error ;
  
  if(duoji_duty<dj_left_max)
    duoji_duty=dj_left_max;
  if(duoji_duty>dj_right_max)
    duoji_duty=dj_right_max;
  ftm_pwm_duty(FTM3 ,FTM_CH6,duoji_duty);                  //给舵机占空比  duoji_duty       1450
  if( abs(duoji_duty-dj_mid) <= 35 )
    zhidao_flag = 1 ;
  else
    zhidao_flag = 0 ;
}


/****************************baodi电机驱动**********************************/
void dianji_baodi()
{
  
  float A;                      //
  float B;
  float K;
  uint16 V;
  if(zhidao_flag==1)
  {
    V =75; A = 1.0 ;  B = 1.0 ;  K = 0.0 ;
  }
  else
  {
    V =75; A = 1.0 ;  B = 1.0 ;  K = 0.15 ;
  }
  
  
  
  SpeedKP_right=0;
  SpeedKI_right=50;
  SpeedKD_right=0;
  
  SpeedKP_left=0;
  SpeedKI_left=50;
  SpeedKD_left=0;
  
  
  
  dianji_right_speed = (int)( A*V*( B+ K*(duoji_duty-dj_mid)/70.0 ) );
  dianji_left_speed  = (int)( A*V*( B- K*(duoji_duty-dj_mid)/70.0 ) );
  dianji_Left_baodi();
  dianji_Right_baodi();
  
  
}

void dianji_Left_baodi()
{
  //  if(zhidao_flag==1)
  //  {
  //    SpeedKP_left=0;
  //    SpeedKI_left=65;
  //    SpeedKD_left=0;
  //  }
  //  else
  //  {
  //    SpeedKP_left=25;
  //    SpeedKI_left=5;
  //    SpeedKD_left=0;
  //  }
  //  
  
  left_E[2]=left_E[1];
  left_E[1]=left_E[0];
  left_E[0]=dianji_left_speed+val_left;
  // left_E[0]=75+val_left;
  left_speed=left_speed
    +SpeedKP_left*( left_E[0] - left_E[1])
      +SpeedKI_left*( left_E[0] )
        +SpeedKD_left*(left_E[0] - 2*left_E[1]+  left_E[2]);
  if(left_speed>=1800)
    left_speed=1800;
  if(left_speed<=700)
    left_speed=700;
  FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=(uint32)(left_speed);        
  FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=0;                       //   第二个电机驱动
  
}

void dianji_Right_baodi()
{
  //  if(zhidao_flag==1)
  //  {
  //    SpeedKP_right=0;
  //    SpeedKI_right=65;
  //    SpeedKD_right=0;
  //  }
  //  else
  //  {
  //    SpeedKP_right=20;
  //    SpeedKI_right=2.5;
  //    SpeedKD_right=0;
  //  }
  
  right_E[2]=right_E[1];
  right_E[1]=right_E[0];
  right_E[0]=dianji_right_speed-val_right;
  //right_E[0]=75-val_right;
  right_speed=right_speed
    +SpeedKP_right*( right_E[0] - right_E[1])
      +SpeedKI_right*( right_E[0] )
        +SpeedKD_right*(right_E[0] - 2*right_E[1]+  right_E[2]);
  if(right_speed>=1800)
    right_speed=1800;
  if(right_speed<=700)
    right_speed=700;
  FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=0;
  FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=(uint32)right_speed;
  
  
}

/********************************初始化******************************************************/
void  initall()
{
  ftm_quad_init(FTM2);                                     //FTM1 正交解码初始化（所用的管脚可查 port_cfg.h ）
  ftm_quad_init(FTM1);
  
  ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,0);          //初始化 电机 PWM             第一个电机驱动
  ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,0);          //初始化 电机 PWM            第二个电机驱动
  ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,0);          //初始化 电机 PWM            第三个电机驱动
  ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,0);          //初始化 电机 PWM             第四个电机驱动
  
  lcd_camera_init();                                     //LCD初始化
  
  ftm_pwm_init(FTM3, FTM_CH6,50,dj_mid);                //初值定义    T1  PTA12
  
  /*************定时器0中断，编码器，舵机，电机******************/
  pit_init_ms(PIT0,10);                                     //初始化PIT0，定时时间为： 10ms
  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);       //设置PIT0的中断服务函数为 PIT0_IRQHandler
  enable_irq (PIT0_IRQn);                                  //使能PIT0中断
  
  /*************定时器1中断，上位机i调试蓝牙发送******************/
  /*pit_init_ms(PIT1,10);                                     //初始化PIT1，定时时间为： 10ms
  set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);       //设置PIT1的中断服务函数为 PIT1_IRQHandler
  enable_irq (PIT1_IRQn);                                  //使能PIT1中断
  */
  
  /*************初始化串口UART0，TOF或者蓝牙******************/
  uart_init(UART0, 9600);                       
  set_vector_handler(UART0_RX_TX_VECTORn,uart0_test_handler);
  uart_rx_irq_en(UART0);
  
  /*************初始化串口UART1，陀螺仪******************/
  uart_init(UART1, 115200);                      
  set_vector_handler(UART1_RX_TX_VECTORn,uart1_test_handler);
  uart_rx_irq_en(UART1);
  
  /*******************设置中断优先级******************/                                           
  NVIC_SetPriorityGrouping(4);            //设置优先级分组,4bit 抢占优先级,没有亚优先级
  NVIC_SetPriority(PORTA_IRQn,0);         //配置优先级
  NVIC_SetPriority(DMA0_IRQn,1);          //配置优先级
  NVIC_SetPriority(UART0_RX_TX_IRQn,2);
  NVIC_SetPriority(UART1_RX_TX_IRQn,3);   //配置优先级
  NVIC_SetPriority(PIT0_IRQn,4);          //配置优先级
  NVIC_SetPriority(PIT0_IRQn,5);          //配置优先级
  
  DisableInterrupts;
  EnableInterrupts;
  
  SCCB_WriteByte ( OV7725_CNST, 40 );      //调阈值  //22
}

/********************************定时器中断0，编码器，电机，清中断***************************/
void PIT0_IRQHandler()
{
  val_right=ftm_quad_get(FTM1);
  val_left=ftm_quad_get(FTM2);
  //val_right2 =kalman_filter_right(&kalman_right,val_right);
  //val_left2 =kalman_filter_left(&kalman_left,val_left);
  TOF_1020();
  bizhang_time();
  duoji();
  dianji_baodi();
  //dianjihuang();
  ftm_quad_clean(FTM2);
  ftm_quad_clean(FTM1); 
  PIT_Flag_Clear(PIT0);       //清中断标志位
}

void PIT1_IRQHandler()
{
  
  //vcan_sendware((float *)var, sizeof(var));
  PIT_Flag_Clear(PIT1);       //清中断标志位
}
/********************************UART0蓝牙接收**********************************************/
void uart0_test_handler(void)
{
  UARTn_e uratn = UART0;
  
  if(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK)   //接收数据寄存器满
  {
    //用户需要处理接收数据
    if(rxflag==0)
    {
      if(rxcnt>=15)
      {
        rxcnt=0;
      }
      uart_getchar(UART0,&ch_2019[rxcnt++]);
      if(rxcnt>2)
      {
        
        
        if((ch_2019[rxcnt-1]=='m')&&(ch_2019[rxcnt-2]=='m'))//
        {
          rxcnt--;
          rxflag=1;
          uart_rx_irq_dis(UART0);
        }
        
      }
    }
  }
}

/*********************************UART1陀螺仪接收*******************************************/
void uart1_test_handler(void)
{
  UARTn_e uratn = UART1;
  if(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK)   //接收数据寄存器满
  {
    uart_getchar(UART1,&ucRxBuffer[ucRxCnt++]);
    if (ucRxBuffer[0]!=0x55) 
    {
      ucRxCnt=0;
      return;																																	  
    }
    if (ucRxCnt<11) {
      return;}
    else
    {
      switch(ucRxBuffer[1])
      {
      case 0x51:
        Acc[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
        Acc[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
        Acc[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
        acc_x=(float)Acc[0]/32768*16*9.8+0.315;
        acc_y=(float)Acc[1]/32768*16*9.8-0.6;
        acc_z=(float)Acc[2]/32768*16*9.8;
        // getGyroDistance(acc_x);
        
        break;
      case 0x53:
        Angle[0] = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
        Angle[1] = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
        Angle[2] = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
        angle_x=(float)(Angle[0]/32768.0*180);
        angle_y=(float)(Angle[1]/32768.0*180);
        angle_z=(float)(Angle[2]/32768.0*180);
        break;
      default:
        break;
        
      }
      ucRxCnt=0;
      
    }
  }
}

/*********************************避障计时**************************************************/
void bizhang_time()
{
  if(BZ==1)
  {
    time+=0.01;
    mid_j=50;
    if(time>=0.10)mid_j=55;
    if(time>=0.25)mid_j=50;
    if(time>=0.30)mid_j=45;
    if(time>=0.35)mid_j=32;
    if(time>=0.45)
    {
      BZ=0;
      time=0;
      mid_j=30;
    }
  }
}

/*********************************TOF测距函数***********************************************/
void TOF_1020()
{
  if(rxflag)
  {
    for(i_2019=0; i_2019<=rxcnt; i_2019++)
    {
      if(ch_2019[i_2019]=='m')
      {
        if(ch_2019[i_2019+1]=='m')	//ASCII
        {
          length_val[2] = length_val[1];
          length_val[1] = length_val[0];                                   //距离储存
          
          if((i_2019>0)&&(ch_2019[i_2019-1]>='0')&&(ch_2019[i_2019-1]<='9'))
            length_val[0]=ch_2019[i_2019-1]-'0';
          if((i_2019>1)&&(ch_2019[i_2019-2]>='0')&&(ch_2019[i_2019-2]<='9'))
            length_val[0]+=(ch_2019[i_2019-2]-'0')*10;
          if((i_2019>2)&&(ch_2019[i_2019-3]>='0')&&(ch_2019[i_2019-3]<='9'))
            length_val[0]+=(ch_2019[i_2019-3]-'0')*100;
          if((i_2019>3)&&(ch_2019[i_2019-4]>='0')&&(ch_2019[i_2019-4]<='9'))
            length_val[0]+=(ch_2019[i_2019-4]-'0')*1000;
          break;
        }
      }
    }
    rxflag = 0;
    rxcnt = 0;
    uart_rx_irq_en(UART0);
  }
  
  
  if( length_val[0] <= 650 && length_val[0] >= 100 )
  {
    if( BZ == 0 )       //不是在避障过程
    {
      BZ = 1 ;
    }
    
  }
  
}

/*************************图像处理************************************/
void  processImage()
{
  camera_get_img();
  img_extract((uint8 *)img,(uint8 *)imgbuff,CAMERA_SIZE);       //解压为二维数组
  //vcan_sendimg((uint8 *)imgbuff, sizeof(imgbuff));
  analyzeRoad();
  //findFlag();
  getMidLine();
}

/************************************分析道路****************************************/
void analyzeRoad()
{
  uint8 mid_back=40;
  for(int i=i_max;i>=i_min;i--)
  {
    for(int j=mid_back;j>1;j--)                        //左边赛道
    {
      if(img[i][j]==0xff && img[i][j-1]==0x00)
      {
        mid_L_line[i]=j-1;
        flag_L[i]=1;
        break;
      }
      else
      {
        flag_L[i]=0;
      }
    }
    for(int j=mid_back;j<78;j++)                       //右边赛道
    {
      if(img[i][j]==0xff && img[i][j+1]==0x00)
      {
        mid_R_line[i]=j+1;
        flag_R[i]=1;
        break;
      }
      else
      {
        flag_R[i]=0;
      }
    }
    if(mid[i] <= 0)     mid_back=1;
    else if (mid[i] >= 79)    mid_back=78;
    else if(mid[i] > 0 && mid[i] < 79)   mid_back=mid[i];
    else  mid_back=40;
  }
}

/************************************直道检测****************************************/
void findFlag()
{
  uint8 daolu_i = 0 , daolu_Max = i_max , daolu_Min = i_min ;
  uint8 daolu_Mid_lose = 0;
  uint8 daolu_sum = 0 ;
  for( int i = daolu_Min ; i <= daolu_Max -  5  ; i ++ )         //直道
  {
    if( flag_L[i] == 1 && flag_R[i] == 1 )
    {
      daolu_sum ++ ;
    }
    else   daolu_Mid_lose ++ ;
    
    if( daolu_Mid_lose >= 6 )     break;
  }
  
  if( daolu_sum >= 24 )
    zhidao_flag = 1 ;
  else
    zhidao_flag = 0 ;
  
}

/*************************寻找中心线************************************/
void  getMidLine()
{
  for(int i=i_max;i>=i_min;i--)
  {
    //左右都有
    if(flag_L[i]==1 && flag_R[i]==1)
    {
      mid[i]=(mid_R_line[i]+mid_L_line[i])/2;
      mid1[i]=(mid_R_line[i]-mid_L_line[i])/2;
      // mid[i]=mid_R_line[i]-xz[i];             //调矫正数组用
      // mid[i]=mid_L_line[i]+xz[i];             //调矫正数组用
      
    }
    //左丢线
    if(flag_L[i]==0 && flag_R[i]==1)
    {
      mid[i]=mid_R_line[i]-xz1[i];
    }
    //右丢线
    if(flag_L[i]==1 && flag_R[i]==0)
    {
      mid[i]=mid_L_line[i]+xz1[i];
      
    }
    //左右同时丢线
    if(flag_L[i]==0 && flag_R[i]==0)                      //未详细分解
    {
      mid[i] = mid[i+1]; 
    }
  }
}

/****************************障碍物判断****************************************/
int getObstacleDistance()
{
  for(int i=i_max;i>=i_min-5;i--)
  {
    if(mid_R_line[i]<mid_R_line[i-2])
      return i;
  }
  return 0;
}

/**********************************画出中线**************************************/
void drawingMidLine()
{
  for(int i=1;i<=59;i++)
  {
    if(mid[i]>0&&mid[i]<80)
    {
      Site_t midline[1]={39,i};//40
      LCD_points(midline,1,BLUE);
    }
  }
  for(int i=i_min;i<=i_max;i++)
  {
    if(mid[i]>0&&mid[i]<80)
    {
      Site_t midline[1]={mid[i],i};//40
      LCD_points(midline,1,RED);
    }
  }
  
  for(int i=0;i<80;i++)
  {
    Site_t hangf[1]={i,30};
    Site_t hangs[1]={i,20};
    Site_t hangt[1]={i,40};
    Site_t hangq[1]={i,10};
    
    LCD_points(hangf,1,BLUE);
    LCD_points(hangs,1,BLUE);
    LCD_points(hangt,1,BLUE);
    LCD_points(hangq,1,BLUE);
    
  }
  for(int i=1;i<60;i++)
  {
    Site_t hangL[1]={straig_Lline[i],i};
    Site_t hangR[1]={straig_Rline[i],i};
    
    LCD_points(hangL,1,GREEN);
    LCD_points(hangR,1,GREEN);
  }
}

/**********************液晶摄像头初始化*******************************************/
void lcd_camera_init()
{
  LCD_init();                                             //LCD_init
  camera_init(imgbuff);
  //配置中断服务函数
  set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置 PORTA 的中断服务函数为 PORTA_IRQHandler
  set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //设置 DMA0 的中断服务函数为 PORTA_IRQHandler
}

/*************************************PORTA中断服务函数********************************************/
void PORTA_IRQHandler()
{
  uint8  n;    //引脚号
  uint32 flag;
  
  while(!PORTA_ISFR);
  flag = PORTA_ISFR;
  PORTA_ISFR  = ~0;                                   //清中断标志位
  
  n = 29;                                             //场中断
  if(flag & (1 << n))                                 //PTA29触发中断
  {
    camera_vsync();
  }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
  n = 28;
  if(flag & (1 << n))                                 //PTA28触发中断
  {
    camera_href();
  }
#endif
  
  
}

/*************************************DMA0中断服务函数********************************************/
void DMA0_IRQHandler()
{
  camera_dma();
}

/***************************************陀螺仪工具函数**************************************************/
/***************************************获取陀螺仪两次差值****************************************/
float getGyroDate(float x1,float x2)
{
  float angle;
  if(x1>=x2)
  {
    if(x1-x2>=180.0)
      angle=360.0-(x1-x2);
    else
      angle=x2-x1;
  }
  else
  {
    if(x2-x1>=180.0)
      angle=(x2-x1)-360.0;
    else
      angle=x2-x1;
  }
  return angle;
}

float getGyroDistance(float a)
{
  gy_speed=gy_speed+a*0.01;
  gy_distance=gy_distance+gy_speed*0.01;
  return gy_distance;
}
/***************************************开根号***********************************************/

float m_sqrt(unsigned int x)
{
  uint8 ans=0,p=0x80;
  while( p != 0)
  {
    ans += p;
    if(ans * ans > x)
    {
      ans -= p;
    }
    p=(uint8)(p/2);
  }
  return(ans);
}

/*******************************************曲率计算******************************************/
float Qulv(int x1,int x2,int x3,int y1,int y2,int y3)
{
  int S;
  float a,b,c;
  S=((x2-x1)*(y3-y1)-(x3-x1)*(y2-y1))/2;
  a=m_sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
  b=m_sqrt((x3-x2)*(x3-x2)+(y3-y2)*(y3-y2));
  c=m_sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1));
  return 4*S/(a*b*c);
}

/*******************************************角度计算******************************************/
int Atan (float Bz_K)
{
  float param = Bz_K, result;
  result = (atan (param) * 180 / PI);  //将弧度转换为度
  return (int)result;
}

/*************************************归一化函数********************************************/
int GYH(int AD_max,int AD_min,int value)
{
  float val = value;
  val = (val > AD_max) ? AD_max : val;
  val = (val < AD_min) ? AD_min : val;
  val = (val - AD_min)/(AD_max - AD_min);
  if( val < 0.01 )   val=0.01;
  if( val > 1.0 )   val=1.00;
  val *= 100;
  return (int)val;
}

/**************************************递归函数******************************************/
/*uint8 digui( int8 x1 , int8 x2)
{
if( mid_R_line[x1] - mid_R_line[x1-2] <= 3 && mid_R_line[x1] - mid_R_line[x1-2] >= 0  && mid_R_line[x1] <= 78
&& mid_L_line[x1 - 2] - mid_L_line[x1] <= 3 && mid_L_line[x1 - 2] - mid_L_line[x1] >= 0  && mid_L_line[x1] >= 3 )
{
if( mid_R_line[x1] - mid_L_line[x1] >=  mid_R_line[x1 - 2] - mid_L_line[x1 - 2] )
{
return( 1 + digui(x1) );


if( mid_R_line[QIE_i] - mid_L_line[QIE_i] <= 10)    return( );

    }
  }

}                  
*/