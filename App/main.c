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
  0 ,27 ,26 ,25 ,24 ,23,22,21,21,20,
  19,18,17,16,16,15,14,13,13,12,
  11,11,10,9 ,8 ,7 ,7 ,6 ,6 ,5 ,
  5 ,4 ,3 ,3 ,2 ,2 ,1 ,1 ,0 ,0 ,
  0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
  0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0
};
uint8 straig_Rline[60]={                             //右边直线储存数组
  79,48,49,50,51,52,53,54,55,56,
  56,57,58,59,60,60,61,62,63,64,
  64,65,66,67,68,69,69,70,71,72,
  72,73,74,74,75,76,76,77,77,78,
  78,78,79,79,79,79,79,79,79,79,
  79,79,79,79,79,79,79,79,79,79
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

/********************************************************舵机/电机变量模块*******************************************/

//舵机定义
#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH2          //A5
#define MOTOR2_PWM  FTM_CH3          //A6
#define MOTOR3_PWM  FTM_CH4          //A7
#define MOTOR4_PWM  FTM_CH5          //D5

#define MOTOR1_PWM_IO  FTM0_CH3
#define MOTOR2_PWM_IO  FTM0_CH4
#define MOTOR3_PWM_IO  FTM0_CH2
#define MOTOR4_PWM_IO  FTM0_CH5


//#define PI 3.14

float  SpeedKP_left;
float  SpeedKI_left;
float  SpeedKD_left;

float  SpeedKP_right;
float  SpeedKI_right;
float  SpeedKD_right;

float left_E[3]={0.0,0.0,0.0};          //左偏差
float right_E[3]={0.0,0.0,0.0};         //右偏差

double right_speed=0.0,left_speed=0.0;  //左轮输出PWM 右轮输出PWM
int dianji_left_speed=0,dianji_right_speed=0;   // 左轮理论速度  右轮理论速度 (求出来的值)

//  下面是  s30110 舵机  频率  50Hz  时间控制
#define dj_mid              680                    //舵机居中  10000-1500              680.0
#define dj_left_max         610                 //左极限    10000-1675              650.0
#define dj_right_max        750                  //右极限    10000-1375              820.0

float KP=5.6 ;   // 5.6                               //PD算法常量
float KD=8.6;

float KP1=5.6 ;   //摄像头斜率
float KD1=8.6;

float KPV=5.6 ;   // 5.6                               //PD算法常量
float KDV=3.1;       //3.1


uint16 duoji_duty=800 , duoji_duty_K = 0;                                //舵机占空比
uint16 duoji_duty_V = 800 ,duoji_last_dutyV=0;
uint16 BZ_duty1 = 0 ;                                                   //避障过程中的舵机占空比
uint16 BZ_dutyV = 0 ;                                                  //避障最后一次舵机的占空比（回来时的角度）
uint16 BZ_duty = 0 ;                                                   //避障占空比

/**********************************************************************************************************/
float duoji_last_error=0.0,duoji_error=0.0;             //中线平均偏差值
float duoji_last_errorV=0.0,duoji_errorV=0.0;             //中线平均偏差值
float sum=0.0,ave=0.0;

float duoji_K1 = 0.0 , duoji_K2 = 0.0 ;             //舵机斜率
float duoji_error_hunhe = 0.0 ;

uint8 duo_ji_SX=40,duo_ji_XX=13;                     //控制舵机距离上下限     (45 30)

//电机
#define MOTOR_HZ    (20*1000)                        //电机频率


/********************************************************赛道分析变量模块*******************************************/

int mid[60],mid1[60];                                                               //中线存储数组    赛道一半宽储存数组
int mid_L_line[60]={0},mid_R_line[60]={0},mid_L_line_last=0,mid_R_line_last=0;      //中线储存数组
int panduanL[60]={0},panduanR[60]={0};                                            //0 是找到点  1 是未找到点

uint8 left_loss_line=0,right_loss_line=0;                                           //暂时置为全局变量 左丢线计数  右丢线计数

uint8 left_currve_flag,right_currve_flag;                                           //  左弯道   右弯道
uint8 currve_flag ;                                                                 //弯道标志
uint8 zhidao_flag ;                                                                 //直道标志

uint8 base_all_white_left,base_all_white_right;                                    //基准行无路判断划分全白行参量

/********************************************************圆环变量模块*******************************************/

uint8 hudao1RLF=0,hudao2RLF=0,hudao1RRF=0,hudao2RRF=0;              //环道标记 左圆环第一标志 左圆环第二标志 右圆环第一标志 右圆环第二标志
uint8 hudao1CLF=0,hudao1CRF=0;                                      // 出左圆环第一标志  出右圆环第一标志   出圆环第二标志
float huandaoLK=0.00,huandaoRK=0.00;                               //入左圆环斜率
float huandaoLB=0.00,huandaoRB=0.00;                               //入左圆环B


/********************************************************十字变量模块*******************************************/

uint8 shziF1=0,shziF2=0,shziFF=0;                                //十字标记
float shiziL_K=0.0,shiziR_K=0.0;                                //十字斜率
float shiziL_B=0.0,shiziR_B=0.0;                                //十字B


/********************************************************蓝牙模块*******************************************/

float var[4];
int16 val_left=0,val_right=0;

/**********************************************电磁模块变量***********************************************/

uint16  AD_data[4][10]={0};                             //电压储存数组
uint16  AD_average[4]={0};                              //电压平均值
uint8  AD_GYH[4]={0};                                   //电压归一化后的数组
uint8  AD_GYH_C[4][10]={0};                             //电压归一化后线路储存数组
uint16  AD_GYH_A[4][10]={0};                            //电压平均值后线路储存数组


/*uint8 AD_F_MAX = 151 , AD_F_MIN = 6  ;
uint8 AD_B_MAX = 144 , AD_B_MIN = 14 ;
uint8 AD_L_MAX = 95  , AD_L_MIN = 9  ;
uint8 AD_R_MAX = 50  , AD_R_MIN = 3  ;                  */

uint8 AD_F_MAX = 100 , AD_F_MIN = 10  ;
uint8 AD_B_MAX = 100 , AD_B_MIN = 10 ;
uint8 AD_L_MAX = 100  , AD_L_MIN = 2  ;
uint8 AD_R_MAX = 100  , AD_R_MIN = 7  ;

uint8 DIYH_flag_R = 0  ;                               //电磁入右圆环标志
uint8 DIYH_flag_L = 0  ;                               //电磁入左圆环标志
uint8 DIYH_flag_CR = 0 ;                               //电磁出右圆环标志
uint8 DIYH_flag_CL = 0 ;                               //电磁出左圆环标志


uint8 VW_L=0 , VW_R=0;                                 // VW_L入左弯标志 VW_R入右弯标志

float chuan[5]={0.0};

uint8 Daolu_WaodaoV = 0 ;
/***********************************************摄像头/电磁切换模块*******************************************/

uint8 QieHuan_Flag = 0 ;
/***********************************************坡道模块*******************************************/


/*******************************超声波避障模块*********************************/

#define TRIG    PTE4                                                            //超声波接收引脚
#define ECHG    PTE5                                                            //超声波发送引脚
#define FLAGWAIT    0xAFFF                                                      //最大距离1.19m
#define JULI    65                                                              //超声波检测避障距离 50cm

uint8 PIT2_Flag = 0 ;
uint8 PITE_jishu = 0 ;
uint32 timevar=0;

uint32 BZ_timevar = 0 ;                                                         //避障出去计时


uint8 BZ_flag=0;                                                                //超声波避障标志 BZ_flag
uint8 NBZ=0;
uint8 BZ_flag1=0,BZ=0;;
uint8 BZ_dianci = 0 ;

void PIT2_IQRHandler(void);                                                    //PIT2定时中断函数      发射超声波
void PORTE_IRQHandler(void);                                                   //超声波测距中断服务函数  PORTE端口
void bizhang_fx();
void bizhang_gc();
void bizhang_V(void);                                                          //电磁避障过程函数

/**************************************TOF_1020**************************************/
unsigned char rxbuf[16], waitflag;
char ch_2019[16];
int rxflag = 0 ,rxcnt = 0 ;
int length_val[3] = {0} ;
int i_2019;

unsigned char ucRxBuffer[12];
unsigned char  ucRxCnt = 0;
short Angle_X;
short Angle_Y;
short Angle_Z;
int x,y,z;
void uart0_test_handler(void);
void uart1_test_handler(void);
void TOF_1020();

/********************************************************函数声明模块*******************************************/

void PORTA_IRQHandler();                                                           //中断
void DMA0_IRQHandler();
void PIT0_IRQHandler2();

void lcd_camera_init();                                                            //LCD初始化
void lcd_display();                                                                //LCD显示

void QieHuan();

void duoji();                                                                     //摄像头舵机
void duoji_V();                                                                   //电磁舵机

void dianji_baodi();                                                              //dianji'0'
void dianji_Right_baodi();
void dianji_Left_baodi();

void dianji_disu();                                                               //disu_PID
void dianji_Left_disu();
void dianji_Right_disu();
void disu_duoji();

void dianjihuang();
void dianji_7_2();
void dianji_Left_7_2();
void dianji_Right_7_2();


void tiqianCL();                                                                  //提前分析赛道
void huamid();                                                                    //画中线
void midline();                                                                   //找中线
void daolu_fenxi();                                                               //弯直道判断

void shizi();                                                                     //十字判断
void shizi1();                                                                    //十字判断
void shizi2();                                                                    //十字判断



void GetVoltage();                                                                //获取电压

int GYH(int AD_max,int AD_min,int value);                                        //归一化

float m_sqrt(unsigned int x);                                                   //开根号函数
float Qulv(int x1,int x2,int x3,int y1,int y2,int y3);                         //曲率
int Atan (float Bz_K);                                                           //反正切弧度转化为角度

/**********************液晶显示**************************************************/
void lcd_display()
{
  Site_t site     = {0, 0};                             //显示图像左上角位置
  Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小
  Size_t size;                                        //显示区域图像大小
  size.H = 60;
  size.W = 80;
  
  LCD_Img_Binary_Z(site, size, imgbuff, imgsize);
  
  
  
  site.x =60;
  site.y =110;
  LCD_num(site,val_left,FCOLOUR,BCOLOUR);
  site.y =95;
  LCD_num(site,val_right,FCOLOUR,BCOLOUR);
  site.y =70;
  LCD_num_C(site,length_val[0],FCOLOUR,BCOLOUR);
  
  Site_t site1  = {0, 0};
  site1.x =0;
  site1.y =60;
  
  x= abs(Angle_X/32768.0*180);
  y=abs(Angle_Y/32768.0*180);
  z=abs(Angle_Z/32768.0*180);
  LCD_num_C(site1,x,FCOLOUR,BCOLOUR);
  site1.y=80;
  LCD_num_C(site1,y,FCOLOUR,BCOLOUR);
  site1.y=100;
  LCD_num_C(site1,z,FCOLOUR,BCOLOUR);

}

/***************************salted_fish电机驱动**********************************/
void dianjihuang()
{
  //right
      FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=2200;
      //left
      FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=2200;
}
/*****************************主函数*********************************************/

void  main(void)
{
  
  ftm_quad_init(FTM2);                                     //FTM1 正交解码初始化（所用的管脚可查 port_cfg.h ）
  ftm_quad_init(FTM1);
  
  ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,0);          //初始化 电机 PWM             第一个电机驱动
  ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,0);          //初始化 电机 PWM            第二个电机驱动
  ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,0);          //初始化 电机 PWM            第三个电机驱动
  ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,0);          //初始化 电机 PWM             第四个电机驱动
  
  lcd_camera_init();                                     //LCD初始化
  
  ftm_pwm_init(FTM3, FTM_CH6,50,dj_mid);                //初值定义:居中10000-1550    T1  PTA12
  
  /*************定时器0中断，编码器，舵机，电机******************/
  pit_init_ms(PIT0,10);                                     //初始化PIT0，定时时间为： 10ms
  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler2);       //设置PIT0的中断服务函数为 PIT0_IRQHandler
  enable_irq (PIT0_IRQn);                                  //使能PIT0中断
  
  
  /*************初始化串口UART0，TOF或者蓝牙******************/
  uart_init(UART0, 9600);                       
  set_vector_handler(UART0_RX_TX_VECTORn,uart0_test_handler);
  uart_rx_irq_en(UART0);
  
  /*************初始化串口UART1，陀螺仪******************/
  uart_init(UART1, 9600);                      
  set_vector_handler(UART1_RX_TX_VECTORn,uart1_test_handler);
  uart_rx_irq_en(UART1);
  
  
  NVIC_SetPriorityGrouping(4);            //设置优先级分组,4bit 抢占优先级,没有亚优先级
  
  NVIC_SetPriority(PORTA_IRQn,0);         //配置优先级
  //   NVIC_SetPriority(PORTE_IRQn,1);       //配置优先级
  NVIC_SetPriority(DMA0_IRQn,2);          //配置优先级
  NVIC_SetPriority(PIT0_IRQn,3);          //配置优先级
  //   NVIC_SetPriority(PIT2_IRQn,4);        //配置优先级
  NVIC_SetPriority(UART0_RX_TX_IRQn,1);
  
  DisableInterrupts;
  
  EnableInterrupts;
  rxflag=0;
  rxcnt=0;                                               
  
  SCCB_WriteByte ( OV7725_CNST, 40 );      //调阈值
  
  
  while(1)
  {
    midline();
    huamid();
    duoji();
    TOF_1020();
    
    //      if( BZ == 1 )
    //      {
    //        disable_irq(UART0_RX_TX_IRQn);
    //        disable_irq(PORTA_IRQn);
    //        disable_irq(DMA0_IRQn);
    //        //  PITE_jishu = 0 ;
    //        bizhang_gc();
    //        enable_irq (UART0_RX_TX_IRQn) ;
    //        enable_irq(PORTA_IRQn);
    //        enable_irq(DMA0_IRQn);
    //      }
    //      
    //      
    
          
             var[0]= (float)Angle_X/32768.0*180;
        var[1]=(float)Angle_Y/32768.0*180;
        var[2]=(float)Angle_Z/32768.0*180;
          var[3]=0;
          vcan_sendware((float *)var, sizeof(var));             
  }
}

/********************************编码器，电机，清中断************************************************/
void PIT0_IRQHandler2()
{
  static uint8 js = 0 ;
  js ++ ;
  val_right = -ftm_quad_get(FTM2);          //获取FTM 正交解码 的脉冲数(负数表示反方向)
  val_left  = ftm_quad_get(FTM1);
  
  
  //GetVoltage();
  //Dianci_YuHU();
  //duoji_V();
  
  //dianji_7_2();
  dianjihuang();
  /*  var[0] =DIYH_flag_R;
  var[1] =DIYH_flag_L;
  var[2] = 0;
  var[1] = 0 ;
  vcan_sendware((int16_t *)var, sizeof(var));         */
  ftm_quad_clean(FTM2);
  ftm_quad_clean(FTM1);
  
  PIT_Flag_Clear(PIT0);       //清中断标志位
  
}


/********************************************UART0接收************************************/
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
/********************************************UART1接收************************************/
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
      case 0x53:
        Angle_X = ((unsigned short)ucRxBuffer[3]<<8)|ucRxBuffer[2];
        Angle_Y = ((unsigned short)ucRxBuffer[5]<<8)|ucRxBuffer[4];
        Angle_Z = ((unsigned short)ucRxBuffer[7]<<8)|ucRxBuffer[6];
      default:
        break;
        
      }
      ucRxCnt=0;
      
    }
  }
}
/***************************************TOF测距函数************************************/
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
 
 
  
  
//  if( length_val[0] <= 850 && length_val[0] >= 300 )
//  {
//    if( length_val[2] > length_val[1] && length_val[1] > length_val[0] )//&& length_val[2] <= 750 )
//    {
//      if( PIT2_Flag == 1 )
//      {
//        //  PIT2_Flag = 0 ;                             避障之前清零，不再这里清零
//        BZ = 1 ;
//      }
//      else
//      {
//        PIT2_Flag = 0 ;
//        BZ = 0 ;
//      }
//    }
//  }
//  else
//  {
//    PIT2_Flag = 0 ;
//  }
}


/*************************LCD寻找中线判断路况************************************/
void  midline()
{
  uint8 i=0,j_l=0,j_r=0 , i_max = 50 , i_min = 1 ;
  int mid_back=40;
  uint8 flag_L=0,flag_R=0;                   //0 是未找到  1 是找到
  // int Panduan_R_line[30]={0},Panduan_L_line[30]={0};  //储存控制范围行的左右丢线的数组
  // Wan_line_Rflag=0,Wan_line_Lflag=0,Zhi_line_Rflag=0,Zhi_line_Lflag=0,Zhi_line_flag=0;
  
  mid[50]=40;         //基准线
  mid[51]=40;
  //  uint8 buchang=5;
  
  
  camera_get_img();
  img_extract((uint8 *)img,(uint8 *)imgbuff,CAMERA_SIZE);       //解压为二维数组
  // vcan_sendware((int16_t *)var, sizeof(var)); 
 // vcan_sendimg((uint8 *)imgbuff, sizeof(imgbuff));
  tiqianCL();
  QieHuan();
  //Ch_Sh_Bo();
  //podao();
  daolu_fenxi();
  
  //  shizi1();
  

  //  huandao1L();
  for(i=i_max;i>=i_min;i--)
  {
    
    for(j_l=mid_back;j_l>1;j_l--)                        //左边赛道
    {
      if(img[i][j_l]==0xff && img[i][j_l-1]==0x00)
      {
        mid_L_line_last=mid_L_line[i];      //把这一行找的坐标记录下来，当下一行坐标找不到时引用上一次的坐标
        mid_L_line[i]=j_l-1;
        flag_L=1;
        panduanL[i]=0;
        break;
      }
      else
      {
        panduanL[i]=1;
        flag_L=0;
      }
    }
    for(j_r=mid_back;j_r<78;j_r++)                       //右边赛道
    {
      if(img[i][j_r]==0xff && img[i][j_r+1]==0x00)
      {
        mid_R_line_last=mid_R_line[i];       //把这一行找的坐标记录下来，当下一行坐标找不到时引用上一次的坐标
        mid_R_line[i]=j_r+1;
        flag_R=1;
        panduanR[i]=0;
        break;
      }
      else
      {
        panduanR[i]=1;
        flag_R=0;
      }
    }
    
    //左右都有
    if(flag_L==1 && flag_R==1)
    {
      mid[i]=(mid_R_line[i]+mid_L_line[i])/2;
      mid1[i]=(mid_R_line[i]-mid_L_line[i])/2;
      // mid[i]=mid_R_line[i]-xz[i];             //调矫正数组用
      // mid[i]=mid_L_line[i]+xz[i];             //调矫正数组用
      if(shziF1 == 1)   //都有就不再补线了
      {
        if(i > 20)
        {
          mid_L_line[i]=(int)(i*shiziL_K+shiziL_B);
          mid_R_line[i]=(int)(i*shiziR_K+shiziR_B);
          mid[i]=(mid_R_line[i]+mid_L_line[i])/2;
        }
      }
    }
    //左丢线
    if(flag_L==0 && flag_R==1)
    {
      mid[i]=mid_R_line[i]-xz1[i];
      //  mid[i]=mid_R_line[i]-xz1[i]-buchang;              //半路宽补线
      if(i<40) left_loss_line++;                       //左丢线计数
      
      if(mid[i]<=4) mid_back=4;       //下一帧限位
      else if(mid[i]>=76) mid_back=76;
      else mid_back=mid[i];
      
      
      if( hudao2RLF==1 )
      {
        if(i >= duo_ji_XX && i <= duo_ji_SX && img[20][40] == 0xff ) //&& img[10][40] == 0xff)                              //要不要修改控制补线行数
        {
          mid_L_line[i]=(int)(huandaoLK*i+huandaoLB) ;
          mid[i]=  mid_L_line[i] + (uint8)( xz1[i] * 0.6 ) ;      //xz1[i] -
          if(  mid[i]> 40 && i <= duo_ji_SX )  mid[i]= mid_L_line[i] + (uint8)( xz1[i] * 0.5 );
          if(  mid[i]> 40)  mid[i]=40;
          if(  mid[i]< 0 )  mid[i]=1;
        }
      }
      /*   else if( hudao1CRF == 1 )
      {
      if( i >= duo_ji_XX && i <= duo_ji_SX && img[20][40] == 0xff)
      {
      mid_R_line[i]=(int)(huandaoRK*i+huandaoRB);
      mid[i]=mid_R_line[i] - (uint8)( xz1[i] * 0.3 );
      
      if(  mid[i] <= 40 && i <= 35 )  mid[i] = mid_R_line[i] - (uint8)( xz1[i] * 0.2 );
      if( mid[i] < 40 ) mid[i]= 40 ;
      if(  mid[i] > 79 )  mid[i]=78;
    }
    }                                                                                              */
      else if( hudao1CLF == 1)
      {
        if(i >= duo_ji_XX && i <= duo_ji_SX && img[20][40] == 0xff )                  //要不要修改控制补线行数
        {
          mid_L_line[i]=(int)(huandaoLK*i+huandaoLB);
          mid[i]=  mid_L_line[i] + (uint8)( xz1[i] * 0.5 );      //xz1[i] -
          if(  mid[i]> 40 &&i <= duo_ji_SX )  mid[i]= mid_L_line[i] + (uint8)( xz1[i] * 0.4 );
          if(  mid[i]> 40)  mid[i]=40;
          if(  mid[i]< 0 )  mid[i]=1;
        }
      }
      
      if(shziF1==1)
      {
        mid_L_line[i]=(int)(i*shiziL_K+shiziL_B);
        mid_R_line[i]=(int)(i*shiziR_K+shiziR_B);
        mid[i]=(mid_R_line[i]+mid_L_line[i])/2;
      }
    }
    //右丢线
    if(flag_L==1 && flag_R==0)
    {
      mid[i]=mid_L_line[i]+xz1[i];
      // mid[i]=mid_L_line[i]+xz1[i]+buchang;              //半路宽补线
      if(i<40) right_loss_line++;                     //右丢线计数
      
      if(mid[i]<=4) mid_back=4;      //下一帧限位
      else if(mid[i]>=76) mid_back=76;
      else mid_back=mid[i];
      
      if( hudao2RRF == 1 )
      {
        if( i >= duo_ji_XX && i <= duo_ji_SX && img[20][40] == 0xff )
        {
          mid_R_line[i]=(int)(huandaoRK*i+huandaoRB);
          mid[i]=mid_R_line[i] - (uint8)( xz1[i] * 0.6 );
          
          if( mid[i] <= 40 && i <= duo_ji_SX )  mid[i] = mid_R_line[i] - (uint8)( xz1[i] * 0.5 );
          if( mid[i] < 40 ) mid[i]= 40 ;
          if( mid[i] > 79 ) mid[i]=78;
        }
      }
      /*    else if( hudao1CLF == 1)
      {
      if(i >= duo_ji_XX && i <= duo_ji_SX && img[20][40] == 0xff) //&& img[10][40] == 0xff)                              //要不要修改控制补线行数
      {
      mid_L_line[i]=(int)(huandaoLK*i+huandaoLB);
      mid[i]=  mid_L_line[i] + (uint8)( xz1[i] * 0.7 );      //xz1[i] -
      if(  mid[i]> 40)  mid[i]=40;
      if(  mid[i]<= 0 )  mid[i]=1;
    }
    }                                                                                        */
      else if( hudao1CRF == 1 )
      {
        if( i >= duo_ji_XX && i <= duo_ji_SX  && img[20][40] == 0xff)                 //&& mid_L_line[i] > mid_L_line[i - 2]
        {
          mid_R_line[i]=(int)(huandaoRK*i+huandaoRB);
          mid[i]=mid_R_line[i] - (uint8)( xz1[i] * 0.5 );
          if( mid[i] <= 40 && i <= duo_ji_SX )  mid[i] = mid_R_line[i] - (uint8)( xz1[i] * 0.4 );
          if( mid[i] < 40 ) mid[i]= 40 ;
          if( mid[i] > 80 ) mid[i]=80;
        }
      }
      
      if(shziF1==1)
      {
        mid_L_line[i]=(int)(i*shiziL_K+shiziL_B);
        mid_R_line[i]=(int)(i*shiziR_K+shiziR_B);
        mid[i]=(mid_R_line[i]+mid_L_line[i])/2;
      }
    }
    //左右同时丢线
    if(flag_L==0 && flag_R==0)                      //未详细分解
    {
      
      //  mid_L_line[i]=mid_L_line_last;
      //  mid_R_line[i]=mid_R_line_last;
      
      
      
      if(shziF1==1)
      {
        mid_L_line[i]=(int)(i*shiziL_K+shiziL_B);
        mid_R_line[i]=(int)(i*shiziR_K+shiziR_B);
        mid[i]=(mid_R_line[i]+mid_L_line[i])/2;
      }
      else if( hudao2RLF == 1 )
      {
        if( i >= duo_ji_XX && i <= duo_ji_SX )                              //要不要修改控制补线行数
        {
          mid_L_line[i]=(int)(huandaoLK*i+huandaoLB);
          mid[i]= mid_L_line[i] + (uint8)( xz1[i] * 0.6 );       //xz1[i] +
          if(  mid[i]>= 40 && i <= duo_ji_SX )  mid[i]= mid_L_line[i] + (uint8)( xz1[i] * 0.5 );
          if(  mid[i]> 40 )  mid[i]=40;
          if(  mid[i]< 0 )  mid[i]=0;
        }
      }
      else if( hudao2RRF == 1 )
      {
        if( i >= duo_ji_XX && i <= duo_ji_SX )
        {
          mid_R_line[i]=(int)(huandaoRK*i+huandaoRB);
          mid[i]= mid_R_line[i] - (uint8)( xz1[i] * 0.6 );
          if(  mid[i] <= 40 && i <= duo_ji_SX )  mid[i] = mid_R_line[i] - (uint8)( xz1[i] * 0.5 );
          if( mid[i] < 40 ) mid[i]= 40 ;
          if(  mid[i] > 80 ) mid[i]=80 ;
        }
      }
      else if( hudao1CRF == 1 )
      {
        if( i >= duo_ji_XX && i <= duo_ji_SX  && img[20][40] == 0xff )
        {
          mid_R_line[i]=(int)(huandaoRK*i+huandaoRB);
          mid[i]=mid_R_line[i] - (uint8)( xz1[i] * 0.5 ) ;
          if( mid[i] <= 40 && i <= duo_ji_SX )  mid[i] = mid_R_line[i] - (uint8)( xz1[i] * 0.4 );
          if( mid[i] < 40 )  mid[i] = 40 ;
          if( mid[i] > 80 )  mid[i] = 80 ;
        }
      }
      else if( hudao1CLF == 1)
      {
        if(i >= duo_ji_XX && i <= duo_ji_SX && img[20][40] == 0xff )                  //要不要修改控制补线行数
        {
          mid_L_line[i]=(int)(huandaoLK*i+huandaoLB);
          mid[i]=  mid_L_line[i] + (uint8)( xz1[i] * 0.5 ) ;      //xz1[i] -
          if(  mid[i]>= 40 && i <= duo_ji_SX )  mid[i]= mid_L_line[i] + (uint8)( xz1[i] * 0.4 );
          if(  mid[i]> 40)  mid[i]=40;
          if(  mid[i]< 0 )  mid[i]=0;
        }
      }
      else
      {
        mid[i] = 40;
      }
    }
    
    
    if(mid[i] <= 0)     mid_back=1;
    else if (mid[i] >= 79)    mid_back=78;
    else if(mid[i] > 0 && mid[i] < 79)   mid_back=mid[i];
    else  mid_back=40;
  }
  lcd_display();      //lcd显示
  
}

/*****************************************十字**************************************/
void shizi()
{
  uint8 shziLD=0,shziLU=0,shziRD=0,shziRU=0;
  uint8 midpoint=40;
  uint8 ii=45,jj=45,Ni=0;
  uint8 shziNF=0,shziNS=0,shziS=0;
  for(ii=45;ii>25;ii--)             //行进行
  {
    for(jj=20;jj<=60;jj++)         //列进行检测
    {
      if(img[ii][jj]==0xff)        //判断是否为白色
      {
        shziNF=shziNF+1;
        if(shziNF>=30)         //连续shziNF个白色
        {
          shziNS=shziNS+1;
          if(shziNS>=5)      //在连续shziNS行内出现全白，提前跳出循环
          {
            break;
          }
        }
      }
      else                  //在一行内出现黑色的点跳出此行，并进行shziNF清零
      {
        shziNF=0;
        break;
      }
    }
    if(shziNS>=5)
    {
      Ni=ii;
      break;
    }
  }
  
  if(shziNS>=5)            //进行下面的判断
  {
    for(ii=60;ii>30;ii--)        //左下区域进行扫描
    {
      for(jj=midpoint;jj>1;jj--)
      {                              //先左后右                            //img[ii-3][jj]==0xff && img[ii+1][jj+3]==0xff
        if(img[ii][jj+1]==0xff && img[ii][jj]==0x00 && mid_L_line[ii]>mid_L_line[ii-2]  )
        {
          shziLD=1;                    //左下标记
          break;                      //满足标记提前跳出循环
        }
        else
        {
          shziLD=0;
          //  break;
        }
      }
      if(shziLD==1)
      {
        break;
      }
      else
      {
        shziLD=0;
      }
    }
    if(shziLD==1)
    {
      for(ii=60;ii>30;ii--)        //右下区域进行扫描
      {
        for(jj=midpoint;jj<78;jj++)
        {                                             //img[ii-3][jj]==0xff&& img[ii+1][jj-3]==0xff
          if(img[ii][jj-1]==0xff && img[ii][jj]==0x00 && mid_R_line[ii]<mid_R_line[ii-2] )//img[ii+2][jj]=0xff &&
          {
            shziRD=1;                    //右下标记
            break;
          }
          else
          {
            shziRD=0;
            //  break;
          }
        }
        if(shziRD==1)
        {
          break;
        }
        else
        {
          shziRD=0;
        }
      }
    }
    if(shziRD==1)
    {
      for(ii=Ni;ii>9;ii--)        //右上区域进行扫描
      {
        for(jj=midpoint;jj<78;jj++)
        {                                             // img[ii+3][jj+3]==0xff&& img[ii-1][jj-3]==0xff
          if(img[ii][jj-1]==0xff && img[ii][jj]==0x00 && (img[ii-2][jj-5]==0xff || img[ii+2][jj+5]==0xff))//img[ii+2][jj]=0xff &&
          {
            shziRU=1;                    //右下标记
            break;
          }
          else
          {
            shziRU=0;
            //  break;
          }
        }
        if(shziRU==1)
        {
          break;
        }
        else
        {
          shziRU=0;
        }
      }
    }
    if(shziRU==1)
    {
      for(ii=Ni;ii>9;ii--)        //左上区域进行扫描
      {
        for(jj=midpoint;jj>3;jj--)
        {                                                                      //&& img[ii-3][jj+1]==0xff
          if(img[ii][jj+1]==0xff && img[ii][jj]==0x00 && (img[ii+2][jj-5]==0xff || img[ii-2][jj+5]==0xff) )//img[ii+2][jj]=0xff &&
          {
            shziLU=1;                    //右下标记
            break;
          }
          else
          {
            shziLU=0;
            break;
          }
        }
        if(shziLU==1)
        {
          break;
        }
        else
        {
          shziLU=0;
        }
      }
    }
    if(shziLD==1 && shziLU==1 && shziRD==1 && shziRU==1)
    {
      shziFF=1;
    }
    else
    {
      shziFF=0;
    }
  }
  //第二种
  for(ii=45;ii>15;ii--)
  {
    if(panduanL[ii]==1 && panduanR[ii]==1)
    {
      shziS=shziS+1;
    }
  }
  if(shziS>5)
  {
    shziF1=1;          //改标记
  }
  else
  {
    shziF1=0;
  }
}

/*****************************************十字1**************************************/
void shizi1()
{
  uint8 shziS=0,shziRF=0,shziLF=0,ii=0;
  uint8 jishu=0;
  int16 Chucun_L=0,Chucun_R=0;
  shiziL_K=0,shiziR_K=0,shiziL_B=0,shiziR_B=0;
  
  uint8  White_Line=3;                     //全白线行数
  
  for(ii=30;ii>=20;ii--)                 //全白被控行
  {
    if(panduanL[ii]==1 && panduanR[ii]==1 && img[ii][1]==0xff && img[ii][78]==0xff)
    {
      shziS=shziS+1;
    }
    if(shziS >= White_Line)
    {
      break;
    }
  }
  if(shziS>=White_Line)
  {
    for(ii=20;ii>=5;ii--)
    {
      if(mid_L_line[ii]>10 && mid_R_line[ii]<70 )
      {
        if(( mid_L_line[ii] - mid_L_line[ii + 1] <= 1 ) && ( mid_L_line[ii - 1] - mid_L_line[ii] <= 1 ) &&
           ( mid_R_line[ii + 1] - mid_R_line[ii] ) <= 1  && ( mid_R_line[ii] - mid_R_line[ii-1] <= 1 ) )
        {
          uint8 q=ii;
          for(ii=q+1;ii>=q-1;ii--)    //斜率被控行
          {
            jishu++;
            Chucun_L=Chucun_L+mid_L_line[ii];
            Chucun_R=Chucun_R+mid_R_line[ii];
          }
          break;
        }
      }
    }
    if(jishu >= 3 )        //‘5’随     ii>=q-4 控制
    {
      /*  if( (mid_L_line[ii+1]+mid_L_line[ii+jishu])/2 <= Chucun_L/jishu+2 &&
      (mid_L_line[ii+1]+mid_L_line[ii+jishu])/2 >= Chucun_L/jishu-2)       */
      {
        shiziL_K=(mid_L_line[ii+jishu] - mid_L_line[ii+1])/2.0;  //斜率计算
        shiziL_B=mid_L_line[ii+1]-shiziL_K*ii;  //B的计算
        shziLF=1;
      }
      /*  if( (mid_R_line[ii+1]+mid_R_line[ii+jishu])/2 <= Chucun_R/jishu+2 &&
      (mid_R_line[ii+1]+mid_R_line[ii+jishu])/2 >= Chucun_R/jishu-2)       */
      {
        shiziR_K=(mid_R_line[ii+jishu] - mid_R_line[ii+1])/2.0;   //斜率计算
        shiziR_B=mid_R_line[ii+1]-shiziR_K*ii;  //B的计算
        shziRF=1;
      }
    }
    
    if(shziLF==1 && shziRF==1)
    {
      shziF1=1;
    }
    else
    {
      shziF1=0;
    }
  }
  else
  {
    shziF1=0;
  }
  
  
}

/*****************************************十字2********************************************/
void  shizi2()
{
  uint8 shziS=0,shziRF=0,shziLF=0,ii=0;
  uint8 jishu=0;
  int Chucun_L=0,Chucun_R=0;
  shiziL_K=0,shiziR_K=0,shiziL_B=0,shiziR_B=0;
  
  for(ii=50;ii>=40;ii--)         //全白被控行
  {
    if(panduanL[ii]==1 && panduanR[ii]==1 && img[ii][1]==0xff && img[ii][78]==0xff)
    {
      shziS=shziS+1;
    }
    if(shziS>4)
    {
      break;
    }
  }
  
  if(shziS>=4)
  {
    for(ii=30;ii>=15;ii--)
    {
      if(mid_L_line[ii]>10 && mid_R_line[ii]<70 )
      {
        if(mid_L_line[ii+1] < mid_L_line[ii] && mid_L_line[ii] < mid_L_line[ii-1] &&
           mid_R_line[ii+1] > mid_R_line[ii] && mid_R_line[ii] > mid_R_line[ii-1])
        {
          uint8 q=ii;
          for(;ii>=q-3;ii--)    //斜率被控行
          {
            jishu++;
            Chucun_L=Chucun_L+mid_L_line[ii];
            Chucun_R=Chucun_R+mid_R_line[ii];
          }
          break;
        }
      }
    }
    
    if(jishu >= 4)        //‘5’随     ii>=q-4 控制
    {
      if( (mid_L_line[ii]+mid_L_line[ii+jishu])/2 <= Chucun_L/jishu+1.5 &&
         (mid_L_line[ii]+mid_L_line[ii+jishu])/2 >= Chucun_L/jishu-1.5)
      {
        shiziL_K=(mid_L_line[ii+jishu] - mid_L_line[ii])/jishu;  //斜率计算
        shiziL_B=mid_L_line[ii]-shiziL_K*ii;  //B的计算
        shziLF=1;
      }
      if( (mid_R_line[ii]+mid_R_line[ii+jishu])/2 <= Chucun_R/jishu+1.5 &&
         (mid_R_line[ii]+mid_R_line[ii+jishu])/2 >= Chucun_R/jishu-1.5)
      {
        shiziR_K=(mid_R_line[ii+jishu] - mid_R_line[ii])/jishu;   //斜率计算
        shiziR_B=mid_R_line[ii]-shiziR_K*ii;  //B的计算
        shziRF=1;
      }
    }
    
    if( shziLF==1 && shziRF==1 )
    {
      shziF2=1;
    }
    else
    {
      shziF2=0;
    }
  }
  else
  {
    shziF2=0;
  }
}

/***************************************切换+识别**********************************/
void QieHuan()
{
  uint8 QIE_i, QIE_i_Max = 38 , QIE_i_Min = 10 , QIE_JiShu=0 , QIE_JiShu2 = 0 ;
  int8  QIE_JiShu1=0 ;
  // uint8 QIE_YuZhi1 = 8 ,QIE_YuZhi2 = 5 ;
  uint8 QIE_ChuCun = 0 ;
  // uint8 QieHuan_Flag1 = 0,QieHuan_Flag2 = 0;
  
  /***************************清标志******************************/    //夜晚 还可以 较稳定
  if( QieHuan_Flag == 1 )
  {
    QIE_JiShu  = 0 ;
    QIE_JiShu1 = 0 ;
    QIE_JiShu2 = 0 ;
    QIE_ChuCun = 0 ;
    
    uint8 QIE_j = 0 ;
    
    for( QIE_i = 10 ; QIE_i <= 40 ; QIE_i ++ )
    {
      if( mid[QIE_i] - mid[QIE_i + 1] >= -3 && mid[QIE_i] - mid[QIE_i + 1] <= 3  &&
         ( ( mid_L_line[QIE_i] <= 40 && mid_L_line[QIE_i] >= straig_Lline[QIE_i] - 3 ) ||
          ( mid_R_line[QIE_i] >= 40 && mid_R_line[QIE_i] <= straig_Rline[QIE_i] + 3 )
            )  )       QIE_JiShu ++ ;
    }
    
    if( QIE_JiShu >= 25 )
    {
      QIE_JiShu = 0 ;
      for( QIE_i = 15 ; QIE_i <= 35 ; QIE_i ++ )             //宽                 //区域控制来控制稳定度与精确度
      {                                                      //长
        //if( mid_L_line[QIE_i] >= straig_Lline[QIE_i] - 3 && mid_L_line[QIE_i] <= straig_Lline[QIE_i] + 3 &&
        //   mid_R_line[QIE_i] >= straig_Rline[QIE_i] - 3 && mid_R_line[QIE_i] <= straig_Rline[QIE_i] + 3 )
        
        //  if( mid_L_line[QIE_i] >= 10 && mid_L_line[QIE_i] <= 70 &&
        //      mid_R_line[QIE_i] >= 10 && mid_R_line[QIE_i] <= 70 )
        //  {
        for( QIE_j = mid_L_line[QIE_i] ; QIE_j <= mid_R_line[QIE_i] ; QIE_j ++ )
        {
          
          if( img[QIE_i][QIE_j] == 0xff )
          {
            QIE_JiShu ++ ;
          }
          else
          {
            QIE_JiShu = 0 ;
          }
          
          if( QIE_JiShu >= straig_Rline[QIE_i] - straig_Lline[QIE_i] - 5 )
          {
            QIE_JiShu = 0 ;
            QIE_JiShu1 ++ ;
            break;
          }
          
          if( QIE_JiShu1 >= 15 )
          {
            QieHuan_Flag = 0 ;
            break ;
          }
          
        }
        //  }
        
        if( QIE_JiShu1 >= 15)
        {
          QieHuan_Flag = 0 ;
          BZ_dianci = 0 ;
          break ;
        }
      }
    }
  }
  
  
  /***********************************************直道检测断路*************************************************/
  if( QieHuan_Flag == 0 )              //直道检测距离为300mm
  {
    QIE_JiShu  = 0 ;
    QIE_JiShu1 = 0 ;
    QIE_JiShu2 = 0 ;
    QIE_ChuCun = 0 ;
    
    for( QIE_i = QIE_i_Max ; QIE_i >= QIE_i_Min ; QIE_i-- )      //从图像下向上找
    {
      if( img[QIE_i][40] == 0x00 )                             //判断前方中间点的颜色来大概确定前方是否有路
      {
        QIE_JiShu ++ ;
        if( QIE_JiShu == 2 )    QIE_ChuCun = QIE_i + 2 ;           //储存
      }
    }
    
    if( QIE_JiShu >= (uint8)( ( QIE_i_Max - QIE_i_Min ) * 0.2 ) )            //如果白点过即前方无路，标志成立 0.2为灵敏度
    {
      for( QIE_i = QIE_ChuCun ; QIE_i <= QIE_i_Max ; QIE_i ++ )
      {
        if( ( mid_R_line[QIE_i] < straig_Rline[QIE_i] + 5 ) && ( mid_L_line[QIE_i] > straig_Lline[QIE_i] - 5 ) &&
           mid_R_line[QIE_i] > 40 && mid_L_line[QIE_i] < 40 )
        {
          QIE_JiShu1 ++ ;
        }
      }
      
      if( mid_L_line[QIE_ChuCun] >= straig_Lline[QIE_i] - 5 && mid_R_line[QIE_ChuCun] <= straig_Rline[QIE_i] + 5 &&
         mid_R_line[QIE_i] > 40 && mid_L_line[QIE_i] < 40 )
        for( QIE_i = mid_L_line[QIE_ChuCun] ; QIE_i <= mid_R_line[QIE_ChuCun] ; QIE_i ++)
        {
          if( img[QIE_ChuCun - 3][QIE_i] == 0x00 && img[QIE_ChuCun + 3][QIE_i] == 0xff )
          {
            QIE_JiShu2 ++ ;
          }
        }
      
    }
    
    if( QIE_JiShu2 >= (uint8)( 0.8 * (mid_R_line[QIE_ChuCun] - mid_L_line[QIE_ChuCun] ) )
       &&  QIE_JiShu1 >= (uint8)( 0.5 * ( QIE_i_Max - QIE_ChuCun ) ) )
    {
      QieHuan_Flag = 1 ;
    }
    
    /*****************************************弯道检测断路***************************************************/
    
    
    /****************************************第二次弯道*********************************************/
    //弯道检测距离最大为500mm 待检测
    QIE_JiShu=0 ; QIE_JiShu2 = 0 ;  QIE_JiShu1 = 0 ;
    
    for( QIE_i = QIE_i_Max - 10  ; QIE_i > QIE_i_Min - 5; QIE_i -- )
    {
      if( mid_L_line[QIE_i] > straig_Lline[QIE_i] && mid_R_line[QIE_i] < straig_Rline[QIE_i] ||
         ( mid_L_line[QIE_i] > straig_Lline[QIE_i] + 10  && mid_R_line[QIE_i] < straig_Rline[QIE_i] - 5 ) ||
           ( mid_L_line[QIE_i] > straig_Lline[QIE_i] - 5  && mid_R_line[QIE_i] < straig_Rline[QIE_i] + 10 ) )  //小于直道宽度  有问题
      {
        if(  mid_R_line[QIE_i] - mid_R_line[QIE_i - 1] <= 5 && mid_R_line[QIE_i] - mid_R_line[QIE_i - 1] > 0 &&   //趋势减小
           mid_L_line[QIE_i - 1] - mid_L_line[QIE_i] <= 5 && mid_L_line[QIE_i - 1] - mid_L_line[QIE_i] > 0 )
        {
          if( mid_R_line[QIE_i] - mid_L_line[QIE_i] >  mid_R_line[QIE_i - 1] - mid_L_line[QIE_i - 1] )  //  宽度减小
          {
            QIE_JiShu2 ++ ;
            if( ( ( mid_R_line[QIE_i] - mid_L_line[QIE_i] <= 10 ) ||( mid_R_line[QIE_i - 1 ] - mid_L_line[QIE_i - 1] <= 10 )
                 || ( mid_R_line[QIE_i + 1] - mid_L_line[QIE_i + 1] <= 10 ) ) && QIE_JiShu2 >= 3 )                                            //接近于一个尖
            {
              QieHuan_Flag = 1 ;
              break ;
            }
          }
        }
      }
    }
    
  }
}

/************************************直道弯道检测****************************************/
void daolu_fenxi()
{
  uint8 daolu_i = 0 , daolu_Max = 40 , daolu_Min = 7 ,daolu_Min1 = 10 ;
  uint8 daolu_Left_lose = 0 , daolu_Right_lose = 0;
  uint8 daolu_Left_line = 0 , daolu_Right_line = 0;
  uint8 daolu_Mid_lose = 0 ,  daolu_Mid_line = 0 , daolu_Mid_Lline = 0 , daolu_Mid_Rline = 0 ;
  uint8 daolu_sum = 0 ;
  
  for( daolu_i = daolu_Min1 ; daolu_i <= daolu_Max  ; daolu_i ++ )         //右弯
  {
    if( panduanL[daolu_i] == 0 && panduanR[daolu_i] == 1 && daolu_i < daolu_Max - 5 )
    {
      daolu_Right_lose ++ ;
    }
    else if ( panduanL[daolu_i] == 0 && panduanR[daolu_i] == 1 )         daolu_Right_lose ++ ;
    else if ( panduanR[daolu_i] == 1 && daolu_i >= daolu_Max - 5 )       daolu_Right_lose ++ ;
    else  daolu_sum ++ ;
    
    if( daolu_sum >= 6 )     break;
  }
  
  if( daolu_Right_lose >= 25 )
  {
    for( daolu_i = daolu_Min1 ; daolu_i <= daolu_Max  ; daolu_i ++ )
    {
      if( mid_L_line[daolu_i] > ( straig_Lline[daolu_i] + 1 ) && mid_L_line[daolu_i] <= 70 && mid[daolu_i] > 41 )     //左边线弯曲 中线偏右
      {
        daolu_Right_line ++ ;
      }
      else if( mid_L_line[daolu_i] <= 3 && mid[daolu_i] >= 41 && daolu_i < daolu_Max - 5 )     daolu_Right_line ++ ;
    }
  }
  
  if( daolu_Right_line >= 25 )      right_currve_flag = 1 ;
  else                              right_currve_flag = 0 ;
  
  daolu_sum = 0 ;                                                        //清零
  for( daolu_i = daolu_Min1 ; daolu_i <= daolu_Max  ; daolu_i ++ )      //左弯
  {
    if( panduanR[daolu_i] == 0 && panduanL[daolu_i] == 1 && daolu_i < daolu_Max - 5 )
    {
      daolu_Left_lose ++ ;
    }
    else if( panduanR[daolu_i] == 0 && panduanL[daolu_i] == 1 )            daolu_Left_lose ++ ;
    else if( panduanL[daolu_i] == 1 && daolu_i >= daolu_Max - 5   )        daolu_Left_lose ++ ;
    else  daolu_sum ++ ;
    
    if( daolu_sum >= 6 )     break;
  }
  
  if( daolu_Left_lose >= 25)
  {
    for( daolu_i = daolu_Min1 ; daolu_i <= daolu_Max  ; daolu_i ++ )
    {
      if( mid_R_line[daolu_i] < ( straig_Rline[daolu_i] - 1 ) && mid_R_line[daolu_i] >= 10 && mid[daolu_i] < 39 )     //左边线弯曲 中线偏右
      {
        daolu_Left_line ++ ;
      }
      else if( mid_R_line[daolu_i] >= 77 && mid[daolu_i] <= 39 && daolu_i < daolu_Max - 5)    daolu_Left_line ++ ;
    }
  }
  
  if( daolu_Left_line >= 25 )       left_currve_flag = 1 ;
  else                              left_currve_flag = 0 ;
  
  daolu_sum = 0 ;                                                           //清零
  for( daolu_i = daolu_Min1 ; daolu_i <= daolu_Max -  5  ; daolu_i ++ )         //直道
  {
    if( panduanL[daolu_i] == 1 && panduanR[daolu_i] == 1 )
    {
      daolu_Mid_lose ++ ;
    }
    else   daolu_sum ++ ;
    
    if( daolu_sum >= 6 )     break;
  }
  
  if( daolu_Mid_lose <= 6 )
  {
    for( daolu_i = daolu_Min1 ; daolu_i <= daolu_Max  ; daolu_i ++ )
    {
      if( mid_L_line[daolu_i] > ( straig_Lline[daolu_i] - 8 ) && mid_L_line[daolu_i] < ( straig_Lline[daolu_i] + 8 ) &&
         mid_R_line[daolu_i] > ( straig_Rline[daolu_i] - 8 ) && mid_R_line[daolu_i] < ( straig_Rline[daolu_i] + 8 ) )     //左边线弯曲 中线偏右
      {
        daolu_Mid_line ++ ;
      }
    }
  }
  
  if( daolu_Mid_line >= 22 )     zhidao_flag = 1 ;
  else                           zhidao_flag = 0 ;
  
  
  
  
}

/************************************提前分析赛道****************************************/
void tiqianCL()
{
  uint8 i = 0 ,i_max = 50 ,i_min = 1 ;
  uint8 j_l=0,j_r=0;
  uint8 mid_back=40;
  for( i = i_max ; i >= i_min ; i-- )                         //范围
  {
    for(j_l=mid_back;j_l>1;j_l--)                        //左边赛道
    {
      if(img[i][j_l]==0xff && img[i][j_l-1]==0x00)
      {
        mid_L_line[i]=j_l-1;
        panduanL[i]=0;
        break;
      }
      else
      {
        panduanL[i]=1;
        mid_L_line[i]=0;
      }
    }
    for(j_r=mid_back;j_r<78;j_r++)                       //右边赛道
    {
      if(img[i][j_r]==0xff && img[i][j_r+1]==0x00)
      {
        mid_R_line[i]=j_r+1;
        panduanR[i]=0;
        break;
      }
      else
      {
        panduanR[i]=1;
        mid_R_line[i]=80;
      }
    }
    
    mid_back=(mid_L_line[i]+mid_R_line[i])/2;
    
    mid[i]=(mid_L_line[i]+mid_R_line[i])/2;
    
  }
  
}

/*************************************舵机;由平均值计算出占空比**************************************/
void duoji()
{
  uint8 s = 0 ;
  float x = 0.0 ;
  
  
  /*  for( s = duo_ji_SX - 3 ; s > duo_ji_XX ; s-- )                   //限幅滤波
  {
  if( mid[s] - mid[s + 1] >= 10 || mid[s] - mid[s + 1] <= -10 )
  {
  mid[s] = mid[s + 1] ;
}
}                                                                                             */
  
  
  if( mid[30] != 40 )
  {
    duoji_K1 = ( 30.0 - 40.0 ) / ( mid[30] - 40.0 );
    duoji_K1 = 1.0 / duoji_K1 * 10  ;
  }
  else    duoji_K1 = 0.0 ;
  
  if( mid[20] != mid[30] )
  {
    duoji_K2 = ( 20.0 - 30.0 ) / ( mid[20] - mid[30] ) ;
    duoji_K2 = 1.0 / duoji_K2 * 10 ;
  }
  else    duoji_K2 = 0.0 ;
  
  
  for( s = duo_ji_SX ; s >= duo_ji_XX ; s-- )                   //计算均值
  {
    sum+=mid[s];
    x++;
  }
  
  ave=sum/x ;
  sum=0 ;
  duoji_error=ave-39 ; //角速度                               //计算偏差
  
  
  disu_duoji();
  
  KP1=0.8 ;
  // KD1=5.6;
  
  
  
  
  duoji_error_hunhe =  duoji_K1 + duoji_K2 ;
  
  duoji_duty_K=(uint16)(KP1*duoji_error_hunhe) ;   //dj;
  
  
  duoji_duty=(uint16)(dj_mid+KP*duoji_error+KD*(duoji_error-duoji_last_error) ) ;   //dj;
  
  // duoji_duty = duoji_duty + duoji_duty_K ;
  duoji_last_error=duoji_error ;
  
  if(duoji_duty<dj_left_max)
    duoji_duty=dj_left_max;
  if(duoji_duty>dj_right_max)
    duoji_duty=dj_right_max;
  
  //if( QieHuan_Flag == 0 )  //不切换同时  电磁圆环不成立
    ftm_pwm_duty(FTM3 ,FTM_CH6,duoji_duty);                  //给舵机占空比  duoji_duty       1450
  
}

/*************************************电磁舵机;**************************************/
void duoji_V()
{
  uint8 VI = 0,VJ = 0 ,VJS= 0 ;
  int8 temp1 = 0 , temp2 = 0 ;
  VW_L=0 ; VW_R=0;      // VW_L入左弯标志 VW_R入右弯标志
  uint8  DIYH_j1 =0, DIYH_i = 0  ;
  uint8 x = 0 ;
  float av_average1 = 0 , av_average2 = 0 ;
  uint16 av_sum1 = 0 , av_sum2 = 0 ;
  for(x = 0 ; x < 3 ; x ++)
  {
    av_sum1 = +  AD_GYH_C[1][x] ;
    av_sum2 = +  AD_GYH_C[2][x] ;
  }
  av_average1 = av_sum1 / 3.0 ;
  av_average2 = av_sum2 / 3.0 ;
  
  /*********************************************电磁弯道/防抖动*******************************************************/
  
  uint8  VW_L_YZ = 4, VW_R_YZ = 4 ;                                         //比较连续点个数的阈值
  uint8  VW_L_YZ1 = 2 , VW_R_YZ1 = 2 ;                                         //比较连续点个数的阈值
  uint8  wucha = 0 ;
  
  for( VI = 0 ; VI < 5; VI ++ )
  {
    if( AD_GYH_A[1][VI] > AD_GYH_A[1][VI+1] )
    {
      temp1 ++;
    }
    else if( AD_GYH_A[1][VI] < AD_GYH_A[1][VI+1])
    {
      /*  if( temp1 >= 1 )   temp1 -- ;
           else              temp1 = 0 ;              */
      // temp1 -- ;
    }
    
    if( AD_GYH_A[2][VI] > AD_GYH_A[2][VI+1])
    {
      temp2 ++;
    }
    else if( AD_GYH_A[2][VI] < AD_GYH_A[2][VI+1] )
    {
      /* if(temp2 >= 1 )   temp2 -- ;
           else              temp2 = 0 ;              */
      //  temp2 -- ;
    }
  }
  
  
  
  duoji_last_errorV = duoji_errorV;
  duoji_last_dutyV = duoji_duty_V;
  
  duoji_errorV = (int8)( 600 * ( m_sqrt( AD_GYH_C[0][0] ) - m_sqrt( AD_GYH_C[3][0] ) )/( AD_GYH_C[0][0] + AD_GYH_C[3][0] ) );
  /* if(AD_GYH_C[2][0] >= AD_GYH_C[1][0] )    duoji_errorV = -fabs( duoji_errorV) ;
   else                                      duoji_errorV = fabs( duoji_errorV);          */
  
  
  // duoji_errorV =  600 * ( m_sqrt( av_average1 ) - m_sqrt( av_average2 ) )/( av_average1 + av_average2 ) ;
  //if( duoji_errorV < 10 && duoji_errorV > -10 )      duoji_errorV =   duoji_errorV  ;
  
  
  /*  float Error[1]={0};
  Error[0]=duoji_errorV;                                     //发送偏差
  vcan_sendware((int16_t *)Error, sizeof(Error));                            */
  
  /*  if( duoji_errorV >= 0 )    KPV = 2.0 + duoji_errorV / 30  ;
      else                       KPV = 2.0 - duoji_errorV / 30 ;
  KDV = 0 ;                                                           */
  
  if( Daolu_WaodaoV == 1 )
  {
    KPV = 4.2;
    KDV = 2.6;
  }
  else
  {
    KPV = 2.6;
    KDV = 1.2;
  }
  
  
  duoji_duty_V=(uint16)(dj_mid+( KPV )*duoji_errorV+KDV*( duoji_errorV-duoji_last_errorV ) );
  
  /*************************************环倒模式***********************************************************/
  if( DIYH_flag_L == 2 )         //入环模式
  {                             //5.6                3.1
    duoji_duty_V=(uint16)(dj_mid+( 5.6 )*duoji_errorV+( 4.1 )*(duoji_errorV-duoji_last_errorV));  //分类讨论
    
    DIYH_j1 = 0 ; DIYH_i = 0 ;                                       //电感选择
    if( AD_GYH_A[DIYH_i][0] >=  AD_GYH_A[DIYH_i][1])    DIYH_j1 = AD_GYH_A[DIYH_i][0] - AD_GYH_A[DIYH_i][1];
    else                                                DIYH_j1 = AD_GYH_A[DIYH_i][1] - AD_GYH_A[DIYH_i][0];
    
    duoji_duty_V = (uint16)( duoji_duty_V + ( ( - 0.2 ) * DIYH_j1 * DIYH_j1 + 6 * DIYH_j1 + 30 ) );    //入环打角算法
    if( duoji_duty_V < dj_mid + 10 )   duoji_duty_V = (uint16)(dj_mid + ( - 0.2 ) * DIYH_j1 * DIYH_j1 + 6 * DIYH_j1 + 20 );  //强制打角
    if( DIYH_flag_CL == 1)  duoji_duty_V=(uint16)(dj_mid+( 5.6 )*duoji_errorV+( 4.1 )*(duoji_errorV-duoji_last_errorV));  //分类讨论
  }
  
  if( DIYH_flag_CL == 3  && DIYH_flag_L ==  2)        //出环模式
  {
    duoji_duty_V=(uint16)( dj_mid - ( 0.9 ) * duoji_errorV - ( 2.6 ) * ( duoji_errorV - duoji_last_errorV ) );
  }                                   //1.9
  
  
  if( DIYH_flag_R == 2 )
  {                            //5.6  4.8              3.1
    duoji_duty_V=(uint16)(dj_mid+( 5.6 )*duoji_errorV+(4.1)*(duoji_errorV-duoji_last_errorV));  //分类讨论
    DIYH_j1 = 0 ; DIYH_i = 3 ;                                       //电感选择
    if( AD_GYH_A[DIYH_i][0] >=  AD_GYH_A[DIYH_i][1])    DIYH_j1 = AD_GYH_A[DIYH_i][0] - AD_GYH_A[DIYH_i][1];
    else                                                DIYH_j1 = AD_GYH_A[DIYH_i][1] - AD_GYH_A[DIYH_i][0];
    
    duoji_duty_V = (uint16)( duoji_duty_V - ( ( - 0.2 ) * DIYH_j1 * DIYH_j1 + 6 * DIYH_j1 + 30 ) );    //入环打角算法
    if( duoji_duty_V > dj_mid - 10 )   duoji_duty_V = (uint16)( duoji_duty_V - ( ( - 0.2 ) * DIYH_j1 * DIYH_j1 + 6 * DIYH_j1 + 20 ) );   //强制打角
    if( DIYH_flag_CR == 1)  duoji_duty_V=(uint16)(dj_mid-( 5.6 )*duoji_errorV-( 4.1 )*(duoji_errorV-duoji_last_errorV));  //分类讨论
    
  }
  
  if( DIYH_flag_CR == 3  && DIYH_flag_R ==  2)        //出环模式
  {
    duoji_duty_V=(uint16)( dj_mid + ( 0.9 ) * duoji_errorV + ( 2.6 ) * ( duoji_errorV - duoji_last_errorV ) );
  }                                  //1.9
  
  
  /* if(AD_data[1][0] < 5 || AD_data[2][0] < 8 )          //此时电感采集的值无效，舵机实行上一次打角
  if( duoji_duty_V > dj_mid )   duoji_duty_V =  (uint16)(1.1*duoji_duty_last);
  else    duoji_duty_V =  (uint16)(0.9*duoji_duty_last);                               */
  
  if(duoji_duty_V>dj_left_max)
    duoji_duty_V=dj_left_max;
  if(duoji_duty_V<dj_right_max)
    duoji_duty_V=dj_right_max;
  
  /*  if( BZ_dianci == 1 )
  {
  if( AD_GYH_C[0][0] <= 10 && AD_GYH_C[3][0] >= 10 )       ftm_pwm_duty(FTM3 ,FTM_CH6,BZ_dutyV);
  else                                                   ftm_pwm_duty(FTM3 ,FTM_CH6,duoji_duty_V);
}                                                                                               */
  if( QieHuan_Flag == 1 )       ftm_pwm_duty(FTM3 ,FTM_CH6,duoji_duty_V);
  //  ftm_pwm_duty(FTM3 ,FTM_CH6,duoji_duty_V);
}

/**********************************画出中线**************************************/
void huamid()
{
  uint8 ii;
  for(ii=5;ii<45;ii++)
  {
    if(mid[ii]>0&&mid[ii]<80)
    {
      Site_t midline[1]={mid[ii],ii};//40
      LCD_points(midline,1,RED);
      //  Site_t midlineq[1]={midq[ii],ii};//40
      //  LCD_points(midlineq,1,RED);
    }
  }
  for(ii=0;ii<80;ii++)
  {
    Site_t hangf[1]={ii,30};
    Site_t hangs[1]={ii,20};
    Site_t hangt[1]={ii,40};
    Site_t hangq[1]={ii,10};
    
    
    LCD_points(hangf,1,GREEN);
    LCD_points(hangs,1,GREEN);
    LCD_points(hangt,1,GREEN);
    LCD_points(hangq,1,GREEN);
    
  }
  for(ii=1;ii<60;ii++)
  {
    /* Site_t lie[1]={10,ii};
    Site_t liee[1]={70,ii};
    LCD_points(lie,1,GREEN);
    LCD_points(liee,1,GREEN);              */
    
    Site_t hangL[1]={straig_Lline[ii],ii};
    Site_t hangR[1]={straig_Rline[ii],ii};
    
    LCD_points(hangL,1,GREEN);
    LCD_points(hangR,1,GREEN);
  }
}


/****************************baodi电机驱动********************************1*/

void dianji_Left_baodi()
{
  SpeedKP_left=32;
  SpeedKI_left=34;
  SpeedKD_left=45;
  left_E[2]=left_E[1];
  left_E[1]=left_E[0];
  left_E[0]=dianji_left_speed-val_left;
  left_speed=left_speed
    +SpeedKP_left*( left_E[0] - left_E[1])
      +SpeedKI_left*( left_E[0] )
        +SpeedKD_left*(left_E[0] - 2*left_E[1]+  left_E[2]);
  if(left_speed>=2500)
    left_speed=2500;
  if(left_speed<=-1000)
    left_speed=1000;
  FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=0;//1500}          //   第一个电机驱动
  FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=(uint32)(left_speed);                       //   第二个电机驱动
  //  FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=(uint32)(left_speed);//1500}          //   第一个电机驱动
  //  FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=0;                       //   第二个电机驱动
  
  
}
void dianji_Right_baodi()
{
  SpeedKP_right=32;
  SpeedKI_right=34;
  SpeedKD_right=45;
  right_E[2]=right_E[1];
  right_E[1]=right_E[0];
  right_E[0]=dianji_right_speed+val_right;
  right_speed=right_speed
    +SpeedKP_right*( right_E[0] - right_E[1])
      +SpeedKI_right*( right_E[0] )
        +SpeedKP_right*(right_E[0] - 2*right_E[1]+  right_E[2]);
  if(right_speed>=2500)
    right_speed=2500;
  if(right_speed<=-1000)
    right_speed=1000;
  FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=0;//1500}          //   第一个电机驱动
  FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=500;                       //   第二个电机驱动
  //   FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=(uint32)(right_speed);//1500}          //   第一个电机驱动
  //   FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=0;                       //   第二个电机驱动
  
}

void dianji_baodi()
{
  
  float A = 1.0 ;
  float B = 1.0 ;
  uint16 V = 60 ;
  
  dianji_right_speed=(int)(A*V*( B + (duoji_duty-dj_mid)*(duoji_duty-dj_mid)/90 /100.0));
  dianji_left_speed=(int)(A*V*( B - (duoji_duty-dj_mid)*(duoji_duty-dj_mid)/90 /100.0));
  
  
  dianji_Right_baodi();
  dianji_Left_baodi();
  
}


/****************************disu电机驱动*********************************/
void dianji_Left_7_2()
{
  
  if( BZ == 1 )
  {
    SpeedKP_left=40;
    SpeedKI_left=80;
    SpeedKD_left=0;
  }
  else if( left_currve_flag == 1 || right_currve_flag == 1 )
  {
    SpeedKP_left=20;
    SpeedKI_left=50;
    SpeedKD_left=10;
  }
  else
  {
    SpeedKP_left=10;
    SpeedKI_left=85;
    SpeedKD_left=100;
  }
  
  left_E[2]=left_E[1];
  left_E[1]=left_E[0];
  left_E[0]=dianji_left_speed-val_left;
  left_speed=left_speed
    +SpeedKP_left*( left_E[0] - left_E[1])
      +SpeedKI_left*( left_E[0] )
        +SpeedKD_left*(left_E[0] - 2*left_E[1]+  left_E[2]);
  left_speed=1000;
  
  if ( length_val[0] <= 980 )                                                //减速第一
  {
    if( left_speed >= 0 )
    {
      FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=(uint32)(left_speed);
    }
    else
    {
      left_speed = -1*left_speed;
      FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=(uint32)(left_speed);
      FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=0;
    }
  }
  else if( left_speed >= 2500 )                                                    //上限内分
  {
    if( left_currve_flag == 1 || right_currve_flag == 1 )
    {
      FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=(uint32)(left_speed);
    }
    else if( zhidao_flag == 1 && BZ == 0 && length_val[0] >= 980 &&
            hudao2RLF == 0 && hudao2RRF == 0 && hudao1CLF == 0 && hudao1CRF == 0 )
    {
      if( left_speed >= 3000 )
      {
        left_speed = 3000 ;
      }
      FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=(uint32)(left_speed);
    }
    else if( left_speed >= 2500 )
    {
      left_speed = 2500 ;
      FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=(uint32)(left_speed);
    }
  }
  else if( left_speed <= 0 )                                                //下线内分
  {
    if( left_currve_flag == 1 || right_currve_flag == 1 )
    {
      left_speed = -1*left_speed;
      FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=(uint32)(left_speed);
      FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=0;
    }
    else
    {
      left_speed = 500 ;
      FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=(uint32)(left_speed);
      
    }
  }
  else                                                               //正常
  {
    FTM_CnV_REG(FTMN[FTM0],MOTOR2_PWM)=0;
    FTM_CnV_REG(FTMN[FTM0],MOTOR3_PWM)=(uint32)(left_speed);
  }
  
  
  
  
  
}

void dianji_Right_7_2()
{
  if( BZ == 1 )
  {
    SpeedKP_right=40;
    SpeedKI_right=80;                           //避障之前已减速  可适当减小吧!!
    SpeedKD_right=0;
  }
  else if( left_currve_flag == 1 || right_currve_flag == 1 )
  {
    SpeedKP_right=20;
    SpeedKI_right=50;
    SpeedKD_right=10;
  }
  else
  {
    SpeedKP_right=10;
    SpeedKI_right=85;
    SpeedKD_right=100;
    
  }
  
  
  right_E[2]=right_E[1];
  right_E[1]=right_E[0];
  right_E[0]=dianji_right_speed+val_right;
  right_speed=right_speed
    +SpeedKP_right*( right_E[0] - right_E[1] )
      +SpeedKI_right*( right_E[0] )
        +SpeedKD_right*(right_E[0] - 2*right_E[1]+  right_E[2]);
  right_speed=1000;
  
  if ( length_val[0] <= 980 )                                                    //减速第一
  {
    if( right_speed >= 0)
    {
      FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=(uint32)(right_speed);
    }
    else
    {
      right_speed = -1*right_speed;
      FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=(uint32)(right_speed);
      FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=0;
    }
  }
  else if( right_speed >= 2500 )                                                    //上限内分
  {
    if( left_currve_flag == 1 || right_currve_flag == 1 )
    {
      FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=(uint32)(right_speed);
      
    }
    else if( zhidao_flag == 1 && BZ == 0 && length_val[0] >= 980 &&
            hudao2RLF == 0 && hudao2RRF == 0 && hudao1CLF == 0 && hudao1CRF == 0 )
    {
      if( right_speed >= 3000 )
      {
        right_speed = 3000 ;
      }
      FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=(uint32)(right_speed);
      
    }
    else if( right_speed >= 2500 )
    {
      right_speed = 2500 ;
      FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=(uint32)(right_speed);
    }
  }
  else if( right_speed <= 0 )                                                //下线内分
  {
    if( left_currve_flag == 1 || right_currve_flag == 1 )
    {
      right_speed = -1*right_speed;
      FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=(uint32)(right_speed);
      FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=0;
    }
    else
    {
      right_speed = 500 ;
      FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=0;
      FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=(uint32)(right_speed);
    }
  }
  else
  {
    FTM_CnV_REG(FTMN[FTM0],MOTOR4_PWM)=0;
    FTM_CnV_REG(FTMN[FTM0],MOTOR1_PWM)=(uint32)(right_speed);
  }
}

void dianji_7_2()
{
  float A = 1.0 ;                      //
  float B = 1.0 ;
  float K = 0.15 ;
  uint16 V = 80 ;
  
  //uint8 L = 20 , G =18;
  
  /*****************************请标志********************************/
  
  dianji_left_speed  = 80 ;
  dianji_right_speed = 80 ;
  
  if( BZ == 1 )
  {
    V = 20 ; A = 1.5 ;  B = 0.9 ;  K = 0.5 ;
    dianji_right_speed = (int)( A*V*( B+ K*(BZ_duty-dj_mid)/75.0 ) );
    dianji_left_speed  = (int)( A*V*( B- K*(BZ_duty-dj_mid)/75.0 ) );
  }
  else if ( length_val[0] <= 980 )
  {
    
      V= 10 ;
      dianji_right_speed = (int)( A*V*( B+ K*(duoji_duty-dj_mid)/75.0 ) );
      dianji_left_speed  = (int)( A*V*( B- K*(duoji_duty-dj_mid)/75.0 ) );
    
  }
  else  if( left_currve_flag == 1 || right_currve_flag == 1 )
  {
    V = 65 ; A = 1.2 ;  B = 0.9 ;  K = 0.25 ;
    dianji_right_speed = (int)( A*V*( B+ K*(duoji_duty-dj_mid)/75.0 ) );
    dianji_left_speed  = (int)( A*V*( B- K*(duoji_duty-dj_mid)/75.0 ) );
  }
  else if( zhidao_flag == 1 )
  {
    dianji_left_speed  = 90 ;
    dianji_right_speed = 90 ;
  }
  
  dianji_Left_7_2();
  dianji_Right_7_2();
  
  
  
}


void disu_duoji()
{
  
  if( hudao2RLF == 1 || hudao2RRF == 1 )
  {
    KP = 5.6 ;
    KD = 9.6 ;
  }
  else if( hudao1CLF == 1 || hudao1CRF == 1 )
  {
    KP = 5.8 ;
    KD = 9.4 ;
  }
  else if ( right_currve_flag == 1 || left_currve_flag == 1 )
  {
    KP = 8.8 ;
    KD = 9.6 ;
  }
  else if( zhidao_flag == 1)
  {
    KP = 1.6 ;
    KD = 4.2 ;
  }
  else
  {
    KP = 5.2 ;
    KD = 7.9 ;
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

/*********************************************获取电压*********************************************************/
void  GetVoltage()
{
  uint8 GVI = 0,GVJ = 0,GVK=0,temp=0;                             //函数内部循环计数变量
  uint16  AD_sum[4]={0};                                          //十次电压数值求和
  
  for( GVI = 0 ; GVI < 4 ; GVI ++ )                              //采集5次电压
    for( GVJ = 0 ; GVJ < 10 ; GVJ ++ )
    {
      switch(GVI)
      {
      case 0:    AD_data[GVI][GVJ]=adc_once(ADC1_SE14, ADC_8bit);continue;
      case 1:    AD_data[GVI][GVJ]=adc_once(ADC1_SE15, ADC_8bit);continue;
      case 2:    AD_data[GVI][GVJ]=adc_once(ADC0_SE11, ADC_8bit);continue;
      case 3:    AD_data[GVI][GVJ]=adc_once(ADC0_SE16, ADC_8bit);continue;
      }
    }
  
  for( GVI = 0 ; GVI < 4 ; GVI ++ )                         //冒泡排序
    for( GVJ = 0 ; GVJ < 9 ; GVJ ++ )
      for( GVK = 0; GVK < 9 - GVJ ; GVK++)
      {
        if(AD_data[GVI][GVK] > AD_data[GVI][GVK+1])        //前面的比后面的大  则进行交换
        {
          temp = AD_data[GVI][GVK+1];
          AD_data[GVI][GVK+1] = AD_data[GVI][GVK];
          AD_data[GVI][GVK] = temp;
        }
      }
  
  
  for( GVI = 0 ; GVI < 4 ; GVI ++ )                       //8次电压数值求和
    for( GVJ = 1 ; GVJ < 9 ; GVJ ++ )
    {
      AD_sum[GVI] = AD_sum[GVI] + AD_data[GVI][GVJ];
    }
  
  for( GVI = 0 ; GVI < 4 ; GVI ++ )     AD_average[GVI] = AD_sum[GVI] / 8;     //8次电压数值取平均                                                                                  */
  
  AD_GYH[0]=GYH( AD_F_MAX,AD_F_MIN,AD_average[0] );                            //归一化
  AD_GYH[1]=GYH( AD_L_MAX,AD_L_MIN,AD_average[1] );
  AD_GYH[2]=GYH( AD_R_MAX,AD_R_MIN,AD_average[2] );
  AD_GYH[3]=GYH( AD_B_MAX,AD_B_MIN,AD_average[3] );
  
  
  
  for( GVI = 9 ; GVI > 0 ; GVI -- )                        //循环9次    归一化
  {
    AD_GYH_C[0][GVI] =  AD_GYH_C[0][GVI-1];
    AD_GYH_C[1][GVI] =  AD_GYH_C[1][GVI-1];
    AD_GYH_C[2][GVI] =  AD_GYH_C[2][GVI-1];
    AD_GYH_C[3][GVI] =  AD_GYH_C[3][GVI-1];
  }
  
  for( GVI = 0 ; GVI < 4 ; GVI ++ )          //数据更新
  {
    AD_GYH_C[GVI][0] =  AD_GYH[GVI];
    //     AD_GYH[GVI] = 0 ;
  }
  
  for( GVI = 9 ; GVI > 0 ; GVI -- )                        //循环9次    平均值
  {
    AD_GYH_A[0][GVI] =  AD_GYH_A[0][GVI-1];
    AD_GYH_A[1][GVI] =  AD_GYH_A[1][GVI-1];
    AD_GYH_A[2][GVI] =  AD_GYH_A[2][GVI-1];
    AD_GYH_A[3][GVI] =  AD_GYH_A[3][GVI-1];
  }
  
  for( GVI = 0 ; GVI < 4 ; GVI ++ )          //数据更新
  {
    AD_GYH_A[GVI][0] =  AD_average[GVI];
  }
  
  /*  for( GVI = 0 ; GVI < 4 ; GVI ++ )            //限幅滤波
  {
  uint8 cha = 0 , FuCha = 1 ;
  if( AD_GYH_C[GVI][0] > AD_GYH_C[GVI][1] )
  {
  cha = AD_GYH_C[GVI][0] - AD_GYH_C[GVI][1] ;
  if( cha <= FuCha )
  {
  AD_GYH_C[GVI][0] = AD_GYH_C[GVI][1] ;
  AD_GYH[GVI] = AD_GYH_C[GVI][0] ;
}
}
      else
  {
  cha = AD_GYH_C[GVI][1] - AD_GYH_C[GVI][0] ;
  if( cha <= FuCha )
  {
  AD_GYH_C[GVI][0] = AD_GYH_C[GVI][1] ;
  AD_GYH[GVI] = AD_GYH_C[GVI][0] ;
}
}
}                                                                           */
  
  
  /*  for( GVI = 0 ; GVI < 4 ; GVI ++ )
  {
  uint8 ave1 = 0 ;
  uint16 sum1 = 0 ;
  sum1 = AD_GYH_C[GVI][0] + AD_GYH_C[GVI][1] + AD_GYH_C[GVI][2] ;
  ave1 = sum1 / 3 ;
  if( ave1 >= AD_GYH_C[GVI][0] )
  
}           */
  
  
  
  
  
  for( GVI = 0 ; GVI < 4 ; GVI ++ )
  {
    AD_GYH_C[GVI][0] = (uint8)( 0.6*AD_GYH_C[GVI][0]+ 0.3*AD_GYH_C[GVI][1]+ 0.1*AD_GYH_C[GVI][2]);
    if( AD_GYH_C[GVI][0] <= 0)       AD_GYH_C[GVI][0] = 1 ;
    chuan[GVI] = (float)AD_GYH_C[GVI][0] ;
  }
  
  if( chuan[1] >= chuan[2] &&  chuan[2] > 0 )      chuan[4] = ((chuan[1] * chuan[1] ) /( chuan[2] *chuan[2] ));
  else  if( chuan[1] <= chuan[2] && chuan[1] > 0 )          chuan[4] = ((chuan[2] * chuan[2] ) /( chuan[1] *chuan[1] ));
  else                                              chuan[4] = 100;
  
  //   vcan_sendware((int16_t *)chuan, sizeof(chuan));
}

/***************************************电磁道路*********************************************/
void Dianci_Daolu()
{
  uint8 Daolu_i = 0 , Daolu_i_Max = 6 , Daolu_i_Min = 0 ;
  uint8 Daolu_wandao1 = 0 , Daolu_wandao2 = 0 ;
  
  for( Daolu_i = Daolu_i_Max ; Daolu_i >= Daolu_i_Min ; Daolu_i -- )
  {
    if( AD_GYH_C[0][Daolu_i] > AD_GYH_C[0][Daolu_i - 1] && AD_GYH_C[3][Daolu_i] > AD_GYH_C[3][Daolu_i - 1] )
    {
      Daolu_wandao1 ++ ;
    }
  }
  
  for( Daolu_i = Daolu_i_Max ; Daolu_i >= Daolu_i_Min ; Daolu_i -- )
  {
    if( AD_GYH_C[1][Daolu_i] > AD_GYH_C[1][Daolu_i - 1] && AD_GYH_C[2][Daolu_i] < AD_GYH_C[2][Daolu_i - 1] )
    {
      Daolu_wandao2 ++ ;
    }
  }
  
  if( Daolu_wandao1 >= 5 && Daolu_wandao2 >= 4 )    Daolu_WaodaoV  = 1 ;
  else                                              Daolu_WaodaoV  = 0 ;
  
  
  
  
}

/***************************************电磁圆环*********************************************/
void Dianci_YuHU()
{
  uint8  DIYH_i = 0 ,DIYH_j = 0,DIYH_js = 0,DIYH_jx = 0;
  uint8  DIYH_flag1 = 0, DIYH_flag2 = 0, DIYH_flag3 = 0, DIYH_flag4 = 0;
  
  
  if( DIYH_flag_L == 2 && DIYH_flag_CL == 0)                                  //打开电磁出环
  {
    if(AD_GYH_A[1][0] < AD_L_MAX &&  AD_GYH_A[2][0] <  AD_R_MAX)       DIYH_flag_CL = 1 ;
  }
  
  /******************************************出左圆环*********************************************/
  
  if( DIYH_flag_CL == 3 )
  {
    if(AD_GYH_A[1][0] >= AD_L_MAX - 5 || AD_GYH_A[2][0] >= AD_R_MAX - 5 )
    {
      AD_GYH_A[1][0] = AD_L_MAX - 3 ;
      AD_GYH_A[2][0] = AD_R_MAX - 3 ;
      DIYH_flag_L  = 2 ;
      DIYH_flag_R  = 0 ;
      DIYH_flag_CL = 3 ;
    }
    else
    {
      DIYH_flag_CL = 0;
      DIYH_flag_L  = 0 ;
    }
  }
  
  if( ( DIYH_flag_L == 2 && DIYH_flag_CL == 1 ) )
  {
    if(DIYH_flag_CL == 1)
    {
      for( DIYH_i = 0 ; DIYH_i < 9 ; DIYH_i ++ )                                   //左斜
      {
        for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ; DIYH_j ++ ,DIYH_jx ++)
        {
          if( ( AD_GYH_A[0][DIYH_j] >  AD_GYH_A[0][DIYH_j+1] ) && AD_GYH_A[0][9] != 0 )   DIYH_js ++;        //变大
          else
          {
            DIYH_js = 0;
            break;
          }
          
          if( DIYH_js >= 6 )
          {
            DIYH_flag1 = 1;
            break;
          }
        }
        if( DIYH_flag1 == 1) break;
      }
      
      DIYH_js = 0;
      DIYH_jx = 0;
      for( DIYH_i = 0 ; DIYH_i < 9 ; DIYH_i ++)                                   //右斜
      {
        for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ; DIYH_j ++ ,DIYH_jx ++)
        {
          if( ( AD_GYH_A[3][DIYH_j] <  AD_GYH_A[3][DIYH_j+1] ) && AD_GYH_A[3][9] != 0)   DIYH_js ++;          //变小
          else
          {
            DIYH_js = 0 ;
            break;
          }
          
          if( DIYH_js >= 6 )
          {
            DIYH_flag2 = 1 ;
            break;
          }
        }
        if( DIYH_flag2 == 1)  break;
      }
      
      if( DIYH_flag_CL == 1 )
      {
        if(DIYH_flag1 == 1 && DIYH_flag2 == 1 && ( AD_GYH_A[1][0] > AD_L_MAX  || AD_GYH_A[2][0] > AD_R_MAX ) )     DIYH_flag_CL = 2 ;
        // ( AD_GYH_A[0][0] < AD_GYH_A[0][1] || AD_GYH_A[3][0] > AD_GYH_A[3][1])
        else                                                                                                        DIYH_flag_CL = 1 ;
      }
    }
  }
  if ( DIYH_flag_CL == 2)
  {
    if( AD_GYH_A[0][0] < AD_GYH_A[0][1] || AD_GYH_A[3][0] > AD_GYH_A[3][1] )    DIYH_flag_CL = 3;
  }
  
  DIYH_flag1 = 0;
  DIYH_flag2 = 0;
  DIYH_flag3 = 0;
  DIYH_flag4 = 0;
  DIYH_js = 0;
  DIYH_jx = 0;
  
  
  
  /******************************************左圆环识别*****************************************************/
  if(DIYH_flag_R == 0 && DIYH_flag_CL == 0 )
  {
    for( DIYH_i = 0 ; DIYH_i < 7 ; DIYH_i ++)
    {
      for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ; DIYH_j ++ ,DIYH_jx ++)
      {
        if( AD_GYH_A[0][DIYH_j] < AD_GYH_A[0][DIYH_j+1] )   DIYH_js ++;                    //左斜
        else
        {
          DIYH_js = 0;
          break;
        }
        if( DIYH_js >= 3 )
        {
          DIYH_flag1 = 1;
          break;
        }
      }
      if( DIYH_flag1 == 1) break;
    }
    
    DIYH_js = 0;
    DIYH_jx = 0;
    for( DIYH_i = 0 ; DIYH_i < 7 ; DIYH_i ++)                                           //右斜
    {
      for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ; DIYH_j ++ ,DIYH_jx ++)
      {
        if( AD_GYH_A[3][DIYH_j] > AD_GYH_A[3][DIYH_j+1] )      DIYH_js ++;
        else
        {
          DIYH_js = 0;
          break;
        }
        
        if( DIYH_js >= 3 )
        {
          DIYH_flag2 = 1;
          break;
        }
      }
      if( DIYH_flag2 == 1) break;
    }
    
    DIYH_js = 0;
    DIYH_jx = 0;
    for( DIYH_i = 0 ; DIYH_i < 8 ; DIYH_i ++)                                  //左平
    {
      for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ;DIYH_j ++ ,DIYH_jx ++)
      {
        if( AD_GYH_A[1][DIYH_j] <=  AD_GYH_A[1][DIYH_j+1] )      DIYH_js ++;
        else
        {
          DIYH_js = 0;
          break;
        }
        
        if( DIYH_js >= 2 )
        {
          DIYH_flag3 = 1;
          break;
        }
      }
      if( DIYH_flag3 == 1) break;
    }
    
    DIYH_js = 0;
    DIYH_jx = 0;
    for( DIYH_i = 0 ; DIYH_i < 8 ; DIYH_i ++)                                   //右平
    {
      for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ; DIYH_j ++ , DIYH_jx ++)
      {
        if( AD_GYH_A[2][DIYH_j] >=  AD_GYH_A[2][DIYH_j + 1] )      DIYH_js ++;
        else
        {
          DIYH_js = 0;
          break;
        }
        
        if( DIYH_js >= 2 )
        {
          DIYH_flag4 = 1;
          break;
        }
      }
      if( DIYH_flag4 == 1) break;
    }
    if( DIYH_flag_L == 0 )
    {
      if( DIYH_flag1 == 1 && DIYH_flag2 == 1 && DIYH_flag3 == 1 && DIYH_flag4 == 1 )
      {
        if( AD_GYH_A[1][0] > AD_L_MAX - 10 && AD_GYH_A[2][0] > AD_R_MAX - 10 )                     DIYH_flag_L = 1;
      }
      else
      {
        DIYH_flag_L = 0;
      }
    }
    
    if( DIYH_flag_L == 1  )
    {
      if( AD_GYH_A[3][0] < AD_GYH_A[3][1] )
      {
        uint8   GVI = 0 ;
        DIYH_flag_L = 2 ;
        for( GVI = 9 ; GVI > 1 ; GVI -- )                //清除数据
        {
          AD_GYH_A[0][GVI] =  0;
          AD_GYH_A[1][GVI] =  0;
          AD_GYH_A[2][GVI] =  0;
          AD_GYH_A[3][GVI] =  0;
        }
      }
      
    }
  }
  
  
  if( DIYH_flag_R == 2 && DIYH_flag_CR == 0)                                  //打开电磁出环
  {
    if(AD_GYH_A[1][0] < AD_L_MAX &&  AD_GYH_A[2][0] <  AD_R_MAX)       DIYH_flag_CR = 1 ;
  }
  
  /******************************************出右圆环*********************************************/
  
  if( DIYH_flag_CR == 3 )
  {
    if(AD_GYH_A[1][0] >= AD_L_MAX - 5 || AD_GYH_A[2][0] >= AD_R_MAX - 5 )
    {
      AD_GYH_A[1][0] = AD_L_MAX - 3 ;
      AD_GYH_A[2][0] = AD_R_MAX - 3 ;
      DIYH_flag_L  = 0 ;
      DIYH_flag_R  = 2 ;
      DIYH_flag_CR = 3 ;
    }
    else
    {
      DIYH_flag_CR = 0;
      DIYH_flag_R  = 0 ;
    }
  }
  
  if( ( DIYH_flag_R == 2 && DIYH_flag_CR == 1 ) )
  {
    if(DIYH_flag_CR == 1)
    {
      for( DIYH_i = 0 ; DIYH_i < 9 ; DIYH_i ++ )                                   //左斜
      {
        for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ; DIYH_j ++ ,DIYH_jx ++)
        {
          if( ( AD_GYH_A[0][DIYH_j] <  AD_GYH_A[0][DIYH_j+1] ) && AD_GYH_A[0][9] != 0 )   DIYH_js ++;        //变小
          else
          {
            DIYH_js = 0;
            break;
          }
          
          if( DIYH_js >= 6 )
          {
            DIYH_flag1 = 1;
            break;
          }
        }
        if( DIYH_flag1 == 1) break;
      }
      
      DIYH_js = 0;
      DIYH_jx = 0;
      for( DIYH_i = 0 ; DIYH_i < 9 ; DIYH_i ++)                                   //右斜
      {
        for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ; DIYH_j ++ ,DIYH_jx ++)
        {
          if( ( AD_GYH_A[3][DIYH_j] >  AD_GYH_A[3][DIYH_j+1] ) && AD_GYH_A[3][9] != 0)   DIYH_js ++;          //变大
          else
          {
            DIYH_js = 0 ;
            break;
          }
          
          if( DIYH_js >= 6 )
          {
            DIYH_flag2 = 1 ;
            break;
          }
        }
        if( DIYH_flag2 == 1)  break;
      }
      
      if( DIYH_flag_CR == 1 )
      {
        if(DIYH_flag1 == 1 && DIYH_flag2 == 1 && ( AD_GYH_A[1][0] > AD_L_MAX  || AD_GYH_A[2][0] > AD_R_MAX ) )     DIYH_flag_CR = 2 ;
        // ( AD_GYH_A[0][0] < AD_GYH_A[0][1] || AD_GYH_A[3][0] > AD_GYH_A[3][1])
        else                                                                                                        DIYH_flag_CR = 1 ;
      }
    }
  }
  if ( DIYH_flag_CR == 2)
  {
    if( AD_GYH_A[0][0] > AD_GYH_A[0][1] || AD_GYH_A[3][0] < AD_GYH_A[3][1] )    DIYH_flag_CR = 3;
  }
  
  /******************************************右圆环识别*****************************************************/
  DIYH_flag1 = 0;
  DIYH_flag2 = 0;
  DIYH_flag3 = 0;
  DIYH_flag4 = 0;
  
  DIYH_js = 0;
  DIYH_jx = 0;
  
  if(DIYH_flag_L == 0 && DIYH_flag_CL == 0)
  {
    for( DIYH_i = 0 ; DIYH_i < 7 ; DIYH_i ++)                                   //左斜
    {
      for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ; DIYH_j ++ ,DIYH_jx ++)
      {
        if( AD_GYH_A[0][DIYH_j] >  AD_GYH_A[0][DIYH_j+1] )   DIYH_js ++;
        else
        {
          DIYH_js = 0;
          break;
        }
        
        if( DIYH_js >= 3 )
        {
          DIYH_flag1 = 1;
          break;
        }
      }
      if( DIYH_flag1 == 1) break;
    }
    
    DIYH_js = 0;
    DIYH_jx = 0;
    for( DIYH_i = 0 ; DIYH_i < 7 ; DIYH_i ++)                                   //右斜
    {
      for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ; DIYH_j ++ ,DIYH_jx ++)
      {
        if( AD_GYH_A[3][DIYH_j] <  AD_GYH_A[3][DIYH_j+1] )      DIYH_js ++;
        else
        {
          DIYH_js = 0;
          break;
        }
        
        if( DIYH_js >= 3 )
        {
          DIYH_flag2 = 1;
          break;
        }
      }
      if( DIYH_flag2 == 1) break;
    }
    
    DIYH_js = 0;
    DIYH_jx = 0;
    for( DIYH_i = 0 ; DIYH_i < 8 ; DIYH_i ++)                                   //左平
    {
      for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ;DIYH_j ++ ,DIYH_jx ++)
      {
        if( AD_GYH_A[1][DIYH_j] >=  AD_GYH_A[1][DIYH_j+1] )      DIYH_js ++;
        else
        {
          DIYH_js = 0;
          break;
        }
        
        if( DIYH_js >= 2 )
        {
          DIYH_flag3 = 1 ;
          break ;
        }
      }
      if( DIYH_flag3 == 1) break;
    }
    
    DIYH_js = 0;
    DIYH_jx = 0;
    for( DIYH_i = 0 ; DIYH_i < 8 ; DIYH_i ++)                                   //右平
    {
      for( DIYH_j = DIYH_i ; DIYH_jx < 9 - DIYH_i ; DIYH_j ++ , DIYH_jx ++)
      {
        if( AD_GYH_A[2][DIYH_j] <=  AD_GYH_A[2][DIYH_j + 1] )      DIYH_js ++;
        else
        {
          DIYH_js = 0;
          break;
        }
        
        if( DIYH_js >= 2 )
        {
          DIYH_flag4 = 1;
          break;
        }
      }
      if( DIYH_flag4 == 1) break;
    }
    if( DIYH_flag_R == 0 )
    {
      if( DIYH_flag1 == 1 && DIYH_flag2 == 1 && DIYH_flag3 == 1 && DIYH_flag4 == 1 )
      {
        if( AD_GYH_A[1][0] > AD_L_MAX - 10  && AD_GYH_A[2][0] > AD_R_MAX - 10 )       DIYH_flag_R = 1;
        else                                                                 DIYH_flag_R = 0;
      }
      else
      {
        DIYH_flag_R = 0;
      }
    }
    
    if( DIYH_flag_R == 1  )
    {
      if( AD_GYH_A[0][0] < AD_GYH_A[0][1] )
      {
        uint8   GVI = 0 ;
        DIYH_flag_R = 2 ;
        for( GVI = 9 ; GVI > 1 ; GVI -- )                //清除数据
        {
          AD_GYH_A[0][GVI] =  0;
          AD_GYH_A[1][GVI] =  0;
          AD_GYH_A[2][GVI] =  0;
          AD_GYH_A[3][GVI] =  0;
        }
      }
    }
  }
  
  
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


/*********************************避障摄像头分析函数*********************************/

void bizhang_fx()
{
  uint8 i=0,j=0;
  uint8 j_w=0,i_b=0;
  uint8 i_bz=0;
  
  for(i=15;i>5;i--)
  {
    for(j=30;j<=50;j++)
    {
      if( (img[i][j]==0xff & img[i-1][j]==0x00 & img[i-2][j]==0x00 ) || ( img[i-1][j]==0xff & img[i-2][j]==0x00 & img[i-3][j]==0x00) )
      {
        j_w++;
      }
    }
    
    if(j_w>=10)
    {
      i_bz=i-1;
      break;
    }
    
    j_w=0;
    
  }
  
  
  for(i=i_bz;i>5;i--)
  {
    for(j=30;j<=50;j--)
    {
      if( (img[i][j]==0x00 && img[i-1][j]==0x00) || (img[i-1][j]==0x00 && img[i-2][j]==0x00) )
      {
        i_b++;
      }
    }
    
    if(i_b>=35);
    {
      BZ_flag1=1;
      break;
    }
  }
  
  
}

/*******************************避障过程函数******************************/
void bizhang_gc( )
{
  float BZ_L = 20.0 ;                                                          //避障宽度
  float BZ_K = 0.0 ;                                                            // 斜率和反斜率
  
  int BZ_jiaodu;
  
  // disable_irq(PIT2_IRQn);                                                       //关闭PIT2 避障中不打开PIT2
  // disable_irq(PORTE_IRQn);                                                      //关闭PITE 避障中不打开PITE
  
  /****************************计算斜率*********************************/
  PIT2_Flag = 0 ;                                                              //TOF标志清零
  
  
  BZ_K = BZ_L / length_val[0] *10 ;
  BZ_jiaodu = ( int )( 4.0 * Atan( BZ_K ) ) ;       // 2.5为每度转化为数字量    BZ_jiaodu 所求舵机增量
  
  BZ_duty  = dj_mid + ( int )( BZ_jiaodu ) ;
  
  if(BZ_duty>dj_left_max)
    BZ_duty=dj_left_max;
  if(BZ_duty<dj_right_max)
    BZ_duty=dj_right_max;
  
  ftm_pwm_duty( FTM3 ,FTM_CH6, BZ_duty ) ;
  // DELAY_MS( 400 ) ;
  // pit_time_start  (PIT1);                                                      //开始计时
  while( 1 )
  {
    if( AD_GYH_C[3][0] <= 5 ) break ;                                        //3测量与精确程度
  }
  // BZ_timevar = pit_time_get_ms(PIT1) ;                                          //停止计时，获取计时时间
  
  BZ_duty = dj_mid - (int)( BZ_jiaodu * 0.8  ) ;
  
  if(BZ_duty>dj_left_max)
    BZ_duty=dj_left_max;
  if(BZ_duty<dj_right_max)
    BZ_duty=dj_right_max;
  
  ftm_pwm_duty( FTM3 ,FTM_CH6, BZ_duty ) ;
  // DELAY_MS((uint32)(1.5*BZ_timevar));
  // BZ_dutyV = BZ_duty ;                                                    //结束时把把避障的最后一次的舵机打角数据传给电磁舵机输出
  // BZ_dianci = 1 ;
  
  
  while( 1 )
  {
    if( AD_GYH_C[3][0] > 15 )  break ;                                        //15测量与精确程度
  }
  QieHuan_Flag = 1 ;
  BZ = 0 ;
  //duoji_duty = 870 ;
  // ftm_pwm_duty(FTM3 ,FTM_CH6, 870 ) ;                                          //范围值
  // DELAY_MS( 50 ) ;                                                                                          */
  
  //  enable_irq (PIT2_IRQn) ;                                                      //打开PIT2
  //  enable_irq (PORTE_IRQn) ;                                                     //打开PITE
  
}

void bizhang_gc1()
{
  float BZ_L = 20.0 ;                                                          //避障宽度
  float BZ_K = 0.0;                                                            // 斜率和反斜率
  uint16 BZ_duty = 0  ;
  int BZ_jiaodu;
  
  float a= 0.00,b = 0.00 ,Tz = 10.0;
  uint16 t = 0 ;
  
  // disable_irq(PIT2_IRQn);                                                       //关闭PIT2 避障中不打开PIT2
  // disable_irq(PORTE_IRQn);                                                     //关闭PITE 避障中不打开PITE
  
  /****************************计算斜率*********************************/
  
  BZ_K = BZ_L / length_val[0] *10 ;
  BZ_jiaodu = ( int )( 3.0 * Atan( BZ_K ) ) ;       // 2.5为每度转化为数字量    BZ_jiaodu 所求舵机增量
  
  BZ_duty  = dj_mid + ( int )( BZ_jiaodu ) ;
  
  duoji_duty = BZ_duty ;
  //避障过程中差速
  BZ_duty =   dj_left_max ;
  
  a = ( 800.0 - BZ_duty ) / ( -1 * Tz * Tz ) ;       //500ms
  b= -2.0 * Tz * a;
  
  
  
  // DELAY_MS( 200 ) ;
  
  while( 1 )
  {
    
    DELAY_MS( 500 ) ;
    t ++ ;
    if( t > (uint16)(2 * Tz) ) break;
    
    if( t <= (uint16)Tz ) BZ_duty1 = (uint16) (a * t * t + b * t + BZ_duty );
    else     BZ_duty1 =(uint16) ( 800.0 - ( a * t * t + b * t + BZ_duty - 800.0 ) );
    
    if(BZ_duty1>dj_left_max)
      BZ_duty1=dj_left_max;
    if(BZ_duty1<dj_right_max)
      BZ_duty1=dj_right_max;                                  //3测量与精确程度
    ftm_pwm_duty(FTM3 ,FTM_CH6, BZ_duty1 ) ;
    
    
    
  }
  
  
  
  //  enable_irq (PIT2_IRQn) ;                                                      //打开PIT2
  //  enable_irq (PORTE_IRQn) ;                                                     //打开PITE
  
}


/*!
*  @brief      PORTA中断服务函数
*  @since      v5.0
*/
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

/*!
*  @brief      DMA0中断服务函数
*  @since      v5.0
*/
void DMA0_IRQHandler()
{
  camera_dma();
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

}                                                                                  */


