/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
//car_a


/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K66 平台主程序
 * @author     山外科技
 * @version    v6.0
 * @date       2017-11-04
 */

#include "common.h"6+
#include "include.h"

//------------初始化参数定义--------------
//舵机初始化参数定义     12.2\14.4\16.3   blue

#define S3010_PORT   PTE5


//电机初始化参数定义


#define MOTOR3_FTM   FTM0
#define MOTOR4_FTM   FTM0
#define MOTOR3_PWM  FTM_CH2
#define MOTOR4_PWM  FTM_CH4


#define MOTOR3_PWM_IO  FTM0_CH3_PIN
#define MOTOR4_PWM_IO  FTM1_CH3_PIN

//滑行模式下，频率应该是 30~100。
//常规模式下，频率应该是 20k 左右
#if 0
#define MOTOR_HZ    (50)
#else
#define MOTOR_HZ    (5*1000)
#endif


//鹰眼初始化参数定义
#define SEE_H 3      //比SEE_H小的行视为盲区,都置黑
#define SEE_W CAMERA_W
#define SEE_SIZE SEE_H*SEE_W/8  //比SEE_SIZE小的区视为盲区,都置黑
#define SEE_MID SEE_W/2

/*--------------------全局变量定义--------------*/
//----------------拨码开关---------------
uint8 PTD5_data;
uint8 PTD4_data;
uint8 PTD3_data;
uint8 PTD2_data;

//-------------摄像头和LCD参数----------
Site_t site_dis;                                        //LCD显示变量
uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
uint8 img[CAMERA_H][CAMERA_W];
Site_t point={0,0};                                     //定义中线存储数组
int point_site[CAMERA_H][2]={0};
Site_t site     = {0, 0};                           //显示图像左上角位置
Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小
Size_t size     = {CAMERA_W, CAMERA_H};                   //显示区域图像大小
uint8 Aslan[CAMERA_H][CAMERA_W]={0};            //保存连通域标号的数组
//---------------------计时标志位------
uint8 TIME0flag_5ms   ;
uint8 TIME0flag_10ms  ;
uint8 TIME0flag_15ms  ;
uint8 TIME0flag_20ms  ;
uint8 TIME0flag_50ms  ;
uint8 TIME0flag_80ms  ;
uint8 TIME0flag_100ms ;
uint8 TIME0flag_1s;
uint8 TimeCount=0;
uint8 menu_flag=0;
//-----------电机速度参数----------------
float speed_right=0;
//int speed = 0;
uint16 LastSpeed=0;

float SpeedP=0.4;
float SpeedI=0.13;
float SpeedD=0.5;


float ErrorSpeed=0;
float SpeedErrorP=0;                                      //电机PID控制
float SpeedErrorD=0;
float SpeedErrorI=0;
float gain=0;
float LastGain=0;
float ExpectSpeed=0;                             //还受到image_analyze里控制   20%占空比为 312
float LastErrorSpeed=0;
float Last2ErrorSpeed=0;

//------------舵机转向参数--------

float S3010_mid=1580;
float S3010_right=1290;
float S3010_left=1820;
float turn_temp=0;
float TurnP=0.4;
float TurnI=0.3;
float TurnD=0.1;
uint8 turn_straight=0;
uint8 turn_left=0;
uint8 turn_right=0;


float TurnErrorP=0;                                      //舵机PID控制
float TurnErrorI=0; 
float TurnErrorD=0;
int turn=0;
float ExpectTurn=0;
float LastErrorTurn=0;
float Last2ErrorTurn=0;


//-----------------图像处理参数-------------------
uint8 middle_ave[CAMERA_H]={0};
float mid_sum=0;
float mid_val=0;
uint8 middle=CAMERA_W/2;
int ERROR[CAMERA_H][2];                         //两个差分数组
int E_ERROR[CAMERA_H][2];
int chafen0_min=0,chafen0_max=0,chafen1_min=0,chafen1_max=0;
int saidao_flag=2;
uint8 middle_count;
float cam_ratio=1.0;

float Error=0;                      //图像中线均值和中间的差值
float LastError=0;                      //图像中线均值和中间的差值
float T=1;                    //转角调节系数    1.3
float S=1;                    //转角调节系数    1.3
float ErrorP=0;                    //转角调节系数    1.3
float ErrorSD=60;
float ErrorS=7;                 //速度调节系数    在main_menu.H
float ErrorD=500;
float K_m=0.75;
float K_K=2.25;
float MINK=0;
float ref_speed =100 ;            //参考速度
uint8 flag1=0;
uint8 flag2=0;
uint8 flag3=0;
uint8 flag=0;
uint8 cross_left_flag=0;
uint8 cross_right_flag=0;
uint8 cross_flag=0;

uint8 annular_left_flag=0;
uint8 annular_right_flag=0;
uint32 annular_dis=0;

uint8 Ri=CAMERA_H-1;                    //最远的有效行，即和赛道标号一致的连通域中的最远的行,越远的行值越小
uint8 RunWayNum=0;
uint8 RunWayNum_H=CAMERA_H-1;         //RunWayNum所在的行号

uint8 dif_num_left = 0;
uint8 dif_num_right = 0;
uint8 start_count=0;

//-----------------会车参数-------------------------
/*uint32 mid_dis = 17000;                            
uint32 meet_dis = 0;                            //两车会车的距离，可根据赛道调节
uint32 whole_dis = 32350;                          //总距离
uint32 dis_right;                          //右轮的距离
uint32 dis_left;                           //左轮的距离
uint32 dis;



uint8 meet_flag_a=0;
uint8 meet_flag_b=0;
uint32 meet_times=0;
*/
uint8 start_flag=0;
uint32 start_time=0;
uint8 stop_flag=0;
uint32 stop_time=0;

//-------——————初始化函数定义-----------------
void motor_init();		//电机初始化
void serve_motor_init();	//舵机初始化
void encoder_init();		//编码器初始化
void LED_init();		//LED初始化
void eagle_init();              //鹰眼初始化
void switch_gpio_init();             //拨码开关初始化

//————————自定义函数声明——————————

void PORTA_IRQHandler();
void DMA0_IRQHandler();
void image_analyze();
void PIT0_IRQHandler();
void PIT3_IRQHandler();
void eagle_run();
void pointline(void);


void speed_get();                               //编码器速度获得
void speed_control();                           //电机PID控制
void motor_F(float n);                            //电机前进
void motor_B(float n);                            //电机后退

void serve_motor_control();                      //舵机PID控制
void serve_motor_turn(float n);	                //舵机转向
void set_serv();
void see_cir();
float min_k(void);
float get_ErrorT(int num);
float get_ErrorS(int num);

void switch_gpio_get();                      //编码器状态检测

//-------------------------系统参数选择--------------------------
sys_con system_config=0;

void get_system_config(void)
{
  uint8 sw_sta=0;
  sw_sta=get_sw_status();
   if((sw_sta&0x01)!=0)//BM1拨上去，使用摄像头
     system_config.c1=1;
   if((sw_sta&0x02)!=0)//BM2拨上去，使用LCD显示图像
     system_config.c2=1;
   if((sw_sta&0x04)!=0)//BM3拨上去，使用串口打印图像信息
     system_config.c3=1;
   if((sw_sta&0x08)!=0)//BM4拨上去，使用无线模块
     system_config.c4=1;
}
 

  
/*!
 *  @brief      main函数.

 *  @since      v5.3
 *  @note       山外摄像头 LCD 测试实验
 */
void  main(void)
{
      DisableInterrupts;                           //关总中断，初始化
      
      
    //配置中断服务函数
     // systick_timing_ms(100);                                     //初始化滴答定时器，定时时间为： 100ms
  // set_vector_handler(SysTick_VECTORn ,SysTick_IRQHandler);    //设置滴答定时器的中断服务函数为 SysTick_IRQHandler
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置 PORTA 的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //设置 DMA0 的中断服务函数为 DMA0_IRQHandler
    set_vector_handler(PIT0_VECTORn,PIT0_IRQHandler);	//设置PIT0中断服务函数为PIT0_IRQHandler
    set_vector_handler(PIT3_VECTORn,PIT3_IRQHandler);	//设置PIT0中断服务函数为PIT0_IRQHandler
    pit_init_ms(PIT0,5);			//设置PIT0,定时时间为5ms
   
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);            //设置优先级分组,4bit 抢占优先级,没有亚优先级    
    
     NVIC_SetPriority(PORTA_IRQn,0);         //配置优先级    
//     NVIC_SetPriority(PORTE_IRQn,1);         //配置优先级    
     NVIC_SetPriority(DMA0_VECTORn,1);          //配置优先级    
     NVIC_SetPriority(PIT0_IRQn,2);          //配置优先级   
     NVIC_SetPriority(PIT3_IRQn,3);          //配置优先级   


        motor_init();
       serve_motor_init();
        LED_init(LED_MAX);  
       key_init(KEY_MAX);
     get_system_config();
     
         if(system_config.c2==1)
   {
     SpeedP=1;
     SpeedI=0.4;
     SpeedD=0.4;
     TurnP=1;
     TurnI=0.3;
     TurnD=0.3;
     //ErrorP=40;
     ref_speed = 80;
   }
 
                 if(system_config.c3==1)
   {
     SpeedP=1;
     SpeedI=0.4;
     SpeedD=0.4;
     TurnP=1;
     TurnI=0.3;
     TurnD=0.3;
    // ErrorT=40;
     ref_speed = 80 ;
   }
                 if(system_config.c2==1&&system_config.c3==1)
   {
     SpeedP=1;
     SpeedI=0.4;
     SpeedD=0.4;
     TurnP=1;
     TurnI=0.3;
     TurnD=0.3;     
    // ErrorT=40;   
     ref_speed = 80;
   }
   
    encoder_init();
    eagle_init();
    switch_gpio_init();
       
    EnableInterrupts;                                //开串口接收中断    
    enable_irq (PIT0_IRQn);
 //-------------------------------初始化---------------------------------------//
//           display_logo();//开机显示欢迎画面4
           OLED_Init();//OLED初始化
          

   
 //-------------------------------------------------
//    while(1)
//    {
////    motor_F(30);
////    DELAY_MS(1000);
//    motor_F(30);
//    DELAY_MS(1000);
//    }
   
//    meet_flag_a=1;
    //main_menu_task();       //菜单函数
  // ErrorS=(ref_speed-16)/18;
          
    while(1){
     eagle_run();
     serve_motor_control();
     speed_control();
//     if (flag1==1)
//     {
//       led(LED0,LED_ON);
//       flag1=0;
//       led(LED0,LED_OFF);   
//     }
//     if (flag2==1)
//     {
//       led(LED1,LED_ON);
//       flag2=0;
//       pflag2=0;
//       led(LED1,LED_OFF);    
//     }
//      if (flag3==1)
//     {
//       led(LED2,LED_ON);
//        
//     }
//      if (flag4==1)
//     {
//       led(LED3,LED_ON);
//       flag4=0;
//       pflag4=0;
//       led(LED3,LED_OFF);
//       flag3=0;
//       pflag3=0
//       led(LED2,LED_OFF);  
//     }
    }
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

//              计时器中断服务函数
void PIT0_IRQHandler()
{	
  TimeCount ++ ;
  
  if(TimeCount%1 == 0 )   
  {
   
    TIME0flag_5ms = 1;  
    serve_motor_control();
  } 
  if(TimeCount%2 == 0 )
  {
    TIME0flag_10ms = 1;  
    key_IRQHandler();
    key_msg_process();
  } 
  if(TimeCount%3 == 0 )
  {
    TIME0flag_15ms = 1;    
  }     
  if(TimeCount%4 == 0 )
  {
    TIME0flag_20ms = 1;  
    

    gpio_set(S3010_PORT,0);
    set_serv();
  }
  
  if(TimeCount%10 == 0 )
  {
    TIME0flag_50ms = 1;
  }
  if(TimeCount%16 == 0 )
  {
      //see_cir();
      TIME0flag_80ms = 1;
  }  
  if(TimeCount%20 == 0 )
  {
    TIME0flag_100ms = 1;
   
    if (system_config.c4==1)
    {
    OLED_process();   
    }
  }  
  if(TimeCount%200 == 0 )
  {
    TIME0flag_1s = 1;
    TimeCount = 0 ;
    flag1=0;
    led(LED3,LED_OFF);
   

  }	
  
  PIT_Flag_Clear(PIT0);	
}
void PIT3_IRQHandler()
{
  gpio_set(S3010_PORT,1);
  PIT_Flag_Clear(PIT3);
}

/*------------初始化函数---------------*/

//            鹰眼初始化
void eagle_init()
{ 
  camera_init(imgbuff);
  
}
//舵机初始化
void serve_motor_init()
{
 
 gpio_init(PTE5,GPO,1);


// pit_delay_us(PIT1,S3010_mid);
// gpio_turn(S3010_PORT);
 
  
}
void set_serv(void)
{
  if(ref_speed==0)
  {
    turn=1570;
  }
    
    pit_init_us(PIT3,turn);//舵机高电平时间 范围0-3ms
    set_vector_handler(PIT3_VECTORn ,PIT3_IRQHandler);//设置PIT1的中断服务函数为 PIT1_IRQHandler
    enable_irq (PIT3_IRQn); //使能PIT1中断

}

//电机初始化
void motor_init()
{
  ftm_pwm_init(MOTOR3_FTM, MOTOR3_PWM,MOTOR_HZ,0);      //初始化 电机 PWM
  ftm_pwm_init(MOTOR4_FTM, MOTOR4_PWM,MOTOR_HZ,0);      //初始化 电机 PWM

}

//LED初始化
void LED_init()
{
  led_init(LED0);                         //初始化LED0，LED1，LED2，LED3
  led_init(LED1);                        
  led_init(LED2);
  led_init(LED3);
}

//编码器初始化
void encoder_init()
{
  ftm_quad_init(FTM2);
}

//拨码开关初始化
void switch_gpio_init()
{
  gpio_init(PTD2,GPI,1);
  gpio_init(PTD3,GPI,1);
  gpio_init(PTD4,GPI,1);
  gpio_init(PTD5,GPI,1);
}

/*-----------自定义函数----------------*/
//速度获取
void speed_get()
{
  speed_right=ftm_quad_get(FTM2) ;
  ftm_quad_clean(FTM2);
}

//速度控制
void speed_control()
{
  
  
  
  speed_get();
  
  //位置式PID
//  ErrorSpeed=ExpectSpeed-speed_right; 
//  SpeedErrorP=ErrorSpeed;
//  SpeedErrorI+=ErrorSpeed;
//  SpeedErrorD=ErrorSpeed - LastErrorSpeed;
//  gain=(SpeedErrorP*SpeedP)+(SpeedErrorI*SpeedI)+(SpeedErrorD*SpeedD) ;
   ErrorSpeed=ExpectSpeed-speed_right; 
  SpeedErrorP=ErrorSpeed-LastErrorSpeed;
  SpeedErrorI=ErrorSpeed;
  SpeedErrorD=ErrorSpeed - 2*LastErrorSpeed+Last2ErrorSpeed;
  gain=gain+(SpeedErrorP*SpeedP)+(SpeedErrorI*SpeedI)+(SpeedErrorD*SpeedD) ;
  Last2ErrorSpeed=LastErrorSpeed;
  LastErrorSpeed=ErrorSpeed;
  /*SpeedErrorD=ErrorSpeed - LastErrorSpeed; 
  if(SpeedErrorD>12)
  {
    ErrorSpeed=LastErrorSpeed+10;
  }
    if(SpeedErrorD<-12)
  {
    ErrorSpeed=LastErrorSpeed-10;
  } 
  */
  
//  if((meet_flag_a==1)&&(meet_flag_b==0))
//  {
//    motor_F(0);
//  }
//  else if((meet_flag_a==2)&&(meet_flag_b!=2))
//  {
//    motor_F(0);
//  }
//  else if((meet_flag_a==2)&&(meet_flag_b==2))
//  {
//    motor_F(0);
//  }  
  if(gain>0)
  {
    if(gain>80)
      gain=80;
    motor_F(gain);
  }
  else
  {
    if(gain < -50)
      gain=-50;
    motor_B(-gain);
  }
    
//  if(speed > 270)
//  {
//    motor_B(0,-200);
//  }
  
  
  
  
}

//电机前进
void motor_F(float n)                                     //m = 0,左边电机前进；m = 1，右边电机前进      还未测试
{       
  if(n >50)
  {
    n = 50;
  }
  if(n>=0 && n<=50)                                            //MOTOR3,MOTOR4 控制右轮，MOTOR1,MOTOR2 控制左轮
  {
//    ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,n);
//    ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);
//    ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,n);
//    ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
      ftm_pwm_duty(MOTOR3_FTM, MOTOR3_PWM,0);
      ftm_pwm_duty(MOTOR4_FTM, MOTOR4_PWM,n);
  }
}

//电机后退
void motor_B(float n)                                     //m = 0,左边电机前进；m = 1，右边电机前进
{
    if(n > 50)
    {
      n = 50;
    }
    if(n>=0 && n<=50)
    {     
//      ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
//      ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,n);
//      ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);                     
//      ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,n);
        ftm_pwm_duty(MOTOR3_FTM, MOTOR3_PWM,n);
        ftm_pwm_duty(MOTOR4_FTM, MOTOR4_PWM,0);

     }
}

//舵机转向
void serve_motor_turn(float n)
{
  if(n>S3010_left)
  {
    n=S3010_left;}
  
  if(n<S3010_right)
  { n=S3010_right;
  }
  turn=n;
		
}

//             舵机控制
void serve_motor_control()
{ 
  /* TurnErrorP=ExpectTurn-S3010_mid;
  TurnErrorI+=TurnErrorP;
  TurnErrorD=TurnErrorP-LastErrorTurn;
  TurnErrorD=TurnErrorP-2*LastErrorTurn+Last2ErrorTurn;
  Last2ErrorTurn=LastErrorTurn;
  LastErrorTurn=TurnErrorP;
  turn=(TurnErrorP*TurnP)+(TurnErrorD*TurnD)+(TurnErrorI*TurnI)+S3010_mid;
  serve_motor_turn(turn);
 if(ExpectTurn==S3010_mid) 
 {
  TurnErrorI=0; 
 } */

    if(abs(Error)<5)                                               //直道速度，转向控制
    {
      //ExpectSpeed=ref_speed-ErrorS*Error;                        //260
      ExpectTurn=(S3010_mid-Error*ErrorP+ErrorD*(Error-LastErrorTurn));
    }                                                              //小于150右转，大于150左转
    else  if(Error>5.1)                                                       //普通弯道速度，转向控制
    {
      ExpectTurn=1290;
      //ExpectSpeed=(-abs(Error)*ErrorS+ref_speed);               //180
    }
    else  if(Error<-5.1)                                                       //普通弯道速度，转向控制
    {
      ExpectTurn=1820;
      //ExpectSpeed=(-abs(Error)*ErrorS+ref_speed);               //180
    }
  serve_motor_turn(ExpectTurn);
}

//编码器状态获取
void switch_gpio_get()
{
  PTD5_data = gpio_get(PTD5);
  PTD4_data = gpio_get(PTD4);
  PTD3_data = gpio_get(PTD3);
  PTD2_data = gpio_get(PTD2);
  
}

//          鹰眼运行
void eagle_run()
{
  uint16 i,j;
  camera_get_img();                                   //摄像头获取图像
  img_extract(img, imgbuff, CAMERA_SIZE);          //解压为灰度图像，方便发送到上位机显
  /*for(i=0;i<20;i++)
  {
    for(j=0;j<80;j++)
    {
      img[i][j]=0x00;
    }
  }*/
  image_analyze();
//  lcd_img_binary_z(site, size, imgbuff, imgsize,BLACK,WHITE);      // lcd_img_gray_z(site, size, imgbuff, imgsize);
  //vcan_sendimg(imgbuff, sizeof(imgbuff));
 //vcan_sendimg(img, sizeof(img));

}

//              图像处理分析
void image_analyze()
{
  uint8 i,j;
  int ii=0,jj=0;//如果用uint8，采用倒序时判断ii>=0会提示 pointless
  uint8 canum=1;//保存当前标号的变量，保存标号的数组为Aslan
  
  
  for(i=0;i<SEE_SIZE;i++)
  {
    imgbuff[i]=255;
  }
  for(j=0;j<SEE_W;j++)
  {
    for(i=0;i<SEE_H;i++)
    {
            img[i][j]=0;
    }
  }
    //竖着滤
 for(j=0;j<SEE_W;j++)
  {
	for(i=SEE_H+1;i<CAMERA_H;i++)
	{
	  if(img[i][j]==255 && img[i-1][j]==0 && img[i+1][j]==0)
	  {
		img[i][j]=0;
	  }
	}
  }
  //横着滤
  for(i=SEE_H;i<CAMERA_H;i++)
  {
	for(j=1;j<SEE_W-1;j++)
	{
	  if(img[i][j]==255 && img[i][j-1]==0 && img[i][j+1]==0)
	  {
		img[i][j]=0;
	  }
	}
  }
  
  for(ii=CAMERA_H-1;ii>=SEE_H;ii--)
  {
	for(jj=0;jj<CAMERA_W;jj++)
	{
	  //Aslan[ii][jj]=254;
	  if(img[ii][jj]==0xff)//此点为白色的几种情况判断
	  {
		if(ii==CAMERA_H-1)//第一行的情况  从下往上就是最下面一行
		{
		  if(jj==0)//第一个点的情况
		  {
			Aslan[ii][jj]=canum;
			canum++;
		  }
		  else if(img[ii][jj-1]==0xff)//左边是白色的情况
		  {
			Aslan[ii][jj]=Aslan[ii][jj-1];
		  }
		  else
		  {
			Aslan[ii][jj]=canum;
			canum++;
		  }
		}
		else//其他行
		{
		  if(jj==0)//第一个点的情况
		  {
			if(img[ii+1][jj]==0xff)//下面是白色的情况 从下往上，改成判断下面的点
			{
			  Aslan[ii][jj]=Aslan[ii+1][jj];
			}
			else
			{
			  Aslan[ii][jj]=canum;
			  canum++;
			}
		  }
		  else if(img[ii+1][jj]==0xff && img[ii][jj-1]==0xff)//下面和左边为白色，统一标号
		  {
			if(Aslan[ii+1][jj] > Aslan[ii][jj-1])
			{
			  Aslan[ii][jj]=Aslan[ii][jj-1];
			}
			else
			{
			  Aslan[ii][jj]=Aslan[ii+1][jj];
			}
		  }
		  else if(img[ii+1][jj]==0xff)//下面为白色，左边为黑，标号等于下面
		  {
			Aslan[ii][jj]=Aslan[ii+1][jj];
		  }
		  else if(img[ii][jj-1]==0xff)//下面为黑，左边是白色。则标号等于左边的。
		  {
			Aslan[ii][jj]=Aslan[ii][jj-1];
		  }
		  else//上面左边都不是白色，标记为新区域。
		  {
			Aslan[ii][jj]=canum;
			canum++;
		  }
		}
	  }
	  else
	  {
		Aslan[ii][jj]=255;
	  }
	}
	for(j=CAMERA_W-1;j>0;j--)//把一行的标号统一
	{
	  if(img[ii][j]==0xff && img[ii][j-1]==0xff)
	  {
		if(Aslan[ii][j]>Aslan[ii][j-1])
		{
		  Aslan[ii][j]=Aslan[ii][j-1];
		}
		else
		{
		  Aslan[ii][j-1]=Aslan[ii][j];
		}
	  }
	}
  }
  
   //获取赛道标号
//  uint8 RunWayNum=0;
//  uint8 RunWayNum_H=CAMERA_H-4;         //RunWayNum所在的行号
  if(img[RunWayNum_H][middle]==255)//车头正前方的位置
  {
	RunWayNum=Aslan[RunWayNum_H][middle];
//        led(LED0, LED_ON);                  //LED0 亮    
//        led(LED1, LED_OFF);                  //LED0 亮    
//        led(LED2, LED_OFF);                  //LED0 亮    
	//led_turn(0);
  }
  else if(img[RunWayNum_H][middle/2]>128)//左边在赛道内
  {
	RunWayNum=Aslan[RunWayNum_H][middle/2];
  }
  else if(img[RunWayNum_H][middle*3/2]>128)//右边在赛道内
  {
	RunWayNum=Aslan[RunWayNum_H][middle*3/2];
  }
  else
  {
	RunWayNum=0;
  }
  
  
  //查找图像中点                      oxff是白点，0x00是黑点
  
  for(i=SEE_H;i<CAMERA_H;i++)                                     //point_site初始化,j=0
  {
      point_site[i][0]=SEE_W-1;       //左边界
      point_site[i][1]=0; //右边界
  }
  
//   uint8 Ri=CAMERA_H-1;      //最远的有效行，即和赛道标号一致的连通域中的最远的行,越远的行值越小
  Ri=CAMERA_H-1;
  middle_count = 0;
 int island=0;
  for(i=CAMERA_H;i>(SEE_H-1);i--)                                     //第一个连通域查找白黑突变点
  {
    for(j=0;j<SEE_W;j++)                                       //从左向右扫描
    {
      if(Aslan[i][j]==RunWayNum)                                //只在车
      {
        if(Ri>i)
        {
          Ri=i;
        }
        if(point_site[i][0]>j)
        {
          point_site[i][0]=j;				//该行的左端
        }
        if(point_site[i][1]<j)
        {
          point_site[i][1]=j;				//该行右端
        }
      }
     
    }
    if(i>25&&i<29)
    {
      for(j=40;j<SEE_W;j++) 
      {
        if(point_site[i][1]-point_site[i][0] > 65 )//赛道很宽；需要判断是否是环岛
                {
                  for(jj=point_site[i][0];jj<point_site[i][1];jj++)
                  {
                    if(Aslan[ii][jj]!=RunWayNum)
                    {
                      island++;
                      
                    }
                  }
                  if(island > 10)//是环岛，按照上次的中点选择以左边或右边扫线
                  {
                          flag1=1;
                     //islandCount++;
                     //islandi=ii;
                  }
                }
      }
    }
    see_cir();
  middle=(int)((point_site[i][0]+point_site[i][1]+1)/2);              //求中点位置
  middle_ave[i]=middle;                                               //保存中线位置
  }
  
  
  
 /*for (i=0;i<40;i++)
 {
   j=point_site[i+1][1]-point_site[i][1];
   if(j>15)
   {
     led(LED3,LED_ON);
   }
 }*/
  
  
  mid_sum=0;                                                          //计算中线偏差，可能需要修改
  for(i=CAMERA_H;i>(Ri-1);i--)
  {
    mid_sum += middle_ave[i];
  }
 mid_val = mid_sum/(CAMERA_H-Ri+1);
  MINK=min_k();
  Error=K_m*(mid_val-SEE_MID)+K_K*MINK;
  if(abs(Error-LastError)<10)
  {
  ErrorP=get_ErrorT(T);
//  ErrorS=get_ErrorS(S);
//  ErrorP=get_ErrorT(T);
//  ErrorS=6;
  
  if((start_flag==1)&&(start_time==0))
  {

    
    
  }
  else if(start_time<=4)
  {
    
  }



  start_count = 0;
  for(j=point_site[60][0];j<point_site[60][1];j++)
  {
    if(img[40][j] == 0xff && img[40][j+1]==0x00)
    {
      start_count++;
    }
  }
  if(start_count > 5)
  {
    stop_flag = 1;
  }
  
//    ExpectTurn=(-Error*ErrorT+S3010_mid);
    
  if(stop_flag)
  {
    /*int i=30;
    while(1)
    {
                ExpectSpeed=0;
    speed_control();
    i--;
    }
          ErrorSpeed=i;
    speed_control();
    
    ref_speed=0;*/
  }
  
  
  
 else
  {
    //差速度
    
      ExpectSpeed=ref_speed-ErrorS*abs(Error);                        //260
//  -ErrorSD*abs(Error-LastError)
                                                                //小于150右转，大于150左转
//  else if(Error<-3.2)
//  {
//    ExpectSpeed=5.88*Error+78.8;
//  }
//  else if(Error>3.2)
//  {
//    ExpectSpeed=-5.88*Error+78.8;
//  }
  }
  LastErrorTurn=Error;
 if (ExpectSpeed<40)
  {
  ExpectSpeed=40;
}
 LastError=Error;
  }
 }


//画线
void pointline(void)
{
  uint8 i;
    for(i=Ri;i<CAMERA_H;i++)
    {
      point.y=i/2;                                                          //赛道左边界
      point.x=point_site[i][0]/2;   
      lcd_point(point,RED);
      
      point.x=point_site[i][1]/2;                                           //赛道右边界
      lcd_point(point,RED);
      
      point.x=middle_ave[i]/2;             
      lcd_point(point,GREEN);                                               //画中线
      
    }
}

void show_pic_on_oled(void)
{
  OLED_Img_gray_Z(site, size, img, imgsize);
}
float min_k()
{
  float num=CAMERA_H+1;
  int count=0;
  int mul=0;
  int sumx=0;
  int sumy=0;
  int sumx2=0;
  float mink=0;
  float n=1/num;
  for (count=0;count<num;count++)
  {
    sumy+=count;
  }
  for (count=0;count<num;count++)
  {
    sumx+=middle_ave[count];
  }
  for (count=0;count<num;count++)
  {
    sumx2+=middle_ave[count]*middle_ave[count];
  }
    for (count=0;count<num;count++)
  {
    mul+=middle_ave[count]*count;
  }
  mink =  ((mul-n*sumx*sumy)/(sumx2-n*sumx*sumx));
  return mink;
   
}
float get_ErrorT(int num)
{
 
  float Et=ErrorP;
  //float p2 =     0.05568  ;
 // float p3 =     0.595 ;
  //float p4 =     -3.61  ;
 // float p5 =     15.72 ; 
 //float     p1 =    0.002623  ;
 //float     p2 =    -0.02137 ;
 //float     p3 =     -0.4227  ;
// float     p4 =        2.19  ;
 //float     p5 =       9.314 ;
 //float     p6 =       7.864 ;
  float       p1 =    0.001256  ;
   float     p2 =   -0.003586 ;
   float     p3 =     -0.1505  ;
    float    p4 =      0.3962 ;
    float    p5 =       4.175  ;
      float  p6 =       24.27;
  if(Error>-8.3&&Error<8.8)
  {
  switch (num)
  {
  case 1:
    //Et=p2*pow(Error,3)+ p3*pow(Error,2)+ p4*Error+ p5;
    Et=p1*pow(Error,5)+p2*pow(Error,4)+p3*pow(Error,3)+ p4*pow(Error,2)+ p5*Error+ p6;
    break;
  default:
    Et=35;
    break;
    
  }
  }
  return Et;
}
float get_ErrorS(int num)
{
  
    float Es=ErrorS;

//float      p1 =   4.581e-05  ;
//float      p2 =   2.285e-05;
//float      p3 =  -0.008431  ;
//float      p4 =   -0.001855   ;
//float      p5 =    0.5863 ;
//float      p6 =      -0.0661 ;
float         p1 =     0.03669  ;
float         p2 =  -4.248e-17 ;
 float        p3 =     -0.6799;
  float       p4 =   3.878e-16 ;
 float        p5 =       3.863;
 float        p6 =  -8.269e-16 ;
 float        p7 =      -1.744 ;
 float        p8 =   2.249e-16 ;

   if(abs(Error)<3.2)
  { 
    switch (num)
  {
  case 1:
    Es=p1*pow(Error,7)+p2*pow(Error,6)+p3*pow(Error,5)+p4*pow(Error,4)+p5*pow(Error,3)+p6*pow(Error,2)+p7*pow(Error,1)+p8;
    break;
  default:
    Es=0;
    break;
    
  }  
 
  }

  return Es;
}
void see_cir()
{
//int i,j;
//if(Error>-0.5&&Error<1.5)
//{
//  flag2=1;
//}
//if(flag2==1)
//{
//    for(i=25;i<30;i++)
//    {
//        if(point_site[i][0]<55)
//        {
//          pflag3++;
//        }
//    }
//    if(pflag3>2)
//    {
//        flag3=1;
//    }
//    if(flag3==1)
//    {
//        for(i=25;i<30;i++)
//        {
//        if(point_site[i][1]-40>26)
//        {
//          pflag4++;
//        }
//        }
//        if(pflag4>2)
//        {
//          flag4=1;
//        }
//    }
//    
//}
}

/*!
 *  @brief      VCAN_PORT中断服务函数
 *  @since      v5.0
 */

