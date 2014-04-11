/*
Author: Shehan Malaka Caldera
Changelog:
          * Added function-wise initiation
          * Added BLDC initializing function, BLDC function is optimized to
            power-on each motor with a 10ms delay to each.
          * Added PWM calulating function
          * Added support for 16x2 LCD display.
          * Added support for printing various info on LCD, at selected points.

Status: OK for powering ON the all motors
        NO stable hovering
        Printing various status on LCD
*/

int i=0;
int speed_ratio=0;
char pwm_val[3];
char adc_read[3];
int adcval=0x00;


unsigned char text=0x00;

unsigned GYRO_XOUT_H=0x00;
unsigned GYRO_XOUT_L=0x00;
unsigned GYRO_YOUT_H=0x00;
unsigned GYRO_YOUT_L=0x00;
unsigned GYRO_ZOUT_H=0x00;
unsigned GYRO_ZOUT_L=0x00;

char text1[4];



int GYRO_XOUT_OFFSET_1000SUM=0x00;
int GYRO_YOUT_OFFSET_1000SUM=0x00;
int GYRO_ZOUT_OFFSET_1000SUM=0x00;

int GYRO_XOUT_OFFSET=0x00;
int GYRO_YOUT_OFFSET=0x00;
int GYRO_ZOUT_OFFSET=0x00;

char GYRO_X_OFF[5];
char GYRO_Y_OFF[5];
char GYRO_Z_OFF[5];



 unsigned int maxduty1;
 unsigned int maxduty2;
 unsigned int maxduty3;
 unsigned int maxduty4;

int pwm=0;

// Lcd pinout settings
sbit LCD_RS at LATE5_bit;
sbit LCD_EN at LATE4_bit;
sbit LCD_D7 at LATE3_bit;
sbit LCD_D6 at LATE2_bit;
sbit LCD_D5 at LATE1_bit;
sbit LCD_D4 at LATE0_bit;

// Pin direction
sbit LCD_RS_Direction at TRISE5_bit;
sbit LCD_EN_Direction at TRISE4_bit;
sbit LCD_D7_Direction at TRISE3_bit;
sbit LCD_D6_Direction at TRISE2_bit;
sbit LCD_D5_Direction at TRISE1_bit;
sbit LCD_D4_Direction at TRISE0_bit;

//Register Addresses of MPU-6050
#define MPU6050_ADDRESS 0b11010010 // Address with end write bit
#define MPU6050_RA_XG_OFFS_TC 0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC 0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC 0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN 0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN 0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN 0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H 0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC 0x07
#define MPU6050_RA_YA_OFFS_H 0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC 0x09
#define MPU6050_RA_ZA_OFFS_H 0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC 0x0B
#define MPU6050_RA_XG_OFFS_USRH 0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL 0x14
#define MPU6050_RA_YG_OFFS_USRH 0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL 0x16
#define MPU6050_RA_ZG_OFFS_USRH 0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL 0x18
#define MPU6050_RA_SMPLRT_DIV 0x19
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_FF_THR 0x1D
#define MPU6050_RA_FF_DUR 0x1E
#define MPU6050_RA_MOT_THR 0x1F
#define MPU6050_RA_MOT_DUR 0x20
#define MPU6050_RA_ZRMOT_THR 0x21
#define MPU6050_RA_ZRMOT_DUR 0x22
#define MPU6050_RA_FIFO_EN 0x23
#define MPU6050_RA_I2C_MST_CTRL 0x24
#define MPU6050_RA_I2C_SLV0_ADDR 0x25
#define MPU6050_RA_I2C_SLV0_REG 0x26
#define MPU6050_RA_I2C_SLV0_CTRL 0x27
#define MPU6050_RA_I2C_SLV1_ADDR 0x28
#define MPU6050_RA_I2C_SLV1_REG 0x29
#define MPU6050_RA_I2C_SLV1_CTRL 0x2A
#define MPU6050_RA_I2C_SLV2_ADDR 0x2B
#define MPU6050_RA_I2C_SLV2_REG 0x2C
#define MPU6050_RA_I2C_SLV2_CTRL 0x2D
#define MPU6050_RA_I2C_SLV3_ADDR 0x2E
#define MPU6050_RA_I2C_SLV3_REG 0x2F
#define MPU6050_RA_I2C_SLV3_CTRL 0x30
#define MPU6050_RA_I2C_SLV4_ADDR 0x31
#define MPU6050_RA_I2C_SLV4_REG 0x32
#define MPU6050_RA_I2C_SLV4_DO 0x33
#define MPU6050_RA_I2C_SLV4_CTRL 0x34
#define MPU6050_RA_I2C_SLV4_DI 0x35
#define MPU6050_RA_I2C_MST_STATUS 0x36
#define MPU6050_RA_INT_PIN_CFG 0x37
#define MPU6050_RA_INT_ENABLE 0x38
#define MPU6050_RA_DMP_INT_STATUS 0x39
#define MPU6050_RA_INT_STATUS 0x3A
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40
#define MPU6050_RA_TEMP_OUT_H 0x41
#define MPU6050_RA_TEMP_OUT_L 0x42
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS 0x61
#define MPU6050_RA_I2C_SLV0_DO 0x63
#define MPU6050_RA_I2C_SLV1_DO 0x64
#define MPU6050_RA_I2C_SLV2_DO 0x65
#define MPU6050_RA_I2C_SLV3_DO 0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL 0x67
#define MPU6050_RA_SIGNAL_PATH_RESET 0x68
#define MPU6050_RA_MOT_DETECT_CTRL 0x69
#define MPU6050_RA_USER_CTRL 0x6A
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_PWR_MGMT_2 0x6C
#define MPU6050_RA_BANK_SEL 0x6D
#define MPU6050_RA_MEM_START_ADDR 0x6E
#define MPU6050_RA_MEM_R_W 0x6F
#define MPU6050_RA_DMP_CFG_1 0x70
#define MPU6050_RA_DMP_CFG_2 0x71
#define MPU6050_RA_FIFO_COUNTH 0x72
#define MPU6050_RA_FIFO_COUNTL 0x73
#define MPU6050_RA_FIFO_R_W 0x74
#define MPU6050_RA_WHO_AM_I 0x75



char I2CRead(char deviceAddress, char memoryAddress)
{
 unsigned char txt=0x00;

 I2C1_Start();
 I2C1_Write(deviceAddress);
 I2C1_Write(memoryAddress);
 I2C1_Restart();
 I2C1_Write(deviceAddress+1);
 txt=I2C1_Read(0);
 I2C1_Stop();

 return txt;
}

void I2CWrite(unsigned char deviceAddress, unsigned char memoryAddress, unsigned char dataSent)
{
 I2C1_Start();
 I2C1_Write(deviceAddress);
 I2C1_Write(memoryAddress);
 I2C1_Write(dataSent);
 I2C1_Stop();
}

void Gyro_Chk()
{
 text=I2CRead(MPU6050_ADDRESS,MPU6050_RA_WHO_AM_I);

 //Lcd_Out(1,1,"Gyro Recognized");
 if(text==0x68)
 {
  Lcd_Out(2,1,"Gyro Connected");
 }
 else
 {
  Lcd_Out(2,1,"Gyro ???");
 }
}

void Gyro_Config()
{
//General Config
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_SMPLRT_DIV,0x07);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_CONFIG,0x03);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_GYRO_CONFIG,0b00001000);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_ACCEL_CONFIG,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_FF_THR,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_FF_DUR,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_MOT_THR,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_MOT_DUR,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_ZRMOT_THR,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_ZRMOT_DUR,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_FIFO_EN,0x00);

 //AUX I2C config
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_MST_CTRL,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV0_ADDR,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV0_REG,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV0_CTRL,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV1_ADDR,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV1_REG,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV1_CTRL,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV2_ADDR,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV2_REG,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV2_CTRL,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV3_ADDR,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV3_REG,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV3_CTRL,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV4_ADDR,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV4_REG,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV4_CTRL,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV4_DO,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_I2C_SLV4_DI,0x00);

 //Interrupts
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_INT_PIN_CFG,0x00);
 I2CWrite(MPU6050_ADDRESS,MPU6050_RA_INT_ENABLE,0x00);

 //More Slave config
 I2CWrite(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV0_DO, 0x00);
 I2CWrite(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV1_DO, 0x00);
 I2CWrite(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV2_DO, 0x00);
 I2CWrite(MPU6050_ADDRESS, MPU6050_RA_I2C_SLV3_DO, 0x00);

 I2CWrite(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
 I2CWrite(MPU6050_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
 I2CWrite(MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_CTRL, 0x00);
 I2CWrite(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);
 I2CWrite(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0b00000010);
 I2CWrite(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
 I2CWrite(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, 0x00);

 Lcd_Out(1,1,"***** ONE  *****");
 Lcd_Out(2,1," ONE: Gyro UP   ");
}



 //////////////////////////////////////////////////////////////////////////


void Init()
{
 Lcd_Init();
 Lcd_Cmd(_LCD_CLEAR);
 Lcd_Cmd(_LCD_CURSOR_OFF);
 I2C1_Init(100000);

 TRISB.B0=1;
 ADC1_Init();
 
 TRISD=0;
 maxduty1=PWM_Init(50,1,1,2);              //1650 high, 1020 low
 maxduty2=PWM_Init(50,2,1,2);              //speed : 0 - 100
 maxduty3=PWM_Init(50,3,1,3);
 maxduty4=PWM_Init(50,4,1,3);
 PWM_Start(1);
 PWM_Start(2);
 PWM_Start(3);
 PWM_Start(4);
}

 void Lcd_Progress(int rounds)
{

for (i=0;i<rounds;i++)
 {
 Lcd_Cmd(_LCD_CLEAR);
 Lcd_Out(2,1,"**");
 Delay_ms(200);
 Lcd_Out(2,1,"****");
 Delay_ms(200);
 Lcd_Out(2,1,"******");
 Delay_ms(200);
 Lcd_Out(2,1,"********");
 Delay_ms(200);
 Lcd_Out(2,1,"**********");
 Delay_ms(200);
 Lcd_Cmd(_LCD_CLEAR);
 }
 Lcd_Out(2,5,"  The ONE");
}

void set_Motors_toDev()
{
 PWM_Set_Duty(1650,1);
 Delay_ms(10);
 PWM_Set_Duty(1650,2);
 Delay_ms(10);
 PWM_Set_Duty(1650,3);
 Delay_ms(10);
 PWM_Set_Duty(1650,4);
 Delay_ms(5000);

 PWM_Set_Duty(1020,1);
 Delay_ms(10);
 PWM_Set_Duty(1020,2);
 Delay_ms(10);
 PWM_Set_Duty(1020,3);
 Delay_ms(10);
 PWM_Set_Duty(1020,4);
 Delay_ms(2000);
 Lcd_Out(2,1," ONE: Motors UP ");
 Delay_ms(2000);
}

int getPWM_forBLDC(int speed)
{
 if(speed<0)
 {
  speed=0;
 }

 if(speed>100)
 {
  speed=100;
 }

 pwm=6.3*speed+1020;

 if(pwm<1020)
 {
  pwm=1020;
 }
 if(pwm>1650)
 {
  pwm=1650;
 }

 return pwm;
}

//Left
void run_LeftFront(int speed)
{
 PWM_Set_Duty(getPWM_forBLDC(speed),1);
}
void run_LeftRear(int speed)
{
 PWM_Set_Duty(getPWM_forBLDC(speed),2);
}

//Right
void run_RightFront(int speed)
{
 PWM_Set_Duty(getPWM_forBLDC(speed),3);
}
void run_RightRear(int speed)
{
 PWM_Set_Duty(getPWM_forBLDC(speed),4);
}

void stop_Motors()
{
  run_LeftFront(0);
  //Delay_ms(10);

  run_LeftRear(0);
  //Delay_ms(10);

  run_RightFront(0);
  //Delay_ms(10);

  run_RightRear(0);
  Delay_ms(10);
}

void QuadInit()
{
 Init();
 Lcd_Progress(4);
 Lcd_Cmd(_LCD_CLEAR);
 Gyro_Config();
 set_Motors_toDev();
}

void main() 
{
 QuadInit();
 Lcd_Out(1,1,"ONE:runnin @ 60%");
 
 while(1)
 {
  adcval=ADC1_Get_Sample(0);
  IntToStr(adcval,adc_read);

  Lcd_Out(2,1,"PWM: 1020-1650");
  
  speed_ratio=60;
  run_LeftFront(speed_ratio);
  run_LeftRear(speed_ratio);
  run_RightFront(speed_ratio);
  run_RightRear(speed_ratio);
 }
}
