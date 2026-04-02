/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dac.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdatomic.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AS5600_ADDRESS            0x36        /**< iic device address */
#define AS5600_REG_RAW_ANGLE_H        0x0C        /**< raw angle register high */
#define PI                            3.1415926f
#define Ts_100us                      0.0001f
#define Ts_PWM                        Ts_100us
#define SQRT_3                        1.732051



#define MPU9250_WHO_AM_I      0x75                                          //dv
#define MPU9250_PWR_MGMT_1    0x6B                                          //dv
#define MPU9250_USER_CTRL     0x6A                                          //dv
#define MPU9250_SMPLRT_DIV    0x19                                          //dv
#define MPU9250_CONFIG        0x1A                                          //dv
#define MPU9250_GYRO_CONFIG   0x1B                                          //dv
#define MPU9250_ACCEL_CONFIG  0x1C                                          //dv
#define MPU9250_ACCEL_CONFIG2 0x1D                                          //dv
#define MPU9250_INT_PIN_CFG   0x37                                          //dv
#define ACCEL_XOUT_H          0x3B                                          //dv
#define MPU9250_INT_ENABLE    0x38                                          //dv
#define GYRO_XOUT_H           0x43                                          //dv
#define MPU9250_SPI_WRITE     0x00                                          //dv
#define MPU9250_SPI_READ      0x80                                          //dv




#define AS5048_READ_CMD  0x8000
#define AS5048_ANGLE_REG 0x3FFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Uart5Buf[8] = {0};
uint8_t UartTransBuf[4] = {0,0,0,0};
float VdcFbk = 12.0f;
float IqMax = 0.0f,IqMin = 0.0f;
float OmegaXFbk = 0;
float OmegaYFbk = 0;
float OmegaZFbk = 0;

uint32_t UartError = 0;

uint8_t I2c2TxBuf[4] = {0,0,0,0};
uint8_t I2c2RxBuf[2] = {0,0};


typedef struct{
    float A;
    float B;
    float C;
}Var3s_s;

typedef struct{
    float alpha;
    float beta;
}Var2s_s;

typedef struct{
    float d;
    float q;
}Var2r_s;


typedef struct{
    int16_t X;
    int16_t Y;
    int16_t Z;
}MpuData_s;


typedef struct{
    MpuData_s buf[4];
    atomic_uint_fast16_t latestidx;
    uint16_t write_idx;
}OmegaRawBuf_s;
OmegaRawBuf_s OmegaRawBuf = {0};

typedef struct{
    uint16_t buf[4];
    atomic_uint_fast16_t latestidx;
    uint16_t write_idx;
}AngleRawBuf_s;
AngleRawBuf_s PitchRawBuf = {0},YawRawBuf = {0};


typedef struct{
    float PitchOmegaBuf[4];
    float YawOmegaBuf[4];
    atomic_uint_fast16_t latestidx;
    uint16_t write_idx;
}UartRxBuf_s;
UartRxBuf_s UartRxBuf = {0};

typedef struct{
    float TeCmd;
    float AngleElec;
    float OmegaMechErr;
    float OmegaMechIntg;
    float OmegaMechCmd;
    float OmegaMechFbk;
    float KpOmega;
    float KiOmega;
    float Np;
    float Rs;
    float Psif;
    Var2r_s Curr2r;
    Var2r_s Volt2r;
    Var2s_s Volt2s;
    Var3s_s Volt3s;
}SpdCtrlHandler_s;
SpdCtrlHandler_s PitchSpdCtrl = {0},YawSpdCtrl = {0};



float AngleMechPitch = 0.0f;


typedef struct{                                                             //dv
    float raw_prev;                                                         //dv
    float filted_prev;                                                      //dv
}LPFBuf;                                                                    //dv

typedef struct{
    float Sloped_prev;
}SlopeBuf;

typedef struct{
    float x_prev;
    float x_prev_prev;
    float y_prev;
    float y_prev_prev
}NotchBuf_s;


///
typedef struct
{
    float Torq;
    bool IsValid;
}TorqForGravityCompensate_s;
TorqForGravityCompensate_s TorqForGravityCompensate = {0.0f,0};

float UartTransBuf1[200] = {0};
float UartTransBuf2[200] = {0};

uint16_t Spi1TxBuf = 0xFFFF;
uint16_t Spi1RxBuf = 0;

uint8_t Spi4TxBuf[7] = {GYRO_XOUT_H | MPU9250_SPI_READ, 0,0,0,0,0,0};            //dv
uint8_t Spi4RxBuf[7] = {0};

uint8_t I2c4RxBuf[2] = {0};




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Trans_2rto2s(Var2r_s* pVar2r,Var2s_s* pVar2s,float* AngleElec);
void LPF(float* raw,float* filted,LPFBuf* s,float wc,float Ts);
void Trans_2sto3s(Var2s_s* pVar2s,Var3s_s* pVar3s);
void SVPWM2(TIM_HandleTypeDef* htim,Var2s_s* pVolt2s,Var3s_s* pVolt3s);
void ASC(TIM_HandleTypeDef* htim);
void Slope(SlopeBuf* Buf,float* raw,float* Sloped,float* SlopeMin,float* SlopeMax,float Ts);
void Sat(float* raw,float* Min,float* Max);
HAL_StatusTypeDef MPU6050_Write_Reg(uint16_t devAddr, uint8_t reg, uint8_t value);
void NotchFliter(float* raw,float* notched,NotchBuf_s* NotchBuf,float OmegaNotch,float zeta);
void MPU9250_SPI_WriteByte(uint8_t reg_addr, uint8_t data);
uint8_t MPU9250_SPI_ReadByte(uint8_t reg_addr);
uint8_t Mpu9250Init(void);
void SpdCtrlInit(void);
void SpdCtrl(SpdCtrlHandler_s* h,TIM_HandleTypeDef* htim);
void SpdCtrlStart(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  IqMin = -1.0f;
  IqMax = 1.0f;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_DAC1_Init();
  MX_I2C4_Init();
  MX_SPI4_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);

  ASC(&htim1);
  ASC(&htim2);

  HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);

  HAL_UART_Receive_IT(&huart5,Uart5Buf,8);

  Mpu9250Init();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)&Spi1TxBuf, (uint8_t*)&Spi1RxBuf, 1);

  SpdCtrlInit();
  SpdCtrlStart();

  HAL_I2C_EnableListen_IT(&hi2c2);
  HAL_I2C_Slave_Seq_Receive_IT(&hi2c2, I2c2RxBuf, 2, I2C_LAST_FRAME);  //开启从机中断接收
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1)
  {
      static uint8_t i = 0;
      i = (i+1)%2;
      if(i == 0)
      {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);

///// 250dps 需要与GyroConfig同步修改
//        OmegaXFbk = ((float)(OmegaRawBuf.buf[OmegaRawBuf.latestidx].X))/32768.f*4.3633f - 0.1157f;
//        OmegaYFbk = ((float)(OmegaRawBuf.buf[OmegaRawBuf.latestidx].Y))/32768.f*4.3633f - 0.0069f;
//        OmegaZFbk = ((float)(OmegaRawBuf.buf[OmegaRawBuf.latestidx].Z))/32768.f*4.3633f + 0.0035f;

        /// 1000dps 需要与GyroConfig同步修改
        OmegaXFbk = ((float)(OmegaRawBuf.buf[OmegaRawBuf.latestidx].X))/32768.f*17.4533f - 0.058f;
        OmegaYFbk = ((float)(OmegaRawBuf.buf[OmegaRawBuf.latestidx].Y))/32768.f*17.4533f - 0.003f;
        OmegaZFbk = ((float)(OmegaRawBuf.buf[OmegaRawBuf.latestidx].Z))/32768.f*17.4533f + 0.0017f;
        AngleMechPitch = (PitchRawBuf.buf[PitchRawBuf.latestidx]/16384.f)*2*PI;

        PitchSpdCtrl.AngleElec = ((PitchRawBuf.buf[PitchRawBuf.latestidx] - 1339 + 16384)%16384)%2340/2340.f*2*PI;
        PitchSpdCtrl.OmegaMechCmd = UartRxBuf.PitchOmegaBuf[UartRxBuf.latestidx];
        PitchSpdCtrl.OmegaMechFbk = OmegaYFbk;
        SpdCtrl(&PitchSpdCtrl,&htim1);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
      }

  }
  if(htim->Instance == TIM2)
  {
      static uint8_t i = 0;
      i = (i+1)%2;
      if(i == 0)
      {
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET);
        YawSpdCtrl.AngleElec = (((YawRawBuf.buf[YawRawBuf.latestidx]-27+4096)%4096)%585)/585.f*2*PI;
        //        /// 扫频信号给定 start
//        static float FreqInit = 2.0f;
//        static float Sweept = 0.0f;
//        Sweept = Sweept + 0.0001f;
////        TeRef = 0.035f*sinf(2*PI*(FreqInit*Sweept+2.0f*Sweept*Sweept));     //线性扫频
//        OmegaMechCmdYaw = PI*sinf(2*PI*         (      FreqInit* expf(0.2f*Sweept)   )     *Sweept);     //对数扫频
//        /// 扫频信号 end

        YawSpdCtrl.OmegaMechCmd = UartRxBuf.YawOmegaBuf[UartRxBuf.latestidx];
        YawSpdCtrl.OmegaMechFbk = -sinf(AngleMechPitch)*OmegaXFbk + cosf(AngleMechPitch)*OmegaZFbk;
        SpdCtrl(&YawSpdCtrl,&htim2);




        /// 串口发送数据 start
//        static uint8_t BufToRecord = 1;
//        static uint8_t OmegaRecordCntr = 0;
//        if(BufToRecord == 1)
//        {
//          UartTransBuf1[OmegaRecordCntr] = OmegaMechFbkYaw;
//          UartTransBuf1[OmegaRecordCntr + 100] = OmegaMechCmdYaw;
//        }
//        else
//        {
//          UartTransBuf2[OmegaRecordCntr] = OmegaMechFbkYaw;
//          UartTransBuf2[OmegaRecordCntr + 100] = OmegaMechCmdYaw;
//        }
//
//        if(OmegaRecordCntr == 99)
//        {
//          if(BufToRecord == 1)
//          {
//            BufToRecord = 2;
//            HAL_UART_Transmit_IT(&huart4,(uint8_t*)&UartTransBuf1,800);
//          }
//          else
//          {
//            BufToRecord = 1;
//            HAL_UART_Transmit_IT(&huart4,(uint8_t*)&UartTransBuf2,800);
//          }
//        }
//        OmegaRecordCntr = (OmegaRecordCntr+1)%100;
/// 串口发送数据End
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_RESET);

      }
  }
}



void NotchFliter(float* raw,float* notched,NotchBuf_s* NotchBuf,float OmegaNotch,float zeta)
{
  static float a = 0.0f;                         //dv
  static float b = 0.0f;
  static float c = 0.0f;

  static float d = 0.0f;
  static float e = 0.0f;
  static float f = 0.0f;

  a = 2.0f;
  b = -cos(Ts_PWM*OmegaNotch)*4.0f;
  c = 2.0f;

  d = zeta*sin(Ts_PWM*OmegaNotch) + 2.0f;
  e = -cos(Ts_PWM*OmegaNotch)*4.0f;
  f = -zeta*sin(Ts_PWM*OmegaNotch) + 2.0f;

  *notched = ( a*(*raw) + b*NotchBuf->x_prev + c*NotchBuf->x_prev_prev - e*NotchBuf->y_prev - f*NotchBuf->y_prev_prev)/d;        //dv
  NotchBuf->x_prev_prev = NotchBuf->x_prev;                                           //dv
  NotchBuf->y_prev_prev = NotchBuf->y_prev;                                           //dv
  NotchBuf->x_prev = *raw;                                                            //dv
  NotchBuf->y_prev = *notched;                                                        //dv
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5)
    {
      HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_10);
      memcpy(&(UartRxBuf.PitchOmegaBuf[UartRxBuf.write_idx]), Uart5Buf, 4);
      memcpy(&(UartRxBuf.YawOmegaBuf[UartRxBuf.write_idx]), &(Uart5Buf[4]), 4);
      atomic_store(&(UartRxBuf.latestidx),UartRxBuf.write_idx);
      UartRxBuf.write_idx = (UartRxBuf.write_idx+1)%4;
      UartError = HAL_UART_GetError(&huart5);
      HAL_UART_Receive_IT(&huart5,Uart5Buf,8);
    }
}


void Slope(SlopeBuf* Buf,float* raw,float* Sloped,float* SlopeMin,float* SlopeMax,float Ts)             //dv
{
  float delta = *raw - Buf->Sloped_prev;                  //dv
  if (delta/Ts > *SlopeMax)                               //dv
  {
    *Sloped = Buf->Sloped_prev + *SlopeMax*Ts;            //dv
  }
  else if(delta/Ts < *SlopeMin)                           //dv
  {
    *Sloped = Buf->Sloped_prev + *SlopeMin*Ts;            //dv
  }
  else                                                    //dv
  {
    *Sloped = *raw;                                       //dv
  }
  Buf->Sloped_prev = *Sloped;                             //dv
}


void Sat(float* raw,float* Min,float* Max)                          //dv
{
  *raw = (*raw>*Max)?*Max:((*raw<*Min)?*Min:*raw);                  //dv
}





void SVPWM2(TIM_HandleTypeDef* htim,Var2s_s* pVolt2s,Var3s_s* pVolt3s)                                      //dv
{
  Var2s_s Volt2s_Scaled;
  Volt2s_Scaled.alpha = pVolt2s->alpha/(VdcFbk/SQRT_3);                             //dv
  Volt2s_Scaled.beta = pVolt2s->beta/(VdcFbk/SQRT_3);                             //dv
  Trans_2sto3s(&Volt2s_Scaled,pVolt3s);     //dv
  float mid = 0.0f;                                                        //dv
  if(pVolt3s->A > pVolt3s->B)                                               //dv
  {
    if(pVolt3s->B > pVolt3s->C)                                             //dv
    {
      mid = pVolt3s->B;                                                 //dv
    }
    else if(pVolt3s->A > pVolt3s->C)                                        //dv
    {
      mid = pVolt3s->C;                                                 //dv
    }
    else                                                            //dv
    {
      mid = pVolt3s->A;                                                 //dv
    }
  }
  else                                                              //dv
  {
    if (pVolt3s->A > pVolt3s->C)                                            //dv
    {
      mid = pVolt3s->A;                                                 //dv
    }
    else if(pVolt3s->B > pVolt3s->C)                                        //dv
    {
      mid = pVolt3s->C;                                                 //dv
    }
    else                                                            //dv
    {
      mid = pVolt3s->B;                                                 //dv
    }
  }
  float DutyA = (pVolt3s->A+mid*0.5)*0.57735+0.5;                        //dv
  float DutyB = (pVolt3s->B+mid*0.5)*0.57735+0.5;                        //dv
  float DutyC = (pVolt3s->C+mid*0.5)*0.57735+0.5;                        //dv
  DutyA = DutyA>1 ? 1 : (DutyA<0 ? 0: DutyA);                       //dv
  DutyB = DutyB>1 ? 1 : (DutyB<0 ? 0: DutyB);                       //dv
  DutyC = DutyC>1 ? 1 : (DutyC<0 ? 0: DutyC);                       //dv


  uint32_t CompareA = ((uint32_t)((1-DutyA)*12000.0f))%12000;
  uint32_t CompareB = ((uint32_t)((1-DutyB)*12000.0f))%12000;
  uint32_t CompareC = ((uint32_t)((1-DutyC)*12000.0f))%12000;

  if(CompareA == 0)                                                          //dv
    CompareA = 1;                                                          //dv
  if(CompareB == 0)                                                          //dv
    CompareB = 1;                                                          //dv
  if(CompareC == 0)                                                         //dv
    CompareC = 1;                                                          //dv

    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_1,CompareA);                //dv
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_2,CompareB);                //dv
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_3,CompareC);                //dv
}




void Trans_2rto2s(Var2r_s* pVar2r,Var2s_s* pVar2s,float* AngleElec)         //dv
{
  float cos_theta = cosf(*AngleElec);                                             //dv
  float sin_theta = sinf(*AngleElec);                                             //dv
  pVar2s->alpha = cos_theta*(pVar2r->d) - sin_theta*(pVar2r->q);                                           //dv
  pVar2s->beta = sin_theta*(pVar2r->d) + cos_theta*(pVar2r->q);                                            //dv
}




void Trans_2sto3s(Var2s_s* pVar2s,Var3s_s* pVar3s)                //dv
{
  pVar3s->A = pVar2s->alpha;                                                                       //dv
  pVar3s->B = -0.5f*(pVar2s->alpha) + 0.5*SQRT_3*(pVar2s->beta);                                          //dv
  pVar3s->C = -0.5f*(pVar2s->alpha) - 0.5*SQRT_3*(pVar2s->beta);                                          //dv
}



void ASC(TIM_HandleTypeDef* htim)
{
  __HAL_TIM_SetCompare(htim,TIM_CHANNEL_1,1);                //dv
  __HAL_TIM_SetCompare(htim,TIM_CHANNEL_2,1);                //dv
  __HAL_TIM_SetCompare(htim,TIM_CHANNEL_3,1);                //dv
}



/*
 *低通滤波器
 */
void LPF(float* raw,float* filted,LPFBuf* s,float wc,float Ts)                        //dv
{
  *filted = Ts*wc*(s->raw_prev) + (1-Ts*wc)*(s->filted_prev);                       //dv
  s->raw_prev = *raw;                                                               //dv
  s->filted_prev = *filted;                                                         //dv
}



void MPU9250_SPI_WriteByte(uint8_t reg_addr, uint8_t data)                                      //dv
{
  uint8_t tx_data[2] = {reg_addr & ~MPU9250_SPI_READ, data};                          //dv
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);                //dv
  HAL_SPI_Transmit(&hspi4, tx_data, 2, HAL_MAX_DELAY);                //dv
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);                  //dv
}



uint8_t MPU9250_SPI_ReadByte(uint8_t reg_addr)                                                                  //dv
{
  uint8_t tx_data[2] = {reg_addr | MPU9250_SPI_READ, 0x00};                                           //dv
  uint8_t rx_data[2] = {0};                                                                                   //dv

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);                               // 拉低CS
  HAL_SPI_TransmitReceive(&hspi4, tx_data, rx_data, 2, HAL_MAX_DELAY);     //dv
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);                                 //dv

  return rx_data[1];                                                                                          //dv
}


uint8_t Mpu9250Init(void)
{
  uint8_t who_am_i = 0;                                                                                   //dv
  MPU9250_SPI_WriteByte(MPU9250_PWR_MGMT_1, 0x80);                                          //dv 重置
  HAL_Delay(100);                                                                                   //dv 延迟100ms
  MPU9250_SPI_WriteByte(MPU9250_PWR_MGMT_1, 0x01);                                           //dv 自动选择最佳时钟源
  HAL_Delay(10);                                                                                      //dv 延迟10ms
  MPU9250_SPI_WriteByte(MPU9250_USER_CTRL, 0x10);                                             //dv 禁用IIC，启用SPI
  MPU9250_SPI_WriteByte(MPU9250_SMPLRT_DIV, 0x00);                                            //dv
  MPU9250_SPI_WriteByte(MPU9250_CONFIG, 0x00);                                                //dv
  MPU9250_SPI_WriteByte(MPU9250_GYRO_CONFIG, 0x19);                                           //dv
  MPU9250_SPI_WriteByte(MPU9250_ACCEL_CONFIG, 0x10);                                          //dv
  MPU9250_SPI_WriteByte(MPU9250_ACCEL_CONFIG2, 0x08);                                         //dv
  MPU9250_SPI_WriteByte(MPU9250_INT_ENABLE, 0x00);                                            //dv 禁用中断
  MPU9250_SPI_WriteByte(MPU9250_INT_PIN_CFG, 0x10);
  who_am_i = MPU9250_SPI_ReadByte(MPU9250_WHO_AM_I);                                               //dv
  if (who_am_i == 0x71 || who_am_i == 0x73) {                                                               //dv
    return 0;                                                                                              //dv
  } else {                                                                                                    //dv
    return 1;                                                                                               //dv
  }
}



void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)                                                              //dv
{
  if(hspi->Instance == SPI4)                                                                                      //dv
  {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    //角速度数据处理

    OmegaRawBuf.buf[OmegaRawBuf.write_idx].X = (int16_t)((Spi4RxBuf[1] << 8) | Spi4RxBuf[2]);
    OmegaRawBuf.buf[OmegaRawBuf.write_idx].Y = (int16_t)((Spi4RxBuf[3] << 8) | Spi4RxBuf[4]);
    OmegaRawBuf.buf[OmegaRawBuf.write_idx].Z = (int16_t)((Spi4RxBuf[5] << 8) | Spi4RxBuf[6]);
    atomic_store(&(OmegaRawBuf.latestidx),OmegaRawBuf.write_idx);
    OmegaRawBuf.write_idx = (OmegaRawBuf.write_idx+1)%4;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)&Spi1TxBuf, (uint8_t*)&Spi1RxBuf, 1);

  }
  if(hspi->Instance == SPI1)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

    // 角度数据处理
    PitchRawBuf.buf[PitchRawBuf.write_idx] = ((int16_t)(Spi1RxBuf & 0x3FFF) + 1227)%16384;
    atomic_store(&(PitchRawBuf.latestidx),PitchRawBuf.write_idx);
    PitchRawBuf.write_idx = (PitchRawBuf.write_idx + 1)%4;

    HAL_I2C_Mem_Read_IT(&hi2c4,AS5600_ADDRESS << 1,AS5600_REG_RAW_ANGLE_H,1,I2c4RxBuf,2);
  }
}



void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance == I2C4)
  {
    YawRawBuf.buf[YawRawBuf.write_idx] = ((uint16_t)((I2c4RxBuf[0] & 0xF) << 8) | I2c4RxBuf[1]);
    atomic_store(&(YawRawBuf.latestidx),YawRawBuf.write_idx);
    YawRawBuf.write_idx = (YawRawBuf.write_idx+1)%4;

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);                                     // 跳转到SPI1读取角度
    HAL_SPI_TransmitReceive_IT(&hspi4, Spi4TxBuf, Spi4RxBuf, 7);
  }
}


void SpdCtrlInit()
{
  PitchSpdCtrl.KpOmega = 0.0069f;
  PitchSpdCtrl.KiOmega = 0.1597f;
  PitchSpdCtrl.Np = 7;
  PitchSpdCtrl.Rs = 3.76f;
  PitchSpdCtrl.Psif = 0.001761f;

  YawSpdCtrl.KpOmega = 0.01403f;
  YawSpdCtrl.KiOmega = 0.2006f;
  YawSpdCtrl.Np = 7;
  YawSpdCtrl.Rs = 2.823275f;
  YawSpdCtrl.Psif = 0.003418f;
}



void SpdCtrl(SpdCtrlHandler_s* h,TIM_HandleTypeDef* htim)
{
//        OmegaMechCmdPitch = 0;
  h->OmegaMechErr = h->OmegaMechCmd - h->OmegaMechFbk;
  h->OmegaMechIntg = h->OmegaMechIntg + h->OmegaMechErr*Ts_PWM;
  h->TeCmd = h->KpOmega*h->OmegaMechErr + h->KiOmega*h->OmegaMechIntg;
  h->Curr2r.d = 0.0f;
  h->Curr2r.q = h->TeCmd/(1.5f*h->Np*h->Psif);
  h->Volt2r.d = h->Rs*h->Curr2r.d;
  h->Volt2r.q = h->Rs*h->Curr2r.q;
  h->Volt2r.q = fmax(fmin(h->Volt2r.q,VdcFbk/SQRT_3),-VdcFbk/SQRT_3);                 /// SVPWM前对电压进行限幅
  Trans_2rto2s(&(h->Volt2r),&(h->Volt2s),&h->AngleElec);
  SVPWM2(htim,&(h->Volt2s),&(h->Volt3s));
}


void SpdCtrlStart(void)
{
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);     //dv 使能驱动器1
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);     //dv 使能驱动器2
}






void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if(hi2c->Instance == I2C2)
    {
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

        if(TransferDirection == I2C_DIRECTION_RECEIVE)      //需要发送
        {
            // 数据处理
            uint16_t temp = (PitchRawBuf.buf[PitchRawBuf.latestidx])%16384;
            I2c2TxBuf[0] = (temp >> 8) & 0xFF;
            I2c2TxBuf[1] = temp & 0xFF;
            temp = YawRawBuf.buf[YawRawBuf.latestidx];
            I2c2TxBuf[2] = (temp >> 8) & 0xFF;
            I2c2TxBuf[3] = temp & 0xFF;
            //
            HAL_I2C_Slave_Seq_Transmit_IT(&hi2c2, I2c2TxBuf, 4, I2C_FIRST_FRAME);
//		HAL_I2C_Slave_Transmit_IT(&hi2c1, (tx_buff + rx_buff[0]), 1);   //该函数发送会使总线死掉
        }
        else if(TransferDirection == I2C_DIRECTION_TRANSMIT)//需要接收
        {
            //数据处理
            //
            //
            HAL_I2C_Slave_Seq_Receive_IT(&hi2c2, I2c2RxBuf, 2, I2C_FIRST_AND_NEXT_FRAME);
//		HAL_I2C_Slave_Receive_IT(&hi2c1, rx_buff, 1);                   //该函数接受会使总线死掉
        }
        else
        {}
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)  //监听中断回调
{
    if(hi2c->Instance == I2C2)
    {
        HAL_I2C_EnableListen_IT(hi2c); // Restart
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)  //全部发送完成回调
{
    if(hi2c->Instance == I2C2)
    {
        // 数据处理
        //
        //
        HAL_I2C_Slave_Seq_Transmit_IT(&hi2c2, I2c2TxBuf, 2, I2C_FIRST_FRAME);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)  //全部接收完成回调
{
    if(hi2c->Instance == I2C2)
    {
        // 数据处理
        if(TorqForGravityCompensate.IsValid == 0)
        {
            TorqForGravityCompensate.Torq = ((float)((int16_t)(I2c2RxBuf[0] << 8 | I2c2RxBuf[1])))/800000.0f;
            TorqForGravityCompensate.IsValid = 1;
        }
        HAL_I2C_Slave_Seq_Receive_IT(&hi2c2, I2c2RxBuf, 2, I2C_FIRST_AND_NEXT_FRAME);
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//      ASC(&htim1);
//      ASC(&htim2);
//  }

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
