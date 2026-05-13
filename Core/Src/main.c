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
#include "dma.h"
#include "quadspi.h"
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
#define AS5600_ADDRESS                0x36        /**< iic device address */
#define AS5600_REG_RAW_ANGLE_H        0x0C        /**< raw angle register high */
#define PI                            3.1415926f
#define Ts_100us                      0.0001f
#define Ts_2ms                        0.002f
#define Ts_PWM                        Ts_100us
#define SQRT_3                        1.732051f
#define Ts_SpdCtrl                    0.0001f



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
#define TRUE                  1
#define FALSE                 0




#define CALIB_PARAM_BASE_ADDR 0x000000



#define UART5_RXBUF_LEN       100                                           //dv


#define AS5048_READ_CMD  0x8000
#define AS5048_ANGLE_REG 0x3FFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__attribute__((section(".RAM"))) uint8_t Uart5RXBuf[UART5_RXBUF_LEN] = {0};
float VdcFbk = 12.0f;

typedef struct{
    float A;
    float B;
    float C;
}Var3s_t;

typedef struct{
    float alpha;
    float beta;
}Var2s_t;

typedef struct{
    float d;
    float q;
}Var2r_t;

typedef struct{
    float In;
    float In_1delay;
    float In_2delay;
    float In_3delay;
    float In_4delay;
    float Out;
    float Out_1delay;
    float Out_2delay;
    float Out_3delay;
    float Out_4delay;
}Compensator_t;

typedef struct{
    float a0;
    float a1;
    float a2;
    float a3;
    float a4;
    float b1;
    float b2;
    float b3;
    float b4;
}CompCoeff_t;

typedef enum{
  SpdMode,
  VoltMode,
  CurrMode,
  AscMode,
}CtrlMode_e;


typedef struct{
    float AngleElec;
    float OmegaMechCmd;
    float OmegaMechFbk;
    float OmegaMechIntg;
    float OmegaMechIntgMax;
    float KpOmega;
    float KiOmega;
    float Np;
    float Rs;
    float Psif;
    Var2r_t Curr2r;
    Var2r_t Curr2r_Prev;
    Var2r_t Volt2r;
    Var2s_t Volt2s;
    Var3s_t Volt3s;
    Compensator_t Comp;
    CompCoeff_t CompCoeff;
    GPIO_TypeDef* GpioPort;
    TIM_HandleTypeDef* htim;
    uint16_t SpdLoopIRQ;
    uint16_t FocIRQ;
    uint16_t EnbPin;
    CtrlMode_e Mode;
    bool NeedInit;
}MotorCtrlHandler_t;
MotorCtrlHandler_t PitchChanel = {0},RollChanel = {0};

typedef struct{                                                             //dv
    float filted;                                                         //dv
    float filted_prev;                                                      //dv
    float alpha;
}LpfHandler_t;                                                                    //dv

typedef struct{
    float x_prev;
    float x_prev_prev;
    float y_prev;
    float y_prev_prev
}NotchBuf_t;

typedef struct
{
    float Iq[100];
    float Omega[100];
}gimbal_data_t;

typedef struct
{
    uint16_t header;
    uint16_t length;
    gimbal_data_t data;
}Uart5TxFrame_t;
//__attribute__((section(".RAM"))) Uart5TxFrame_t Uart5TxFrame[2];

// ForSweep Start
__attribute__((section(".RAM"))) gimbal_data_t SweepData[2];
// ForSweep End

typedef struct{
    uint16_t PitchElecOffSet;
    uint16_t PitchMechOffSet;
    uint16_t PitchMechOffsetForIMU;
    uint16_t RollElecOffSet;
    uint16_t RollMechOffSet;
    float OmegaXOffset;
    float OmegaYOffset;
    float OmegaZOffset;
    uint8_t IsValid;
}CalibParams_t;
CalibParams_t CalibParams = {.PitchElecOffSet = 11844,.PitchMechOffsetForIMU = 13700,.PitchMechOffSet = 200,.RollElecOffSet = 14796,.RollMechOffSet = 14127,.OmegaXOffset = -0.0115f,.OmegaYOffset = 0.0063f,.OmegaZOffset = 0.0115f};

typedef enum{
    Chirp,
    Sin,
    Step,
}SignalType_e;

typedef struct{
    float Ts;
    float t;
    float FreqInit;
    float Magnitude;
    float FreqSlope;
    float Signal;
    SignalType_e Type;
}SignalHandler_t;

typedef struct{
    float Prev;
    float Now;
    float filted;
}Buf_t;

typedef struct{
    uint16_t Rx;
    uint16_t Tx;
    uint16_t AngleMechRaw;
    uint16_t AngleElecRaw;
    uint16_t AngleMechRawForIMU;
    Buf_t AngleElec;
    Buf_t AngleMech;
    Buf_t AngleMechForIMU;
}AngleSensor_t;
AngleSensor_t PitchData = {.Tx = 0xFFFF},RollData = {.Tx = 0xFFFF};

typedef struct{
    uint8_t Tx[7];
    uint8_t Rx[7];
    Buf_t X;
    Buf_t Y;
    Buf_t Z;
    Buf_t Roll;
    LpfHandler_t XLpf;
    LpfHandler_t YLpf;
    LpfHandler_t ZLpf;
}OmegaSensor_t;
OmegaSensor_t OmegaData = {.Tx={GYRO_XOUT_H | MPU9250_SPI_READ, 0,0,0,0,0,0}};

SignalHandler_t CurrSigForCalib = {0};

typedef struct{
    uint16_t Header;
    uint16_t Length;
    uint16_t PitchAngleMech;
    uint16_t RollAngleMech;
}AngleData_t;
__attribute__((section(".RAM"))) AngleData_t AngleData = {.Header = 0x5A89,.Length = 8,};

typedef struct{
    uint16_t Header;
    uint16_t Length;
    uint64_t TimeStamp;
    float PitchAngleMech;
    float PitchOmegaCmd;
    float PitchOmegaFbk;
    float RollAngleMech;
    float RollOmegaCmd;
    float RollOmegaFbk;
}LogData_t;
__attribute__((section(".RAM"))) LogData_t LogData = {.Header = 0x5A89,.Length = 36};

uint8_t Q128Ret = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Trans_2rto2s(Var2r_t* pVar2r,Var2s_t* pVar2s,float* AngleElec);
void Lpf(float* raw,LpfHandler_t* h);
void Trans_2sto3s(Var2s_t* pVar2s,Var3s_t* pVar3s);
void SVPWM2(TIM_HandleTypeDef* htim,Var2s_t* pVolt2s,Var3s_t* pVolt3s);
HAL_StatusTypeDef MPU6050_Write_Reg(uint16_t devAddr, uint8_t reg, uint8_t value);
void NotchFliter(float* raw,float* notched,NotchBuf_t* NotchBuf,float OmegaNotch,float zeta);
void MPU9250_SPI_WriteByte(uint8_t reg_addr, uint8_t data);
uint8_t MPU9250_SPI_ReadByte(uint8_t reg_addr);
uint8_t Mpu9250Init(void);
void SetMotorCtrlParams(void);
void ParamsCalib(void);
void MotorCtrl_SpdMode_Start(MotorCtrlHandler_t* h);
void MotorCtrl_HdwrEnb(MotorCtrlHandler_t* h);
void MotorCtrl_HdwrDisab(MotorCtrlHandler_t* h);
float SignalGenerate(SignalHandler_t* h);

void LpfInit(LpfHandler_t* h,float Ts,float CutoffFreq);
void SetLpfParams(float Ts,float CutoffFreq);
void SensorReadingStart(void);
void SpdCtrl(MotorCtrlHandler_t* h);
void CurrCtrl(MotorCtrlHandler_t* h);
void SpdModeInit(MotorCtrlHandler_t* h);
void MotorCtrl_Break(MotorCtrlHandler_t* h);
void MotorCtrl_CurrMode_Start(MotorCtrlHandler_t* h);
void MotorCtrl_AscMode_Start(MotorCtrlHandler_t* h);
void MotorCtrl_VoltMode_Start(MotorCtrlHandler_t* h);
void Asc(TIM_HandleTypeDef* htim);

uint8_t NeedCalib(void);
uint8_t W25Q128_WaitBusy(void);
uint8_t W25Q128_EraseSector(uint32_t sector_addr);
uint8_t W25Q128_Read(uint32_t addr, uint8_t *pData, uint32_t len);
uint8_t W25Q128_Write(uint32_t addr, uint8_t *pData, uint32_t len);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_QUADSPI_Init();
  MX_TIM6_Init();
  MX_SPI3_Init();
  MX_SPI6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart5,Uart5RXBuf,UART5_RXBUF_LEN);
  Mpu9250Init();

  SetMotorCtrlParams();                                                                 //dv 电机控制参数设置
//  ParamsCalib();                                                                      //dv 进行参数标定
//  if(NeedCalib())                                                                     //dv 检测是否需要标定
//  {
//      ParamsCalib();                                                                  //dv 进行参数标定
//  }

  SetLpfParams(Ts_SpdCtrl,100.f);                                         //dv 设置低通滤波器参数
  SetMotorCtrlParams();
  MotorCtrl_HdwrEnb(&PitchChanel);
  MotorCtrl_SpdMode_Start(&PitchChanel);
  MotorCtrl_HdwrEnb(&RollChanel);
  MotorCtrl_AscMode_Start(&RollChanel);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)           //dv 定时器更新中断
{
  // 俯仰通道电流环
  if(htim->Instance == TIM1)                                          //dv TIM1更新中断
  {
    if((htim->Instance->CR1 & TIM_CR1_DIR) == 0)                      //dv 判断中断是否由三角波底部产生
    {
      CurrCtrl(&PitchChanel);                                      //dv 进行电流控制
    }
  }

  // 滚转通道电流环
  if(htim->Instance == TIM8)                                          //dv TIM8更新中断
  {
    if((htim->Instance->CR1 & TIM_CR1_DIR) == 0)                      //dv 判断中断是否由三角波底部生成
    {
      CurrCtrl(&RollChanel);                                       //dv 进行电流控制
    }
  }

  //数据采集器
  if(htim->Instance == TIM6)                                          //dv TIM6更新中断、用于传感器数据采集
  {
    // 俯仰与滚转通道角度数据
    PitchData.AngleElec.Prev = PitchData.AngleElec.Now;               //dv 将上一时刻电角度存入Prev
    PitchData.AngleMech.Prev = PitchData.AngleMech.Now;
    PitchData.AngleMechForIMU.Prev = PitchData.AngleMechForIMU.Now;   //dv 将上一时刻机械角度存入Prev
    RollData.AngleElec.Prev = RollData.AngleElec.Now;                 //dv 将上一时刻电角度存入Prev
    RollData.AngleMech.Prev = RollData.AngleMech.Now;                 //dv 将上一时刻机械角度存入Prev

    Lpf(&OmegaData.X.Now,&(OmegaData.XLpf));                  //dv 对X轴角速度进行滤波
    Lpf(&OmegaData.Y.Now,&(OmegaData.YLpf));                  //dv 对Y轴角速度进行滤波
    Lpf(&OmegaData.Z.Now,&(OmegaData.ZLpf));                  //dv 对Z轴角速度进行滤波

    OmegaData.X.Prev = OmegaData.XLpf.filted;                         //dv 滤波值存入Prev
    OmegaData.Y.Prev = OmegaData.YLpf.filted;                         //dv 滤波值存入Prev
    OmegaData.Z.Prev = OmegaData.ZLpf.filted;                         //dv 滤波值存入Prev
    OmegaData.Roll.Prev = OmegaData.X.Prev*cosf(-PitchData.AngleMechForIMU.Prev) - OmegaData.Z.Prev*sinf(-PitchData.AngleMechForIMU.Prev);

    SensorReadingStart();                                             //dv 开始房前周期的DMA读取，读取后在中断中对数据进行处理，处理结果存入Now中



//      // 方波信号 Start
//      static uint64_t k1 = 0;
//      static float Signal = 0.28f;
//      k1 = (k1+1)%5000;
//      if(k1==0)
//      {
//          Signal = Signal*(-1.f);
//      }
//      // 方波信号 End


//      // 扫频信号生成 start
//      static float t = 0.00f;
//      static float FreqInit = 0.05f;
//      static float Mag = 0.05f;
//      static float Signal = 0;
//      static float FreqSlope = 0.00f;
//      t = t + Ts_PWM;
//      Signal =0.0f + Mag*sinf(2*PI*(FreqInit* expf(FreqSlope*t))*t);
//      // 扫频信号生成 End
//      PitchChanel.OmegaMechCmd = Signal;


//      PitchChanel.OmegaMechCmd = Signal;

    PitchChanel.OmegaMechFbk = OmegaData.Y.Prev;                      //dv 前一时刻的角速度反馈赋值给Pitch通道
    PitchChanel.AngleElec = PitchData.AngleElec.Prev;                 //dv 前一时刻的电角度反馈赋值给Pitch通道
    SpdCtrl(&PitchChanel);                                         //dv 进行速度控制

    RollChanel.OmegaMechFbk = OmegaData.Roll.Prev;                    //dv 前一时刻的角速度反馈赋值给Roll通道
    RollChanel.AngleElec = RollData.AngleElec.Prev;                   //dv 前一时刻的电角度反馈赋值给Roll通道
    SpdCtrl(&RollChanel);                                          //dv 进行速度控制


//      // Uart数据传输 Start
//      static uint32_t k = 0;
//      static uint8_t idx = 0;
//      SweepData[idx].Iq[k] = PitchChanel.OmegaMechCmd;
//      SweepData[idx].Omega[k] = PitchChanel.OmegaMechFbk;
//      k = (k+1)%100;
//      if(k == 0)
//      {
//          HAL_UART_Transmit_DMA(&huart5,(uint8_t*)(&(SweepData[idx])),sizeof(SweepData[idx]));
//          idx = (idx+1)%2;
//      }
//      // Uart数据传输 End

  }

  // 信号生成器
  if(htim->Instance == TIM7)                  // 信号生成器
  {
    PitchChanel.Curr2r.q = SignalGenerate(&CurrSigForCalib);
  }
}


// 速度控制
void SpdCtrl(MotorCtrlHandler_t* h)                                                 //dv 速度控制、内部带有初始化逻辑
{
  if(h->Mode == SpdMode)                                                            //dv 如果为速度模式
  {
    if(h->NeedInit == true)                                                         //dv 首次进入速度模式时，需进行初始化
    {
      SpdModeInit(h);                                                               //dv 初始化
      h->NeedInit = false;                                                          //dv 初始化标志位清零
    }
    h->Curr2r_Prev.d = h->Curr2r.d;                                                 //dv 电流指令存储
    h->Curr2r_Prev.q = h->Curr2r.q;                                                 //dv 电流指令存储

    h->Comp.In = h->OmegaMechCmd - h->OmegaMechFbk;                                 //dv 计算补偿器输入

//    // 开环输入 start
//    h->Comp.In = h->OmegaMechCmd;
//    // 开环输入End

    h->Comp.Out = h->CompCoeff.a0*h->Comp.In \
                + h->CompCoeff.a1*h->Comp.In_1delay \
                + h->CompCoeff.a2*h->Comp.In_2delay \
                + h->CompCoeff.a3*h->Comp.In_3delay \
                + h->CompCoeff.a4*h->Comp.In_4delay \
                - h->CompCoeff.b1*h->Comp.Out_1delay \
                - h->CompCoeff.b2*h->Comp.Out_2delay \
                - h->CompCoeff.b3*h->Comp.Out_3delay \
                - h->CompCoeff.b4*h->Comp.Out_4delay;                               //dv 计算补偿器输出
    h->OmegaMechIntg = h->Comp.Out*Ts_SpdCtrl + h->OmegaMechIntg;                   //dv 计算积分值
    h->OmegaMechIntg = fmaxf(fminf(h->OmegaMechIntgMax,h->OmegaMechIntg),-h->OmegaMechIntgMax);             //dv 积分值限幅
    h->Curr2r.d = 0.0f;                                                             //dv 计算d轴电流指令
    h->Curr2r.q = h->KpOmega*h->Comp.Out + h->KiOmega*h->OmegaMechIntg;             //dv 计算转矩指令

    h->Comp.In_4delay = h->Comp.In_3delay;
    h->Comp.In_3delay = h->Comp.In_2delay;                                          //dv 存储补偿器输入
    h->Comp.In_2delay = h->Comp.In_1delay;                                          //dv
    h->Comp.In_1delay = h->Comp.In;                                                 //dv
    h->Comp.Out_4delay = h->Comp.Out_3delay;
    h->Comp.Out_3delay = h->Comp.Out_2delay;                                        //dv 存储补偿器输出
    h->Comp.Out_2delay = h->Comp.Out_1delay;                                        //dv
    h->Comp.Out_1delay = h->Comp.Out;                                               //dv
  }
  else                                                                              //dv 如果为非速度模式
  {
    h->NeedInit = true;                                                             //dv 设置初始化标志，用于下次速度模式的初始化
  }
}

void SpdModeInit(MotorCtrlHandler_t* h)                             //dv 速度模式初始化
{
  h->Comp.Out = 0.f;                                                //dv 补偿器输出清零
  h->Comp.Out_1delay = 0.f;                                         //dv 补偿器历史输出清零
  h->Comp.Out_2delay = 0.f;                                         //dv
  h->Comp.Out_3delay = 0.f;                                         //dv
  h->Comp.In = 0.f;                                                 //dv 补偿器输入清零
  h->Comp.In_1delay = 0.f;                                          //dv 补偿器历史输入清零
  h->Comp.In_2delay = 0.f;                                          //dv
  h->Comp.In_3delay = 0.f;                                          //dv
  h->OmegaMechIntg = 0.f;                                           //dv 积分器清零
  h->Curr2r.d = 0.f;                                                //？？？
  h->Curr2r.q = 0.f;                                                //？？？
}

void SensorReadingStart(void)                                                                                                   //dv 开始读取所有传感器值
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);                                                  //dv 拉低片选引脚
  HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)&(PitchData.Tx), (uint8_t*)&(PitchData.Rx), 1);       //dv SPI1开始DMA传输

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);                                                  //dv 拉低片选引脚
  HAL_SPI_TransmitReceive_IT(&hspi3, (uint8_t*)&(RollData.Tx), (uint8_t*)&(RollData.Rx), 1);         //dv SPI3开始DMA传输

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);                                                   //dv 拉低片选引脚
  HAL_SPI_TransmitReceive_IT(&hspi6, (uint8_t*)&(OmegaData.Tx), (uint8_t*)&(OmegaData.Rx), 7);       //dv SPI6开始DMA传输
}

// SPI中断回调函数
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)                                                //dv SPI中断回调函数（俯仰角度处理）
{
  if(hspi->Instance == SPI1)                                                                          //dv SPI1中断回调
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);                          //dv 拉高片选引脚
    uint16_t PitchRaw = (PitchData.Rx) & 0x3FFF;                                                      //dv 读取14位编码器原始值
    PitchData.AngleElecRaw = (PitchRaw - CalibParams.PitchElecOffSet + 0x4000)%0x4000;                //dv 消除零偏（电）
    PitchData.AngleElec.Now = ((float)(PitchData.AngleElecRaw%2340))/2340.f*2*PI;                     //dv 计算电角度
    PitchData.AngleMechRaw = (PitchRaw - CalibParams.PitchMechOffSet - CalibParams.PitchMechOffsetForIMU + 0x8000)%0x4000;
    PitchData.AngleMech.Now = ((float)(RollData.AngleMechRaw)/16384.f)*2*PI;
    PitchData.AngleMechRawForIMU = (PitchRaw - CalibParams.PitchMechOffsetForIMU + 0x4000)%0x4000;    //dv 消除零偏（机械）
    PitchData.AngleMechForIMU.Now = ((float)(PitchData.AngleMechRawForIMU)/16384.f)*2*PI;             //dv 计算机械角度
  }
  if(hspi->Instance == SPI3)                                                                          //dv SPI3中断回调函数（滚转角度处理）
  {
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);                          //dv 拉高片选引脚
    uint16_t RollRaw = (RollData.Rx) & 0x3FFF;                                                        //dv 读取14位编码器原始值
    RollData.AngleElecRaw = (RollRaw - CalibParams.RollElecOffSet + 0x4000)%0x4000;                   //dv 消除零偏（电）
    RollData.AngleElec.Now = ((float)(RollData.AngleElecRaw%2340))/2340.f*2*PI;                       //dv 计算电角度
    RollData.AngleMechRaw = (RollRaw - CalibParams.RollMechOffSet + 0x4000)%0x4000;                   //dv 消除零偏（机械）
    RollData.AngleMech.Now = ((float)(RollData.AngleMechRaw)/16384.f)*2*PI;                           //dv 滚转机械角度
  }
  if(hspi->Instance == SPI6)                                                                          //dv SPI6中断函数
  {
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);                          //dv 拉高片选引脚
    int16_t OmegaXRaw = (int16_t)((OmegaData.Rx[1] << 8) | OmegaData.Rx[2]);                          //dv 读取16位原始值
    int16_t OmegaYRaw = (int16_t)((OmegaData.Rx[3] << 8) | OmegaData.Rx[4]);                          //dv 读取16位原始值
    int16_t OmegaZRaw = (int16_t)((OmegaData.Rx[5] << 8) | OmegaData.Rx[6]);                          //dv 读取16位原始值

    OmegaData.X.Now = ((float)OmegaXRaw)/32768.f*17.4533f - CalibParams.OmegaXOffset;                 //dv 减去零偏
    OmegaData.Y.Now = ((float)OmegaYRaw)/32768.f*17.4533f - CalibParams.OmegaYOffset;                 //dv 减去零偏
    OmegaData.Z.Now = ((float)OmegaZRaw)/32768.f*17.4533f - CalibParams.OmegaZOffset;                 //dv 减去零偏
  }
}



void NotchFliter(float* raw,float* notched,NotchBuf_t* NotchBuf,float OmegaNotch,float zeta)
{
  static float a = 0.0f;                         //dv
  static float b = 0.0f;
  static float c = 0.0f;

  static float d = 0.0f;
  static float e = 0.0f;
  static float f = 0.0f;

  a = 2.0f;
  b = -cosf(Ts_PWM*OmegaNotch)*4.0f;
  c = 2.0f;

  d = zeta*sinf(Ts_PWM*OmegaNotch) + 2.0f;
  e = -cosf(Ts_PWM*OmegaNotch)*4.0f;
  f = -zeta*sinf(Ts_PWM*OmegaNotch) + 2.0f;

  *notched = ( a*(*raw) + b*NotchBuf->x_prev + c*NotchBuf->x_prev_prev - e*NotchBuf->y_prev - f*NotchBuf->y_prev_prev)/d;        //dv
  NotchBuf->x_prev_prev = NotchBuf->x_prev;                                           //dv
  NotchBuf->y_prev_prev = NotchBuf->y_prev;                                           //dv
  NotchBuf->x_prev = *raw;                                                            //dv
  NotchBuf->y_prev = *notched;                                                        //dv
}

void SVPWM2(TIM_HandleTypeDef* htim,Var2s_t* pVolt2s,Var3s_t* pVolt3s)                                      //dv
{
  Var2s_t Volt2s_Scaled;
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
  float DutyA = (pVolt3s->A+mid*0.5f)*0.57735f+0.5f;                        //dv
  float DutyB = (pVolt3s->B+mid*0.5f)*0.57735f+0.5f;                        //dv
  float DutyC = (pVolt3s->C+mid*0.5f)*0.57735f+0.5f;                        //dv
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

  if(htim->Instance == TIM1)                                          //dv timer1 采用Chanel1、2、3
  {
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_1,CompareA);                //dv
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_2,CompareB);                //dv
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_3,CompareC);                //dv
  }
  if(htim->Instance == TIM8)                                          //dv timer8 采用chanel2、3、4
  {
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_2,CompareA);                //dv
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_3,CompareB);                //dv
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_4,CompareC);                //dv
  }

}




void Trans_2rto2s(Var2r_t* pVar2r,Var2s_t* pVar2s,float* AngleElec)         //dv
{
  float cos_theta = cosf(*AngleElec);                                             //dv
  float sin_theta = sinf(*AngleElec);                                             //dv
  pVar2s->alpha = cos_theta*(pVar2r->d) - sin_theta*(pVar2r->q);                                           //dv
  pVar2s->beta = sin_theta*(pVar2r->d) + cos_theta*(pVar2r->q);                                            //dv
}




void Trans_2sto3s(Var2s_t* pVar2s,Var3s_t* pVar3s)                //dv
{
  pVar3s->A = pVar2s->alpha;                                                                       //dv
  pVar3s->B = -0.5f*(pVar2s->alpha) + 0.5f*SQRT_3*(pVar2s->beta);                                          //dv
  pVar3s->C = -0.5f*(pVar2s->alpha) - 0.5f*SQRT_3*(pVar2s->beta);                                          //dv
}



void Asc(TIM_HandleTypeDef* htim)
{
  if(htim->Instance == TIM1)
  {
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_1,1);                //dv
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_2,1);                //dv
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_3,1);                //dv
  }
  if(htim->Instance == TIM8)
  {
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_2,1);                //dv
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_3,1);                //dv
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_4,1);                //dv
  }
}



// 低通滤波器
void Lpf(float* raw,LpfHandler_t* h)                                            //dv 低通滤波器（前向欧拉）
{
  h->filted = (h->alpha*(*raw) + h->filted_prev)/(1+h->alpha);                  //dv 计算当前滤波值
  h->filted_prev = h->filted;                                                   //dv 存储当前滤波值
}



void MPU9250_SPI_WriteByte(uint8_t reg_addr, uint8_t data)                                      //dv
{
  uint8_t tx_data[2] = {reg_addr & ~MPU9250_SPI_READ, data};                          //dv
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);                //dv
  HAL_SPI_Transmit(&hspi6, tx_data, 2, HAL_MAX_DELAY);                //dv
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);                  //dv
}


uint8_t MPU9250_SPI_ReadByte(uint8_t reg_addr)                                                                //dv
{
  uint8_t tx_data[2] = {reg_addr | MPU9250_SPI_READ, 0x00};                                           //dv
  uint8_t rx_data[2] = {0};                                                                                   //dv

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);                               // 拉低CS
  HAL_SPI_TransmitReceive(&hspi6, tx_data, rx_data, 2, HAL_MAX_DELAY);     //dv
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);                                 //dv

  return rx_data[1];                                                                                          //dv
}


uint8_t Mpu9250Init(void)
{
  uint8_t who_am_i = 0;                                                                                     //dv
  MPU9250_SPI_WriteByte(MPU9250_PWR_MGMT_1, 0x80);                                            //dv 重置
  HAL_Delay(100);                                                                                     //dv 延迟100ms
  MPU9250_SPI_WriteByte(MPU9250_PWR_MGMT_1, 0x01);                                            //dv 自动选择最佳时钟源
  HAL_Delay(10);                                                                                      //dv 延迟10ms
  MPU9250_SPI_WriteByte(MPU9250_USER_CTRL, 0x10);                                             //dv 禁用IIC，启用SPI
  MPU9250_SPI_WriteByte(MPU9250_SMPLRT_DIV, 0x00);                                            //dv
  MPU9250_SPI_WriteByte(MPU9250_CONFIG, 0x00);                                                //dv
  MPU9250_SPI_WriteByte(MPU9250_GYRO_CONFIG, 0x19);                                           //dv
  MPU9250_SPI_WriteByte(MPU9250_ACCEL_CONFIG, 0x10);                                          //dv
  MPU9250_SPI_WriteByte(MPU9250_ACCEL_CONFIG2, 0x08);                                         //dv
  MPU9250_SPI_WriteByte(MPU9250_INT_ENABLE, 0x00);                                            //dv 禁用中断
  MPU9250_SPI_WriteByte(MPU9250_INT_PIN_CFG, 0x10);
  who_am_i = MPU9250_SPI_ReadByte(MPU9250_WHO_AM_I);                                                //dv
  if (who_am_i == 0x71 || who_am_i == 0x73) {                                                               //dv
    return 0;                                                                                               //dv
  } else {                                                                                                  //dv
    return 1;                                                                                               //dv
  }
}


void SetMotorCtrlParams()
{
//  PitchChanel.KpOmega = 0.4578f;
//  PitchChanel.KiOmega = 28.03f;
  PitchChanel.KpOmega = 0.4578f*1.5f;
  PitchChanel.KiOmega = 28.03f*1.5f;
  PitchChanel.Np = 7;
  PitchChanel.Rs = 3.76f;
  PitchChanel.Psif = 0.001761f;                         // 退磁前
//  PitchChanel.Psif = 0.001629f;                       // 退磁后
  PitchChanel.CompCoeff.a0 = 1.f;
  PitchChanel.CompCoeff.a1 = 0.f;
  PitchChanel.CompCoeff.a2 = 0.f;
  PitchChanel.CompCoeff.a3 = 0.f;
  PitchChanel.CompCoeff.a4 = 0.f;
  PitchChanel.CompCoeff.b1 = 0.f;
  PitchChanel.CompCoeff.b2 = 0.f;
  PitchChanel.CompCoeff.b3 = 0.f;
  PitchChanel.CompCoeff.b4 = 0.f;
  PitchChanel.OmegaMechIntgMax = VdcFbk/sqrtf(3)/PitchChanel.Rs;
  PitchChanel.GpioPort = GPIOE;
  PitchChanel.EnbPin = GPIO_PIN_8;
  PitchChanel.Mode = AscMode;
  PitchChanel.htim = &htim1;
  PitchChanel.FocIRQ = TIM1_UP_IRQn;


  RollChanel.KpOmega = 0.4125f*3.f;
  RollChanel.KiOmega = 5.112f*3.f;
  RollChanel.Np = 7;
  RollChanel.Rs = 3.76f;
  RollChanel.Psif = 0.001761f;
  RollChanel.CompCoeff.a0 = 1.0f;
  RollChanel.CompCoeff.a1 = 0;
  RollChanel.CompCoeff.a2 = 0;
  RollChanel.CompCoeff.a3 = 0;
  RollChanel.CompCoeff.a4 = 0;
  RollChanel.CompCoeff.b1 = 0;
  RollChanel.CompCoeff.b2 = 0;
  RollChanel.CompCoeff.b3 = 0;
  RollChanel.CompCoeff.b4 = 0;
  RollChanel.OmegaMechIntgMax = VdcFbk/sqrtf(3)/RollChanel.Rs;
  RollChanel.GpioPort = GPIOC;
  RollChanel.EnbPin = GPIO_PIN_6;
  RollChanel.Mode = AscMode;
  RollChanel.htim = &htim8;
  RollChanel.FocIRQ = TIM8_UP_TIM13_IRQn;
}


// 电流控制
void CurrCtrl(MotorCtrlHandler_t* h)                                                //dv 电流环，在0.1ms高频中断中运行
{
  switch(h->Mode)                                                                   //dv 判断模式
  {
    case SpdMode:                                                                   //dv 速度模式
    case CurrMode:                                                                  //dv 电流模式
      h->Volt2r.d = h->Rs*h->Curr2r_Prev.d;                                         //dv 计算d轴电压
      h->Volt2r.q = h->Rs*h->Curr2r_Prev.q;                                         //dv 计算q轴电压
      h->Volt2r.q = fmax(fmin(h->Volt2r.q,VdcFbk/SQRT_3),-VdcFbk/SQRT_3);           //dv q轴电压限幅
      Trans_2rto2s(&(h->Volt2r),&(h->Volt2s),&(h->AngleElec));        //dv 仅限电压变换
    case VoltMode:                                                                  //dv 电压模式
      SVPWM2(h->htim,&(h->Volt2s),&(h->Volt3s));                     //dv 进行电压调制
      break;                                                                        //dv
    case AscMode:                                                                   //dv 主动短路模式
      Asc(h->htim);                                                                 //dv 主动短路
      break;                                                                        //dv
  }
}




void MotorCtrl_HdwrDisab(MotorCtrlHandler_t* h)
{
  HAL_GPIO_WritePin(h->GpioPort,h->EnbPin,GPIO_PIN_RESET);
}

void MotorCtrl_HdwrEnb(MotorCtrlHandler_t* h)
{
  HAL_GPIO_WritePin(h->GpioPort,h->EnbPin,GPIO_PIN_SET);
}

void MotorCtrl_AscMode_Start(MotorCtrlHandler_t* h)
{
  HAL_NVIC_DisableIRQ(h->FocIRQ);
  h->Mode = AscMode;
  HAL_NVIC_EnableIRQ(h->FocIRQ);
}


void MotorCtrl_SpdMode_Start(MotorCtrlHandler_t* h)                   //dv 开启速度模式
{
  HAL_NVIC_DisableIRQ(h->SpdLoopIRQ);                           //dv 关闭速度环所在的中断，//？？？
  h->Mode = SpdMode;                                                  //dv 更改模式
  h->OmegaMechCmd = 0.f;
  HAL_NVIC_EnableIRQ(h->SpdLoopIRQ);
}

void MotorCtrl_VoltMode_Start(MotorCtrlHandler_t* h)
{
  HAL_NVIC_DisableIRQ(h->FocIRQ);
  h->Mode = VoltMode;
  h->Volt2s.alpha = 0.0f;
  h->Volt2s.beta = 0.0f;
  HAL_NVIC_EnableIRQ(h->FocIRQ);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)                                 //dv 空闲中断回调函数
{
  if(huart->Instance == UART5)                                                                            //dv UART5
  {
    // 数据处理
    // 解析数据类型
    uint16_t Header = (Uart5RXBuf[1] << 8) | Uart5RXBuf[0];                                               //dv 解析帧头
    uint16_t Length = (Uart5RXBuf[3] << 8) | Uart5RXBuf[2];                                               //dv 解析长度
    uint8_t Type = Uart5RXBuf[4];                                                                         //dv 解析类型
    uint8_t Cmd = Uart5RXBuf[5];                                                                          //dv 解析指令


    // 判断是否为RK发送的数据
    if(Header == 0x5A89)                                                                                  //dv 帧头判断
    {
      // 发送数据
      if(Type == 1)                                                                                       //dv 发送数据
      {
        // 发送日志                                                                                         //dv
        if(Cmd == 1)                                                                                      //dv 发送日志数据
        {
          // 准备数据
          LogData.Header = 0x5A89;
          LogData.Length = 36;
          LogData.TimeStamp = 0;                                                                          //dv 时间戳
          LogData.PitchAngleMech = PitchData.AngleMech.Prev;                                              //dv 俯仰机械角度
          LogData.PitchOmegaCmd = PitchChanel.OmegaMechCmd;                                               //dv 角速度指令
          LogData.PitchOmegaFbk = PitchChanel.OmegaMechFbk;                                               //dv 角速度反馈
          LogData.RollAngleMech = RollData.AngleMech.Prev;                                                //dv 滚转机械角
          LogData.RollOmegaCmd = RollChanel.OmegaMechCmd;                                                 //dv 角速度指令
          LogData.RollOmegaFbk = RollChanel.OmegaMechFbk;                                                 //dv 角速度反馈
          // 启动串口发送
          HAL_UART_Transmit_DMA(&huart5,(uint8_t*)(&LogData),sizeof(LogData));           //dv 启动串口DMA传输
        }

        // 发送编码器角度
        if(Cmd == 3)                                                                                      //dv 获取编码器角度
        {
          // 准备数据
          AngleData.Header = 0x5A89;
          AngleData.Length = 8;
          AngleData.PitchAngleMech = PitchData.AngleMechRaw;                                              //dv 俯仰机械角度
          AngleData.RollAngleMech = RollData.AngleMechRaw;                                                //dv 滚转机械角度
          // 启动串口发送
          HAL_UART_Transmit_DMA(&huart5,(uint8_t*)(&AngleData),sizeof(AngleData));       //dv 启动串口DMA传输
        }
      }

      // 接收数据
      if(Type == 2)                                                                                       //dv 接收数据
      {
        // 接收角速度指令
        if(Cmd == 2)                                                                                      //dv 接收角速度指令
        {
          HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0);                                            //dv GPIO翻转
          // 解析角速度指令
          float temp = 0;                                                                                 //dv 临时浮点变量
          memcpy(&temp,&(Uart5RXBuf[6]),4);                                                               //dv 解析俯仰角速度指令
          PitchChanel.OmegaMechCmd = temp;                                                                //dv 赋值
          memcpy(&temp,&(Uart5RXBuf[10]),4);                                                              //dv 解析滚转角速度指令
          RollChanel.OmegaMechCmd = temp;                                                                 //dv 赋值
        }
      }
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5,Uart5RXBuf,UART5_RXBUF_LEN);                    //dv 重新启动接收
  }
}


//
void SetLpfParams(float Ts,float CutoffFreq)
{
  HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  LpfInit(&(OmegaData.XLpf),Ts,CutoffFreq);
  LpfInit(&(OmegaData.YLpf),Ts,CutoffFreq);
  LpfInit(&(OmegaData.ZLpf),Ts,CutoffFreq);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

// 参数标定函数
void ParamsCalib(void)
{
    memset(&CalibParams,0,sizeof(CalibParams));



    /// 陀螺仪零偏标定
    // 软件刹车
    MotorCtrl_HdwrEnb(&PitchChanel);
    MotorCtrl_Break(&PitchChanel);
    MotorCtrl_HdwrEnb(&RollChanel);
    MotorCtrl_Break(&RollChanel);

    // 设置滤波器截止频率、采样周期
    SetLpfParams(Ts_SpdCtrl,0.1f);
    HAL_Delay(5000);
    CalibParams.OmegaXOffset = OmegaData.X.Prev;
    CalibParams.OmegaYOffset = OmegaData.Y.Prev;
    CalibParams.OmegaZOffset = OmegaData.Z.Prev;

    /// 电角度零偏标定
    MotorCtrl_VoltMode_Start(&PitchChanel);
    MotorCtrl_VoltMode_Start(&RollChanel);
    PitchChanel.Volt2s = (Var2s_t){.alpha = 5.0f,.beta = 0.f};
    RollChanel.Volt2s = (Var2s_t){.alpha = 8.0f,.beta = 0.f};
    HAL_Delay(100);
    CalibParams.PitchElecOffSet = PitchData.AngleElecRaw;
    CalibParams.RollElecOffSet = RollData.AngleElecRaw;

//    /// 机械角度零位标定
//    // 更新角速度滤波参数
//    SetLpfParams(Ts_2ms,0.1f);
//
//    // 俯仰通道保持固定
//    MotorCtrl_SpdMode_Start(&PitchChanel);
//    PitchChanel.OmegaMechCmd = 0.2f;

//    // 滚转通道启用电流模式，并启用指令生成器，给定正弦波电流指令
//    MotorCtrl_CurrMode_Start(&RollChanel);
//    HAL_NVIC_DisableIRQ(TIM7_IRQn);
//    CurrSigForCalib.t = 0.f;
//    CurrSigForCalib.Ts = Ts_100us;
//    CurrSigForCalib.Type = Sin;
//    CurrSigForCalib.Magnitude = 0.3f;
//    CurrSigForCalib.FreqInit = 2.f;
//    HAL_NVIC_EnableIRQ(TIM7_IRQn);

//    // 检测角速度幅值，使用滑动窗口滤波
//    float SquareOmegaX[30]={0};
//    float Sum = 0.f;
//    float Average = 0;
//    uint8_t idx = 0;
//    uint8_t idx_prev = 0;
//    do{
//      idx_prev = (idx + 29)%30;
//      SquareOmegaX[idx] = OmegaData.X.filted*OmegaData.X.filted;
//      Sum = Sum + SquareOmegaX[idx] - SquareOmegaX[idx_prev];
//      idx = (idx + 1)%30;
//      Average = Sum/30;
//      HAL_Delay(10);
//    }while(Average > 0.01f);
//    // 关闭指令生成器
//    HAL_NVIC_DisableIRQ(TIM7_IRQn);
//    RollChanel.Curr2r_Prev.q = 0.f;
//    PitchChanel.OmegaMechCmd = 0.f;
//    CalibParams.PitchMechOffSet = PitchData.AngleMechRaw;

    CalibParams.IsValid = TRUE;

    Q128Ret = W25Q128_EraseSector(CALIB_PARAM_BASE_ADDR);                                                         //dv 擦除扇区

    W25Q128_Write(CALIB_PARAM_BASE_ADDR,(uint8_t*)(&(CalibParams)),sizeof(CalibParams));
}


// 刹车
void MotorCtrl_Break(MotorCtrlHandler_t* h)
{
  HAL_NVIC_DisableIRQ(h->FocIRQ);
  h->Mode = AscMode;
  HAL_NVIC_EnableIRQ(h->FocIRQ);
}

// 启动电流模式
void MotorCtrl_CurrMode_Start(MotorCtrlHandler_t* h)
{
  HAL_NVIC_DisableIRQ(h->FocIRQ);
  h->Mode = CurrMode;
  h->Curr2r_Prev.d = 0.f;
  h->Curr2r_Prev.q = 0.f;
  HAL_NVIC_EnableIRQ(h->FocIRQ);
}

// 擦除扇区
uint8_t W25Q128_EraseSector(uint32_t sector_addr)                   //dv 擦除扇区
{
  QSPI_CommandTypeDef cmd = {0};                                    //dv 指令

  cmd.Instruction = 0x06;                                           //dv 写使能
  cmd.AddressMode = QSPI_ADDRESS_NONE;                              //dv 无地址
  cmd.DataMode = QSPI_DATA_NONE;                                    //dv 无数据
  if(HAL_QSPI_Command(&hqspi, &cmd, 5000) != HAL_OK)        //dv 发送指令
    return 1;                                                       //dv

  cmd.Instruction = 0x20;                                           //dv 扇区擦除
  cmd.AddressMode = QSPI_ADDRESS_1_LINE;                            //dv 1线地址模式
  cmd.AddressSize = QSPI_ADDRESS_24_BITS;                           //dv 24位地址
  cmd.Address = sector_addr;                                        //dv 扇区地址
  cmd.DataMode = QSPI_DATA_NONE;                                    //dv 无数据
  if(HAL_QSPI_Command(&hqspi, &cmd, 5000) != HAL_OK)        //dv 发送指令
    return 2;

  if(W25Q128_WaitBusy() != 0)                                       //dv 等待FLASH空闲
    return 3;                                                       //dv

  return 0;                                                         //dv
}


uint8_t W25Q128_Read(uint32_t addr, uint8_t *pData, uint32_t len)
{
  QSPI_CommandTypeDef cmd = {0};                                              //dv 指令

  cmd.Instruction = 0x03;                                                     //dv 读数据
  cmd.AddressMode = QSPI_ADDRESS_1_LINE;                                      //dv 1线地址模式
  cmd.AddressSize = QSPI_ADDRESS_24_BITS;                                     //dv 24位地址
  cmd.Address = addr;                                                         //dv 目标地址
  cmd.DummyCycles = 0;                                                        //dv 不需要dummycycle
  cmd.DataMode = QSPI_DATA_1_LINE;                                            //dv 1线数据模式
  cmd.NbData = len;                                                           //dv

  if (HAL_QSPI_Command(&hqspi, &cmd, 1000) != HAL_OK)                 //dv 发送指令
    return 1;

  if (HAL_QSPI_Receive(&hqspi, pData, 1000) != HAL_OK)                //dv 接收数据
    return 2;                                                                 //dv

  return 0;                                                                   //dv
}


uint8_t W25Q128_Write(uint32_t addr, uint8_t *pData, uint32_t len)            //dv 向FLASH写入数据
{
  QSPI_CommandTypeDef cmd = {0};                                              //dv 指令
  uint32_t page_remain;                                                       //dv 当前页剩余字节数
  uint32_t write_size;                                                        //dv 写字节数

  if(len == 0) return 0;                                                      //dv 如果len==0,立即返回

  while(len > 0)                                                              //dv 如果len > 0
  {
    page_remain = 256 - (addr & 0xFF);                                        //dv 计算当前页剩余字节
    write_size = (len > page_remain) ? page_remain : len;                     //dv 如果要求写入字节数大于当前页剩余字节数，那么实际写入字节数设为当前剩余字节数，否则取为要求写入字节数

    cmd.Instruction = 0x06;                                                   //dv 写使能
    cmd.AddressMode = QSPI_ADDRESS_NONE;                                      //dv 无地址
    cmd.DataMode = QSPI_DATA_NONE;                                            //dv 无数据
    if(HAL_QSPI_Command(&hqspi, &cmd, 1000) != HAL_OK)                //dv 发送指令
      return 1;                                                               //dv

    cmd.Instruction = 0x02;                                                   //dv 页写入
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;                                    //dv 1线地址模式
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;                                   //dv 12位地址
    cmd.Address = addr;                                                       //dv 写入地址
    cmd.DataMode = QSPI_DATA_1_LINE;                                          //dv 1线数据模式
    cmd.NbData = write_size;                                                  //dv 数据大小

    if(HAL_QSPI_Command(&hqspi, &cmd, 1000) != HAL_OK)                //dv 发送页编程指令
      return 1;

    if(HAL_QSPI_Transmit(&hqspi, pData, 1000) != HAL_OK)              //dv 开始写入
      return 1;

    if(W25Q128_WaitBusy() != 0)                                               //dv 等待FLASH空闲
      return 1;

    addr += write_size;                                                       //dv 目标地址增加
    pData += write_size;                                                      //dv 数据地址增加
    len -= write_size;                                                        //dv 长度减小
  }

  return 0;                                                                   //dv
}



uint8_t W25Q128_WaitBusy(void)                                                //dv 等待空闲
{
  QSPI_CommandTypeDef cmd = {0};                                              //dv 指令
  uint8_t status;                                                             //dv 状态

  cmd.Instruction = 0x05;                                                     //dv 读状态寄存器1
  cmd.AddressMode = QSPI_ADDRESS_NONE;                                        //dv 无地址
  cmd.DataMode = QSPI_DATA_1_LINE;                                            //dv 1线数据模式
  cmd.NbData = 1;                                                             //dv 1个字节

  do                                                                          //dv
  {
    if(HAL_QSPI_Command(&hqspi, &cmd, 5000) != HAL_OK)                //dv 发送指令
      return 1;                                                               //dv
    if(HAL_QSPI_Receive(&hqspi, &status, 5000) != HAL_OK)       //dv 实际读取
      return 1;                                                               //dv
  } while((status & 1) == 1);                                                 //dv 循环检查忙位

  return 0;                                                                   //dv
}


float SignalGenerate(SignalHandler_t* h)
{
  h->t = h->t + h->Ts;
  switch(h->Type)
  {
      case Chirp:
          h->Signal = h->Magnitude*sinf(2*PI*(h->FreqInit* expf(h->FreqSlope*h->t))*h->t);
          break;
      case Sin:
          h->Signal = h->Magnitude*sinf(2*PI*h->FreqInit*h->t);
          break;
      case Step:
          h->Signal = h->Magnitude;
          break;
      default:
          h->Signal = 0.f;
  }
  return h->Signal;
}




uint8_t NeedCalib(void)                                                                       //dv 检测是否需要标定
{
    uint8_t NeedCalib = 0;                                                                    //dv 定义标定标志
    if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == GPIO_PIN_RESET)                  //dv 如果上电检测Usr按钮是否被按下
    {
        HAL_Delay(1000);                                                                //dv 延迟1s后
        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == GPIO_PIN_RESET)              //dv 依然被按下
        {
            NeedCalib = 1;                                                                    //dv 那么需要进行标定
        }
    }
    else                                                                                                    //dv 如果上电检测USR按钮没有被按下
    {
        W25Q128_Read(CALIB_PARAM_BASE_ADDR,(uint8_t*)(&CalibParams),sizeof(CalibParams));   //dv 读取标定参数
        if(CalibParams.IsValid == FALSE)                                                                    //dv 如果从未进行过标定
        {
            NeedCalib = 1;                                                                                  //dv 那么需要进行标定
        }
    }
    return NeedCalib;                                                                                       //dv 返回标定需求标志位
}

// 低通滤波器初始化
void LpfInit(LpfHandler_t* h,float Ts,float CutoffFreq)
{
    h->alpha = Ts*2*PI*CutoffFreq;
    h->filted_prev = 0.f;
    h->filted = 0.f;
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
//    // 硬件级停止电机控制
//    MotorCtrl_HdwrDisab(&PitchChanel);
//    MotorCtrl_HdwrDisab(&RollChanel);
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
