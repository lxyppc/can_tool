/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Custom HID demo main file
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#ifdef STM32L1XX_MD
#include "stm32l1xx.h"
#else
#include "stm32f10x.h"
#endif /* STM32L1XX_MD */
 
#include "usb_lib.h"
#include "hw_config.h"
#include "can.h"
#include "string.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define GPIO_RES  GPIOB
#define RCC_RES_Periph RCC_APB2Periph_GPIOB
// for product, it is controlled by PC6,7,8,9
#define CH1_A     GPIO_Pin_12
#define CH1_B     GPIO_Pin_13
#define CH2_A     GPIO_Pin_14
#define CH2_B     GPIO_Pin_15

#define   LED1_R_ON()     do{ GPIOA->BRR =  GPIO_Pin_0; }while(0)
#define   LED1_R_OFF()    do{ GPIOA->BSRR = GPIO_Pin_0; }while(0)
#define   LED1_G_ON()     do{ GPIOA->BRR =  GPIO_Pin_1; }while(0)
#define   LED1_G_OFF()    do{ GPIOA->BSRR = GPIO_Pin_1; }while(0)

#define   LED2_R_ON()     do{ GPIOA->BRR =  GPIO_Pin_2; }while(0)
#define   LED2_R_OFF()    do{ GPIOA->BSRR = GPIO_Pin_2; }while(0)
#define   LED2_G_ON()     do{ GPIOA->BRR =  GPIO_Pin_3; }while(0)
#define   LED2_G_OFF()    do{ GPIOA->BSRR = GPIO_Pin_3; }while(0)

#define   LED3_R_ON()     do{ GPIOB->BRR =  GPIO_Pin_0; }while(0)
#define   LED3_R_OFF()    do{ GPIOB->BSRR = GPIO_Pin_0; }while(0)
#define   LED3_G_ON()     do{ GPIOB->BRR =  GPIO_Pin_1; }while(0)
#define   LED3_G_OFF()    do{ GPIOB->BSRR = GPIO_Pin_1; }while(0)

#define   LED_R_ON()      do{ LED1_R_ON();  LED2_R_ON();  LED3_R_ON(); }while(0)
#define   LED_R_OFF()     do{ LED1_R_OFF(); LED2_R_OFF(); LED3_R_OFF(); }while(0)

#define   LED_G_ON()      do{ LED1_G_ON();  LED2_G_ON();  LED3_G_ON(); }while(0)
#define   LED_G_OFF()     do{ LED1_G_OFF(); LED2_G_OFF(); LED3_G_OFF(); }while(0)

#define   LED_ALL_ON()    do{ LED_R_ON();  LED_G_ON();     }while(0)
#define   LED_ALL_OFF()   do{ LED_R_OFF(); LED_G_OFF();    }while(0)

#define  CAN1_ERROR_ON()  do{ LED3_R_ON();  }while(0)
#define  CAN1_ERROR_OFF() do{ LED3_R_OFF(); }while(0)

#define  CAN2_ERROR_ON()  do{ LED2_R_ON();  }while(0)
#define  CAN2_ERROR_OFF() do{ LED2_R_OFF(); }while(0)

/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
__IO uint8_t PrevXferComplete = 1;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);

DeviceSetting  deviceSetting;
DeviceSetting  newSetting;
uint8_t        bReConfigCAN = 0;
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CAN_IT_Config(void);
static void CAN_UpdateConfig(CAN_TypeDef* CANx)
{
    CAN_InitTypeDef        CAN_InitStructure;
    BTR  btr;
    if (CANx == CAN1)
    {
        btr.raw = deviceSetting.btr[0];
        deviceSetting.baudrate[0] = deviceSetting.CANfreq/btr.REG.BRP/(btr.REG.TS1+btr.REG.TS2+3);
    }
    else if(CANx == CAN2)
    {
        btr.raw = deviceSetting.btr[1];
        deviceSetting.baudrate[1] = deviceSetting.CANfreq/btr.REG.BRP/(btr.REG.TS1+btr.REG.TS2+3);
    }
    else
    {
        return;
    }
    CAN_StructInit(&CAN_InitStructure);
    
  CAN_InitStructure.CAN_TTCM = ENABLE;
  CAN_InitStructure.CAN_ABOM = ENABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  if( btr.REG.SILENT && btr.REG.LOOKBACK)
      CAN_InitStructure.CAN_Mode = CAN_Mode_Silent_LoopBack;
  else if ( btr.REG.SILENT )
      CAN_InitStructure.CAN_Mode = CAN_Mode_Silent;
  else if ( btr.REG.LOOKBACK )
      CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
  else
      CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = btr.REG.SJW;
  CAN_InitStructure.CAN_BS1 = btr.REG.TS1;
  CAN_InitStructure.CAN_BS2 = btr.REG.TS2; 
  CAN_InitStructure.CAN_Prescaler = btr.REG.BRP;
  CAN_Init(CANx, &CAN_InitStructure);
}

static void CAN_ReConfig(void)
{
   /* CAN1 and CAN2 register init */
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  CAN_UpdateConfig(CAN1);
  CAN_UpdateConfig(CAN2);
    /* CAN1 filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 1;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  //CAN_SlaveStartBank(14);
  
  /* CAN2 filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 15;
  CAN_FilterInit(&CAN_FilterInitStructure);
}

volatile uint16_t test_in;
void test_init_can(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  return;
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure CAN2 TX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  while(1) {
      GPIO_SetBits(GPIOB, GPIO_Pin_6);
      test_in = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
        
      GPIO_ResetBits(GPIOB, GPIO_Pin_6);
      test_in = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
      
      GPIO_SetBits(GPIOB, GPIO_Pin_9);
      test_in = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8);
        
      GPIO_ResetBits(GPIOB, GPIO_Pin_9);
      test_in = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8);
  }
}

void init_can(void)
{
    
  GPIO_InitTypeDef  GPIO_InitStructure;
  /* Configure CAN1 and CAN2 IOs **********************************************/
  /* GPIOB and AFIO clocks enable */
  //can_io_for_test();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
     
  /* Configure CAN1 RX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  /* Configure CAN2 RX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  test_init_can();
  /* Configure CAN1 TX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure CAN2 TX pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Remap CAN1 and CAN2 GPIOs */
  GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_CAN2, ENABLE);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);  
  
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);
  
  CAN_ReConfig();
  
  CAN_IT_Config();
}
void CAN_IT_Config(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;
 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN2_SCE_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  // enable last error, error passive, error warning and bus off error
  CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_FMP1 | CAN_IT_TME | CAN_IT_ERR | CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF, ENABLE);
  CAN_ITConfig(CAN2, CAN_IT_FMP0 | CAN_IT_FMP1 | CAN_IT_TME | CAN_IT_ERR | CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF, ENABLE);
}
void init_io(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
    
    // Init IO for relay control and LEDs
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    
    // Setup PIN for relay control and LED3
    GPIO_InitStructure.GPIO_Pin = CH1_A | CH1_B | CH2_A | CH2_B | GPIO_Pin_0 | GPIO_Pin_1;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIO_RES, &GPIO_InitStructure);
    // Setup PIN for LED1 and LED2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // because we use a latch coil relay
    // config timer to shutdown the relay driver
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	// APB1 is 36MHz
	TIM_TimeBaseStructure.TIM_Period = 500;  // 2Hz
  	TIM_TimeBaseStructure.TIM_Prescaler = 36000; // 1KHz
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Single);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	/* TIM4 enable counter */
  	//TIM_Cmd(TIM4, ENABLE);

	
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

void init_timestamp(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    // because we use a latch coil relay
    // config timer to shutdown the relay driver
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	// APB1 is 36MHz
	TIM_TimeBaseStructure.TIM_Period = 1999;  // 1Hz
  	TIM_TimeBaseStructure.TIM_Prescaler = 36000-1; // 1KHz
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
    
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

void EnableTerminalResistor(uint32_t ch1, uint32_t ch2)
{
	uint32_t  bsrr = ((uint32_t)(CH1_A | CH1_B | CH2_A | CH2_B)) << 16; // reset all pins
	if(ch1){
		bsrr |= CH1_B;
	}else{
		bsrr |= CH1_A;
	}
	if(ch2){
		bsrr |= CH2_B;
	}else{
		bsrr |= CH2_A;
	}
	GPIO_RES->BSRR = bsrr;
	TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		GPIO_RES->BSRR = ((uint32_t)(CH1_A | CH1_B | CH2_A | CH2_B)) << 16;
	}
}
uint32_t second = 0;
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		second++;
        //GPIOA->ODR ^=  GPIO_Pin_0;
	}
}

static void update_can_config(void)
{
  if( deviceSetting.btr[0] != newSetting.btr[0] )
  {
      deviceSetting.btr[0] = newSetting.btr[0];
      CAN_UpdateConfig(CAN1);
      CAN1_ERROR_OFF();
  }
  if( deviceSetting.btr[1] != newSetting.btr[1] )
  {
      deviceSetting.btr[1] = newSetting.btr[1];
      CAN_UpdateConfig(CAN2);
      CAN2_ERROR_OFF();
  }
}

static uint8_t res1 = 1;
static uint8_t res2 = 1;
void send_can_frame(void);
int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  BTR btr;
  //Set_System();
    
  init_io();
  LED_ALL_OFF();
    
  btr.raw = 0;
  btr.REG.SILENT = 0;
  btr.REG.LOOKBACK = 0;
  btr.REG.SJW = CAN_SJW_1tq;
  btr.REG.TS1 = CAN_BS1_13tq;
  btr.REG.TS2 = CAN_BS2_2tq;
  btr.REG.BRP = 18;  // 36000000 / 18 / (3+13+2) = 125KHz
  btr.REG.RESISTOR = 1; // enable terminal resistor
  deviceSetting.btr[0] = btr.raw;
  deviceSetting.btr[1] = btr.raw;
  newSetting.btr[0] = btr.raw;
  newSetting.btr[1] = btr.raw;
  deviceSetting.channelCount = CAN_CHANNEL_COUNT;
    
  
  RCC_GetClocksFreq(&RCC_Clocks);
  deviceSetting.CANfreq = RCC_Clocks.PCLK1_Frequency; // for can 
  
  EnableTerminalResistor(res1, res2);
  
  USB_Interrupts_Config();

  Set_USBClock();

  USB_Init();
  
  init_can();
  
  init_timestamp();


  while (1)
  {
      update_can_config();
      send_can_frame();
      if(0)// &&  (deviceSetting.btr[0] != newSetting.btr[0]) || (deviceSetting.btr[1] != newSetting.btr[1]) )
      {
          
          BTR  btr;
          uint8_t send_buffer[64];
          uint16_t pres;
          btr.raw = newSetting.btr[0];
          send_buffer[0] = 0xaa;
          send_buffer[1] = 0xbb;
          pres = btr.REG.BRP;
          memcpy(send_buffer+2, &pres, 2);
          send_buffer[4] = btr.REG.TS1;
          send_buffer[5] = btr.REG.TS2;
          send_buffer[6] = btr.REG.SJW;
          send_buffer[7] = btr.REG.SILENT | (btr.REG.LOOKBACK<<1);
          
          
          btr.raw = newSetting.btr[1];
          send_buffer[0+8] = 0xcc;
          send_buffer[1+8] = 0xdd;
          pres = btr.REG.BRP;
          memcpy(send_buffer+2+8, &pres, 2);
          send_buffer[4+8] = btr.REG.TS1;
          send_buffer[5+8] = btr.REG.TS2;
          send_buffer[6+8] = btr.REG.SJW;
          send_buffer[7+8] = btr.REG.SILENT | (btr.REG.LOOKBACK<<1);
          if (PrevXferComplete)
          {   
              USB_SIL_Write(EP1_IN, send_buffer, 64);  
              
        #ifndef STM32F10X_CL
              SetEPTxValid(ENDP1);
        #endif /* STM32F10X_CL */
              PrevXferComplete = 0;
          }
          
          deviceSetting.btr[0] = newSetting.btr[0];
          deviceSetting.btr[1] = newSetting.btr[1];
          CAN_ReConfig();
      }
  }
}

enum{
    BAUD_1M,
    BAUD_800K,
    BAUD_500K,
    BAUD_250K,
    BAUD_125K,
    BAUD_50K,
    BAUD_20K,
    BAUD_10K,
};
typedef struct {
    uint16_t pres;
    uint8_t  sjw;
    uint8_t  bs1;
    uint8_t  bs2;
}time_cfg;
static const time_cfg BAUDS[] = {
//         pres         sjw          ts1              ts2
/*   1M*/{  6,    CAN_SJW_1tq,   CAN_BS1_3tq,     CAN_BS2_2tq  },  // 36M / 6 / (1+2+3) = 1MHz, CANOpen recommend total 8 sample@6
/* 800K*/{  5,    CAN_SJW_1tq,   CAN_BS1_6tq,     CAN_BS2_2tq  },  // 36M / 5 / (1+6+2) = 800KHz, CANOpen recommend total 10 sample@8
/* 500K*/{  6,    CAN_SJW_1tq,   CAN_BS1_6tq,     CAN_BS2_2tq  },  // 36M / 6 / (1+9+2) = 500KHz, CANOpen recommend total 16 sample@14
/* 250K*/{  9,    CAN_SJW_1tq,   CAN_BS1_13tq,    CAN_BS2_2tq  },  // 36M / 9 / (1+13+2) = 250KHz, CANOpen recommend total 16 sample@14
/* 125K*/{ 18,    CAN_SJW_1tq,   CAN_BS1_13tq,    CAN_BS2_2tq  },  // 36M /18 / (1+13+2) = 125KHz, CANOpen recommend total 16 sample@14
/*  50K*/{ 45,    CAN_SJW_1tq,   CAN_BS1_13tq,    CAN_BS2_2tq  },  // 36M /45 / (1+13+2) = 50KHz, CANOpen recommend total 16 sample@14
/*  20K*/{100,    CAN_SJW_1tq,   CAN_BS1_14tq,    CAN_BS2_3tq  },  // 36M /100/ (1+14+3) = 20KHz, CANOpen recommend total 16 sample@14
/*  10K*/{225,    CAN_SJW_1tq,   CAN_BS1_13tq,    CAN_BS2_2tq  },  // 36M /225/ (1+13+2) = 10KHz, CANOpen recommend total 16 sample@14
};
void set_time(uint32_t baudrate)
{
    BTR btr;
    const time_cfg* cfg = &BAUDS[BAUD_10K];
    if      (baudrate >= 1000000) cfg = &BAUDS[BAUD_1M];
    else if (baudrate >=  800000) cfg = &BAUDS[BAUD_800K];
    else if (baudrate >=  500000) cfg = &BAUDS[BAUD_500K];
    else if (baudrate >=  250000) cfg = &BAUDS[BAUD_250K];
    else if (baudrate >=  125000) cfg = &BAUDS[BAUD_125K];
    else if (baudrate >=   50000) cfg = &BAUDS[BAUD_50K];
    else if (baudrate >=   20000) cfg = &BAUDS[BAUD_20K];
    else                          cfg = &BAUDS[BAUD_10K];
    
  btr.REG.SJW = cfg->sjw;
  btr.REG.TS1 = cfg->bs1;
  btr.REG.TS2 = cfg->bs2;
  btr.REG.BRP = cfg->pres;
  btr.REG.SILENT = 0;
  btr.REG.LOOKBACK = 0;
  btr.REG.RESISTOR = 1;
}

#define  ID_CH1      1
#define  ID_CH2      2
#define  TYPE_CFG    1
#define  TYPE_FRAME  2
typedef struct _cmd_data
{
    uint8_t  id;
    uint8_t  type;
    uint16_t reserved;
    union{
        struct{
            uint32_t  baudrate;
            uint16_t  pres;
            uint8_t   bs1;
            uint8_t   bs2;
            uint8_t   sjw;
            uint8_t   mode;
            uint8_t   resistor;
        }cfg;
        struct{
            uint32_t id;
            uint8_t  type;
            uint8_t  dlc;
            uint8_t  data[8];
            uint8_t  ts[8];   // second, MS, BS
        }frame;
    }data;
}cmd_data_t;

int32_t TransmitCANFrame(const cmd_data_t* cmd);
void cmd_handler(const void* data)
{
    const cmd_data_t* cmd = (const cmd_data_t*)data;
    BTR btr;
    btr.raw = 0;
    if(cmd->id == ID_CH1){
        if(cmd->type == TYPE_CFG){
            if(res1 != cmd->data.cfg.resistor){
                res1 = cmd->data.cfg.resistor;
                EnableTerminalResistor(res1,res2);
            }
            btr.REG.SILENT = cmd->data.cfg.mode & 1 ? 1: 0;
            btr.REG.LOOKBACK = cmd->data.cfg.mode & 2 ? 1: 0;
            btr.REG.RESISTOR = 1;
            btr.REG.SJW = cmd->data.cfg.sjw - 1;
            btr.REG.TS2 = cmd->data.cfg.bs2 - 1;
            btr.REG.TS1 = cmd->data.cfg.bs1 - 1;
            btr.REG.BRP = cmd->data.cfg.pres;
            newSetting.btr[0] = btr.raw;
        }else if(cmd->type == TYPE_FRAME){
            TransmitCANFrame(cmd);
        }
    }else if(cmd->id == ID_CH2){
        if(cmd->type == TYPE_CFG){
            if(res2 != cmd->data.cfg.resistor){
                res2 = cmd->data.cfg.resistor;
                EnableTerminalResistor(res1,res2);
            }
            btr.REG.SILENT = cmd->data.cfg.mode & 1 ? 1: 0;
            btr.REG.LOOKBACK = cmd->data.cfg.mode & 2 ? 1: 0;
            btr.REG.RESISTOR = 1;
            btr.REG.SJW = cmd->data.cfg.sjw - 1;
            btr.REG.TS2 = cmd->data.cfg.bs2 - 1;
            btr.REG.TS1 = cmd->data.cfg.bs1 - 1;
            btr.REG.BRP = cmd->data.cfg.pres;
            newSetting.btr[1] = btr.raw;
        }else if(cmd->type == TYPE_FRAME){
            TransmitCANFrame(cmd);
        }
    }
}
int32_t TransmitCANFrame(const cmd_data_t* cmd)
{
    uint8_t transmit_mailbox = 0;
    uint32_t tmp = 0;
    uint32_t dx[2];
    CAN_TypeDef* CANx = 0;
    if(cmd->id == ID_CH1){
        CANx = CAN1;
    }else if(cmd->id == ID_CH2){
        CANx = CAN2;
    }
    if(CANx == 0) return -1;
    
    /* Select one empty transmit mailbox */
    if ((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
    {
        transmit_mailbox = 0;
    }
    else if ((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
    {
        transmit_mailbox = 1;
    }
    else if ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
    {
        transmit_mailbox = 2;
    }
    else
    {
        return 1;
    }
    
    tmp = 0;
    if(cmd->data.frame.type & 2) tmp |= CAN_RTR_Remote;
    if(cmd->data.frame.type & 1){
        tmp |= CAN_Id_Extended;
        tmp |= ( cmd->data.frame.id << 3); 
    }else{
        tmp |= ( cmd->data.frame.id << 21);
    }
    memcpy(dx, cmd->data.frame.data, 8);
    CANx->sTxMailBox[transmit_mailbox].TIR = tmp;
    CANx->sTxMailBox[transmit_mailbox].TDTR = cmd->data.frame.dlc & 0xf;
    CANx->sTxMailBox[transmit_mailbox].TDLR = dx[0];
    CANx->sTxMailBox[transmit_mailbox].TDHR = dx[1];
    CANx->sTxMailBox[transmit_mailbox].TIR |= 1;
    return 0;
}

#define  CAN_FRM_FIFO_LEN    16
#define  CAN_FRM_FIFO_MASK   15
struct{
    CANFrame  frames[CAN_FRM_FIFO_LEN];
    uint32_t  rd;
    uint32_t  wr;
    uint32_t  overflow;
}CanRecvFifo;

void push_frame(const CANFrame* pFrame)
{
    uint32_t x = CanRecvFifo.rd ^ CanRecvFifo.wr;
    if(x && ((x&CAN_FRM_FIFO_MASK) == 0) )
    {
        // buffer over flow
        CanRecvFifo.overflow = 1;
        return;
    }
    memcpy(&CanRecvFifo.frames[CanRecvFifo.wr & CAN_FRM_FIFO_MASK], pFrame, sizeof(CANFrame));
    CanRecvFifo.wr++;
}

void send_can_frame(void)
{
    uint32_t x = CanRecvFifo.rd ^ CanRecvFifo.wr;
    if(x && PrevXferComplete){
        SendCANToUSB(&CanRecvFifo.frames[CanRecvFifo.rd & CAN_FRM_FIFO_MASK], 1);
        CanRecvFifo.rd++;
    }
}

uint32_t SendCANToUSB(const CANFrame* pFrame, uint32_t count)
{
    uint8_t send_buffer[64];
    cmd_data_t* cmd = (cmd_data_t*)send_buffer;
    cmd->id = pFrame->frameData.channel;
    cmd->type = TYPE_FRAME;
    cmd->data.frame.id = pFrame->frameData.id;
    cmd->data.frame.type = pFrame->frameData.extend ? 1 : 0;
    cmd->data.frame.type |= pFrame->frameData.remote ? 2 : 0;
    cmd->data.frame.dlc = pFrame->frameData.dlc;
    memcpy(cmd->data.frame.data, pFrame->frameData.data, 8);
    *(uint32_t*)cmd->data.frame.ts = pFrame->frameData.timestampS;
    *(uint16_t*)(cmd->data.frame.ts+4) = pFrame->frameData.timestamp_ms;
    *(uint16_t*)(cmd->data.frame.ts+6) = pFrame->frameData.timestamp_bs;
    if (PrevXferComplete)
    {   
      USB_SIL_Write(EP1_IN, send_buffer, 64);  
      
#ifndef STM32F10X_CL
      SetEPTxValid(ENDP1);
#endif /* STM32F10X_CL */
      PrevXferComplete = 0;
    }
    return 0;
}

static void CAN_Recv_Done(uint32_t chn, uint32_t FIFOn);
static void CAN_TX_Done(CAN_TypeDef* CANx)
{
    uint32_t tsr = CANx->TSR;
    uint32_t flag = 1;
    uint32_t cnt = 0;
    do{
        if(tsr & 1){
            //OnTransmitDone(CANx, cnt, tsr & 0xf);
            CANx->TSR |= flag;
            break;
        }
        flag<<=8;
        tsr>>=8;
        cnt++;
    }while(cnt<3);
}

static void CAN_SCE(CAN_TypeDef* CANx)
{
	uint32_t esr = CANx->ESR;
	CANx->ESR = 0; // clear call esr status
    CANx->MSR = CAN_MSR_ERRI;
	if(esr & CAN_ESR_BOFF){
		// bus off, need reinitilize the can bus, this is done automatically
	}
	//CAN_Error(CANx, esr, CANx->MSR>>8); 
    if  (CANx == CAN1) CAN1_ERROR_ON();
    else               CAN2_ERROR_ON();
}

// CAN1 Tx done
void CAN1_TX_IRQHandler(void)
{
    CAN_TX_Done(CAN1);
}

// CAN2 Tx done
void CAN2_TX_IRQHandler(void)
{
    CAN_TX_Done(CAN2);
}

void CAN1_RX0_IRQHandler(void)
{
    CAN_Recv_Done(ID_CH1,CAN_FIFO0);
}

void CAN1_RX1_IRQHandler(void)
{
    CAN_Recv_Done(ID_CH1,CAN_FIFO1);
}

void CAN2_RX0_IRQHandler(void)
{
    CAN_Recv_Done(ID_CH2,CAN_FIFO0);
}

void CAN2_RX1_IRQHandler(void)
{
    CAN_Recv_Done(ID_CH2,CAN_FIFO1);
}

void CAN1_SCE_IRQHandler(void)
{
    CAN_SCE(CAN1);
}

void CAN2_SCE_IRQHandler(void)
{
    CAN_SCE(CAN2);
}
/** copy recieve message to USB buffer
*/
void CAN_Recv_Done(uint32_t chn, uint32_t FIFOn)
{
    uint32_t tmp;
    CAN_FIFOMailBox_TypeDef* FIFOx;
    CAN_TypeDef* CANx;
    CANFrame frame;
    if(chn == ID_CH1){
        CANx = CAN1;
    }else if(chn == ID_CH2){
        CANx = CAN2;
    }else{
        return;
    }
    
    if(FIFOn > CAN_FIFO1){
        return;
    }
    
    FIFOx = &CANx->sFIFOMailBox[FIFOn];
    tmp = FIFOx->RIR;
    
    if(tmp & CAN_Id_Extended){
        frame.frameData.extend = 1;
        frame.frameData.id = tmp >> 3;
    }else{
        frame.frameData.extend = 0;
        frame.frameData.id = tmp >> 21;
    }
    if(tmp & CAN_RTR_Remote){
        frame.frameData.remote = 1;
    }else{
        frame.frameData.remote = 0;
    }
    frame.frameData.direction = 0;
    frame.frameData.channel = chn;
	frame.frameData.dataType = FT_DATA;
    frame.frameData.dlc = FIFOx->RDTR & 0xf;
    frame.rawData32.dataLow = FIFOx->RDLR;
    frame.rawData32.dataHigh = FIFOx->RDHR;
    frame.frameData.timestamp_bs = FIFOx->RDTR >> 16;
    frame.frameData.timestamp_ms = TIM2->CNT/2;
    frame.frameData.timestampS = second;
    push_frame(&frame);
    //SendCANToUSB(&frame,1);
    
    /* Release the FIFO */
    /* Release FIFO0 */
    if (FIFOn == CAN_FIFO0)
    {
        CANx->RF0R |= CAN_RF0R_RFOM0;
    }
    /* Release FIFO1 */
    else /* FIFONumber == CAN_FIFO1 */
    {
        CANx->RF1R |= CAN_RF1R_RFOM1;
    }
}
#if 0
#endif
/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(__IO uint32_t nCount)
{
  for(; nCount!= 0;nCount--);
}

#ifdef  USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while(1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
