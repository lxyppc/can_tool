/******************** (C) COPYRIGHT 2011 lxyppc ********************
* File Name          : can.h
* Author             : MCD Application Team
* Version            : V1.0.0
* Date               : 27-Dec-2011
* Description        : can data process
********************************************************************************
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H
#define __CAN_H
#include "stm32f10x.h"

#define CAN_CHANNEL_COUNT 			2
#define CAN_SEND_BUF_COUNT			256   // 256 frames
#define CAN_RECV_BUF_COUNT			256   // 256 frames
#define RECV_MASK                   (CAN_RECV_BUF_COUNT-1)
#define SEND_MASK                   (CAN_SEND_BUF_COUNT-1)
#define CAN_FRAME_TYPE              0x3c


#define FT_DATA         		0
#define FT_ERROR        		1
#define FT_ERR_DLC_TRANSMIT 	0	
#define FT_ERR_DLC_ERROR	    1
#define FT_RECV_MASK			2
#define FT_TRST_MASK			1
typedef union tag_CANFrame{

    struct {
		uint32_t    dataType:4;     /* data type */
		uint32_t    channel:4;      /* data channel number */
		uint32_t    dlc:4;          /* data length */
        uint32_t    timestampNS:20; /* Frame timestamp in ns */
        uint32_t    timestampS;     /* Frame timestamp in s */
        uint32_t    timestamp_ms:16;
        uint32_t    timestamp_bs:16;
        uint32_t    id:29;          /* Frame ID */
        uint32_t    extend:1;       /* Extend or not */
        uint32_t    remote:1;       /* Remote or not */
        uint32_t    direction:1;    /* 0 is recieve by spider, 1 is send by spider */        
        uint8_t     data[8];        /* data content */
    }frameData;
    uint8_t     rawData[24];
    struct {
	    uint32_t   type_dlc_channel;
        uint32_t   timeS;
        uint32_t   timeMSBS;
        uint32_t   id_status;
        uint32_t   dataLow;
        uint32_t   dataHigh;
    }rawData32;
}CANFrame, *PCANFrame;

/**
 Combine the CAN configure in a 32bit value
 */
typedef union tagBTR{
    struct
    {
        uint32_t SILENT : 1;
        uint32_t LOOKBACK : 1;
		uint32_t RESISTOR : 1;
        uint32_t reserved1 : 3;
        uint32_t SJW : 2;
        uint32_t reserved2 : 1;
        uint32_t TS2 : 3;
        uint32_t TS1 : 4;
        uint32_t reserved3 : 5;
        uint32_t BRP : 11;
    }REG;
    uint32_t raw;
}BTR;

typedef struct tag_DeviceSetting
{
    uint32_t  channelCount; // device channel count
    uint32_t  CANfreq;      // CAN periph freq
    uint32_t  btr[4];       // Bit timing register value
    uint32_t  baudrate[4];
}DeviceSetting, *PDeviceSetting;

uint32_t SendCANToUSB(const CANFrame* pFrame, uint32_t count);
uint32_t SendUSBToCAN(const CANFrame* pFrame, uint32_t count);
void InitCANBuffer(void);
void InitialUSBTransfer(void);
void CANTestLoop(void);

void InitialTransmit(void);
void OnTransmitDone(CAN_TypeDef* CANx, uint32_t mailBoxNumber, uint32_t status);
void CAN_Recv_Done(uint32_t chn, uint32_t FIFOn);
void CAN_Error(CAN_TypeDef* CANx, uint32_t esr, uint32_t mode);
uint32_t GetBaudRate(CAN_TypeDef* CANx);

#endif

