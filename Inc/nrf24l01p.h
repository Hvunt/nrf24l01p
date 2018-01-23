/**
 * Copyright 2018 Sergey Popov.
 * Licensed under the Apache License, Version 2.0;
 */

#ifndef _NRF24L01P_
#define _NRF24L01P_

#include "main.h"

#ifdef STM32F1
#include "stm32f1xx_hal.h"
#endif
#ifdef STM32F4
#include "stm32f4xx_hal.h"
#endif

/*------===REGISTER MAP===------*/
//#define NRF24_

/*------------CONFIG------------*/
#define NRF24_CONFIG_REG 						((uint8_t)0x00) //Register
#define NRF24_CONFIG_RX_DR 						((uint8_t)0x06)	//here and below the number is the position of the bit
#define NRF24_CONFIG_MASK_TX_DS 				((uint8_t)0x05)
#define NRF24_CONFIG_MASK_MAX_RT 				((uint8_t)0x04)
#define NRF24_CONFIG_EN_CRC 					((uint8_t)0x03)
#define NRF24_CONFIG_CRC0 						((uint8_t)0x02)
#define NRF24_CONFIG_PWR_UP 					((uint8_t)0x01)
#define NRF24_CONFIG_PRIM_RX 					((uint8_t)0x00)

/*------------EN_AA------------*/
#define NRF24_EN_AA_REG 						((uint8_t)0x01) //Register
#define NRF24_EN_AA_P5 							((uint8_t)0x05)
#define NRF24_EN_AA_P4 							((uint8_t)0x04)
#define NRF24_EN_AA_P3 							((uint8_t)0x03)
#define NRF24_EN_AA_P2 							((uint8_t)0x02)
#define NRF24_EN_AA_P1 							((uint8_t)0x01)
#define NRF24_EN_AA_P0 							((uint8_t)0x00)

/*-----------EN_RXADDR----------*/
#define NRF24_EN_RXADDR_REG						((uint8_t)0x02) //Register
#define NRF24_EN_RXADDR_P5						((uint8_t)0x05)
#define NRF24_EN_RXADDR_P4						((uint8_t)0x04)
#define NRF24_EN_RXADDR_P3						((uint8_t)0x03)
#define NRF24_EN_RXADDR_P2						((uint8_t)0x02)
#define NRF24_EN_RXADDR_P1						((uint8_t)0x01)
#define NRF24_EN_RXADDR_P0						((uint8_t)0x00)

/*-----------STATUS-------------*/
#define NRF24_STATUS_REG 						((uint8_t)0x07) //Register
#define NRF24_STATUS_RX_DR 						((uint8_t)0x06)
#define NRF24_STATUS_TX_DS 						((uint8_t)0x05)
#define NRF24_STATUS_MAX_RT 					((uint8_t)0x04)
#define NRF24_STATUS_RX_P_NO 					((uint8_t)0x03) //3 bits, 3:1
#define NRF24_STATUS_TX_FULL 					((uint8_t)0x00)

/*------------RFCH--------------*/
#define NRF24_RF_CH_REG		 					((uint8_t)0x05)
#define NRF24_RF_CH				 				((uint8_t)0x06)	//7 bits, 6:0

/*-----------RFSETUP------------*/
#define NRF24_RFSETUP_REG 						((uint8_t)0x06) //Register
#define NRF24_RFSETUP_CONT_WAVE					((uint8_t)0x07)
#define NRF24_RFSETUP_RF_DR_LOW					((uint8_t)0x05) //see Data rate block
#define NRF24_RFSETUP_PLL_LOCK					((uint8_t)0x04)
#define NRF24_RFSETUP_RF_DR_HIGH				((uint8_t)0x03)	//see Data rate block
#define NRF24_RFSETUP_RF_PWR					((uint8_t)0x02) //2 bits, 2:1. See RF output power in TX block

/*---------ADDRESS_REGS---------*/
#define NRF24_RX_ADDR_P0 							((uint8_t)0x0A) //Register
#define NRF24_RX_ADDR_P1 							((uint8_t)0x0B) //Register
#define NRF24_RX_ADDR_P2 							((uint8_t)0x0C) //Register
#define NRF24_RX_ADDR_P3 							((uint8_t)0x0D) //Register
#define NRF24_RX_ADDR_P4 							((uint8_t)0x0E) //Register
#define NRF24_RX_ADDR_P5 							((uint8_t)0x0F) //Register
#define NRF24_TX_ADDR								((uint8_t)0x10) //Register

/*---------PAYLOAD_REGS---------*/
#define NRF24_RX_PW_P0 								((uint8_t)0x11) //Register
#define NRF24_RX_PW_P1 								((uint8_t)0x12) //Register
#define NRF24_RX_PW_P2 								((uint8_t)0x13) //Register
#define NRF24_RX_PW_P3 								((uint8_t)0x14) //Register
#define NRF24_RX_PW_P4 								((uint8_t)0x15) //Register
#define NRF24_RX_PW_P5 								((uint8_t)0x16) //Register

/*------===REGISTER MAP===------*/
#define NRF24_FIFO_STATUS_REG						((uint8_t)0x17) //Register
#define NRF24_FIFO_STATUS_TX_REUSE					((uint8_t)0x06)
#define NRF24_FIFO_STATUS_TX_FULL					((uint8_t)0x05)
#define NRF24_FIFO_STATUS_TX_EMPTY					((uint8_t)0x04)
#define NRF24_FIFO_STATUS_RX_FULL					((uint8_t)0x01)
#define NRF24_FIFO_STATUS_RX_EMPTY					((uint8_t)0x00)

/*-----===COMMAND WORDS===------*/
#define NRF24_COMMAND_READ_REG						((uint8_t)0x00) //LSByte first
#define NRF24_COMMAND_WRITE_REG						((uint8_t)0x20) //LSByte first. Only in standby or power down modes
#define NRF24_COMMAND_FLUSH_TX						((uint8_t)0xE1)
#define NRF24_COMMAND_FLUSH_RX						((uint8_t)0xE2)
#define NRF24_COMMAND_NOP							((uint8_t)0xFF)
#define NRF24_COMMAND_R_RX_PAYLOAD					((uint8_t)0x61)
#define NRF24_COMMAND_W_TX_PAYLOAD					((uint8_t)0xA0)
#define NRF24_COMMAND_W_TX_PAYLOAD_NOACK			((uint8_t)0xB0)

/*Modes*/
#define NRF24_MODE_RX 								1
#define NRF24_MODE_TX 								0

/*RF output power in TX*/
#define NRF24_RF_PWR_m18DBM							((uint8_t)0x00)
#define NRF24_RF_PWR_m12DBM							((uint8_t)0x01)
#define NRF24_RF_PWR_m6DBM							((uint8_t)0x02)
#define NRF24_RF_PWR_0DBM							((uint8_t)0x03)

/*Data Rate*/
#define NRF24_RF_DR_1MBPS							((uint8_t)0x00)	//[RF_DR_LOW, RF_DR_HIGH]
#define NRF24_RF_DR_2MBPS							((uint8_t)0x08)
#define NRF24_RF_DR_250KBPS							((uint8_t)0x20)

/*Pins managment*/
#define NRF24_CSN_OFF 								GPIO_PIN_RESET
#define NRF24_CSN_ON 								GPIO_PIN_SET
#define NRF24_CE_OFF								GPIO_PIN_RESET
#define NRF24_CE_ON 								GPIO_PIN_SET

/*Auto Acknoledgement state*/
#define NRF24_AUTOACK_OFF							0x00
#define NRF24_AUTOACK_ON							0x01

typedef struct{
	GPIO_TypeDef* GPIO_port_csn;
	GPIO_TypeDef* GPIO_port_ce;
	uint16_t ce_pin;
	uint16_t csn_pin;
} NRF24_CECSN_pin;

//maybe sometimes...
//typedef struct{
//	uint8_t pipe_number;
//	uint8_t pipe_address[5];
//	uint8_t pipe_payload_witdth;
//} pipe_struct;
//
//typedef struct{
//	pipe_struct pipe_0;
//	pipe_struct pipe_1;
//	pipe_struct pipe_2;
//	pipe_struct pipe_3;
//	pipe_struct pipe_4;
//	pipe_struct pipe_5;
//
//
//} NRF24_DEVICE_struct;

uint8_t NRF24_ReadBit(uint8_t reg, uint8_t bit);
uint8_t NRF24_ReadReg(uint8_t reg);
void NRF24_WriteBit(uint8_t reg, uint8_t bit, uint8_t state);
void NRF24_WriteReg(uint8_t reg, uint8_t data);

void NRF24_FlushTx(void);
void NRF24_FlushRx(void);

void NRF24_StartListening(void);
void NRF24_CloseListening(void);

void NRF24_Receive(uint8_t *data, uint8_t length);
void NRF24_Transmit(uint8_t *data, uint8_t length);

void NRF24_CE_ONOFF(uint8_t state);
void NRF24_CSN_ONOFF(uint8_t state);

void NRF24_WakeUp(void);
void NRF24_FallAsleep(void);

uint8_t NRF24_GetStatus(void);
uint8_t NRF24_GetConfig(void);
uint8_t NRF24_GetStatusFIFO(void);
uint8_t NRF24_GetPA(void);
uint8_t NRF24_GetChannel(void);
uint8_t NRF24_GetMode(void);
uint8_t NRF24_GetDataRate(void);
uint8_t NRF24_GetPayloadSize(uint8_t pipe);
uint8_t* NRF24_GetRxAdress(uint8_t pipe);
void NRF24_GetTxAdress(uint8_t* address, uint8_t length);
uint8_t NRF24_GetAutoAck_All(void);
uint8_t NRF24_GetAutoAck_Pipe(uint8_t pipe);
void NRF24_SetPA(uint8_t value);
void NRF24_SetChannel(uint8_t channel);
void NRF24_SetMode(uint8_t mode);
void NRF24_SetDataRate(uint8_t value);
void NRF24_SetPayloadSize(uint8_t value, uint8_t pipe);
void NRF24_OpenRxPipe(uint8_t* address, uint8_t pipe);
void NRF24_SetTxAdress(uint8_t* address);
void NRF24_SetAutoAck_All(uint8_t state);
void NRF24_SetAutoAck_Pipe(uint8_t state, uint8_t pipe);

void NRF24_EnableCRC(void);
void NRF24_DisableCRC(void);

void NRF24_Init(SPI_HandleTypeDef *hspiP, GPIO_TypeDef *GPIO_Port_CE,
		GPIO_TypeDef *GPIO_Port_CSN, uint16_t ce_pin, uint16_t csn_pin);

#endif
