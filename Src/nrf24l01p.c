/**
 * Copyright 2018 Sergey Popov.
 * Licensed under the Apache License, Version 2.0;
 */

#include "nrf24l01p.h"

static SPI_HandleTypeDef *hspi;
static NRF24_CECSN_pin NRF24_CECSN_pins;

//uint8_t temp_nrf;

void NRF24_Init(SPI_HandleTypeDef *hspiP, GPIO_TypeDef *GPIO_Port_CE,
		GPIO_TypeDef *GPIO_Port_CSN, uint16_t ce_pin, uint16_t csn_pin) {

	hspi = hspiP;
	NRF24_CECSN_pins.GPIO_port_ce = GPIO_Port_CE;
	NRF24_CECSN_pins.GPIO_port_csn = GPIO_Port_CSN;
	NRF24_CECSN_pins.ce_pin = ce_pin;
	NRF24_CECSN_pins.csn_pin = csn_pin;
	NRF24_CSN_ONOFF(NRF24_CSN_ON);

	HAL_Delay(10); // wait of settings

	NRF24_WakeUp();

//	NRF24_SetChannel(80);
//	NRF24_SetDataRate(NRF24_RF_DR_1MBPS);
//	NRF24_SetPA(NRF24_RF_PWR_0DBM);
//	NRF24_SetPayloadSize(4, 0);

}
/*===================================================*/
/*===================================================*/
void NRF24_CE_ONOFF(uint8_t state) {
	HAL_GPIO_WritePin(NRF24_CECSN_pins.GPIO_port_ce, NRF24_CECSN_pins.ce_pin,
			state);
}

void NRF24_CSN_ONOFF(uint8_t state) {
	HAL_GPIO_WritePin(NRF24_CECSN_pins.GPIO_port_csn, NRF24_CECSN_pins.csn_pin,
			state);
}
/*===================================================*/
/*===================================================*/
void NRF24_WakeUp(void) {
	uint8_t buffer = NRF24_ReadReg(NRF24_CONFIG_REG);
	buffer |= 1 << NRF24_CONFIG_PWR_UP;
	NRF24_WriteReg(NRF24_CONFIG_REG, buffer);
	HAL_Delay(5);
}

void NRF24_FallAsleep(void) {
	uint8_t buffer = NRF24_ReadReg(NRF24_CONFIG_REG);
	buffer &= ~(1 << NRF24_CONFIG_PWR_UP);
	NRF24_WriteReg(NRF24_CONFIG_REG, buffer);
}
/*===================================================*/
/*===================================================*/

void NRF24_SetPA(uint8_t value) {
	uint8_t buffer;
	buffer = NRF24_ReadReg(NRF24_RFSETUP_REG);
	//clear bits
	buffer &= ~(3 << (NRF24_RFSETUP_RF_PWR - 1));
	switch (value) {
	case NRF24_RF_PWR_m18DBM:
		/*nothing, because -18dBm is 0x00*/
		break;
	case NRF24_RF_PWR_m12DBM:
		buffer |= 1 << NRF24_RF_PWR_m12DBM;
		break;
	case NRF24_RF_PWR_m6DBM:
		buffer |= 1 << NRF24_RF_PWR_m6DBM;
		break;
	case NRF24_RF_PWR_0DBM:
		buffer |= NRF24_RF_PWR_0DBM << 1;
		break;
	}

	NRF24_WriteReg(NRF24_RFSETUP_REG, buffer);
}

uint8_t NRF24_GetPA(void) {
	uint8_t buffer = 0;
//	buffer = NRF24_ReadReg(NRF24_RFSETUP_REG);
//	buffer = buffer >> 1;
//	buffer = buffer & 1;
//	switch(buffer){
//		case NRF24_RF_PWR_m18DBM:
//			buffer = NRF24_RF_PWR_m18DBM;
//		case NRF24_RF_PWR_m12DBM:
//			buffer = NRF24_RF_PWR_m12DBM;
//		case NRF24_RF_PWR_m6DBM:
//			buffer = NRF24_RF_PWR_m6DBM;
//		case NRF24_RF_PWR_0DBM:
//			buffer = NRF24_RF_PWR_0DBM;
//	}
	return buffer;
}

void NRF24_SetDataRate(uint8_t value) {
//	uint8_t buffer;
//	buffer = NRF24_ReadReg(NRF24_RFSETUP_REG);
//	//clear bits
//	buffer &= ~(1 << NRF24_RFSETUP_RF_DR_HIGH);
//	buffer &= ~(1 << NRF24_RFSETUP_RF_DR_LOW);
	switch (value) {
	case NRF24_RF_DR_1MBPS:
		NRF24_WriteBit(NRF24_RFSETUP_REG, NRF24_RFSETUP_RF_DR_HIGH, RESET);
		NRF24_WriteBit(NRF24_RFSETUP_REG, NRF24_RFSETUP_RF_DR_LOW, RESET);
		break;
	case NRF24_RF_DR_2MBPS:
		NRF24_WriteBit(NRF24_RFSETUP_REG, NRF24_RFSETUP_RF_DR_HIGH, SET);
		NRF24_WriteBit(NRF24_RFSETUP_REG, NRF24_RFSETUP_RF_DR_LOW, RESET);
		break;
	case NRF24_RF_DR_250KBPS:
		NRF24_WriteBit(NRF24_RFSETUP_REG, NRF24_RFSETUP_RF_DR_HIGH, RESET);
		NRF24_WriteBit(NRF24_RFSETUP_REG, NRF24_RFSETUP_RF_DR_LOW, SET);
		break;
	}
}

uint8_t NRF24_GetDataRate(void) {
	uint8_t buffer = 0;
//	buffer = NRF24_ReadReg(NRF24_RFSETUP_REG);
////	buffer &= 0x28;
//	switch(buffer){
//		case NRF24_RF_DR_1MBPS:
//			buffer = NRF24_RF_DR_1MBPS;
//		case NRF24_RF_DR_2MBPS:
//			buffer = NRF24_RF_DR_2MBPS;
//		case NRF24_RF_DR_250KBPS:
//			buffer = NRF24_RF_DR_250KBPS;
//	}
	return buffer;
}

void NRF24_SetChannel(uint8_t channel) {
	if (channel < 127) {
		NRF24_WriteReg(NRF24_RF_CH_REG, channel);
	}
}

uint8_t NRF24_GetChannel(void) {
	return NRF24_ReadReg(NRF24_RF_CH_REG);
}

void NRF24_SetMode(uint8_t mode) {
	uint8_t buffer = NRF24_ReadReg(NRF24_CONFIG_REG);

	if (mode != NRF24_MODE_TX)
		buffer |= 1 << NRF24_CONFIG_PRIM_RX;
	else
		buffer &= ~(1 << NRF24_CONFIG_PRIM_RX);
	NRF24_WriteReg(NRF24_CONFIG_REG, buffer);
}

uint8_t NRF24_GetMode(void) {
	uint8_t buffer = NRF24_ReadReg(NRF24_CONFIG_REG);
	return (buffer >> NRF24_CONFIG_PRIM_RX) & 1;
}

void NRF24_SetPayloadSize(uint8_t number_of_bytes, uint8_t pipe) {
	switch (pipe) {
	case 0:
		NRF24_WriteReg(NRF24_RX_PW_P0, number_of_bytes);
		break;
	case 1:
		NRF24_WriteReg(NRF24_RX_PW_P1, number_of_bytes);
		break;
	case 2:
		NRF24_WriteReg(NRF24_RX_PW_P2, number_of_bytes);
		break;
	case 3:
		NRF24_WriteReg(NRF24_RX_PW_P3, number_of_bytes);
		break;
	case 4:
		NRF24_WriteReg(NRF24_RX_PW_P4, number_of_bytes);
		break;
	case 5:
		NRF24_WriteReg(NRF24_RX_PW_P5, number_of_bytes);
		break;
	}
}
uint8_t NRF24_GetPayloadSize(uint8_t pipe) {
	uint8_t buffer = 0;
	switch (pipe) {
	case 0:
		buffer = NRF24_ReadReg(NRF24_RX_PW_P0);
		break;
	case 1:
		buffer = NRF24_ReadReg(NRF24_RX_PW_P1);
		break;
	case 2:
		buffer = NRF24_ReadReg(NRF24_RX_PW_P2);
		break;
	case 3:
		buffer = NRF24_ReadReg(NRF24_RX_PW_P3);
		break;
	case 4:
		buffer = NRF24_ReadReg(NRF24_RX_PW_P4);
		break;
	case 5:
		buffer = NRF24_ReadReg(NRF24_RX_PW_P5);
		break;
	}
	return buffer;
}

void NRF24_OpenRxPipe(uint8_t *address, uint8_t pipe) {
	uint8_t length = sizeof(address);
	uint8_t buffer[length + 1];

	for (int i = 1; i <= length; i++) {
		buffer[i] = address[i];
	}

	switch (pipe) {
	case 0:
		buffer[0] = NRF24_COMMAND_WRITE_REG | NRF24_RX_ADDR_P0;
		break;
	case 1:
		buffer[0] = NRF24_COMMAND_WRITE_REG | NRF24_RX_ADDR_P1;
		break;
	case 2:
		buffer[0] = NRF24_COMMAND_WRITE_REG | NRF24_RX_ADDR_P2;
		break;
	case 3:
		buffer[0] = NRF24_COMMAND_WRITE_REG | NRF24_RX_ADDR_P3;
		break;
	case 4:
		buffer[0] = NRF24_COMMAND_WRITE_REG | NRF24_RX_ADDR_P4;
		break;
	case 5:
		buffer[0] = NRF24_COMMAND_WRITE_REG | NRF24_RX_ADDR_P5;
		break;
	}

	NRF24_CSN_ONOFF(NRF24_CSN_OFF);
	HAL_SPI_Transmit(hspi, buffer, sizeof(buffer), 100);
	NRF24_CSN_ONOFF(NRF24_CSN_ON);

	NRF24_SetMode(NRF24_MODE_RX);

	NRF24_WriteBit(NRF24_EN_RXADDR_REG, pipe, SET);
}

void NRF24_SetTxAdress(uint8_t *address) {
	uint8_t length = sizeof(address);
	uint8_t buffer[length + 1];

	buffer[0] = NRF24_COMMAND_WRITE_REG | NRF24_TX_ADDR;
	for (int i = 1; i <= length + 1; i++) {
		buffer[i] = address[i];
	}

	NRF24_CSN_ONOFF(NRF24_CSN_OFF);
	HAL_SPI_Transmit(hspi, buffer, sizeof(buffer), 100);
	NRF24_CSN_ONOFF(NRF24_CSN_ON);
}

//TODO: add pipe selection
uint8_t* NRF24_GetRxAdress(uint8_t pipe) {
	return 0;
}

void NRF24_GetTxAdress(uint8_t* address, uint8_t length) {
	uint8_t trans_buffer[length + 1];
	uint8_t receive_buffer[length + 1];

	trans_buffer[0] = NRF24_COMMAND_READ_REG | NRF24_TX_ADDR;
	for (int i = 1; i <= length; i++) {
		trans_buffer[i] = NRF24_COMMAND_NOP;
	}

	NRF24_CSN_ONOFF(NRF24_CSN_OFF);
	HAL_SPI_TransmitReceive(hspi, trans_buffer, receive_buffer, (length + 1),
			100);
	NRF24_CSN_ONOFF(NRF24_CSN_ON);

	for (int i = 0; i <= length; i++) {
		address[i] = receive_buffer[i];
	}
}

uint8_t NRF24_GetStatus(void) {
	return NRF24_ReadReg(NRF24_STATUS_REG);
}

uint8_t NRF24_GetConfig(void) {
	return NRF24_ReadReg(NRF24_CONFIG_REG);
}

uint8_t NRF24_GetStatusFIFO(void) {
	return NRF24_ReadReg(NRF24_FIFO_STATUS_REG);
}

uint8_t NRF24_GetAutoAck_All(void) {
	return NRF24_ReadReg(NRF24_EN_AA_REG);
}

void NRF24_SetAutoAck_All(uint8_t state) {
	if (state == NRF24_AUTOACK_ON)
		NRF24_WriteReg(NRF24_EN_AA_REG, 0x3f);
	else
		NRF24_WriteReg(NRF24_EN_AA_REG, 0x00);
}

void NRF24_EnableCRC(void) {
	NRF24_WriteBit(NRF24_CONFIG_REG, NRF24_CONFIG_EN_CRC, SET);
}
void NRF24_DisableCRC(void) {
	NRF24_WriteBit(NRF24_CONFIG_REG, NRF24_CONFIG_EN_CRC, RESET);
}

//TODO: not worked
uint8_t NRF24_GetAutoAck_Pipe(uint8_t pipe) {
	return 0;
}
void NRF24_SetAutoAck_Pipe(uint8_t state, uint8_t pipe) {

}
/*===================================================*/
/*===================================================*/
uint8_t NRF24_ReadBit(uint8_t reg, uint8_t bit) {
	uint8_t buffer;
	buffer = NRF24_ReadReg(reg);
	buffer = (buffer >> bit) & 1;
	return buffer;
}

uint8_t NRF24_ReadReg(uint8_t reg) {
	uint8_t trans_buffer[2];
	uint8_t receiv_buffer[2];

	trans_buffer[0] = NRF24_COMMAND_READ_REG | reg;
	trans_buffer[1] = NRF24_COMMAND_NOP;

	NRF24_CSN_ONOFF(NRF24_CSN_OFF);
	HAL_SPI_TransmitReceive(hspi, trans_buffer, receiv_buffer, 2, 100);
	NRF24_CSN_ONOFF(NRF24_CSN_ON);

	return receiv_buffer[1];
}
/*===================================================*/
/*===================================================*/
void NRF24_WriteBit(uint8_t reg, uint8_t bit, uint8_t state) {
	uint8_t buffer;

	buffer = NRF24_ReadReg(reg);

	if (state == SET)
		buffer |= 1 << bit;
	else
		buffer &= ~(1 << bit);

	NRF24_WriteReg(reg, buffer);
}

void NRF24_WriteReg(uint8_t reg, uint8_t data) {
	uint8_t buffer[2];
	buffer[0] = NRF24_COMMAND_WRITE_REG | reg;
	buffer[1] = data;

	NRF24_CSN_ONOFF(NRF24_CSN_OFF);
	HAL_SPI_Transmit(hspi, buffer, sizeof(buffer), 100);
	NRF24_CSN_ONOFF(NRF24_CSN_ON);
}
/*===================================================*/
/*===================================================*/

void NRF24_FlushTx(void) {
	uint8_t buffer = NRF24_COMMAND_FLUSH_TX;
	NRF24_CSN_ONOFF(NRF24_CSN_OFF);
	HAL_SPI_Transmit(hspi, &buffer, 1, 100);
	NRF24_CSN_ONOFF(NRF24_CSN_ON);
}
void NRF24_FlushRx(void) {
	uint8_t buffer = NRF24_COMMAND_FLUSH_RX;
	NRF24_CSN_ONOFF(NRF24_CSN_OFF);
	HAL_SPI_Transmit(hspi, &buffer, 1, 100);
	NRF24_CSN_ONOFF(NRF24_CSN_ON);
}

//TODO it is necessary?
void NRF24_StartListening(void) {
	NRF24_CE_ONOFF(NRF24_CE_ON);
	NRF24_SetMode(NRF24_MODE_RX);
}
void NRF24_CloseListening(void) {
	NRF24_CE_ONOFF(NRF24_CE_OFF);
	NRF24_SetMode(NRF24_MODE_TX);
}

void NRF24_Receive(uint8_t *data, uint8_t length) {
	uint8_t trans_buffer[length + 1];
	uint8_t receive_buffer[length + 1];

	trans_buffer[0] = NRF24_COMMAND_R_RX_PAYLOAD;

	uint8_t bit_state = NRF24_ReadBit(NRF24_STATUS_REG, NRF24_STATUS_RX_DR);
	if (bit_state != 0) {
		NRF24_CSN_ONOFF(NRF24_CSN_OFF);
		HAL_SPI_TransmitReceive(hspi, trans_buffer, receive_buffer,
				(length + 1), 100);
		NRF24_CSN_ONOFF(NRF24_CSN_ON);
		for (uint8_t i = 0; i < length; i++)
			data[i] = receive_buffer[i + 1];
	} else {
		for (uint8_t i = 0; i < length; i++) //otherwise data sets old values
			data[i] = 0;
	}

	bit_state = NRF24_ReadBit(NRF24_FIFO_STATUS_REG, NRF24_FIFO_STATUS_RX_FULL);
	if (bit_state != 0)
		NRF24_FlushRx();

	NRF24_WriteReg(NRF24_STATUS_REG,
			(NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)
					<< 4);
}

void NRF24_Transmit(uint8_t *data, uint8_t length) {
	uint8_t buffer[length + 1];
	uint8_t rxbuffer[length + 1];

	buffer[0] = NRF24_COMMAND_W_TX_PAYLOAD;
	for (uint8_t i = 1; i < length + 1; i++) {
		buffer[i] = data[i - 1];
	}

	uint8_t bit_state = NRF24_ReadBit(NRF24_STATUS_REG, NRF24_STATUS_TX_FULL);
	if (bit_state != 0) {
		NRF24_FlushTx();
		HAL_Delay(2);
	}

	NRF24_SetMode(NRF24_MODE_TX);

	NRF24_CSN_ONOFF(NRF24_CSN_OFF);
	HAL_SPI_TransmitReceive(hspi, buffer, rxbuffer, (length + 1), 100);
	NRF24_CSN_ONOFF(NRF24_CSN_ON);

	NRF24_CE_ONOFF(NRF24_CE_ON);
	HAL_Delay(2);
	NRF24_SetMode(NRF24_MODE_RX);
	NRF24_CE_ONOFF(NRF24_CE_OFF);

	NRF24_WriteReg(NRF24_STATUS_REG,
			(NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)
					<< 4);
}
