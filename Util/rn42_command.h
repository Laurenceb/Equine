#pragma once
#include <string.h>

#include "sp1ml_command.h"
#include "stm32f10x.h"
#include "usart.h"
#include "main.h"
#include "watchdog.h"


extern volatile uint8_t RN42_aligned_data_ready;
extern volatile SP1ML_tx_rx_state_machine_type RN42_tx_rx_state;
extern volatile uint8_t RN42_tx_sequence_number;

void RN42_conf(void);
uint8_t RN42_get_name(uint8_t SerialNumber[10], uint8_t allowconfig);
uint8_t RN42_get_command(void);
void RN42_generate_packet(uint8_t* data_payload, uint8_t number_bytes, uint8_t device_network_id, uint8_t* sequence_number);
