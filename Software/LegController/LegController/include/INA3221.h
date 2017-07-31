/*
 * INA3221.h
 *
 * Created: 26.02.2017 19:35:17
 *  Author: Alexander Miller
 */ 


#ifndef INA3221_H_
#define INA3221_H_

#pragma region DEFINES

#define INA3221_ADD 0x41

#pragma region INA3221 REGISTERS

#define INA_CFG_R 0x00 //All-register reset
#define INA_C1_SV_R 0x01 //Avg. shunt voltage
#define INA_C1_BV_R 0x02 //Avg. bus voltage
#define INA_C2_SV_R 0x03 //Avg. shunt voltage
#define INA_C2_BV_R 0x04 //Avg. bus voltage
#define INA_C3_SV_R 0x05 //Avg. shunt voltage
#define INA_C3_BV_R 0x06 //Avg. bus voltage
#define INA_C1_CRIT_LIMIT_R 0x07 //Critical alert limit
#define INA_C1_WARN_LIMIT_R 0x08 //Warning alert limit
#define INA_C2_CRIT_LIMIT_R 0x09 //Critical alert limit
#define INA_C2_WARN_LIMIT_R 0x0A //Warning alert limit
#define INA_C3_CRIT_LIMIT_R 0x0B //Critical alert limit
#define INA_C3_WARN_LIMIT_R 0x0C //Warning alert limit
#define INA_SV_SUM_R 0x0D //Shunt voltage sum
#define INA_SV_SUM_LIMIT_R 0x0E //Shunt voltage limit
#define INA_MASK_ENABLE_R 0x0F //Mask/Enable
#define INA_PV_UPPER_LIMIT_R 0x10 //Power-valid Upper Limit
#define INA_PV_LOWER_LIMIT_R 0x11 //Power-valid Lower Limit
#define INA_MANUFACTURER_ID_R 0xFE //Manufacturer ID = 0x5449 = TI
#define INA_DIE_ID_R 0xFF //Die ID = 0x3220

#pragma endregion INA3221 REGISTERS

#pragma region INA3221 CONFIG

#define INA_RST_B 0x8000
#define INA_CH1_EN_B 0x4000
#define INA_CH2_EN_B 0x2000
#define INA_CH3_EN_B 0x1000
#define INA_AVG_MODE_1_B 0x0
#define INA_AVG_MODE_4_B 0x200
#define INA_AVG_MODE_16_B 0x400
#define INA_AVG_MODE_64_B 0x600
#define INA_AVG_MODE_128_B 0x800
#define INA_AVG_MODE_256_B 0xA00
#define INA_AVG_MODE_512_B 0xC00
#define INA_AVG_MODE_1024_B 0xE00
#define INA_BS_CONV_TIME_140us_B 0x0
#define INA_BS_CONV_TIME_204us_B 0x40
#define INA_BS_CONV_TIME_332us_B 0x80
#define INA_BS_CONV_TIME_588us_B 0xC0
#define INA_BS_CONV_TIME_1100us_B 0x100
#define INA_BS_CONV_TIME_2116us_B 0x140
#define INA_BS_CONV_TIME_4156us_B 0x180
#define INA_BS_CONV_TIME_8244us_B 0x1C0
#define INA_SV_CONV_TIME_140us_B 0x0
#define INA_SV_CONV_TIME_204us_B 0x8
#define INA_SV_CONV_TIME_332us_B 0x10
#define INA_SV_CONV_TIME_588us_B 0x18
#define INA_SV_CONV_TIME_1100us_B 0x20
#define INA_SV_CONV_TIME_2116us_B 0x28
#define INA_SV_CONV_TIME_4156us_B 0x30
#define INA_SV_CONV_TIME_8244us_B 0x38
#define INA_OP_MODE_POWER_DOWN0_B 0x0
#define INA_OP_MODE_SV_SINGLE_SHOT_B 0x1
#define INA_OP_MODE_BV_SINGLE_SHOT_B 0x2
#define INA_OP_MODE_SV_AND_BV_SINGLE_SHOT_B 0x3
#define INA_OP_MODE_POWER_DOWN1_B 0x4
#define INA_OP_MODE_SV_CONTINOUS_B 0x5
#define INA_OP_MODE_BV_CONTINOUS_B 0x6
#define INA_OP_MODE_SV_AND_BV_CONTINOUS_B 0x7

#pragma endregion INA3221 CONFIG

#pragma endregion DEFINES

#pragma region VARIABLES
#pragma endregion VARIABLES

#pragma region FUNCTIONS

void ina3221_set_config(uint16_t config);
int16_t ina3221_read_value(char reg);
uint16_t ina3221_get_current(uint16_t channel);
uint16_t ina3221_calculate_current(uint16_t channel);
void ina3221_init();
void ina3221_trigger_measurement();
uint8_t ina3221_check_ground();

#pragma endregion FUNCTIONS


#endif /* INA3221_H_ */