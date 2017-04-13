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

#define INA_RST_B (1<<15)
#define INA_CH1_EN_B (1<<14)
#define INA_CH2_EN_B (1<<13)
#define INA_CH3_EN_B (1<<12)
#define INA_AVG_MODE_1_B (0<<9)
#define INA_AVG_MODE_4_B (1<<9)
#define INA_AVG_MODE_16_B (2<<9)
#define INA_AVG_MODE_64_B (3<<9)
#define INA_AVG_MODE_128_B (4<<9)
#define INA_AVG_MODE_256_B (5<<9)
#define INA_AVG_MODE_512_B (6<<9)
#define INA_AVG_MODE_1024_B (7<<9)
#define INA_BS_CONV_TIME_140us_B (0<<6)
#define INA_BS_CONV_TIME_204us_B (1<<6)
#define INA_BS_CONV_TIME_332us_B (2<<6)
#define INA_BS_CONV_TIME_588us_B (3<<6)
#define INA_BS_CONV_TIME_1100us_B (4<<6)
#define INA_BS_CONV_TIME_2116us_B (5<<6)
#define INA_BS_CONV_TIME_4156us_B (6<<6)
#define INA_BS_CONV_TIME_8244us_B (7<<6)
#define INA_SV_CONV_TIME_140us_B (0<<3)
#define INA_SV_CONV_TIME_204us_B (1<<3)
#define INA_SV_CONV_TIME_332us_B (2<<3)
#define INA_SV_CONV_TIME_588us_B (3<<3)
#define INA_SV_CONV_TIME_1100us_B (4<<3)
#define INA_SV_CONV_TIME_2116us_B (5<<3)
#define INA_SV_CONV_TIME_4156us_B (6<<3)
#define INA_SV_CONV_TIME_8244us_B (7<<3)
#define INA_OP_MODE_POWER_DOWN0_B (0<<0)
#define INA_OP_MODE_SV_SINGLE_SHOT_B (1<<3)
#define INA_OP_MODE_BV_SINGLE_SHOT_B (2<<3)
#define INA_OP_MODE_SV_AND_BV_SINGLE_SHOT_B (3<<3)
#define INA_OP_MODE_POWER_DOWN1_B (4<<3)
#define INA_OP_MODE_SV_CONTINOUS_B (5<<3)
#define INA_OP_MODE_BV_CONTINOUS_B (6<<3)
#define INA_OP_MODE_SV_AND_BV_CONTINOUS_B (7<<3)

#pragma endregion INA3221 CONFIG

#pragma endregion DEFINES

#pragma region VARIABLES
#pragma endregion VARIABLES

#pragma region FUNCTIONS

void ina3221_set_config(char register, uint16_t config);
void ina3221_read_value(char register);

#pragma endregion FUNCTIONS


#endif /* INA3221_H_ */