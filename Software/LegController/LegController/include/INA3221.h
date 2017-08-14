/*
 * INA3221.h
 *
 * Created: 26.02.2017 19:35:17
 *  Author: Alexander Miller
 */ 


#ifndef INA3221_H_
#define INA3221_H_

#pragma region DEFINES

/**
 * @def	INA3221_ADD
 *
 * @brief	A macro that defines ina 3221 address
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA3221_ADD 0x41

#pragma region INA3221 REGISTERS

/**
 * @def	INA_CFG_R
 *
 * @brief	A macro that defines ina Configuration register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_CFG_R 0x00 //All-register reset

/**
 * @def	INA_C1_SV_R
 *
 * @brief	A macro that defines ina channel 1 shuntvoltage register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C1_SV_R 0x01 //Avg. shunt voltage

/**
 * @def	INA_C1_BV_R
 *
 * @brief	A macro that defines ina channel 1 busvoltage register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C1_BV_R 0x02 //Avg. bus voltage

/**
 * @def	INA_C2_SV_R
 *
 * @brief	A macro that defines ina channel 2 shuntvoltage register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C2_SV_R 0x03 //Avg. shunt voltage

/**
 * @def	INA_C2_BV_R
 *
 * @brief	A macro that defines ina channel 2 busvoltage register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C2_BV_R 0x04 //Avg. bus voltage

/**
 * @def	INA_C3_SV_R
 *
 * @brief	A macro that defines ina channel 3 shuntvoltage register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C3_SV_R 0x05 //Avg. shunt voltage

/**
 * @def	INA_C3_BV_R
 *
 * @brief	A macro that defines ina channel 3 busvoltage register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C3_BV_R 0x06 //Avg. bus voltage

/**
 * @def	INA_C1_CRIT_LIMIT_R
 *
 * @brief	A macro that defines ina channel 1 critical alert limit register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C1_CRIT_LIMIT_R 0x07 //Critical alert limit

/**
 * @def	INA_C1_WARN_LIMIT_R
 *
 * @brief	A macro that defines ina channel 1 warning alert limit register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C1_WARN_LIMIT_R 0x08 //Warning alert limit

/**
 * @def	INA_C2_CRIT_LIMIT_R
 *
 * @brief	A macro that defines ina channel 2 critical alert limit register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C2_CRIT_LIMIT_R 0x09 //Critical alert limit

/**
 * @def	INA_C2_WARN_LIMIT_R
 *
 * @brief	A macro that defines ina channel 2 warning alert limit register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C2_WARN_LIMIT_R 0x0A //Warning alert limit

/**
 * @def	INA_C3_CRIT_LIMIT_R
 *
 * @brief	A macro that defines ina channel 3 critical alert limit register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C3_CRIT_LIMIT_R 0x0B //Critical alert limit

/**
 * @def	INA_C3_WARN_LIMIT_R
 *
 * @brief	A macro that defines ina channel 3 warning alert limit register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_C3_WARN_LIMIT_R 0x0C //Warning alert limit

/**
 * @def	INA_SV_SUM_R
 *
 * @brief	A macro that defines ina shuntvoltage sum register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_SV_SUM_R 0x0D //Shunt voltage sum

/**
 * @def	INA_SV_SUM_LIMIT_R
 *
 * @brief	A macro that defines ina shunt voltage sum limit register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_SV_SUM_LIMIT_R 0x0E //Shunt voltage limit

/**
 * @def	INA_MASK_ENABLE_R
 *
 * @brief	A macro that defines ina mask enable register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_MASK_ENABLE_R 0x0F //Mask/Enable

/**
 * @def	INA_PV_UPPER_LIMIT_R
 *
 * @brief	A macro that defines ina power valid upper limit register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_PV_UPPER_LIMIT_R 0x10 //Power-valid Upper Limit register

/**
 * @def	INA_PV_LOWER_LIMIT_R
 *
 * @brief	A macro that defines ina power valid lower limit register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_PV_LOWER_LIMIT_R 0x11 //Power-valid Lower Limit

/**
 * @def	INA_MANUFACTURER_ID_R
 *
 * @brief	A macro that defines ina manufacturer Identifier Register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_MANUFACTURER_ID_R 0xFE //Manufacturer ID = 0x5449 = TI

/**
 * @def	INA_DIE_ID_R
 *
 * @brief	A macro that defines ina die Identifier register
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_DIE_ID_R 0xFF //Die ID = 0x3220

#pragma endregion INA3221 REGISTERS

#pragma region INA3221 CONFIG

/**
 * @def	INA_RST_B
 *
 * @brief	A macro that defines ina Reset bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_RST_B 0x8000

/**
 * @def	INA_CH1_EN_B
 *
 * @brief	A macro that defines ina channel 1 enable bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_CH1_EN_B 0x4000

/**
 * @def	INA_CH2_EN_B
 *
 * @brief	A macro that defines ina channel 2 enable bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_CH2_EN_B 0x2000

/**
 * @def	INA_CH3_EN_B
 *
 * @brief	A macro that defines ina channel 3 enable bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_CH3_EN_B 0x1000

/**
 * @def	INA_AVG_MODE_1_B
 *
 * @brief	A macro that defines ina Average mode 1 bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_AVG_MODE_1_B 0x0

/**
 * @def	INA_AVG_MODE_4_B
 *
 * @brief	A macro that defines ina Average mode 4 bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_AVG_MODE_4_B 0x200

/**
 * @def	INA_AVG_MODE_16_B
 *
 * @brief	A macro that defines ina Average mode 16 bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_AVG_MODE_16_B 0x400

/**
 * @def	INA_AVG_MODE_64_B
 *
 * @brief	A macro that defines ina Average mode 64 bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_AVG_MODE_64_B 0x600

/**
 * @def	INA_AVG_MODE_128_B
 *
 * @brief	A macro that defines ina Average mode 128 bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_AVG_MODE_128_B 0x800

/**
 * @def	INA_AVG_MODE_256_B
 *
 * @brief	A macro that defines ina Average mode 256 bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_AVG_MODE_256_B 0xA00

/**
 * @def	INA_AVG_MODE_512_B
 *
 * @brief	A macro that defines ina Average mode 512 bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_AVG_MODE_512_B 0xC00

/**
 * @def	INA_AVG_MODE_1024_B
 *
 * @brief	A macro that defines ina Average mode 1024 bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_AVG_MODE_1024_B 0xE00

/**
 * @def	INA_BS_CONV_TIME_140us_B
 *
 * @brief	A macro that defines ina busvoltage conversion time 140us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_BS_CONV_TIME_140us_B 0x0

/**
 * @def	INA_BS_CONV_TIME_204us_B
 *
 * @brief	A macro that defines ina busvoltage conversion time 204us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_BS_CONV_TIME_204us_B 0x40

/**
 * @def	INA_BS_CONV_TIME_332us_B
 *
 * @brief	A macro that defines ina busvoltage conversion time 332us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_BS_CONV_TIME_332us_B 0x80

/**
 * @def	INA_BS_CONV_TIME_588us_B
 *
 * @brief	A macro that defines ina busvoltage conversion time 588us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_BS_CONV_TIME_588us_B 0xC0

/**
 * @def	INA_BS_CONV_TIME_1100us_B
 *
 * @brief	A macro that defines ina busvoltage conversion time 1100us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_BS_CONV_TIME_1100us_B 0x100

/**
 * @def	INA_BS_CONV_TIME_2116us_B
 *
 * @brief	A macro that defines ina busvoltage conversion time 2116us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_BS_CONV_TIME_2116us_B 0x140

/**
 * @def	INA_BS_CONV_TIME_4156us_B
 *
 * @brief	A macro that defines ina busvoltage conversion time 4156us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_BS_CONV_TIME_4156us_B 0x180

/**
 * @def	INA_BS_CONV_TIME_8244us_B
 *
 * @brief	A macro that defines ina busvoltage conversion time 8244us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_BS_CONV_TIME_8244us_B 0x1C0

/**
 * @def	INA_SV_CONV_TIME_140us_B
 *
 * @brief	A macro that defines ina shuntvoltage conversion time 140us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_SV_CONV_TIME_140us_B 0x0

/**
 * @def	INA_SV_CONV_TIME_204us_B
 *
 * @brief	A macro that defines ina shuntvoltage conversion time 204us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_SV_CONV_TIME_204us_B 0x8

/**
 * @def	INA_SV_CONV_TIME_332us_B
 *
 * @brief	A macro that defines ina shuntvoltage conversion time 332us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_SV_CONV_TIME_332us_B 0x10

/**
 * @def	INA_SV_CONV_TIME_588us_B
 *
 * @brief	A macro that defines ina shuntvoltage conversion time 588us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_SV_CONV_TIME_588us_B 0x18

/**
 * @def	INA_SV_CONV_TIME_1100us_B
 *
 * @brief	A macro that defines ina shuntvoltage conversion time 1100us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_SV_CONV_TIME_1100us_B 0x20

/**
 * @def	INA_SV_CONV_TIME_2116us_B
 *
 * @brief	A macro that defines ina shuntvoltage conversion time 2116us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_SV_CONV_TIME_2116us_B 0x28

/**
 * @def	INA_SV_CONV_TIME_4156us_B
 *
 * @brief	A macro that defines ina shuntvoltage conversion time 4156us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_SV_CONV_TIME_4156us_B 0x30

/**
 * @def	INA_SV_CONV_TIME_8244us_B
 *
 * @brief	A macro that defines ina shuntvoltage conversion time 8244us bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_SV_CONV_TIME_8244us_B 0x38

/**
 * @def	INA_OP_MODE_POWER_DOWN0_B
 *
 * @brief	A macro that defines ina Operation mode power down bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_OP_MODE_POWER_DOWN0_B 0x0

/**
 * @def	INA_OP_MODE_SV_SINGLE_SHOT_B
 *
 * @brief	A macro that defines ina Operation mode shuntvoltage single shot bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_OP_MODE_SV_SINGLE_SHOT_B 0x1

/**
 * @def	INA_OP_MODE_BV_SINGLE_SHOT_B
 *
 * @brief	A macro that defines ina Operation mode busvoltage single shot bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_OP_MODE_BV_SINGLE_SHOT_B 0x2

/**
 * @def	INA_OP_MODE_SV_AND_BV_SINGLE_SHOT_B
 *
 * @brief	A macro that defines ina Operation mode shuntvoltage and busvoltage single shot bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_OP_MODE_SV_AND_BV_SINGLE_SHOT_B 0x3

/**
 * @def	INA_OP_MODE_POWER_DOWN1_B
 *
 * @brief	A macro that defines ina Operation mode power down bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_OP_MODE_POWER_DOWN1_B 0x4

/**
 * @def	INA_OP_MODE_SV_CONTINOUS_B
 *
 * @brief	A macro that defines ina Operation mode shuntvoltage continous bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_OP_MODE_SV_CONTINOUS_B 0x5

/**
 * @def	INA_OP_MODE_BV_CONTINOUS_B
 *
 * @brief	A macro that defines ina Operation mode busvoltage continous bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_OP_MODE_BV_CONTINOUS_B 0x6

/**
 * @def	INA_OP_MODE_SV_AND_BV_CONTINOUS_B
 *
 * @brief	A macro that defines ina Operation mode shuntvoltage and busvoltage continous bitmask
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define INA_OP_MODE_SV_AND_BV_CONTINOUS_B 0x7

#pragma endregion INA3221 CONFIG

#pragma endregion DEFINES

#pragma region VARIABLES
#pragma endregion VARIABLES

#pragma region FUNCTIONS

/**
 * @fn	void ina3221_set_config(uint16_t config);
 *
 * @brief	Set configuration of INA3221
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	config	The configuration.
 */

void ina3221_set_config(uint16_t config);

/**
 * @fn	int16_t ina3221_read_value(char reg);
 *
 * @brief	Read register value (16bit)
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	reg	The register.
 *
 * @return	An int16_t.
 */

int16_t ina3221_read_value(char reg);

/**
 * @fn	uint16_t ina3221_get_current(uint16_t channel);
 *
 * @brief	Get shunt voltage value for given channel
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	channel	The channel to read.
 *
 * @return	Shunt voltage.
 */

uint16_t ina3221_get_current(uint16_t channel);

/**
 * @fn	uint16_t ina3221_calculate_current(uint16_t channel);
 *
 * @brief	Calculate the current based on the shunt voltage
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	channel	The channel.
 *
 * @return	Current in mA.
 */

uint16_t ina3221_calculate_current(uint16_t channel);

/**
 * @fn	void ina3221_init();
 *
 * @brief	Initializes the sensor
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void ina3221_init();

/**
 * @fn	void ina3221_trigger_measurement();
 *
 * @brief	Trigger one measurement
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void ina3221_trigger_measurement();

/**
 * @fn	uint8_t ina3221_check_ground();
 *
 * @brief	Check for ground contact by measuring current
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @return	An uint8_t.
 */

uint8_t ina3221_check_ground();

#pragma endregion FUNCTIONS


#endif /* INA3221_H_ */