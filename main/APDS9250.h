
/****************************************************/
/**\name	I2C ADDRESS DEFINITIONS  */
/***************************************************/
#define APDS9250_ADDRESS 0x52

/****************************************************/
/**\name	REGISTER ADDRESS DEFINITIONS  */
/***************************************************/
#define APDS9250_REG_MAIN_CTRL (0x00)      // LS operation mode control, SW reset
#define APDS9250_REG_LS_MEAS_RATE (0x04)    // LS measurement rate and resolution in active mode
#define APDS9250_REG_LS_GAIN (0x05)         // LS analog gain range
#define APDS9250_REG_PART_ID (0x06)         // Part number ID and revision ID
#define APDS9250_REG_MAIN_STATUS (0x07)     // Power-on status, interrupt status, data status
#define APDS9250_REG_LS_DATA_IR_0 (0x0A)    // IR ADC measurement data - LSB
#define APDS9250_REG_LS_DATA_IR_1 (0x0B)    // IR ADC measurement data
#define APDS9250_REG_LS_DATA_IR_2 (0x0C)    // IR ADC measurement data - MSB
#define APDS9250_REG_LS_DATA_GREEN_0 (0x0D) // ALS/Green ADC measurement data - LSB
#define APDS9250_REG_LS_DATA_GREEN_1 (0x0E) // ALS/Green ADC measurement data
#define APDS9250_REG_LS_DATA_GREEN_2 (0x0F) // ALS/Green ADC measurement data - MSB
#define APDS9250_REG_LS_DATA_BLUE_0 (0x10)  // Blue ADC measurement data - LSB
#define APDS9250_REG_LS_DATA_BLUE_1 (0x11)  // Blue ADC measurement data
#define APDS9250_REG_LS_DATA_BLUE_2 (0x12)  // Blue ADC measurement data - MSB
#define APDS9250_REG_LS_DATA_RED_0 (0x13)   // Red ADC measurement data - LSB
#define APDS9250_REG_LS_DATA_RED_1 (0x14)   // Red ADC measurement data
#define APDS9250_REG_LS_DATA_RED_2 (0x15)   // Red ADC measurement data - MSB
#define APDS9250_REG_INT_CFG (0x19)         // Interrupt configuration
#define APDS9250_REG_INT_PERSISTENCE (0x1A) // Interrupt persist setting
#define APDS9250_REG_LS_THRES_UP0 (0x21)    // LS interrupt upper threshold, LSB
#define APDS9250_REG_LS_THRES_UP1 (0x22)    // LS interrupt upper threshold
#define APDS9250_REG_LS_THRES_UP2 (0x23)    // LS interrupt upper threshold, MSB
#define APDS9250_REG_LS_THRES_LOW0 (0x24)   // LS interrupt lower threshold, LSB
#define APDS9250_REG_LS_THRES_LOW1 (0x25)   // LS interrupt lower threshold
#define APDS9250_REG_LS_THRES_LOW2 (0x26)   // LS interrupt lower threshold, MSB
#define APDS9250_REG_LS_THRES_VAR (0x27)    // LS interrupt variance threshold

/****************************************************/
/**\name	MEASURE MODE DEFINITIONS  */
/***************************************************/
#define APDS9250_ALS_MODE (0x02)
#define APDS9250_CS_MODE (0x06)

/****************************************************/
/**\name	MEASUREMENT RATE AND RESOLUTION DEFINITIONS  */
/***************************************************/
#define APDS9250_MEASURE_RATE_DEFAUT (0x12)  //100ms 18bits presion


/****************************************************/
/**\name	MEASUREMENT LIGHT SENSOR GAIN DEFINITIONS  */
/***************************************************/
#define APDS9250_LS_GAIN_1 0x00
#define APDS9250_LS_GAIN_3 0x01
#define APDS9250_LS_GAIN_6 0x02
#define APDS9250_LS_GAIN_9 0x03
#define APDS9250_LS_GAIN_18 0x04



/*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
int8_t APDS9250_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
int8_t APDS9250_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);