#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"

// CJ125 register addresses
#define IDENT_REG_RD 0x4800

#define INIT_REG1_RD 0x6C00
#define INIT_REG1_WR 0x5600

#define INIT_REG2_RD 0x7E00
#define INIT_REG2_WR 0x5A00

#define DIAG_REG_RD 0x7800

// CJ125 status constants

#define CJ125_OK 0
#define CJ125_E_NOPOWER 1
#define CJ125_E_SHORTCIRCUITGND 2
#define CJ125_E_SHORTCIRCUITBAT 3

/* CJ125 */

// #define UB_ANALOG_INPUT_PIN A2 // analog read this pin to get battery
uint16_t cj_status;
float oxy;
float UBAT;
unsigned long delayTime;

// Set these as constants for where you plug them in
#define CS_PIN; // SPI CJ125 Chip select line (NSS)
#define CS_BANK;
// #define HTR_PIN; // PWM output for LSU heater
// #define UR_PIN; // CJ125 UR
// #define UA_PIN; // CJ125 UA
// --------------------------------

static SPI_HandleTypeDef *hspi; 

// PID Stuffffs

double Input, Output, Setpoint;
float kp=1;
float ki=0.3; 
float kd=0.1;


// initialize SPI
void initSPI(){
    HAL_SPI_Init(hspi);
}


// Function for transfering SPI data to the CJ125.
uint16_t COM_SPI(uint16_t TX_data)
{

    uint8_t dataArray[2];
    dataArray[0] = TX_data >> 8;
    dataArray[1] = TX_data & 0xFF;

    // Set chip select pin low, chip in use.
    HAL_GPIO_WritePin(CS_BANK, CS_PIN, RESET);

    // Transmit and receive.
    uint8_t highByte;
    HAL_SPI_Receive(hspi, &dataArray[0], highByte, sizeOf(highByte), HAL_MAX_DELAY);
    uint8_t lowByte;
    HAL_SPI_Receive(hspi, &dataArray[1], lowByte, sizeOf(lowByte), HAL_MAX_DELAY);

    // Set chip select pin high, chip not in use.
    HAL_GPIO_WritePin(CS_BANK, CS_PIN, SET)

    // Assemble response in to a 16bit integer and return the value.
    uint16_t Response = (highByte << 8) + lowByte;
    return Response;
}

float get_bat(ADC_HandleTypeDef &UR_PIN)
{
    // UBAT = float(analogRead(UB_ANALOG_INPUT_PIN) ); ****
    UBAT = float(HAL_ADC_GetValue(UR_PIN))
    UBAT = (UBAT * 15) / 1023;
    return UBAT;
}

int check_id(void)
{
	unsigned char result = 0;
	result = COM_SPI(IDENT_REG_RD);
	if ((result&0xF8)!=0x60) return -1;
	else return (result&0x07);
}

int check_stat(void)
{
	unsigned char result=0;
    result = COM_SPI(DIAG_REG_RD);
	result = result>>6;
	switch (result)
	{
		case 0: return CJ125_E_SHORTCIRCUITGND; break;
		case 1: return CJ125_E_NOPOWER; break;
		case 2: return CJ125_E_SHORTCIRCUITBAT; break;
		case 3: return CJ125_OK; break;
	}	
	return -1;
}

int calibrate(float UBAT, ADC_HandleTypeDef &UR_PIN) // UBAT from other function
{
    //There is a risk of water condensed into the O2 sensor, so the proper pre-heating procedure must be maintainted.
    //From what Bosch says, it folllows:
    // - enter the CJ125 calibration mode
    // - apply 2V to the heater for 4 seconds
    // - apply 8.5V to the heater, and within each consecutive second increase by 0.4V up to the battery voltage
    // - store UR value as a PWM reference point, quit CJ125 calibration
    // - change over to PWM heater control mode

	COM_SPI(INIT_REG1_WR|0x9D);	//entering calibration mode
	if (UBAT < 8.5) return -1;	// UBAT is less than 8.5V, hardware problem
	//two volts are equal to  136 from ADC but for convenience, lets stick to the float calculation :)
  	
    float pwm_factor=(2/UBAT)*255; // convert this into 4 bytes to put into SETVALUE

	// analogWrite(HTR_PIN, byte(pwm_factor)); ***
    HAL_DAC_SetValue(HTR_PIN, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ); // find channel for htr_pin, output from pid (can go up to 32 bits of resolution)



	delay(1000); delay(1000); delay (1000); delay (1000);
	float UHTR = 8.5;
	while (UHTR < UBAT)
	{	
        pwm_factor = (UHTR / UBAT) * 255;	//o2 sensor preheating sequence, starting from 8.5V and increasing 0.4V per second
		UHTR+=0.4;
		delay(1000);
		// analogWrite(HTR_PIN,byte(pwm_factor)); ****
        HAL_DAC_SetValue(HTR_PIN, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ); // find channel for htr_pin, output from pid (can go up to 32 bits of resolution)
	}
	// analogWrite(HTR_PIN,0);			//end of pre-heating, power off the heater **
    HAL_DAC_SetValue(HTR_PIN, DAC_CHANNEL_1, DAC_ALIGN_12B_R, RESET); // find channel for htr_pin, output from pid (can go up to 32 bits of resolution)
	Setpoint = HAL_ADC_GetValue(UR_PIN); //UR_PIN
	COM_SPI(INIT_REG1_WR|0x89);	//quit the calibration mode
	return 0;
}

void run(ADC_HandleTypeDef &UR_PIN, DAC_HandleTypeDef &HTR_PIN)
{
	Input = HAL_ADC_GetValue(UR_PIN); // UR_PIN
	// pid.Compute(); make a pid function for this
	// analogWrite(HTR_PIN,byte(Output)); ***
    HAL_DAC_SetValue(HTR_PIN, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ); // find channel for htr_pin, output from pid (can go up to 32 bits of resolution)
}

float get_oxygen(ADC_HandleTypeDef &UA_PIN) 
{
    uint16_t value;
    value = HAL_ADC_GetValue(UA_PIN);  // UA_PIN
    //Declare and set default return value.


    // convert nernst voltage to oxygen concentration and just return that value


    // //Validate ADC range for lookup table.
    // if (value > 854) value = 854;
    
    // if (value >= 307 && value <= 854) {
    //   afr = pgm_read_float_near(afr_table + (value - 307));
    // }
    
    //Return value.
    // return afr;
    // ---------------------------

    
}


