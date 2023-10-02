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

#define UB_ANALOG_INPUT_PIN A2 // analog read this pin to get battery
uint16_t cj_status;
float oxy;
float UBAT;
unsigned long delayTime;

// Set these as constants for where you plug them in
#define CS_PIN; // SPI CJ125 Chip select line (NSS)
#define CS_BANK;
#define HTR_PIN; // PWM output for LSU heater
#define UR_PIN; // CJ125 UR
#define UA_PIN; // CJ125 UA
// --------------------------------

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
    HAL_SPI_Receive(&hspi, &dataArray[0], highByte, sizeOf(highByte), HAL_MAX_DELAY);
    uint8_t lowByte;
    HAL_SPI_tReceive(&hspi, &dataArray[1], lowByte, sizeOf(lowByte), HAL_MAX_DELAY);

    // Set chip select pin high, chip not in use.
    HAL_GPIO_WritePin(CS_BANK, CS_PIN, SET)

    // Assemble response in to a 16bit integer and return the value.
    uint16_t Response = (highByte << 8) + lowByte;
    return Response;
}

