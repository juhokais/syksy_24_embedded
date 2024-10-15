/*
 * opt3001.c
 *
 *  Created on: 22.7.2016
 *  Author: Teemu Leppanen / UBIComp / University of Oulu
 *
 *  Datakirja: http://www.ti.com/lit/ds/symlink/opt3001.pdf
 */

#include <string.h>
#include <math.h>

#include <xdc/runtime/System.h>

#include "sensors/opt3001.h"
#include "Board.h"

void opt3001_setup(I2C_Handle *i2c) {

	I2C_Transaction i2cTransaction;
	char itxBuffer[3];

    i2cTransaction.slaveAddress = Board_OPT3001_ADDR;
    itxBuffer[0] = OPT3001_REG_CONFIG;
    itxBuffer[1] = 0xCE; // continuous mode s.22
    itxBuffer[2] = 0x02;
    i2cTransaction.writeBuf = itxBuffer;
    i2cTransaction.writeCount = 3;
    i2cTransaction.readBuf = NULL;
    i2cTransaction.readCount = 0;

    if (I2C_transfer(*i2c, &i2cTransaction)) {

        System_printf("OPT3001: Config write ok\n");
    } else {
        System_printf("OPT3001: Config write failed!\n");
    }
    System_flush();

}

uint16_t opt3001_get_status(I2C_Handle *i2c) {

	uint16_t e=0;
	I2C_Transaction i2cTransaction;
	char itxBuffer[1];
	char irxBuffer[2];

	/* Read sensor state */
	i2cTransaction.slaveAddress = Board_OPT3001_ADDR;
	itxBuffer[0] = OPT3001_REG_CONFIG;
	i2cTransaction.writeBuf = itxBuffer;
	i2cTransaction.writeCount = 1;
	i2cTransaction.readBuf = irxBuffer;
	i2cTransaction.readCount = 2;

	if (I2C_transfer(*i2c, &i2cTransaction)) {

		e = (irxBuffer[0] << 8) | irxBuffer[1];
	} else {

		e = 0;
	}
	return e;
}

/**************** JTKJ: DO NOT MODIFY ANYTHING ABOVE THIS LINE ****************/

double opt3001_get_data(I2C_Handle *i2c) {

    // JTKJ: Tehtävä 2. Muokkaa funktiota niin että se palauttaa mittausarvon lukseina
    // JTKJ: Exercise 2. Complete this function to return the measured value as lux

	double lux = -1.0; // return value of the function
    // JTKJ: Find out the correct buffer sizes (n) with this sensor?
    // uint8_t txBuffer[ n ];
    // uint8_t rxBuffer{ n ];
	uint8_t txBuffer[1]; //OPT3001_REG_RESULT
	uint8_t rxBuffer[2];
	// JTKJ: Fill in the i2cMessage data structure with correct values
    //       as shown in the lecture material
    I2C_Transaction i2cMessage;
    i2cMessage.slaveAddress = Board_OPT3001_ADDR;
    txBuffer[0] = OPT3001_REG_RESULT;      // Register address to the transmit buffer
    i2cMessage.writeBuf = txBuffer; // Set transmit buffer
    i2cMessage.writeCount = 1;      // Transmitting 1 byte
    i2cMessage.readBuf = rxBuffer;  // Set receive buffer
    i2cMessage.readCount = 2;

	if (opt3001_get_status(i2c) & OPT3001_DATA_READY) {

		if (I2C_transfer(*i2c, &i2cMessage)) {
		    uint8_t expn = rxBuffer[0] >> 4;
		    uint16_t msb = (rxBuffer[0] & 0b00001111) << 8;
		    uint16_t figure1 = msb + rxBuffer[1];
		    lux = 0.01 * (2**expn) * figure1;
	        char lux_string[16];
		    sprintf(lux_string, "%.3f", lux);
	        System_printf(lux_string);
	        System_flush();
		    // JTKJ: Here the conversion from register value to lux

		} else {

			System_printf("OPT3001: Data read failed!\n");
			System_flush();
		}

	} else {
		System_printf("OPT3001: Data not ready!\n");
		System_flush();
	}

	return lux;
}
