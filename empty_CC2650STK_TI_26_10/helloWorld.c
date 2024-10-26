/*
 * task1.c
 *
 *  Created on: 25 Sep 2023
 *      Author: ivan
 */


/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h> //SPECIFIC FOR THE CC2650 SENSOR TAG


/* Board Header files */
#include "Board.h"


int main(void) {

 

    // Greetings from console
    System_printf("Hello World\n");
    System_flush();

   // Program ready
   BIOS_start();

   return (0);
}
