/*
 * task1.c
 *
 *  Created on: 25 Sep 2023
 *      Author: ivan
 */

/* C Standard library */
#include <stdio.h>
#include <string.h>

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

#include <ti/drivers/UART.h>


/* Board Header files */
#include "Board.h"

// Task needs its own stack memory
#define STACKSIZE 1024
Char myTaskStack[STACKSIZE];
enum state {IDLE=0, READY};
enum state programState=IDLE;


// Taskifunktio
Void serialTask(UArg arg0, UArg arg1) {

   Char buff[20];

   // UART-kirjaston asetukset
   UART_Handle uart;
   UART_Params uartParams;

   // Alustetaan sarjaliikenne
   UART_Params_init(&uartParams);
   uartParams.writeDataMode = UART_DATA_TEXT;
   uartParams.readDataMode = UART_DATA_TEXT;
   uartParams.readEcho = UART_ECHO_OFF;
   uartParams.readMode=UART_MODE_BLOCKING;
   uartParams.baudRate = 9600; // nopeus 9600baud
   uartParams.dataLength = UART_LEN_8; // 8
   uartParams.parityType = UART_PAR_NONE; // n
   uartParams.stopBits = UART_STOP_ONE; // 1

   // Avataan yhteys laitteen sarjaporttiin vakiossa Board_UART0
   uart = UART_open(Board_UART0, &uartParams);
   if (uart == NULL) {
      System_abort("Error opening the UART");
   }

   // Ikuinen elämä
   while (1) {

      // Let's send the current state
      sprintf(buff,"Current state: %1d\n\r",programState);
      UART_write(uart, buff, strlen(buff));
      // Let's sleep for one second
      Task_sleep(1000000 / Clock_tickPeriod);
   }
}


int main(void) {
   // Creating task parameters
   Task_Params myTaskParams;
   Task_Handle myTaskHandle;

   // Start device and UART
   Board_initGeneral();
   Board_initUART();


   // Initialize task main parameters
   Task_Params_init(&myTaskParams);
   // Define task stack memory
   myTaskParams.stackSize = STACKSIZE;
   myTaskParams.stack = &myTaskStack;
   // Set task priority
   myTaskParams.priority = 2;


   // Create task
   myTaskHandle = Task_create(serialTask, &myTaskParams, NULL);
   if (myTaskHandle == NULL) {
      System_abort("Task create failed");
   }

    // Greetings from console
    System_printf("Task created!\n");
    System_flush();

   // Program ready
   BIOS_start();

   return (0);
}
