/* C Standard library */
#include <stdio.h>
#include <string.h>
/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"
#include "sensors/mpu9250.h"
#include "buzzer.h"
/* Task */
#define STACKSIZE 2048
void interpret_mpu(void);
Void buzzTaskFxn(void);
void uartFxn(UART_Handle uart, void *rxBuf, size_t len);
void buttonFxn(PIN_Handle handle, PIN_Id pinId);
//char *movement[50] interpret_mpu(void);
Char taskStack[STACKSIZE];
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
// JTKJ: Teht�v� 3. Tilakoneen esittely

//enum state { WAITING=1, DATA_READY };
//enum state programState = WAITING;
enum main_state {MEASURE_ON=0, MEASURE_OFF};
enum main_state main_mode = MEASURE_ON;
enum state {WAITING=1, DATA_READY, SENDING, RECEIVING};
enum state program_state = WAITING;

char move_msg[100] = ".\r\n \r\n -\r\n \r\n \r\n \r\n"; //double ambientLight = -1000.0;
char uartBuffer[30]; // Vastaanottopuskuri
float ax, ay, az, gx, gy, gz = 0;
uint32_t timestamp = 0;
//float tulokset[5][7];
/*struct Mittaus_mpu {
    float ;
    char horizontal[];
};//movements = {{'...', '---'}, {'--.', '..-'}};*/

static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;

static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;
static PIN_Handle hBuzzer;
static PIN_State sBuzzer;

PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};
PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};
// JTKJ: Teht�v� 1. Lis�� painonappien RTOS-muuttujat ja alustus
// JTKJ: Exercise 1. Add pins RTOS-variables and configuration here
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};
// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

Void mpu_sensorFxn(UArg arg0, UArg arg1) {

    //float ax, ay, az, gx, gy, gz;
    I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;
    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);
    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }
    // MPU setup and calibration
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();
    // Loop forever
    while (1) {
        if (main_mode == MEASURE_ON && program_state == WAITING){
        // MPU ask data
            timestamp = Clock_getTicks();
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
            program_state = DATA_READY;
            interpret_mpu();
        }
        // Sleep 100ms
        Task_sleep(100000 / Clock_tickPeriod);
    }

    // Program never gets here..
    // MPU close i2c
    // I2C_close(i2cMPU);
    // MPU power off
    // PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);
}
void interpret_mpu(void){
    //while (1){
        if (main_mode == MEASURE_ON && program_state == DATA_READY){
            //esim. jos ax > 2*az && ax > 2*ay >> liike ylös tms.
            //move_msg = ;
            //float mittaus_data[7] = {timestamp, ax, ay, az, gx, gy, gz};
            char mittausdata[70];
            sprintf(mittausdata, "%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", timestamp, ax, ay, az, gx, gy, gz);
            System_printf(mittausdata);//(move_msg);
            System_flush();
            if(ax > 2.0 | ax < -2.0){ //= LIUKUU??
                char move_msg_temp[100] = ".\r\n.\r\n.\r\n \r\n.\r\n-\r\n.\r\n.\r\n \r\n.\r\n.\r\n \r\n-\r\n.\r\n.\r\n \r\n.\r\n \r\n \r\n \r\n"; //="SLIDE"
                sprintf(move_msg, "%s", move_msg_temp);
            }
        }
        program_state = SENDING;
        buzzTaskFxn();

        //Task_sleep(1000000 / Clock_tickPeriod);
    //}
}

//TOISESTA NAPISTA VIRTA PÄÄLLE/POIS...
void buttonFxn(PIN_Handle handle, PIN_Id pinId){
    /*uint_t pinValue = PIN_getOutputValue(Board_LED0);
    pinValue = !pinValue;
    PIN_setOutputValue(ledHandle, Board_LED0, pinValue);*/
    // JTKJ: Teht�v� 1. Vilkuta jompaa kumpaa ledi�
    // JTKJ: Exercise 1. Blink either led of the device
    if (main_mode == MEASURE_ON){
        main_mode = MEASURE_OFF;
    }else if (main_mode == MEASURE_OFF){
        main_mode = MEASURE_ON;
    }
}
Void buzzTaskFxn(void){//(UArg arg0, UArg arg1) {
    //while (1) {
    if (program_state == RECEIVING){ //SENDING){
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(2000);
        Task_sleep(50000 / Clock_tickPeriod);
        buzzerClose();
        //Task_sleep(950000 / Clock_tickPeriod);
    }else if (program_state == SENDING){//RECEIVING){
        char morsetus[30] = ".\r\n-\r\n  \r\n.\r\n-\r\n";
        sprintf(uartBuffer, "%s", morsetus);
        int i;
        for (i=0;i<sizeof(uartBuffer);i++){
            switch(uartBuffer[i]){
            case '.':
                buzzerOpen(hBuzzer);
                buzzerSetFrequency(2000);
                Task_sleep(35000 / Clock_tickPeriod);
                buzzerClose();
                break;
            case '-':
                buzzerOpen(hBuzzer);
                buzzerSetFrequency(2000);
                Task_sleep(70000 / Clock_tickPeriod);
                buzzerClose();
                break;
            case ' ':
                Task_sleep(70000 / Clock_tickPeriod);
                break;
            }
        }
        program_state = WAITING;
    }

}
/* Task Functions */
void uartFxn(UART_Handle uart, void *rxBuf, size_t len){
    if (program_state == WAITING){
        program_state = RECEIVING;
    }
    if (program_state == RECEIVING){
        buzzTaskFxn();
    }
    UART_read(uart, uartBuffer, 1);
}

Void uartTaskFxn(UArg arg0, UArg arg1) {

    // JTKJ: Teht�v� 4. Lis�� UARTin alustus: 9600,8n1
    UART_Handle uart;
    UART_Params uartParams;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_CALLBACK;//UART_MODE_BLOCKING;
    uartParams.readCallback  = &uartFxn; // Käsittelijäfunktio
    uartParams.baudRate = 9600; // 9600 baud rate
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; //
    uartParams.stopBits = UART_STOP_ONE; // 1

    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
       System_abort("Error opening the UART");
    }
    // Nyt tarvitsee käynnistää datan odotus
    UART_read(uart, uartBuffer, 1);
    while (1){ //(main_state == MEASURE_ON){ (1) {
        //UART_read(uart, uartBuffer, 1);
        /*if (program_state == RECEIVING){ //&& main_mode == MEASURE_ON){
            buzztaskFxn();
        }*/
        if (main_mode == MEASURE_ON && program_state == SENDING){  //for (i = 0; i < 3; i++){ //(programState == DATA_READY){
            /*char move_morse[50];  //char ambientLight_str[10];
            snprintf(move_morse, sizeof(move_morse), "%s\n", move_msg);//snprintf(ambientLight_str, sizeof(ambientLight_str), "%.1f\n", ambientLight);
            System_printf(move_morse);//ambientLight_str);
            System_flush();*/
            // JTKJ: Exercise 3. Print out sensor data as string to debug window if the state is correct
        //       Remember to modify state

        // JTKJ: Teht�v� 4. L�het� sama merkkijono UARTilla
        // JTKJ: Exercise 4. Send the same sensor data string with UART
            char forUart[50]; //[10];
            snprintf(forUart, sizeof(forUart), "%s\n\r", move_msg); //ambientLight_str);
            UART_write(uart, forUart, strlen(forUart));
            program_state = WAITING;//WAITING;
        // Just for sanity check for exercise, you can comment this out
            //System_printf("uartTask\n");
            //System_flush();
        }
        // Once per second, you can modify this
        Task_sleep(1000000 / Clock_tickPeriod);
    }
}

Void sensorTaskFxn(UArg arg0, UArg arg1) {

    I2C_Handle      i2c;
    I2C_Params      i2cParams;

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    i2c = I2C_open(Board_I2C_TMP, &i2cParams);

    if (i2c == NULL) {
       System_abort("Error Initializing I2C\n");
    }
    I2C_close(i2c);
    // JTKJ: Teht�v� 2. Avaa i2c-v�yl� taskin k�ytt��n
    // JTKJ: Exercise 2. Open the i2c bus
    //Task_sleep(100000 / Clock_tickPeriod);
    //opt3001_setup(&i2c);
    // JTKJ: Teht�v� 2. Alusta sensorin OPT3001 setup-funktiolla
    //       Laita enne funktiokutsua eteen 100ms viive (Task_sleep)

    while (0) {
        if (main_mode == MEASURE_ON && program_state == WAITING){
          // ambientLight = opt3001_get_data(&i2c);
        // JTKJ: Teht�v� 2. Lue sensorilta dataa ja tulosta se Debug-ikkunaan merkkijonona
           program_state = DATA_READY;
        // JTKJ: Teht�v� 3. Tallenna mittausarvo globaaliin muuttujaan
        //       Muista tilamuutos
        // Just for sanity check for exercise, you can comment this out
            System_printf("sensorTask\n");
            System_flush();
        }
        // Once per second, you can modify this
        Task_sleep(1000000 / Clock_tickPeriod);
    }
}

Int main(void) {
    // Task variables
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
    Task_Handle task;
    Task_Params taskParams;
    Task_Handle buzzTask;
    Task_Params buzzTaskParams;
    // Initialize board
    Board_initGeneral();

    Board_initI2C();
    Board_initUART();
    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }
    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
      if (hBuzzer == NULL) {
        System_abort("Pin open failed!");
      }

    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
          System_abort("Error initializing button pins\n");
       }
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
          System_abort("Error initializing LED pins\n");
       }
    if(PIN_registerIntCb(buttonHandle, &buttonFxn) != 0){
        System_abort("Error registering button callback function");
    }
    // JTKJ: Teht�v� 1. Ota painonappi ja ledi ohjelman k�ytt��n
    //       Muista rekister�id� keskeytyksen k�sittelij� painonapille
    // JTKJ: Exercise 1. Open the button and led pins
    //       Remember to register the above interrupt handler for button
    Task_Params_init(&buzzTaskParams);
    taskParams.stackSize = STACKSIZE;
    taskParams.stack = &taskStack;
    buzzTask = Task_create((Task_FuncPtr)buzzTaskFxn, &buzzTaskParams, NULL);
    if (buzzTask == NULL) {
      System_abort("Buzzertask create failed!");
    }

    Task_Params_init(&taskParams);
    taskParams.stackSize = STACKSIZE;
    taskParams.stack = &taskStack;
    task = Task_create((Task_FuncPtr)mpu_sensorFxn, &taskParams, NULL);
    if (task == NULL) {
        System_abort("Task create failed!");
    }
    /* Task */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();
    /* Start BIOS */
    BIOS_start();

    return (0);
}
