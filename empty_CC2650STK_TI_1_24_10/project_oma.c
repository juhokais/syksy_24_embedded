/* C Standard library */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
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
Void buzzTaskFxn(UArg arg0, UArg arg1);
void uartFxn(UART_Handle uart, void *rxBuf, size_t len);
//void uartFxn(UART_Handle uart);
void buttonFxn(PIN_Handle handle, PIN_Id pinId);
void buttonFxn_pwr(PIN_Handle handle_pwr, PIN_Id pinId_pwr);
void interpretUart(void);
//void interpretUart(void *rxBuf, size_t len);
void buzzing(int duration);

Char taskStack[STACKSIZE];
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
Char buzzTaskStack[STACKSIZE];// TÄMÄ LISÄTTY

//enum state { WAITING=1, DATA_READY };
//enum state programState = WAITING;
enum main_state {MEASURE_ON=0, MEASURE_OFF};
enum main_state main_mode = MEASURE_ON;
enum state {WAITING=1, DATA_READY, SENDING, RECEIVING};
enum state program_state = WAITING;

char move_msg[30] = ".\r\n \r\n-\r\n \r\n \r\n \r\n"; //double ambientLight = -1000.0;
char uartBuffer[30] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '\0'}; // Vastaanottopuskuri
float ax, ay, az, gx, gy, gz = 0;
uint32_t timestamp = 0;
float ax_sum, ay_sum, az_sum = 0;

static PIN_Handle buttonHandle;
static PIN_State buttonState;
//static PIN_Handle ledHandle;
//static PIN_State ledState;
static PIN_Handle buttonHandle_pwr;
static PIN_State buttonState_pwr;

static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;
static PIN_Handle hBuzzer;
static PIN_State sBuzzer;

PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};
PIN_Config buttonConfig_pwr[] = {
   Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};
/*PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};*/
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
        //System_printf("MPU...\n");
        //System_flush();
        if (main_mode == MEASURE_ON && program_state == WAITING){
        // MPU ask data
            //System_printf("Kohta mittaus...");
            //System_flush();
            timestamp = Clock_getTicks();
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
            program_state = DATA_READY;
            interpret_mpu();
            //System_printf("Mitattu...");
            //System_flush();
            //Task_sleep(100000 / Clock_tickPeriod);
        }
        Task_sleep(100000 / Clock_tickPeriod);//TÄHÄN 100000!!!!
    }
    // Program never gets here..
    // MPU close i2c
    // I2C_close(i2cMPU);
    // MPU power off
    // PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);
}
float *ax_po = &ax;
float *ay_po = &ay;
float *az_po = &az;
float *gx_po = &gx;
float *gy_po = &gy;
float *gz_po = &gz;
float *ax_sum_po = &ax_sum;
float *ay_sum_po = &ay_sum;
float *az_sum_po = &az_sum;

void interpret_mpu(void){
    //while (1){
        if (main_mode == MEASURE_ON && program_state == DATA_READY){
            //ax_sum, ay_sum = 0.0;
            char mittausdata[70];
            sprintf(mittausdata, "%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", timestamp, *ax_po, *ay_po, *az_po, *gx_po, *gy_po, *gz_po);
            System_printf(mittausdata);//(move_msg);
            System_flush();
            if (fabs(*ax_po) > 0.03){
                *ax_sum_po += *ax_po;
            }
            if (fabs(ay) > 0.03){
                *ay_sum_po += *ay_po;
            }
            if (fabs(az+1.0) > 0.01){
                *az_sum_po += (*az_po + 1.0);
            }
            if (((fabs(*ax_sum_po) + fabs(*ay_sum_po)) > 0.04) && (fabs(*az_sum_po) <= 0.15)){ //ax > 2.0 | ax < -2.0){ //= LIUKUU??
                //ax_sum, ay_sum = 0.0;
                char move_msg_temp[100] = ".\r\n.\r\n.\r\n \r\n.\r\n-\r\n.\r\n.\r\n \r\n.\r\n.\r\n \r\n-\r\n.\r\n.\r\n \r\n.\r\n \r\n \r\n \r\n"; //="SLIDE"
                sprintf(move_msg, "%s\n\r", move_msg_temp);
                //char axx[30];
                //sprintf(axx, "LIUKU"); //ax: %.3f ax_sum: %.3f\n", *ax_po, *ax_sum_po);
                System_printf("LIUKU"); //axx); //"LIUKU%.3fH%.3f", ax_sum, ay_sum);
                System_flush();
                program_state = SENDING;
                //buzzTaskFxn(0, 0);  //TÄMÄ PÄÄLLE!!!
                buzzing(1);
                buzzerClose();
                *ax_sum_po = 0.0;
                *ay_sum_po = 0.0;
            }else if (fabs(*az_sum_po) > 0.015){
                char move_msg_temp[100] = ".\r\n-\r\n-\r\n-\r\n \r\n.\r\n.\r\n-\r\n \r\n-\r\n-\r\n \r\n.\r\n-\r\n-\r\n.\r\n \r\n \r\n \r\n"; //="JUMP"
                sprintf(move_msg, "%s\n\r", move_msg_temp);
                System_printf("JUMP");
                System_flush();
                program_state = SENDING;
                //buzzTaskFxn(0, 0);  //TÄMÄ PÄÄLLE!!!
                buzzing(1);
                buzzerClose();
                *az_sum_po = 0.0;
            }else{
                program_state = WAITING;
            }

        }
        //program_state = SENDING;
        //buzzTaskFxn(0, 0);  //TÄMÄ PÄÄLLE!!!
        //Task_sleep(1000000 / Clock_tickPeriod);
    //}
}

//TOISESTA NAPISTA VIRTA PÄÄLLE/POIS...!!!
void buttonFxn_pwr(PIN_Handle handle_pwr, PIN_Id pinId_pwr){
    //POWER OFF!!!
    System_printf("Sammuu...pr%d...main%d\n", program_state, main_mode);
    System_flush();
}
void buttonFxn(PIN_Handle handle, PIN_Id pinId){
    /*uint_t pinValue = PIN_getOutputValue(Board_LED0);
    pinValue = !pinValue;
    PIN_setOutputValue(ledHandle, Board_LED0, pinValue);*/
    // JTKJ: Teht�v� 1. Vilkuta jompaa kumpaa ledi�
    if (main_mode == MEASURE_ON){
        main_mode = MEASURE_OFF;
    }else if (main_mode == MEASURE_OFF){
        main_mode = MEASURE_ON;
    }
}
void buzzing(int duration){
    if (duration == 1){
        System_printf("Bz");
        System_flush();
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(2000);
        Task_sleep(20000 / Clock_tickPeriod); //OLI 50000
        buzzerClose();
    }else if (duration == 2){
        System_printf("Bzzz");
        System_flush();
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(2000);
        Task_sleep(150000 / Clock_tickPeriod); //OLI 50000
        buzzerClose();
    }
}
Void buzzTaskFxn(UArg arg0, UArg arg1) { //(void){//(UArg arg0, UArg arg1) {
    //while (1) {
    //Task_sleep(100000 / Clock_tickPeriod);
    //System_printf("Buzzz...\n");
    //System_flush();
    if (arg0 == 0 && arg1 == 0){ //(program_state == SENDING){ //RECEIVING){
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(2000);
        Task_sleep(30000 / Clock_tickPeriod); //OLI 50000
        buzzerClose();
        //Task_sleep(950000 / Clock_tickPeriod);
    }else if (arg0 == 1 && arg1 == 1){ //(program_state == RECEIVING){ //SENDING){ //
        //char morsetus[50] = ".\r\n-\r\n  \r\n.\r\n-\r\n \r\n \r\n \r\n";
        //sprintf(uartBuffer, "%s", morsetus);
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(2000);
        Task_sleep(150000 / Clock_tickPeriod); //OLI 50000
        buzzerClose();
    }
}
//char uartInput;
//void interpretUart(void *rxBuf, size_t len){
void interpretUart(void){
    if (program_state == RECEIVING){
        System_printf("Puskurin: %c#", uartBuffer[0]); //toka: %c, kolmas: %c#", uartBuffer[0], uartBuffer[1], uartBuffer[2]); //neljas:%c, viides: %c#\n", uartBuffer[0], uartBuffer[1], uartBuffer[2], uartBuffer[3], uartBuffer[4]);
        System_flush();
        //int i = 0;
        //int buzzDuration = -1;
        /*if(uartBuffer[0] == '.'){
            buzzTaskFxn(0, 0);
        }else if (uartBuffer[0] == '-'){
            buzzTaskFxn(1, 1);
        }*/
        //for (i=0;i<30;i++){//sizeof(uartBuffer);i++){
        int count = 1;
        int i = 0;
        while(count < 3 ){//|| !terminator){

            //buzzerOpen(hBuzzer);
        //buzzerSetFrequency(5000);
        //Task_sleep(100000 / Clock_tickPeriod);*/
            switch(uartBuffer[i]){
                case '.':
                //Task_sleep(50000 / Clock_tickPeriod);
                    count--;
                    buzzing(1);
                    //i++;
                    //buzzTaskFxn(0, 0);
                    break;
                case '-':
                //buzzerOpen(hBuzzer);
                //buzzerSetFrequency(2000);
                //Task_sleep(200000 / Clock_tickPeriod);
                //buzzerClose();
                    count--;
                    buzzing(2);
                    //i++;
                    //buzzTaskFxn(1, 1);
                    break;
                case ' ':
                    count++;
                    Task_sleep(150000 / Clock_tickPeriod);
                    //i++;
                    break;
                default:
                    count++;
            }
            uartBuffer[i] = 0;
            i++;
        }
        //program_state = WAITING;

    }
    program_state = WAITING;
}
//int count = 1;
void uartFxn(UART_Handle uart, void *rxBuf, size_t len){
    program_state = RECEIVING;
    interpretUart();
    /*if (uartBuffer[0] == '.'){
        buzzing(1);
        //count--;
    }else if (uartBuffer[0] == '-'){
        buzzing(2);
        //count--;
    }else if (uartBuffer[0] == ' '){
        //count++;
        Task_sleep(150000 / Clock_tickPeriod);
    }*/
    //UART_read(uart, uartBuffer, 1);//TÄMÄ OLi 1=yksi
}
//void uart_test(void){

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
    //UART_read(uart, uartBuffer, 1);//TÄMÄ OLI 1=yksi
    while (1){ //(main_state == MEASURE_ON){ (1) {
        /*UART_read(uart, &uartInput, 1);
        System_printf("Eka:%c#", uartInput);
        System_flush();*/
        //if (uartBuffer[0]){
        //Task_sleep(1000 / Clock_tickPeriod);
        //interpretUart();
        /*if (program_state == RECEIVING){ //&& main_mode == MEASURE_ON){
            buzztaskFxn();
        }*/
        UART_read(uart, uartBuffer, 30);
        /*if(uartBuffer[0]){
            program_state = RECEIVING;
            interpretUart();
        }*/
        //UART_write(uart, koe2, strlen(koe2));
        //char koe[50]; // = ".\r\n \r\n-ŗ\n \r\n \r\n \r\n";

        if (program_state == SENDING){ //(main_mode == MEASURE_ON && program_state == SENDING){ //(main_mode == MEASURE_ON && program_state == SENDING){  //for (i = 0; i < 3; i++){ //(programState == DATA_READY){
            /*char move_morse[50];  //char ambientLight_str[10];
            snprintf(move_morse, sizeof(move_morse), "%s\n", move_msg);//snprintf(ambientLight_str, sizeof(ambientLight_str), "%.1f\n", ambientLight);
            System_printf(move_morse);//ambientLight_str);
            System_flush();*/
            // JTKJ: Teht�v� 4. L�het� sama merkkijono UARTilla
            // JTKJ: Exercise 4. Send the same sensor data string with UART
            //char forUart[100]; // = ".\r\n-\r\n \r\n.\r\n.\r\n \r\n \r\n \r\n\n\r"; //[10];
            //snprintf(forUart, sizeof(forUart), "%s\n\r", move_msg);
            System_printf("Liike:%s\n", move_msg); //("Liike:%s\n", forUart);
            System_flush(); //ambientLight_str);

            program_state = WAITING;//WAITING;
        }
        // Once per second, you can modify this
        Task_sleep(100000 / Clock_tickPeriod); //TÄHÄN 1000000!!
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
        System_abort("Error initializing button pin\n");
    }
    buttonHandle_pwr = PIN_open(&buttonState_pwr, buttonConfig_pwr);
    if (!buttonHandle_pwr){
        System_abort("Error initializing button powerpin\n");
    }
    /*ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
          System_abort("Error initializing LED pins\n");
       }*/
    if(PIN_registerIntCb(buttonHandle, &buttonFxn) != 0){
        System_abort("Error registering button callback function");
    }
    if(PIN_registerIntCb(buttonHandle_pwr, &buttonFxn_pwr) != 0){
        System_abort("Error registering powerbutton callback function");
    }
    // JTKJ: Teht�v� 1. Ota painonappi ja ledi ohjelman k�ytt��n
    //       Muista rekister�id� keskeytyksen k�sittelij� painonapille

    Task_Params_init(&buzzTaskParams);
    buzzTaskParams.stackSize = STACKSIZE;//NÄISSÄ taskParams -> buzzTaskParams
    buzzTaskParams.stack = &buzzTaskStack;
    buzzTaskParams.priority=2; //TÄMÄ LISÄTTY
    buzzTask = Task_create((Task_FuncPtr)buzzTaskFxn, &buzzTaskParams, NULL);
    if (buzzTask == NULL) {
      System_abort("Buzzertask create failed!");
    }

    Task_Params_init(&taskParams);
    taskParams.stackSize = STACKSIZE;
    taskParams.stack = &taskStack;
    taskParams.priority=2;
    task = Task_create((Task_FuncPtr)mpu_sensorFxn, &taskParams, NULL);
    if (task == NULL) {
        System_abort("Task create failed!");
    }
    // Task
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
    uartTaskHandle = Task_create((Task_FuncPtr)uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    // Sanity check
    System_printf("Hello world!\n");
    System_flush();
    // Start BIOS
    BIOS_start();

    return (0);
}
