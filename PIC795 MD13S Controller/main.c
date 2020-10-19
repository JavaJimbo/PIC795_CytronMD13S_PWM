/**********************************************************************************
 * PROJECT: PIC795 MD13S CONTROLLER
 * main.c
 * 
 * Compiled for PIC32MX795 XC32 compiler version 1.30 
 * For Robotnik MD13S Controller Board Rev 1.0
 * Adapted from Robotnik_Brain_Board
 * 
 * 8-13-20: Set up IO for new board
 * 9-13-20: Created ADC10_ManualInit()
 * 9-14-20: Fixed ADC bug. Polling works great now.
 * 9-15-20: Got all five PWMs working, also four SW inputs with interrupt on change,
 *          also four encoder counters and encoder direction inputs.
 *          Created USE_PID mode and tested PID with one motor.
 *          Fixed a few PID bugs - works better, and doesn't overrun and go wild.
 *          Added previousPosition, quadCurrent, quadPrevious to PIDtype struct.
 *          Enabled PID for all five motors.
 * 9-16-20: Got DMA working with RS485 Rx at 981600 baud.
 * 9-20-20: Got RC Feather servo board working.
 * 9-21-20: 
 * 9-22-20: Got encoders working nicely in Destination mode - enter positions by text.
 * 9-23-20: Eliminated Velocity mode for now.
 * 9-29-20: Testing RC servo motors with Feather board. Baudrate: 115200
 * 10-2-20: Four RC servos working with pots controlling Feather board.
 * 10-3-20: Added Velocity variable to limit speed.
 * 10-17-20:Got CRC working. 
 * 10-18-20:Constant velocity mode is working nicely using Encoder inputs on ServoCity 26:1 motors.
 *          Got forward/backward/right/left working with joystick on XBEE input.
 *          Re-enabled POT mode for servo #0.
 * 10-29-20:Made adjustments to joystick to work with large motors 
 *          and loads for complementary wheelchair wheels.
 ***********************************************************************************/

enum {
    POT_MODE = 0,
    DESTINATION_MODE,
    CONTINUOUS_MODE
};

// #define USE_FEATHER_BOARD
#define ACCELERATION 0.1
#define DEFAULT_MODE CONTINUOUS_MODE
#define USE_PID
#define SUCCESS 0
#define PCA9685_ADDRESS 0x80 // Basic default address for Adafruit Feather Servo Board 


#define	STX '>'
#define	DLE '/'
#define	ETX '\r'

#define MAXPOTS 10
// For 26:1 motors
#define PWM_OFFSET 0 
#define KP 8.0
#define KI 0.01
#define KD 10.0 

#define KK_PWM_OFFSET 100
#define KKP 8.0
#define KKI 0.04
#define KKD 1000.0 // 30

#define MAX_COMMAND_COUNTS 850
#define MIN_COMMAND_COUNTS 100

#define	STX '>'
#define	DLE '/'
#define	ETX '\r'

#define MAXSERVOS 64  // Was 256
#define NUMMOTORS 5

#define FORWARD 0
#define REVERSE 1
#define MAXSUM 500000
#define PWM_MAX 4000

#define FILTERSIZE 32  // Was 4

#include <xc.h>
#include "Delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "Compiler.h"
#include "I2C_4BUS_EEPROM_PIC32.h"

#include "Delay.h"
#include "Defs.h"
#define _SUPPRESS_PLIB_WARNING

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

/** I N C L U D E S **********************************************************/

#define false FALSE
#define true TRUE

enum 
{
    HALTED=0,
    LOCAL,
    JOG,
    REMOTE    
};

#define TEST_OUT LATCbits.LATC1

#define PWM1 OC1RS
#define PWM2 OC2RS
#define PWM3 OC3RS
#define PWM4 OC4RS
#define PWM5 OC5RS

#define EncoderOne TMR1
#define EncoderTwo TMR5
#define EncoderThree TMR3
#define EncoderFour TMR4

#define EncoderOneDir PORTEbits.RE9
#define EncoderTwoDir PORTEbits.RE5
#define EncoderThreeDir PORTEbits.RE7
#define EncoderFourDir PORTEbits.RE8

#define MOTOR_DIR1 LATGbits.LATG12
#define MOTOR_DIR2 LATDbits.LATD13
#define MOTOR_DIR3 LATDbits.LATD11
#define MOTOR_DIR4 LATAbits.LATA5
#define MOTOR_DIR5 LATDbits.LATD10

#define EEPROM_WRITE_PROTECT LATDbits.LATD8
#define LED LATDbits.LATD9
#define RS485_ENABLE LATBbits.LATB5
#define SW1 PORTBbits.RB0
#define SW2 PORTBbits.RB1
#define SW3 PORTBbits.RB2
#define SW4 PORTCbits.RC13
#define SW5 PORTDbits.RD7

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define XBEEuart UART4
#define XBEEbits U4STAbits
#define XBEE_VECTOR _UART_4_VECTOR

#define RS485uart UART5
#define RS485_RxReg U5RXREG
#define RS485_TxReg U5TXREG
#define RS485bits U5STAbits
#define RS485_VECTOR _UART_5_VECTOR
#define RS485_RX_IRQ _UART5_RX_IRQ
#define RS485_TX_IRQ _UART5_TX_IRQ

#define CR 13
#define LF 10
#define BACKSPACE 8
#define SPACE 32
#define ESCAPE 27
#define RIGHT_ARROW 67
#define LEFT_ARROW 68
#define UP_ARROW 65
#define DOWN_ARROW 66
#define MAXNUM 16
#define MAXBUFFER 255

#define CONSTANT_FORWARD 9999
#define CONSTANT_REVERSE -9999

struct PIDtype
{
    long  error[FILTERSIZE];
    long  sumError;    
    float kP;
    float kI;
    float kD;
    long PWMoffset;
    long PWMvalue;
    short ADActual;    
    long  Destination;
    float  CommandPos;    
    long ActualPos;
    float Velocity;
    float TargetVelocity;
    float ActualVelocity[FILTERSIZE];
    short ADCommand;
    short ErrorCounter;
    unsigned char saturation;
    unsigned char reset;
    short previousPosition;
    unsigned short quadCurrent;
    unsigned short quadPrevious;
    unsigned Mode;    
    unsigned char Halted;
} PID[NUMMOTORS];


/** V A R I A B L E S ********************************************************/
unsigned char DMATxTestFlag = false;
unsigned short offSet;
unsigned char NUMbuffer[MAXNUM + 1];
unsigned char HOSTRxBuffer[MAXBUFFER+1];
unsigned char HOSTRxBufferFull = false;
unsigned char HOSTTxBuffer[MAXBUFFER+1]; 

unsigned char displayFlag = true;

unsigned char XBEERxBuffer[MAXBUFFER+1];
unsigned char XBEEPacket[MAXBUFFER+1];
short XBEEData[MAXBUFFER+1];
float ForwardReverse = 0, RightLeft = 0;
short XBEEPacketLength;

unsigned char RS485RxBufferFull = false;
unsigned char RS485TxBufferFull = false;
unsigned char RS485_DMA_RxBuffer[MAXBUFFER+1];
// unsigned char RS485_DMA_TxBuffer[MAXBUFFER+1] = "This is not a test.\r";
unsigned char RS485RxBuffer[MAXBUFFER+1];
unsigned char RS485TxBuffer[MAXBUFFER+1] = "\rThis has got to be the one";
int RS485TxIndex = 0;
int RS485TxLength = 0;
unsigned char ServoData[MAXBUFFER+1];
short servoPositions[MAXSERVOS];
long ActualRS485BaudRate = 0;
int timeout = 0;

extern unsigned char CheckCRC (unsigned char *ptrPacketData, short packetLength);

void ClearCopyBuffer();
void SetupDMA_Rx(void);
int StartRS485Tx(void);
int ADC10_ManualInit(void);
static void InitializeSystem(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void UserInit(void);
void InitializeUSART(void);
void putcUSART(char c);
unsigned char getcUSART();
extern unsigned char CheckCRC (unsigned char *ptrRxModbus, short RxModbusLength);
unsigned short decodePacket(unsigned char *ptrInPacket, unsigned char *ptrData);
void PrintServoData(short numServos, short *ptrServoPositions, unsigned char command, unsigned char subCommand);
unsigned char SendReceiveSPI(unsigned char dataOut);
void ResetPID();
void InitPID();
long POTcontrol(long servoID, struct PIDtype *PID);
long ENCODERcontrol(long servoID, struct PIDtype *PID);
unsigned char processPacketData(short packetLength, unsigned char *ptrPacket, short *numData, short *ptrData, unsigned char *command, unsigned char *subCommand);

unsigned char DATABufferFull = false;
// void ClearCopyBuffer();
void makeFloatString(float InValue, int numDecimalPlaces, unsigned char *arrAscii);

enum
{
    QUAD_NONE = 0,
    QUAD_ONE,
    QUAD_TWO,
    QUAD_THREE
};

enum
{
    STANDBY = 0,
    RECORD,
    PLAY,
    PAUSE_RECORD,
    PAUSE_PLAY,
    HALT
};

short time, seconds = 0, minutes = 0, hundredths = 0;
unsigned char TestMode = false;
unsigned short ADresult[MAXPOTS];
unsigned short SWRead = 0;
unsigned char SWChangeFlag = false;
unsigned char intFlag = false;
unsigned char memoryFlag = false;
unsigned char runState = HALTED;
short errIndex = 0;
unsigned char startPacket = false;
short XBEEtimeout = 0;

#define PCA9685_ADDRESS 0x80 // Basic default address for Adafruit Feather Servo Board


int main(void) 
{
    short i = 0, j = 0, p = 0, q = 0;        
    long PWMvalue = 0;        
    int JogPWM = 0;
    float floValue;            
    short length = 0;
    unsigned short numBytes;
    unsigned char MessageOut[] = "You can eat the driver and his gloves\r\n ";
    unsigned char MessageIn[64] = "";
    unsigned ADdisplay = true;    
    short SWcounter = 0;    
    unsigned char runMode = LOCAL;
    short startAddress = 0x0000;
    #define NUM_RC_SERVOS 4
    short RCservoPos[NUM_RC_SERVOS] = {0,0,0,0};
    short PreviousRCservoPos[NUM_RC_SERVOS] = {0,0,0,0};
    short DisplayCounter = 0;
    short testCounter = 0;
    short MotorFour;
    short ENCdisplayCounter = 0;           
    short dataLength = 0;    
    short numServos;
    short ServoPositions[MAXSERVOS] = {512,512,512,512};    
    unsigned char ch, command, subCommand, outPacket[MAXBUFFER];
    short numDataIntegers, outData[MAXBUFFER], packetLength;      
    short PreviousXBEEData1 = -9999, PreviousXBEEData2 = -9999;
    
    unsigned char CRCerror = false;    
    
    InitPID();     
    DelayMs(10);
    
    InitializeSystem();          
    
    DelayMs(100);
    
    
#ifdef USE_FEATHER_BOARD
    printf("\rInitializing Feather Servo Board at address 0x80: ");
    if (initializePCA9685(PCA9685_ADDRESS)) printf(" SUCCESS");
    else printf(" ERROR");    
}
#endif
    
#ifdef USE_PID    
    if (DEFAULT_MODE == POT_MODE) printf("\r\rDefault mode: POT MODE\r");    
    else if (DEFAULT_MODE == DESTINATION_MODE) printf("\r\rDefault mode: DESTINATION\r");
    else printf("\r\rDefault mode: CONTINUOUS\r");
        
    if (runState==LOCAL) printf("RunMode = LOCAL");
    else if (runState==REMOTE) printf("RunMode = REMOTE");
    else if (runState==JOG) printf("RunMode = JOG");
    else printf("RunMode = STANDBY");    
    
    printf("\rComplementary wheelchair motors on Servos #2 and #3");
#else
    printf("\r\rSERVOS Disabled\r");    
    if (runState==LOCAL) printf("RunMode = LOCAL");
    else if (runState==REMOTE) printf("RunMode = REMOTE");
    else if (runState==JOG) printf("RunMode = JOG");
    else printf("RunMode = STANDBY");    
    
    DelayMs(10);
    printf("\rInitializing I2C #1");
    initI2C(I2C1);
    numBytes = strlen(MessageOut);
    printf("\rWriting %d bytes to EEprom", numBytes);
    EepromWriteBlock (I2C1, EEPROM_ID, startAddress, MessageOut, numBytes);
    printf("\rReading %d bytes from EEprom", numBytes);
    EepromReadBlock (I2C1, EEPROM_ID, startAddress, MessageIn, numBytes);
    MessageIn[numBytes] = '0';
    printf("\rMessage: %s", MessageIn);
#endif
    
    // printf("\rTesting DMA Rx and Tx on RS485");
    printf("\rTESTING XBEE RX...");
    while(1) 
    {   
        if (SWChangeFlag)
        {
            SWChangeFlag = false;
            printf("\r#%d: ", SWcounter++);
            if (SWRead & 0b0001) printf("SW1 HI, ");
            else printf("SW1 LOW, ");
            if (SWRead & 0b0010) printf("SW2 HI, ");
            else printf("SW2 LOW, ");
            if (SWRead & 0b0100) printf("SW3 HI, ");
            else printf("SW3 LOW, ");    
            if (SWRead & 0b1000) printf("SW4 HI");
            else printf("SW4 LOW");                 
        }        
        
        /*
        if (RS485RxBufferFull)
        {
            RS485RxBufferFull = false;
            printf("\rReceived RS485: %s", RS485RxBuffer);
            strcpy(RS485TxBuffer, "\rReceived: ");
            strcat(RS485TxBuffer, RS485RxBuffer);
            StartRS485Tx();
        }
        */
        
        /*
        if (RS485RxBufferFull)
        {            
            RS485RxBufferFull = false;
            dataLength =  decodePacket(RS485RxBuffer, PacketData);            
            if (!processPacketData(dataLength, PacketData, &numData, , &command, &subCommand)) 
                printf("\rCRC ERROR");
            else PrintServoData(numServos, ServoPositions, command, subCommand);                                    
            ClearCopyBuffer();
        }        
        */
        
        if (XBEEPacketLength)   // $$$$
        {            
            if ( processPacketData(XBEEPacketLength, XBEEPacket, &numDataIntegers, XBEEData, &command, &subCommand))
            {
                // if ( abs(PreviousXBEEData1 - XBEEData[1]) > 2 || abs(PreviousXBEEData2 - XBEEData[2]) > 2)
                {
                    PreviousXBEEData1 = XBEEData[1];
                    PreviousXBEEData2 = XBEEData[2];
                    //printf("\r> XBEE length: %d, int #1: %d, int #2: %d, com: %X, sub: %X, numData: %d", XBEEPacketLength, XBEEData[0], XBEEData[1], command, subCommand, numDataIntegers);
                    ForwardReverse = (float) XBEEData[1];
                    #define DEADZONE 10
                    if (ForwardReverse > 0)
                    {
                        ForwardReverse = ForwardReverse - DEADZONE;
                        if (ForwardReverse < 0) ForwardReverse = 0;
                    }
                    else if (ForwardReverse < 0)
                    {
                        ForwardReverse = ForwardReverse + DEADZONE;
                        if (ForwardReverse > 0) ForwardReverse = 0;
                    }
                    
                    RightLeft = (float) XBEEData[0] ;
                    if (RightLeft > 0)
                    {
                        RightLeft = RightLeft - DEADZONE;
                        if (RightLeft < 0) RightLeft = 0;
                    }
                    else if (RightLeft < 0)
                    {
                        RightLeft = RightLeft + DEADZONE;
                        if (RightLeft > 0) RightLeft = 0;
                    }
                    
                    PID[2].TargetVelocity = (RightLeft + ForwardReverse) / 10;
                    PID[3].TargetVelocity = (RightLeft - ForwardReverse) / 10;
                    //printf("\r>XBEE: %d, %d, Velocity: %0.3f, %0.3f", XBEEData[0], XBEEData[1], PID[2].Velocity, PID[3].Velocity);
                }
            }
            //else printf("\rERR> XBEE length: %d, int #1: %d, int #2: %d, com: %X, sub: %X, numData: %d", XBEEPacketLength, XBEEData[0], XBEEData[1], command, subCommand, numDataIntegers);
            XBEEPacketLength = 0;
        }        

#ifdef USE_PID                    
        if (intFlag)
        {
            intFlag = false;
            
            if (TEST_OUT) TEST_OUT = 0;
            else TEST_OUT = 1;                    
            
            // if (DEFAULT_MODE == POT_MODE)
            {
                IFS1bits.AD1IF = 0;
                while(!IFS1bits.AD1IF);        
                AD1CON1bits.ASAM = 0;        // Pause sampling. 
                for (i = 0; i < MAXPOTS; i++)
                    ADresult[i] = (unsigned short) ReadADC10(i); // read the result of channel 0 conversion from the idle buffer
                AD1CON1bits.ASAM = 1;        // Restart sampling.
            }
            
            if (runState)
            {        
#ifdef USE_FEATHER_BOARD
                for (i = 0; i < NUM_RC_SERVOS; i++) RCservoPos[i] = (short)(ADresult[i+6]/4) + 22;
                // printf("\r#%d: 1 = %d, 2 = %d, 3 = %d, 4 = %d", testCounter++, RCservoPos[0], RCservoPos[1], RCservoPos[2], RCservoPos[3]);                
                
                if (PreviousRCservoPos[0]!=RCservoPos[0])
                {
                    PreviousRCservoPos[0]=RCservoPos[0];
                    if (!setPCA9685outputs (PCA9685_ADDRESS, 0, 0, RCservoPos[0])) printf("\r#0 ERROR");
                }
                if (PreviousRCservoPos[1]!=RCservoPos[1])
                {
                    PreviousRCservoPos[1]=RCservoPos[1];
                    if (!setPCA9685outputs (PCA9685_ADDRESS, 1, 0, RCservoPos[1])) printf("\r#1 ERROR");
                }
                if (PreviousRCservoPos[2]!=RCservoPos[2])
                {
                    PreviousRCservoPos[2]=RCservoPos[2];
                    if (!setPCA9685outputs (PCA9685_ADDRESS, 2, 0, RCservoPos[2])) printf("\r#2 ERROR");
                }
                if (PreviousRCservoPos[3]!=RCservoPos[3])
                {
                    PreviousRCservoPos[3]=RCservoPos[3];
                    if (!setPCA9685outputs (PCA9685_ADDRESS, 3, 0, RCservoPos[3])) printf("\r#3 ERROR");
                }                
#endif                
                
                for (i = 0; i < NUMMOTORS; i++)
                {                             
                    if (runState == JOG) 
                        PWMvalue = JogPWM = 0;
                    else
                    {                                                     
                        if (!PID[i].Halted)
                        {
                            if (PID[i].Mode == POT_MODE) 
                                POTcontrol(i, PID);
                            else ENCODERcontrol(i, PID);                        
                            PWMvalue = PID[i].PWMvalue;
                        }
                        else PWMvalue = 0;
                    }
                    if (i == 0) 
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR1 = REVERSE;                              
                            PWMvalue = 0 - PWMvalue;                            
                        }
                        else MOTOR_DIR1 = FORWARD;                                    
                        PWM1 = (unsigned short)PWMvalue;
                        
                    }       
                    else if (i == 1)
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR2 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else MOTOR_DIR2 = FORWARD;
                        PWM2 = (unsigned short)PWMvalue;
                    }                                                
                    else if (i == 2) 
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR3 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else MOTOR_DIR3 = FORWARD;                                                    
                        PWM3 = (unsigned short)PWMvalue;
                    }
                    else if (i == 3)  // Motor #4
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR4 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else MOTOR_DIR4 = FORWARD;          
                        PWM4 = (unsigned short)PWMvalue;
                    }
                    else
                    {
                        if (PWMvalue < 0)
                        {            
                            MOTOR_DIR5 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else MOTOR_DIR5 = FORWARD;                            
                        PWM5 = (unsigned short)PWMvalue;
                    }
                    
                }                                                
                
                errIndex++; 
                if (errIndex >= FILTERSIZE) errIndex = 0;
                
            }
            else            
            {
                for (i = 0; i < NUMMOTORS; i++) PID[i].sumError = 0;
                PWM1 = PWM2 = PWM3 = PWM4 = PWM5 = 0;
                LED = 0;
            }               
        }    
#endif        
        
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false; 
            // printf("\rReceived: %s", HOSTRxBuffer);
            q = 0;
            command = 0;            
            for (p = 0; p < MAXBUFFER; p++) 
            {
                ch = HOSTRxBuffer[p];
                if (ch < ' ' && ch != '\r')
                {
                    command = ch;
                    break;
                }
                if (isalpha(ch) || (ch==' ' && p==0)) command = ch;                
                putchar(ch);
                if (ch == '\r' || ch == ' ')break;
                if ( (isdigit(ch) || ch == '.' || ch == '-') && q < MAXNUM) NUMbuffer[q++] = ch;
            }
            if (q) 
            {
                NUMbuffer[q] = '\0';
                floValue = atof(NUMbuffer);
            }
            if (command) 
            {
                switch (command) 
                {
                    case 'F':
                        if (!runState)
                        {
                            MOTOR_DIR1 = MOTOR_DIR2 = MOTOR_DIR3 = MOTOR_DIR4 = MOTOR_DIR5 = 1;
                            printf("\rFORWARD");
                        }
                        break;
                    case 'R':
                        if (!runState)
                        {                        
                            MOTOR_DIR1 = MOTOR_DIR2 = MOTOR_DIR3 = MOTOR_DIR4 = MOTOR_DIR5 = 0;
                            printf("\rREVERSE");
                        }
                        break;                        
                    case 'P':
                        if (q) PID[0].kP = PID[1].kP = PID[2].kP = PID[3].kP = floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[0].kP, PID[0].kI, PID[0].kD, PID[0].PWMoffset);
                        break;
                    case 'I':
                        if (q) PID[0].kI = PID[1].kI = PID[2].kI = PID[3].kI = floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[0].kP, PID[0].kI, PID[0].kD, PID[0].PWMoffset);
                        break;
                    case 'D':
                        if (q) PID[0].kD = PID[1].kD = PID[2].kD = PID[3].kD = floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[0].kP, PID[0].kI, PID[0].kD, PID[0].PWMoffset);
                        break;
                    case 'O':
                        if (q) PID[0].PWMoffset = PID[1].PWMoffset = PID[2].PWMoffset = PID[3].PWMoffset = (long) floValue;
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[0].kP, PID[0].kI, PID[0].kD, PID[0].PWMoffset);
                        break;
                    case 'X':
                        runMode = REMOTE;
                        printf("REMOTE MODE ON");
                        break;
                    case 'L':
                        runMode = LOCAL;
                        printf("LOCAL ON");
                        break;       
                    case 'J':
                        JogPWM = (long) floValue;
                        printf("\rJOG ON: %d", JogPWM);
                        runState = JOG;                        
                        break;                        
                    case ' ':
                        ResetPID();
                        if (runState) 
                        {                            
                            runState = STANDBY; 
                            printf("\rHALT");
                        }
                        else
                        {
                            runState = runMode;
                            if (runState==LOCAL) printf("\rRun State = LOCAL");
                            else if (runState==REMOTE) printf("\rRun State = REMOTE");
                            else if (runState==JOG) printf("\rRun State = JOG");
                            else printf("\rERROR RunMode = %d", runMode);
                            ENCdisplayCounter = 0;
                            if (PID[3].Velocity > 0) PID[3].Destination = PID[3].Destination + (abs(PID[3].Destination - PID[3].ActualPos));
                            else PID[3].Destination = PID[3].Destination - (abs(PID[3].Destination - PID[3].ActualPos));
                            printf(", Destination: %ld, Velocity: %0.3f", PID[3].Destination, PID[3].Velocity);
                        }                         
                        break;                        
                    case 'M':
                        if (displayFlag)
                        {
                            displayFlag = false;
                            printf("\rDisplay OFF");
                        }
                        else {
                            displayFlag = true;
                            printf("\rDisplay ON");
                        }   
                        break;                        
                    case 'Q':
                        printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[0].kP, PID[0].kI, PID[0].kD, PID[0].PWMoffset);
                        break;                        
                    case 'T':
                        if (ADdisplay)
                        {
                            ADdisplay = false;
                            printf("\rPot display OFF");
                        }
                        else
                        {
                            ADdisplay = true;
                            printf("\rPot display ON");
                        }
                        break;                        
                    case 'V':
                        if (q) PID[3].Velocity = (float)floValue;
                        printf("Velocity: %0.3f", PID[3].Velocity);
                        PID[2].Velocity = PID[3].Velocity;
                        break;                        
                    case 'Z':
                        if (q) PID[3].Destination = (long)floValue;
                        
                        //if (PID[3].Destination > PID[3].previousPosition && PID[3].Velocity < 0)
                        //    PID[3].Velocity = 0 - PID[3].Velocity;                        
                        //else if (PID[3].Destination < PID[3].previousPosition && PID[3].Velocity > 0) 
                        //    PID[3].Velocity = 0 - PID[3].Velocity;                        
                        // PID[2].Velocity = PID[3].Velocity;
                        printf("Destination: %ld, Velocity: %0.3f", PID[3].Destination, PID[3].Velocity);
                        break;                        
                    default:                        
                        if (command < ' ') printf("\rCommand: #%d => %c", command, (command - 1 + 'A'));
                        else printf("\rCommand: %c", command);
                        break;
                } // end switch command                
                
                CONTINUE1: continue;
                command = 0;
            } // End if command             
        } // End if HOSTRxBufferFull        
    } // End while(1))
} // End main())



/*
void ClearCopyBuffer()
{
    int i;
    for (i = 0; i < MAXBUFFER; i++)
    {
        RS485RxBuffer[i] = '\0';
        // RS485_DMA_RxBuffer[i] = '\0';
    }
}
*/


void PrintServoData(short numServos, short *ptrServoPositions, unsigned char command, unsigned char subCommand)
{
    int i;
    printf("\rOK! Com: %d, Sub: %d, servos %d: ", command, subCommand, numServos);
    for (i = 0; i < 10; i++) printf("%d, ", ptrServoPositions[i]);
}





/* 10-18-20
int ADC10_ManualInit(void)
{
    int i, dummy;
    
    AD1CON1bits.ON = 0;
    mAD1IntEnable(INT_DISABLED);   
    mAD1ClearIntFlag();
    
    AD1CON1 = 0;
    AD1CON2 = 0;
    AD1CON3 = 0;
    AD1CHS  = 0;
    AD1CSSL = 0;
    
    // Set each Port B pin for digital or analog
    // Analog = 0, digital = 1
    AD1PCFGbits.PCFG0 = 1; 
    AD1PCFGbits.PCFG1 = 1; 
    AD1PCFGbits.PCFG2 = 1; 
    AD1PCFGbits.PCFG3 = 0; 
    AD1PCFGbits.PCFG4 = 0; 
    AD1PCFGbits.PCFG5 = 1; 
    AD1PCFGbits.PCFG6 = 1; 
    AD1PCFGbits.PCFG7 = 1; 
    AD1PCFGbits.PCFG8 = 0; 
    AD1PCFGbits.PCFG9 = 0; 
    AD1PCFGbits.PCFG10 = 0; 
    AD1PCFGbits.PCFG11 = 0; 
    AD1PCFGbits.PCFG12 = 0; 
    AD1PCFGbits.PCFG13 = 0; 
    AD1PCFGbits.PCFG14 = 0; 
    AD1PCFGbits.PCFG15 = 0;     
    
    AD1CON1bits.FORM = 000;        // 16 bit integer format.
    AD1CON1bits.SSRC = 7;        // Auto Convert
    AD1CON1bits.CLRASAM = 0;    // Normal operation - buffer overwritten by next conversion sequence
    AD1CON1bits.ASAM = 0;        // Not enable Automatic sampling yet.
    
    AD1CON2bits.VCFG = 0;        // Reference AVdd, AVss
    AD1CON2bits.OFFCAL = 0;        // Offset calibration disable.
    AD1CON2bits.CSCNA = 1;        // Scan inputs for CH0+ SHA Input for Mux A input 
    AD1CON2bits.SMPI = 0b1000;        // Interrupt after 9+1 conversion
    AD1CON2bits.BUFM = 0;        // One 16 word buffer
    AD1CON2bits.ALTS = 0;        // Use only Mux A
    AD1CON2bits.SMPI =  MAXPOTS-1;    // Number of channels to sample
    AD1CON2bits.BUFM = 0;                // Single 16-word buffer with CLRASAM.    
    AD1CHSbits.CH0NA = 0; // Mux A Negative input from VR-
    AD1CHSbits.CH0SA = 3; // Mux A Positive input from pin AN3

    // Set conversion clock and set sampling time.
    AD1CON3bits.ADRC = 0;        // Clock derived from peripheral bus clock
    AD1CON3bits.SAMC = 0b11111;        // Sample time max
    AD1CON3bits.ADCS = 0b11111111;   // Conversion time max

    // Select channels to scan. Scan channels = 1, Skip channels = 0
    AD1CSSLbits.CSSL0 = 0;
    AD1CSSLbits.CSSL1 = 0;
    AD1CSSLbits.CSSL2 = 0;
    AD1CSSLbits.CSSL3 = 1;
    AD1CSSLbits.CSSL4 = 1;
    AD1CSSLbits.CSSL5 = 0;
    AD1CSSLbits.CSSL6 = 0;
    AD1CSSLbits.CSSL7 = 0;
    AD1CSSLbits.CSSL8 = 1;
    AD1CSSLbits.CSSL9 = 1;
    AD1CSSLbits.CSSL10 = 1;
    AD1CSSLbits.CSSL11 = 1;
    AD1CSSLbits.CSSL12 = 1;
    AD1CSSLbits.CSSL13 = 1;
    AD1CSSLbits.CSSL14 = 1;
    AD1CSSLbits.CSSL15 = 1;
    
    // Make sure all buffers have been Emptied. 
    for (i = 0; i < 16; i++) dummy = (ADC1BUF0+i*4);    
    
    AD1CON1bits.ASAM = 1;        // Start Automatic Sampling. 
    AD1CON1bits.ON = 1;            // Turn on ADC.
    return (1);
}
*/

/*
void __ISR(_ADC_VECTOR, ipl2) ADInterrupt(void) 
{
    unsigned char i;
    static int counter = 100;

    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    // offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        ADresult[i] = (unsigned short) ReadADC10(i); // read the result of channel 0 conversion from the idle buffer
    counter++;
    if (counter > 100)
    {
        counter = 0;
        ADready = true;    
    }
}
*/


#define ESC 27
#define CR 13
#define BACKSPACE 8
void __ISR(HOST_VECTOR, IPL2AUTO) IntHostUartHandler(void) 
{
unsigned char ch, inByte;
static unsigned long i = 0;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));                 
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;            
        }
        if (UARTReceivedDataIsAvailable(HOSTuart)) 
        {
            inByte = UARTGetDataByte(HOSTuart);
            ch = toupper(inByte);
            if (ch != 0 && ch != '\n') 
            {            
                if ('\r'==ch || (' '==ch && i==0)) 
                {
                    HOSTRxBufferFull = true;
                    HOSTRxBuffer[i++] = ch;
                    HOSTRxBuffer[i] = '\0';
                    i = 0;
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, '\r');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, '\n');
                }                
                else if (ch < ' ')
                {
                    HOSTRxBufferFull = true;
                    HOSTRxBuffer[0] = ch;
                    HOSTRxBuffer[1] = '\0';
                    i = 0;
                }
                else if (ch == BACKSPACE) 
                {
                    if (i != 0) i--;
                    HOSTRxBuffer[i] = '\0';
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, ' ');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, BACKSPACE);                
                } 
                else if (i < MAXBUFFER) 
                {
                    HOSTRxBuffer[i] = ch;
                    i++;
                }            
            }
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));            
}

/******************************************************************************
 *	Change Notice Interrupt Service Routine
 *
 *   Note: Switch debouncing is not performed.
 *   Code comes here if SW2 (CN16) PORTD.RD7 is pressed or released.
 *   The user must read the IOPORT to clear the IO pin change notice mismatch
 *	condition first, then clear the change notice interrupt flag.
 ******************************************************************************/

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) 
{    
    // Step #1 - always clear the mismatch condition first
    SWRead = 0x0000;
    SWRead = PORTB & 0b0000000000000111; // Read RB0, RB1, RB2, mask off the rest of Port B
    if (PORTC & 0b0010000000000000) // Read RC13, mask off the rest of Port C
        SWRead = SWRead | 0b1000;
    SWChangeFlag = true;
    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();
}


// Handler for the DMA channel 0 RECEIVE interrupts
void __ISR(_DMA0_VECTOR, IPL5SOFT) DmaRxHandler(void)
{
    int i;
    unsigned char ch;
	int	evFlags;				// event flags when getting the interrupt

	INTClearFlag(INT_SOURCE_DMA(DMA_CHANNEL0));	// release the interrupt in the INT controller, we're servicing int

	evFlags=DmaChnGetEvFlags(DMA_CHANNEL0);	// get the event flags

    if(evFlags&DMA_EV_BLOCK_DONE)
    { // just a sanity check. we enabled just the DMA_EV_BLOCK_DONE transfer done interrupt
        i = 0;
        do {
            ch = RS485_DMA_RxBuffer[i]; 
            RS485_DMA_RxBuffer[i] = 0x00;
            RS485RxBuffer[i] = ch;
            i++;
        } while (i < MAXBUFFER-1 && ch != ETX);
        RS485RxBuffer[i] = '\0';
        RS485RxBufferFull = true;
        //SetupDMA_Tx();
        timeout = 32;
        DmaChnClrEvFlags(DMA_CHANNEL0, DMA_EV_BLOCK_DONE);       
        DmaChnEnable(DMA_CHANNEL0);
    }
}

// Handler for the DMA channel 1 TRANSMIT interrupts
/*
void __ISR(_DMA1_VECTOR, IPL5SOFT) DmaTxHandler(void)
{
	int	evFlags;				// event flags when getting the interrupt

	INTClearFlag(INT_SOURCE_DMA(DMA_CHANNEL1));	// release the interrupt in the INT controller, we're servicing int

	evFlags=DmaChnGetEvFlags(DMA_CHANNEL1);	// get the event flags
    DMATxTestFlag = true;

    if(evFlags&DMA_EV_BLOCK_DONE)
    { 
        DmaChnClrEvFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE);       
        RS485TxBufferFull = false;        
    }
}
*/
    

void SetupDMA_Rx(void)
{
    int i;
    DelayMs(10);    
    RS485_ENABLE = 0;
    // for (i = 0; i < MAXBUFFER; i++) RS485RxBuffer[i] = 0x00;     
    
    DmaChnOpen(DMA_CHANNEL0, 1, DMA_OPEN_MATCH);    
    DmaChnSetMatchPattern(DMA_CHANNEL0, '\r');	// set < as ending character
    // Set the transfer event control: what event is to start the DMA transfer    
	// We want the UART2 RX interrupt to start our transfer
	// also we want to enable the pattern match: transfer stops upon detection of CR
	DmaChnSetEventControl(DMA_CHANNEL0, DMA_EV_START_IRQ_EN|DMA_EV_MATCH_EN|DMA_EV_START_IRQ(RS485_RX_IRQ));            
	// Set the transfer source and dest addresses, source and dest sizes and the cell size
	DmaChnSetTxfer(DMA_CHANNEL0, (void*)&RS485_RxReg, RS485_DMA_RxBuffer, 1, MAXBUFFER, 1);    
    // Enable the transfer done event flag:
    DmaChnSetEvEnableFlags(DMA_CHANNEL0, DMA_EV_BLOCK_DONE);		// enable the transfer done interrupt: pattern match or all the characters transferred
	INTSetVectorPriority(INT_VECTOR_DMA(DMA_CHANNEL0), INT_PRIORITY_LEVEL_5);		// set INT controller priority
	INTSetVectorSubPriority(INT_VECTOR_DMA(DMA_CHANNEL0), INT_SUB_PRIORITY_LEVEL_3);		// set INT controller sub-priority
	INTEnable(INT_SOURCE_DMA(DMA_CHANNEL0), INT_ENABLED);		// enable the chn interrupt in the INT controller    
    // Once we configured the DMA channel we can enable it
    DmaChnEnable(DMA_CHANNEL0);
    
}

/*
void SetupDMA_Tx(void)
{    
    DelayMs(10);
    RS485_ENABLE = 1;    
    DelayMs(10);    
    DmaChnOpen(DMA_CHANNEL1, 1, DMA_OPEN_MATCH);    
    DmaChnSetMatchPattern(DMA_CHANNEL1, '\r');	// set < as ending character
    // Set the transfer event control: what event is to start the DMA transfer    
	// We want the UART2 TX interrupt to start our transfer
	// also we want to enable the pattern match: transfer stops upon detection of CR
	DmaChnSetEventControl(DMA_CHANNEL1, DMA_EV_START_IRQ_EN|DMA_EV_MATCH_EN|DMA_EV_START_IRQ(RS485_TX_IRQ));            
	// Set the transfer source and dest addresses, source and dest sizes and the cell size
	DmaChnSetTxfer(DMA_CHANNEL1, RS485_DMA_TxBuffer, (void*)&RS485_TxReg, 1, MAXBUFFER, 1);    
    // Enable the transfer done event flag:
    DmaChnSetEvEnableFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE);		// enable the transfer done interrupt: pattern match or all the characters transferred
	INTSetVectorPriority(INT_VECTOR_DMA(DMA_CHANNEL1), INT_PRIORITY_LEVEL_5);		// set INT controller priority
	INTSetVectorSubPriority(INT_VECTOR_DMA(DMA_CHANNEL1), INT_SUB_PRIORITY_LEVEL_3);		// set INT controller sub-priority
	INTEnable(INT_SOURCE_DMA(DMA_CHANNEL1), INT_ENABLED);		// enable the chn interrupt in the INT controller    
    // Once we configured the DMA channel we can enable it
    DmaChnEnable(DMA_CHANNEL1);
    DmaChnStartTxfer(DMA_CHANNEL1, DMA_WAIT_NOT, 0);	// Force the DMA transfer: the UART TX flag it's already been active
}
*/

void InitializeSystem(void) 
{	
    SYSTEMConfigPerformance(80000000);
    
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    // Configure PIC ADC for ten AD input channels
    ADC10_ManualInit();    
    
    // Set up UART Rx DMA interrupts
    SetupDMA_Rx();
    
    // I/O Ports:
    PORTSetPinsDigitalOut(IOPORT_A, BIT_5);  // MOTOR_DIR4 
    PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_1 | BIT_2);  // SW1-3
    PORTSetPinsDigitalOut(IOPORT_B, BIT_5);  // RS485_ENABLE
    PORTSetPinsDigitalIn(IOPORT_C, BIT_13);  // SW4
    PORTSetPinsDigitalOut(IOPORT_C, BIT_1);  // TEST_OUT
    PORTSetPinsDigitalIn(IOPORT_D, BIT_7);  // SW5
    PORTSetPinsDigitalOut(IOPORT_D, BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_13);  // EE_WR, LED, MOTOR DIR #5, #3, #2
    PORTSetPinsDigitalIn(IOPORT_E, BIT_9 | BIT_5 | BIT_7 | BIT_8);  // ENCODER DIRECTION INPUTS 1-4
    PORTSetPinsDigitalOut(IOPORT_G, BIT_12);  // MOTOR DIR #1       
    
    mCNOpen(CN_ON, CN1_ENABLE | CN2_ENABLE | CN3_ENABLE | CN4_ENABLE, CN1_PULLUP_ENABLE | CN2_PULLUP_ENABLE | CN3_PULLUP_ENABLE | CN4_PULLUP_ENABLE);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);    
    
    
    // Set up timers as counters
    T1CON = 0x00;
    T1CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T1CONbits.TCKPS1 = 0; // 1:1 Prescaler
    T1CONbits.TCKPS0 = 0;
    T1CONbits.TSYNC = 1;
    PR1 = 0xFFFF;
    T1CONbits.TON = 1; // Let her rip     

    T3CON = 0x00;
    T3CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T3CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T3CONbits.TCKPS1 = 0;
    T3CONbits.TCKPS0 = 0;
    PR3 = 0xFFFF;
    T3CONbits.TON = 1; // Let her rip 

    T4CON = 0x00;
    T4CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T4CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T4CONbits.TCKPS1 = 0;
    T4CONbits.TCKPS0 = 0;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 0xFFFF;
    T4CONbits.TON = 1; // Let her rip 

    T5CON = 0x00;
    T5CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T5CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T5CONbits.TCKPS1 = 0;
    T5CONbits.TCKPS0 = 0;
    PR5 = 0xFFFF;
    T5CONbits.TON = 1; // Let her rip     
    
    // Set up Timer 2 for PWM time base    
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;    
    PR2 = 4000; // Use 50 microsecond rollover for 20 khz
    T2CONbits.TON = 1; // Let her rip   
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);                    

    // Set up PWM OC1
    OC1CON = 0x00;
    OC1CONbits.OC32 = 0; // 16 bit PWM
    OC1CONbits.ON = 1; // Turn on PWM
    OC1CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC1CONbits.OCM2 = 1; // PWM runState enabled, no fault pin
    OC1CONbits.OCM1 = 1;
    OC1CONbits.OCM0 = 0;
    OC1RS = 0;

    // Set up PWM OC2
    OC2CON = 0x00;
    OC2CONbits.OC32 = 0; // 16 bit PWM
    OC2CONbits.ON = 1; // Turn on PWM
    OC2CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC2CONbits.OCM2 = 1; // PWM runState enabled, no fault pin
    OC2CONbits.OCM1 = 1;
    OC2CONbits.OCM0 = 0;
    OC2RS = 0;

    // Set up PWM OC3
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM runState enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 0;

    // Set up PWM OC4
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM runState enabled, no fault pin
    OC4CONbits.OCM1 = 1;
    OC4CONbits.OCM0 = 0;
    OC4RS = 0;
    
    // Set up PWM OC5
    OC5CON = 0x00;
    OC5CONbits.OC32 = 0; // 16 bit PWM
    OC5CONbits.ON = 1; // Turn on PWM
    OC5CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC5CONbits.OCM2 = 1; // PWM runState enabled, no fault pin
    OC5CONbits.OCM1 = 1;
    OC5CONbits.OCM0 = 0;
    OC5RS = 0;         
    
    // Set up HOST UART    
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 921600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);    
    
    // Set up RS485 UART    
    UARTConfigure(RS485uart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(RS485uart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(RS485uart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    ActualRS485BaudRate = UARTSetDataRate(RS485uart, SYS_FREQ, 57600);
    // ActualRS485BaudRate = UARTSetDataRate(RS485uart, SYS_FREQ, 2000000);
    UARTEnable(RS485uart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure RS485 UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(RS485uart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(RS485uart), INT_DISABLED); 
    INTSetVectorPriority(INT_VECTOR_UART(RS485uart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(RS485uart), INT_SUB_PRIORITY_LEVEL_0);  
    
 
    // Set up XBEE UART    
    UARTConfigure(XBEEuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(XBEEuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(XBEEuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(XBEEuart, SYS_FREQ, 57600);
    // ActualXBEEBaudRate = UARTSetDataRate(XBEEuart, SYS_FREQ, 2000000);
    UARTEnable(XBEEuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure XBEE UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(XBEEuart), INT_ENABLED); 
    INTSetVectorPriority(INT_VECTOR_UART(XBEEuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(XBEEuart), INT_SUB_PRIORITY_LEVEL_0);        
    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();   
}//end UserInit


// RS485 UART interrupt handler it is set at priority level 2
void __ISR(RS485_VECTOR, ipl2) IntRS485UartHandler(void) 
{
    unsigned char ch;
    static unsigned short RS485RxIndex = 0;

    if (INTGetFlag(INT_SOURCE_UART_RX(RS485uart))) 
        INTClearFlag(INT_SOURCE_UART_RX(RS485uart));    
    /*
    if (RS485bits.OERR || RS485bits.FERR) {
        if (UARTReceivedDataIsAvailable(RS485uart))
            ch = UARTGetDataByte(RS485uart);
        RS485bits.OERR = 0;
        RS485RxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(RS485uart))) {
        INTClearFlag(INT_SOURCE_UART_RX(RS485uart));
        if (UARTReceivedDataIsAvailable(RS485uart)) {
            ch = UARTGetDataByte(RS485uart);
            {
                if (ch == LF || ch == 0);
                else if (ch == CR) 
                {
                    if (RS485RxIndex < (MAXBUFFER-1)) 
                    {
                        RS485_DMA_RxBuffer[RS485RxIndex] = CR;
                        RS485_DMA_RxBuffer[RS485RxIndex + 1] = '\0'; 
                        RS485RxBufferFull = true;
                    }
                    RS485RxIndex = 0;
                }                
                else 
                {
                    if (RS485RxIndex < (MAXBUFFER-1))
                        RS485_DMA_RxBuffer[RS485RxIndex++] = ch;                    
                }
            }
        }
    }
    */
    
    if (INTGetFlag(INT_SOURCE_UART_TX(RS485uart))) 
    {        
        INTClearFlag(INT_SOURCE_UART_TX(RS485uart));
        if (RS485TxIndex < RS485TxLength)
        {
            if (!RS485_ENABLE)
            {
                RS485_ENABLE = 1;
                DelayUs(1);  
            }
            while(!UARTTransmitterIsReady(RS485uart));        
            ch = RS485TxBuffer[RS485TxIndex++];
            UARTSendDataByte (RS485uart, ch);
        }
        else
        {
            RS485TxIndex = 0;
            INTEnable(INT_SOURCE_UART_TX(RS485uart), INT_DISABLED);
            DelayUs(10);  
            RS485_ENABLE = 0;
        }
    }
}

int StartRS485Tx()
{
    unsigned char ch;
    RS485TxLength = strlen(RS485TxBuffer);
    if (RS485TxLength > MAXBUFFER) return 0;    
    RS485_ENABLE = 1;
    DelayUs(1);    
    RS485TxIndex = 0;
    INTEnable(INT_SOURCE_UART_TX(RS485uart), INT_ENABLED);
    ch = RS485TxBuffer[RS485TxIndex++];
    while(!UARTTransmitterIsReady(RS485uart));
    UARTSendDataByte (RS485uart, ch);    
    return RS485TxLength;
}

/*
    printf("\r#1: Testing Feather Servos & PID");
    MOTOR_DIR1 = MOTOR_DIR2 = MOTOR_DIR3 = MOTOR_DIR4 = MOTOR_DIR5 = 1;
    PWM1 = PWM2 = PWM3 = PWM4 = PWM5 = 0;
    RS485_ENABLE = 0;    
        
    DelayMs(100);
    printf("\r\rFeather Servo NO USB version");
    printf("\rInitializing Feather Servo Board at address 0x80: ");
    if (initializePCA9685(PCA9685_ADDRESS)) printf(" SUCCESS");
    else printf(" ERROR");        
        
    printf("\rSetting servo pulse: ");
        
    while(1)
    {        
        if (intFlag)
        {
            intFlag = false;
            TEST_OUT = 1;
            IFS1bits.AD1IF = 0;
            while(!IFS1bits.AD1IF);        
            AD1CON1bits.ASAM = 0;        // Pause sampling. 
            for (i = 0; i < MAXPOTS; i++)
                ADresult[i] = (unsigned short) ReadADC10(i); // read the result of channel 0 conversion from the idle buffer
            AD1CON1bits.ASAM = 1;        // Restart sampling.    
            for (i = 0; i < 4; i++)
            {
                RCservoPos[i] = (short)(ADresult[i+6]/4) + 22; 
            }
            DisplayCounter++;
            if (DisplayCounter > 10) 
            {
                DisplayCounter = 0;
                printf("\r#%d: RC Servos: %d, %d, %d, %d, %d, %d, %d, %d", testCounter++, RCservoPos[0], RCservoPos[1], RCservoPos[2], RCservoPos[3], RCservoPos[4], RCservoPos[5], RCservoPos[6], RCservoPos[7]);
            }            
             
            if (!setPCA9685outputs (PCA9685_ADDRESS, 0, 0, RCservoPos[0])) printf(" Servo 0 ERROR");
            if (!setPCA9685outputs (PCA9685_ADDRESS, 1, 0, RCservoPos[1])) printf("  Servo 1 ERROR");
            if (!setPCA9685outputs (PCA9685_ADDRESS, 2, 0, RCservoPos[2])) printf("  Servo 2 ERROR");
            if (!setPCA9685outputs (PCA9685_ADDRESS, 3, 0, RCservoPos[3])) printf("  Servo 3 ERROR");
            if (!setPCA9685outputs (PCA9685_ADDRESS, 4, 0, RCservoPos[0])) printf(" Servo 4 ERROR");
            if (!setPCA9685outputs (PCA9685_ADDRESS, 5, 0, RCservoPos[1])) printf("  Servo 5 ERROR");
            if (!setPCA9685outputs (PCA9685_ADDRESS, 6, 0, RCservoPos[2])) printf("  Servo 6 ERROR");
            if (!setPCA9685outputs (PCA9685_ADDRESS, 7, 0, RCservoPos[3])) printf("  Servo 7 ERROR");
            
            
            mAD1IntEnable(INT_ENABLED);
        }        
        // printf("\r#%d Servo position: %d", counter++, servoPos);           
        
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false;
            printf("\rReceived: %s", HOSTRxBuffer);
        }
        // 
    } // end while(1)    
*/ 





void ClearCopyBuffer()
{
    int i;
    for (i = 0; i < MAXBUFFER; i++)
    {
        RS485RxBuffer[i] = 0;
    }
}

// XBEE UART interrupt handler it is set at priority level 2
void __ISR(XBEE_VECTOR, ipl2) IntXBEEUartHandler(void) 
{
    unsigned char ch;
    static short XBEERxIndex = 0;    
    short i;

    if (XBEEbits.OERR || XBEEbits.FERR) 
    {
        if (UARTReceivedDataIsAvailable(XBEEuart))
            ch = UARTGetDataByte(XBEEuart);
        XBEEbits.OERR = 0;
        XBEERxIndex = 0;
    } 
    else if (INTGetFlag(INT_SOURCE_UART_RX(XBEEuart)))
    {
        INTClearFlag(INT_SOURCE_UART_RX(XBEEuart));
        if (UARTReceivedDataIsAvailable(XBEEuart)) 
        {
            XBEEtimeout = 10;
            ch = UARTGetDataByte(XBEEuart);
            if (!startPacket) 
            {
                if (ch == STX)
                {
                    startPacket = true;
                    XBEERxIndex = 0;
                    XBEERxBuffer[XBEERxIndex++] = STX;
                }
            }
            else if (XBEERxIndex < MAXBUFFER) 
                    XBEERxBuffer[XBEERxIndex++] = ch;
            if (ch == ETX) 
            {
                XBEEPacketLength = XBEERxIndex;
                XBEERxIndex = 0;
                for (i = 0; i < XBEEPacketLength; i++)
                    XBEEPacket[i] = XBEERxBuffer[i];
                startPacket = false;
                XBEEtimeout = 0;
            }                
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(XBEEuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(XBEEuart));
}

unsigned char processPacketData(short packetLength, unsigned char *ptrPacket, short *numData, short *ptrData, unsigned char *command, unsigned char *subCommand)
{
    MConvertType dataValue;    
    short j, i = 0;  
    unsigned char packetDataLength, packetBytes[MAXBUFFER];
    
    packetDataLength = decodePacket(ptrPacket, packetBytes);    
    
    i = 0;
    *command = packetBytes[i++];
    *subCommand = packetBytes[i++];
    *numData = packetBytes[i++];    
    
    // if (*numData > MAXSERVOS) return false;
    // if (*numData != dataLength) return false
        
    j = 0;
    while(j < *numData)
    {
        dataValue.b[0] = packetBytes[i++];
        dataValue.b[1] = packetBytes[i++];
        ptrData[j++] = dataValue.integer;
    }    
    if (!CheckCRC(packetBytes, i)) return false; 
    return true;
}

unsigned short decodePacket(unsigned char *ptrInPacket, unsigned char *ptrData)
{
    unsigned short i, j;
    unsigned char escapeFlag = FALSE;
    unsigned char startFlag = false;
    unsigned char ch;

    j = 0;
    for (i = 0; i < MAXBUFFER; i++) 
    {
        ch = ptrInPacket[i];
        // Escape flag not active
        if (!escapeFlag) 
        {
            if (ch == STX) 
            {
                if (!startFlag) 
                {
                    startFlag = true;
                    j = 0;
                }
                else return (0);
            } 
            else if (ch == ETX) 
                return (j);
            else if (ch == DLE)
                escapeFlag = TRUE;
            else if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else return (0);
        } 
        // Escape flag active
        else 
        {
            escapeFlag = FALSE;
            if (ch == ETX-1) ch = ETX;            
            if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else return(0);
        }
    }
    return (j);
}

// Timer 2 generates an interrupt every 50 microseconds approximately
void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{        
    static int intCounter = 0; 
    static int LEDcounter = 10000;
    static int memoryCounter = 625; 
    
    mT2ClearIntFlag(); // clear the interrupt flag        
    
    if (XBEEtimeout)
    {
        XBEEtimeout--;
        if (!XBEEtimeout)
            startPacket = false;
    }
    
    LEDcounter++;
    if (LEDcounter >= 10000)
    {
        LEDcounter = 0;
        if (LED) LED = 0;
        else LED = 1;
    }
    
    intCounter++;
    if (intCounter >= 80)// -80) // 2000
    {
        intCounter = 0;
        intFlag = true;
    }
    
    if (memoryCounter) memoryCounter--;
    if (!memoryCounter)
    {
        memoryCounter=625;
        memoryFlag = true;
        if (timeout)
        {
            timeout--;
            // if (timeout == 0) LED = 0;
        }
    }
}


void ResetPID()
{
    int i, j;
    
    EncoderOne = 0;
    EncoderTwo = 0;
    EncoderThree = 0;
    EncoderFour = 0;

    for (i = 0; i < NUMMOTORS; i++)
    {
        PID[i].sumError = 0; // For 53:1 ratio Servo City motor
        PID[i].PWMvalue = 0;
        PID[i].ADActual = 0;
        PID[i].ADCommand = 0;        
        PID[i].saturation = false;
        PID[i].ErrorCounter = 0;
        PID[i].Velocity = 0;
        PID[i].ActualPos = 0;
        PID[i].CommandPos = 0;
        PID[i].Halted = false;
        PID[i].Mode = DEFAULT_MODE;
        PID[i].previousPosition = PID[i].ActualPos;
        PID[i].Velocity = 0;
        for (j = 0; j < FILTERSIZE; j++) PID[i].error[j] = 0;
    }
    PID[0].Mode = POT_MODE;
    
    errIndex = 0;
}

void InitPID()
{
    ResetPID();
    int i;
    for (i = 0; i < NUMMOTORS; i++)
    {
        PID[i].Destination = 180;
        PID[i].previousPosition = 0;
        
        PID[i].quadCurrent = QUAD_NONE;
        PID[i].quadPrevious = QUAD_NONE;        
        PID[i].Mode = 0;
        PID[i].TargetVelocity = 0;
        
        
        if (i == 2 || i == 3)
        {
            PID[i].kP = KKP;
            PID[i].kI = KKI;
            PID[i].kD = KKD;            
            PID[i].PWMoffset = KK_PWM_OFFSET;
            PID[i].Mode = DEFAULT_MODE;
            PID[i].Destination = 180;
        }
        else
        {
            PID[i].kP = KP;
            PID[i].kI = KI;
            PID[i].kD = KD;
            PID[i].PWMoffset = PWM_OFFSET;
            PID[i].Mode = POT_MODE;
        }      
        
    }
}


long POTcontrol(long servoID, struct PIDtype *PID)
{
    short Error;     
    short actualPosition;
    short commandPosition; 
    long totalDerError = 0;
    long derError;    
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    unsigned char sensorError = false;
    short i;
    static short displayCounter = 0;         
    
    PID[servoID].ADCommand = (short)(ADresult[servoID+6]);
    if (PID[servoID].ADCommand > MAX_COMMAND_COUNTS) PID[servoID].ADCommand = MAX_COMMAND_COUNTS; 
    if (PID[servoID].ADCommand < MIN_COMMAND_COUNTS) PID[servoID].ADCommand = MIN_COMMAND_COUNTS; 
    PID[servoID].ADActual = (short)(ADresult[servoID+1]);
    
    if (PID[servoID].ADActual < 341) PID[servoID].quadCurrent = QUAD_ONE;
    else if (PID[servoID].ADActual < 682) PID[servoID].quadCurrent = QUAD_TWO;
    else PID[servoID].quadCurrent = QUAD_THREE;
    
    if (PID[servoID].quadCurrent==QUAD_THREE && (PID[servoID].quadPrevious==QUAD_ONE || PID[servoID].previousPosition < 0))
        actualPosition = PID[servoID].ADActual - 1024;
    else if (PID[servoID].quadCurrent==QUAD_ONE && (PID[servoID].quadPrevious==QUAD_THREE || PID[servoID].previousPosition > 1023))
        actualPosition = PID[servoID].ADActual + 1024;
    else actualPosition = PID[servoID].ADActual;
    
    PID[servoID].quadPrevious = PID[servoID].quadCurrent;
    PID[servoID].previousPosition = actualPosition;
    
    commandPosition = PID[servoID].ADCommand;
    Error = actualPosition - commandPosition;    
    PID[servoID].error[errIndex] = Error;
         
    if (!PID[servoID].saturation) PID[servoID].sumError = PID[servoID].sumError + (long)Error; 
        
    totalDerError = 0;
    for (i = 0; i < FILTERSIZE; i++)
        totalDerError = totalDerError + PID[servoID].error[i];
    derError = totalDerError / FILTERSIZE;
    
    if (PID[servoID].sumError > MAXSUM) PID[servoID].sumError = MAXSUM;
    if (PID[servoID].sumError < -MAXSUM) PID[servoID].sumError = -MAXSUM;        
    
    PCorr = ((float)Error) * -PID[servoID].kP;    
    ICorr = ((float)PID[servoID].sumError)  * -PID[servoID].kI;
    DCorr = ((float)derError) * -PID[servoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection == 0) PID[servoID].PWMvalue = 0;
    else if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) (PIDcorrection - PID[servoID].PWMoffset);            
    else PID[servoID].PWMvalue = (long) (PIDcorrection + PID[servoID].PWMoffset);                    
           
    if (abs(Error) < 4) 
    {
        PID[servoID].PWMvalue = 0;
        if (PID[servoID].ErrorCounter > 0) PID[servoID].ErrorCounter--;
    }
    else PID[servoID].ErrorCounter = 100;
    if (PID[servoID].ErrorCounter == 0) PID[servoID].sumError = 0;
        
    if (PID[servoID].PWMvalue > PWM_MAX) 
    {
        PID[servoID].PWMvalue = PWM_MAX;
        PID[servoID].saturation = true;
    }
    else if (PID[servoID].PWMvalue < -PWM_MAX) 
    {
        PID[servoID].PWMvalue = -PWM_MAX;
        PID[servoID].saturation = true;
    }
    else PID[servoID].saturation = false;
    
    if (actualPosition > 1024 && PID[servoID].PWMvalue > 0) PID[servoID].PWMvalue = 0;
    if (actualPosition < 0 && PID[servoID].PWMvalue < 0) PID[servoID].PWMvalue = 0;
    
    if (PID[servoID].ADActual < 10) 
    {
        PID[servoID].PWMvalue = 0;
        PID[servoID].sumError = 0;        
        sensorError = true;
    }        
    
    /*
    displayCounter++;
    if (servoID == 0 && displayFlag)
    {
        if (displayCounter >= 20)
        {
            displayCounter = 0;
            if (sensorError) printf("\rSENSOR ERROR - ACTUAL: %d", PID[servoID].ADActual);
            else printf("\rCOM: %d ACT: %d ERR: %d SUM: %d P: %0.1f D: %0.1f I: %0.1f PWM: %d ", commandPosition, actualPosition, Error, PID[servoID].sumError, PCorr, DCorr, ICorr, PID[servoID].PWMvalue);
        }
    }
    */
    return 1;
}


int ADC10_ManualInit(void)
{
    int i, dummy;
    
    AD1CON1bits.ON = 0;
    mAD1IntEnable(INT_DISABLED);   
    mAD1ClearIntFlag();
    
    AD1CON1 = 0;
    AD1CON2 = 0;
    AD1CON3 = 0;
    AD1CHS  = 0;
    AD1CSSL = 0;
    
    // Set each Port B pin for digital or analog
    // Analog = 0, digital = 1
    AD1PCFGbits.PCFG0 = 1; 
    AD1PCFGbits.PCFG1 = 1; 
    AD1PCFGbits.PCFG2 = 1; 
    AD1PCFGbits.PCFG3 = 0; 
    AD1PCFGbits.PCFG4 = 0; 
    AD1PCFGbits.PCFG5 = 1; 
    AD1PCFGbits.PCFG6 = 1; 
    AD1PCFGbits.PCFG7 = 1; 
    AD1PCFGbits.PCFG8 = 0; 
    AD1PCFGbits.PCFG9 = 0; 
    AD1PCFGbits.PCFG10 = 0; 
    AD1PCFGbits.PCFG11 = 0; 
    AD1PCFGbits.PCFG12 = 0; 
    AD1PCFGbits.PCFG13 = 0; 
    AD1PCFGbits.PCFG14 = 0; 
    AD1PCFGbits.PCFG15 = 0;     
    
    AD1CON1bits.FORM = 000;        // 16 bit integer format.
    AD1CON1bits.SSRC = 7;        // Auto Convert
    AD1CON1bits.CLRASAM = 0;    // Normal operation - buffer overwritten by next conversion sequence
    AD1CON1bits.ASAM = 0;        // Not enable Automatic sampling yet.
    
    AD1CON2bits.VCFG = 0;        // Reference AVdd, AVss
    AD1CON2bits.OFFCAL = 0;        // Offset calibration disable.
    AD1CON2bits.CSCNA = 1;        // Scan inputs for CH0+ SHA Input for Mux A input 
    AD1CON2bits.SMPI = 0b1000;        // Interrupt after 9+1 conversion
    AD1CON2bits.BUFM = 0;        // One 16 word buffer
    AD1CON2bits.ALTS = 0;        // Use only Mux A
    AD1CON2bits.SMPI =  MAXPOTS-1;    // Number of channels to sample
    AD1CON2bits.BUFM = 0;                // Single 16-word buffer with CLRASAM.    
    AD1CHSbits.CH0NA = 0; // Mux A Negative input from VR-
    AD1CHSbits.CH0SA = 3; // Mux A Positive input from pin AN3

    // Set conversion clock and set sampling time.
    AD1CON3bits.ADRC = 0;        // Clock derived from peripheral bus clock
    AD1CON3bits.SAMC = 0b11111;        // Sample time max
    AD1CON3bits.ADCS = 0b11111111;   // Conversion time max

    // Select channels to scan. Scan channels = 1, Skip channels = 0
    AD1CSSLbits.CSSL0 = 0;
    AD1CSSLbits.CSSL1 = 0;
    AD1CSSLbits.CSSL2 = 0;
    AD1CSSLbits.CSSL3 = 1;
    AD1CSSLbits.CSSL4 = 1;
    AD1CSSLbits.CSSL5 = 0;
    AD1CSSLbits.CSSL6 = 0;
    AD1CSSLbits.CSSL7 = 0;
    AD1CSSLbits.CSSL8 = 1;
    AD1CSSLbits.CSSL9 = 1;
    AD1CSSLbits.CSSL10 = 1;
    AD1CSSLbits.CSSL11 = 1;
    AD1CSSLbits.CSSL12 = 1;
    AD1CSSLbits.CSSL13 = 1;
    AD1CSSLbits.CSSL14 = 1;
    AD1CSSLbits.CSSL15 = 1;
    
    // Make sure all buffers have been Emptied. 
    for (i = 0; i < 16; i++) dummy = (ADC1BUF0+i*4);    
    
    AD1CON1bits.ASAM = 1;        // Start Automatic Sampling. 
    AD1CON1bits.ON = 1;            // Turn on ADC.
    return (1);
}

long ENCODERcontrol(long servoID, struct PIDtype *PID)
{
    long Error;         
    long lngCommandPos = 0;    
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    unsigned char EncoderDirection;
    static short displayCounter = 0;
    short i;    
    float ActualVelocity, AverageVelocity, ErrorVelocity;
    
    if (PID[servoID].Halted)
    {
        PID[servoID].PWMvalue = 0;
        return 0;
    }

    if (servoID == 0)
    {
        PID[servoID].Halted = true;
        PID[servoID].PWMvalue = 0;
        return 0;
    }
            
    lngCommandPos = (long) PID[servoID].CommandPos;    
        
    if (PID[servoID].Velocity < PID[servoID].TargetVelocity) 
    {
        PID[servoID].Velocity = PID[servoID].Velocity + ACCELERATION;
        if (PID[servoID].Velocity > PID[servoID].TargetVelocity)
            PID[servoID].Velocity = PID[servoID].TargetVelocity;
    }
    else if (PID[servoID].Velocity > PID[servoID].TargetVelocity)
    {
        PID[servoID].Velocity = PID[servoID].Velocity - ACCELERATION;
        if (PID[servoID].Velocity < PID[servoID].TargetVelocity)
            PID[servoID].Velocity = PID[servoID].TargetVelocity;        
    }
           
    if (!PID[servoID].saturation) PID[servoID].CommandPos = PID[servoID].CommandPos + PID[servoID].Velocity;    
    
    if (PID[servoID].Mode == DESTINATION_MODE)
    {
        if (PID[servoID].Velocity > (float)0.0 && PID[servoID].CommandPos > PID[servoID].Destination)
                PID[servoID].CommandPos = PID[servoID].Destination;
        else if (PID[servoID].Velocity < (float)0.0 && PID[servoID].CommandPos < PID[servoID].Destination)
                PID[servoID].CommandPos = PID[servoID].Destination;
    }    
    
    if (servoID == 1)
    {
        EncoderDirection = !EncoderOneDir;        
        if (EncoderDirection) 
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos + (long)EncoderOne;
            ActualVelocity = (float) EncoderOne;
        }
        else
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos - (long)EncoderOne;
            ActualVelocity = (float) 0 - EncoderOne;
        }
        EncoderOne = 0;
    }    
    else if (servoID == 2)
    {
        EncoderDirection = EncoderTwoDir;
        if (EncoderDirection) 
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos + (long)EncoderTwo;
            ActualVelocity = (float) EncoderTwo;
        }
        else
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos - (long)EncoderTwo;
            ActualVelocity = (float) 0 - EncoderTwo;
        }
        EncoderTwo = 0;
    }    
    else if (servoID == 3)
    {
        EncoderDirection = EncoderThreeDir;
        if (EncoderDirection) 
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos + (long)EncoderThree;
            ActualVelocity = (float) EncoderThree;
        }
        else
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos - (long)EncoderThree;
            ActualVelocity = (float) 0 - EncoderThree;
        }
        EncoderThree = 0;
    }    
    else if (servoID == 4)
    {
        EncoderDirection = !EncoderFourDir;
        if (EncoderDirection) 
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos + (long)EncoderFour;
            ActualVelocity = (float) EncoderFour;
        }
        else
        {
            PID[servoID].ActualPos = PID[servoID].ActualPos - (long)EncoderFour;
            ActualVelocity = (float) 0 - EncoderFour;
        }
        EncoderFour = 0;
    }    
    
    if (PID[servoID].Mode == DESTINATION_MODE)
    {
        if (PID[servoID].Destination >= 0) 
        {
            if (PID[servoID].ActualPos >= (long)(PID[servoID].Destination)) 
            {
                printf("\rHALTED: COM: %ld, ACT: %ld", (long)(PID[servoID].CommandPos), PID[servoID].ActualPos);
                PID[servoID].Halted = true;
                return 0;
            }
        }
        else
        {
            if (PID[servoID].ActualPos <= (long)(PID[servoID].Destination))
            {
                printf("\rHALTED: COM: %ld, ACT: %ld", (long)(PID[servoID].CommandPos), PID[servoID].ActualPos);
                PID[servoID].Halted = true;
                return 0;
            }            
        }
    }
    
    Error = PID[servoID].ActualPos - lngCommandPos;
         
    if (!PID[servoID].saturation) PID[servoID].sumError = PID[servoID].sumError + (long)Error; 

    // derError = Error - PID[servoID].error[errIndex];
    PID[servoID].ActualVelocity[errIndex] = ActualVelocity;    
    AverageVelocity = 0;
    for (i = 0; i < FILTERSIZE; i++)
        AverageVelocity = AverageVelocity + PID[servoID].ActualVelocity[i];
    
    AverageVelocity = AverageVelocity / FILTERSIZE;        
    
    if (PID[servoID].sumError > MAXSUM) PID[servoID].sumError = MAXSUM;
    if (PID[servoID].sumError < -MAXSUM) PID[servoID].sumError = -MAXSUM;        
    
    PCorr = ((float)Error) * -PID[servoID].kP;    
    ICorr = ((float)PID[servoID].sumError)  * -PID[servoID].kI;
    DCorr = (AverageVelocity - PID[servoID].Velocity) * -PID[servoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection == 0) PID[servoID].PWMvalue = 0;
    else if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) (PIDcorrection - PID[servoID].PWMoffset);            
    else PID[servoID].PWMvalue = (long) (PIDcorrection + PID[servoID].PWMoffset);                    
           
        
    if (PID[servoID].PWMvalue >= PWM_MAX) 
    {
        PID[servoID].PWMvalue = PWM_MAX;
        PID[servoID].saturation = true;
    }
    else if (PID[servoID].PWMvalue <= -PWM_MAX) 
    {
        PID[servoID].PWMvalue = -PWM_MAX;
        PID[servoID].saturation = true;
    }
    else PID[servoID].saturation = false;
    
    if (servoID == 2)
    {
        displayCounter++;
        if (displayCounter >= 20 && displayFlag)
        {
            displayCounter = 0;         
            // printf("\r>COM: %0.0f, ACT: %ld, ERR: %ld P: %0.1f I: %0.1f D: %0.1f PWM: %d ", PID[servoID].CommandPos, PID[servoID].ActualPos, Error, PCorr, ICorr, DCorr, PID[servoID].PWMvalue);
            printf("\r>VEL: %0.1f COM: %0.0f, ERR: %ld P: %0.1f I: %0.1f D: %0.1f PWM: %d ", PID[servoID].Velocity, PID[servoID].CommandPos, Error, PCorr, ICorr, DCorr, PID[servoID].PWMvalue);            
        }
    }       
    
    return 1;
}
