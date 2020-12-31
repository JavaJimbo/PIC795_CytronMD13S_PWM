/**********************************************************************************
 * PROJECT: PIC795 MD13S CONTROLLER - STORED ON GIHUB AS PIC795_CytronMD13S_PWM
 * main.c
 * 
 * Compiled for PIC32MX795 XC32 compiler version 1.30 
 * For Robotnik MD13S Controller Board Rev 1.0
 * Adapted from Robotnik_Brain_Board
 * 
 * 8-13-20:     Set up IO for new board
 * 9-13-20:     Created ADC10_ManualInit()
 * 9-14-20:     Fixed ADC bug. Polling works great now.
 * 9-15-20:     Got all five PWMs working, also four SW inputs with interrupt on change,
 *              also four encoder counters and encoder direction inputs.
 *              Created USE_PID mode and tested PID with one motor.
 *              Fixed a few PID bugs - works better, and doesn't overrun and go wild.
 *              Added previousPosition, quadCurrent, quadPrevious to PIDtype struct.
 *              Enabled PID for all five motors.
 * 9-16-20:     Got DMA working with RS485 Rx at 981600 baud.
 * 9-20-20:     Got RC Feather servo board working.
 * 9-21-20: 
 * 9-22-20:     Got encoders working nicely in Destination mode - enter positions by text.
 * 9-23-20:     Eliminated Velocity mode for now.
 * 9-29-20:     Testing RC servo motors with Feather board. Baudrate: 115200
 * 10-2-20:     Four RC servos working with pots controlling Feather board.
 * 10-3-20:     Added Velocity variable to limit speed.
 * 10-17-20:    Got CRC working. 
 * 10-18-20:    Constant velocity mode is working nicely using Encoder inputs on ServoCity 26:1 motors.
 *              Got forward/backward/right/left working with joystick on XBEE input.
 *              Re-enabled POT mode for servo #0.
 * 10-19-20:    Made adjustments to joystick to work with large motors 
 *              and loads for complementary wheelchair wheels.
 * 10-20-20:    Added Watchdog timer. Cleaned up POT control mode, fixed bug. Motor can now go well beyond 360 degrees.
 * 11-14-20: 
 * 12-6-20:     Modified to receive MIDI device data on XBEE at 57600 baud and control first servo with it.
 *              So now it works with MIDI_Device recording and playing back one servo motor.
 * 12-16-20:    Works now with multiple servos recording and playing back on ProTools.
 * 12-17-20:    Cleaned up PID for POT mode. Was very glitchy for AM 218 motors.
 * 12-18-20:    Both RC and PID servos work great.
 *              Use REMOTE at startup. Enabled Feather board RC servo control.
 *              RCservoPos[i] = (short)(ADresult[i+6]/4) + 22;
 * 12-20-20:    XBEE baud rate didn't work well at 115200. Set back to 57600.
 * 12-27-20:    Added errorCode to detect transmission errors on incoming data.
 * 12-31-20:    No big changes at this point. Works beautifully with MIDI_Device
 *              running in either RS232 or USB mode.
 ***********************************************************************************/

enum {
    POT_MODE = 0,
    DESTINATION_MODE,
    CONTINUOUS_MODE    
};

enum{
    NO_ERROR = 0,
    CRC_ERROR,
    STX_ERROR,
    ETX_ERROR,
    OVERRUN_ERROR
};

#define NO_QUAD 0
#define QUAD_ONE 255
#define QUAD_TWO 510
#define QUAD_THREE 1023

#define MAX_COMMAND_COUNTS 856 // 869
#define MIN_COMMAND_COUNTS 91  // 86


#define USE_FEATHER_BOARD
#define USE_PID

#define DEADZONE 64 
#define PWM_MAX 4000
#define ACCELERATION 0.1
#define DEFAULT_MODE CONTINUOUS_MODE

#define SUCCESS 0
#define PCA9685_ADDRESS 0x80 // Basic default address for Adafruit Feather Servo Board 
#define FILTERSIZE 16

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


#define	STX '>'
#define	DLE '/'
#define	ETX '\r'

#define MAXSERVOS 64  // Was 256
#define NUMMOTORS 5
#define NUM_RC_SERVOS 5

#define FORWARD 0
#define REVERSE 1
#define MAXSUM 500000




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
#pragma config FWDTEN   = OFF            // Watchdog Timer enabled
#pragma config WDTPS = PS8192           // Watchdog Timer Postscaler (1:8192) For 31,250 clock divided by 8192 = 262 mS timeout
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
    unsigned short PreviousQuad;
    float Velocity;
    float TargetVelocity;
    float ActualVelocity[FILTERSIZE];
    short ADCommand;
    BYTE saturation;
    BYTE reset;    
    BYTE Mode;    
    BYTE Halted;
    BYTE RemoteDataValid;
} PID[NUMMOTORS];


BYTE flagRemoteTimeout = false;
BYTE DMATxTestFlag = false;
unsigned short offSet;
BYTE NUMbuffer[MAXNUM + 1];
BYTE HOSTRxBuffer[MAXBUFFER+1];
BYTE HOSTRxBufferFull = false;
BYTE HOSTTxBuffer[MAXBUFFER+1]; 

BYTE displayFlag = false;

BYTE XBEERxBuffer[MAXBUFFER+1];
BYTE XBEEPacket[MAXBUFFER+1];
short XBEEData[MAXBUFFER+1];
float ForwardReverse = 0, RightLeft = 0;
short XBEEPacketLength;

BYTE RS485RxBufferFull = false;
BYTE RS485TxBufferFull = false;
BYTE RS485_DMA_RxBuffer[MAXBUFFER+1];
BYTE RS485RxBuffer[MAXBUFFER+1];
BYTE RS485TxBuffer[MAXBUFFER+1];
int RS485TxIndex = 0;
int RS485TxLength = 0;
BYTE ServoData[MAXBUFFER+1];
short servoPositions[MAXSERVOS];
long ActualRS485BaudRate = 0;
int timeout = 0;

extern BYTE CheckCRC (BYTE *ptrPacketData, short packetLength);

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
BYTE getcUSART();
extern BYTE CheckCRC (BYTE *ptrRxModbus, short RxModbusLength);
unsigned short decodePacket(BYTE *ptrInPacket, BYTE *ptrData, BYTE *errorCode);
void PrintServoData(short numServos, short *ptrServoPositions, BYTE command, BYTE subCommand);
BYTE SendReceiveSPI(BYTE dataOut);
void ResetPID();
void InitPID();
long POTcontrol(long servoID, struct PIDtype *PID);
long ENCODERcontrol(long servoID, struct PIDtype *PID);
BYTE processPacketData(short packetLength, BYTE *ptrPacket, short *numData, short *ptrData, BYTE *command, BYTE *subCommand, BYTE *errorCode);
// short BuildPacket(BYTE command, BYTE subcommand, BYTE numData, short *ptrData, BYTE *ptrPacket, short *packetLength);

BYTE DATABufferFull = false;
// void ClearCopyBuffer();
void makeFloatString(float InValue, int numDecimalPlaces, BYTE *arrAscii);


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
// BYTE TestMode = false;
unsigned short ADresult[MAXPOTS];
unsigned short SWRead = 0;
BYTE SWChangeFlag = false;
BYTE intFlag = false;
BYTE memoryFlag = false;

short errIndex = 0;
BYTE startPacket = false;
short XBEEtimeout = 0;

#define PCA9685_ADDRESS 0x80 // Basic default address for Adafruit Feather Servo Board

long ActualXBEEBaudRate, ActualHOSTBaudRate;

int main(void) 
{
    short i = 0, j = 0, p = 0, q = 0;        
    long PWMvalue = 0;        
    int JogPWM = 0;
    float floValue;            
    unsigned ADdisplay = true;    
    short SWcounter = 0;    
    #define NUM_RC_SERVOS 5
    short RCservoPos[NUM_RC_SERVOS] = {0,0,0,0,0};
    short PreviousRCservoPos[NUM_RC_SERVOS] = {0,0,0,0,0};
    BYTE ch, temp, command, subCommand, outPacket[MAXBUFFER];
    short numDataIntegers, outData[MAXBUFFER], packetLength;          
    short packetCounter = 0;
    float tempCommand;
    short ServoCommandPosition;
    BYTE runMode = REMOTE;
    BYTE runState = HALTED;    
    short RCServoID = 0;
    BYTE errorCode = NO_ERROR;
    
    InitPID();     
    DelayMs(10);       
    
    InitializeSystem();                 
    
    runState = runMode = REMOTE;

    //DelayMs(10);
    //printf("\rInitializing I2C #1");
    //initI2C(I2C1);
    //numBytes = strlen(MessageOut);
    //printf("\rWriting %d bytes to EEprom", numBytes);
    //EepromWriteBlock (I2C1, EEPROM_ID, startAddress, MessageOut, numBytes);
    //printf("\rReading %d bytes from EEprom", numBytes);
    //EepromReadBlock (I2C1, EEPROM_ID, startAddress, MessageIn, numBytes);
    //MessageIn[numBytes] = '0';
    //printf("\rMessage: %s", MessageIn);
    
    printf("\r\rSTART ERROR CODE TEST #1");
    printf("\rHOST Baudrate: %ld", ActualHOSTBaudRate);        
    printf("\rXBEE Baudrate: %ld", ActualXBEEBaudRate);

    /*
    ch = 'A';
    while(1)
    {
        DelayMs(200);        
        if (UARTReceivedDataIsAvailable(XBEEuart)) 
        {
            ch = UARTGetDataByte(XBEEuart);   
            printf("\rCH = %c", ch);
            DelayMs(20);        
            
            while(!UARTTransmitterIsReady(XBEEuart));
            UARTSendDataByte(XBEEuart, ch);    
        }
    }
    */
    
    if (runState==LOCAL) printf("\rRunMode = LOCAL");
    else if (runState==REMOTE) printf("\rRunMode = REMOTE");
    else if (runState==JOG) printf("\rRunMode = JOG");
    else printf("\rRunMode = STANDBY");    
    
    if (runState == HALTED) printf("\rRUN STATE: HALTED");
    else printf("\rRUN STATE: ON");
    
#ifdef USE_FEATHER_BOARD
    printf("\rInitializing Feather Servo Board at address 0x80: ");
    if (initializePCA9685(PCA9685_ADDRESS)) printf(" SUCCESS");
    else printf(" ERROR");    
#endif
    while(1) 
    {   
        ClrWdt(); // CLEAR_WATCHDOG
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
        
        if (XBEEPacketLength)   // $$$$
        {         
            if ( processPacketData(XBEEPacketLength, XBEEPacket, &numDataIntegers, XBEEData, &command, &subCommand, &errorCode))
            {
                timeout = 5000;                  
                temp = command & 0xF0;
                if (temp == 0xB0 && subCommand >= 0)
                {                    
                    if (subCommand < NUMMOTORS)
                    {
                        tempCommand = ((float)XBEEData[0]) / 1023;
                        tempCommand = (tempCommand * (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS)) + MIN_COMMAND_COUNTS;
                        ServoCommandPosition = (short)tempCommand;                        
                        if (ServoCommandPosition < MIN_COMMAND_COUNTS) ServoCommandPosition = MIN_COMMAND_COUNTS;
                        if (ServoCommandPosition > MAX_COMMAND_COUNTS) ServoCommandPosition = MAX_COMMAND_COUNTS;                        
                        PID[subCommand].ADCommand = ServoCommandPosition;
                        PID[subCommand].RemoteDataValid = true;
                    }
                    else
                    {
                        RCServoID = subCommand - NUMMOTORS;
                        if (RCServoID < NUM_RC_SERVOS)
                        {
                            ServoCommandPosition = (XBEEData[0] / 4) + 22;
                            if (ServoCommandPosition < 0) ServoCommandPosition = 0;
                            else if (ServoCommandPosition > 277) ServoCommandPosition = 277;
                            RCservoPos[RCServoID] = ServoCommandPosition;
                        }                        
                    }
                    printf("\rREMOTE #%d %02X: Servo #%d: XBEE: %d, COM: %d", packetCounter++, command, subCommand, XBEEData[0], ServoCommandPosition);
                }
            }
            else printf("\rPACKET ERROR CODE: %d", errorCode);            
            XBEEPacketLength = 0;
        }        
                    
        if (intFlag)
        {
            intFlag = false;            
            
            IFS1bits.AD1IF = 0;
            while(!IFS1bits.AD1IF);        
            AD1CON1bits.ASAM = 0;        // Pause sampling. 
            for (i = 0; i < MAXPOTS; i++)            
                ADresult[i] = (unsigned short) ReadADC10(i); // read the result of channel 0 conversion from the idle buffer            
            AD1CON1bits.ASAM = 1;        // Restart sampling.
            
            if (runState)
            {        
#ifdef USE_FEATHER_BOARD
                if (runMode == LOCAL)
                {
                    for (i = 0; i < NUM_RC_SERVOS; i++) RCservoPos[i] = (short)(ADresult[i+6]/4) + 22;
                }
                
                if (PreviousRCservoPos[0]!=RCservoPos[0])
                {
                    PreviousRCservoPos[0]=RCservoPos[0];
                    if (!setPCA9685outputs (PCA9685_ADDRESS, 0, 0, RCservoPos[0])) printf("\r#0 ERROR");
                    //else printf("\rRC Servo #0: %d", RCservoPos[0]);
                }
                if (PreviousRCservoPos[1]!=RCservoPos[1])
                {
                    PreviousRCservoPos[1]=RCservoPos[1];                    
                    if (!setPCA9685outputs (PCA9685_ADDRESS, 1, 0, RCservoPos[1])) printf("\r#1 ERROR");
                    //else printf("\rRC Servo #1: %d", RCservoPos[1]);
                }
                if (PreviousRCservoPos[2]!=RCservoPos[2])
                {
                    PreviousRCservoPos[2]=RCservoPos[2];
                    if (!setPCA9685outputs (PCA9685_ADDRESS, 2, 0, RCservoPos[2])) printf("\r#2 ERROR");
                    //else printf("\rRC Servo #2: %d", RCservoPos[2]);
                }
                if (PreviousRCservoPos[3]!=RCservoPos[3])
                {
                    PreviousRCservoPos[3]=RCservoPos[3];
                    if (!setPCA9685outputs (PCA9685_ADDRESS, 3, 0, RCservoPos[3])) printf("\r#3 ERROR");
                    //else printf("\rRC Servo #3: %d", RCservoPos[3]);
                }     
                if (PreviousRCservoPos[4]!=RCservoPos[4])
                {
                    PreviousRCservoPos[4]=RCservoPos[4];
                    if (!setPCA9685outputs (PCA9685_ADDRESS, 4, 0, RCservoPos[4])) printf("\r#4 ERROR");
                    //else printf("\rRC Servo #4: %d", RCservoPos[4]);
                }                     
#endif
#ifdef USE_PID              
                for (i = 0; i < NUMMOTORS; i++)
                {                             
                    if (runState == JOG) 
                        PWMvalue = JogPWM = 0;
                    else
                    {                                                     
                        if (!PID[i].Halted)
                        {
                            if (PID[i].Mode == POT_MODE) 
                            {
                                if (runState == LOCAL) 
                                {
                                    tempCommand = ((float)ADresult[i+6]) / 1023;
                                    tempCommand = (tempCommand * (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS)) + MIN_COMMAND_COUNTS;
                                    if (tempCommand < MIN_COMMAND_COUNTS) tempCommand = MIN_COMMAND_COUNTS;
                                    if (tempCommand > MAX_COMMAND_COUNTS) tempCommand = MAX_COMMAND_COUNTS;
                                    PID[i].ADCommand = (short)tempCommand;                                    
                                    POTcontrol(i, PID);
                                }
                                else
                                {
                                    if (PID[i].RemoteDataValid) POTcontrol(i, PID);
                                    else PID[i].PWMvalue = 0;
                                }
                            }
                            else ENCODERcontrol(i, PID);                        
                            PWMvalue = PID[i].PWMvalue;
                        }
                        else PWMvalue = 0;
                        command = 0;
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
#endif          
        } // End if intFlag
    
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
                        runState = STANDBY;
                        printf("REMOTE MODE");
                        break;
                    case 'L':
                        runMode = LOCAL;
                        runState = STANDBY;
                        printf("LOCAL MODE");
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
                        printf("Destination: %ld, Velocity: %0.3f", PID[3].Destination, PID[3].Velocity);
                        break;                        
                    default:                        
                        if (command < ' ') printf("\rCommand: #%d => %c", command, (command - 1 + 'A'));
                        else printf("\rCommand: %c", command);
                        break;
                } // end switch command                
                
                CONTINUE1: continue;
                command = 0;
            } // End if (command)
        } // End if HOSTRxBufferFull        
    } // End while(1))
} // End main())





void PrintServoData(short numServos, short *ptrServoPositions, BYTE command, BYTE subCommand)
{
    int i;
    printf("\rOK! Com: %d, Sub: %d, servos %d: ", command, subCommand, numServos);
    for (i = 0; i < 10; i++) printf("%d, ", ptrServoPositions[i]);
}


#define ESC 27
#define CR 13
#define BACKSPACE 8
void __ISR(HOST_VECTOR, IPL2AUTO) IntHostUartHandler(void) 
{
BYTE ch, inByte;
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
    BYTE ch;
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
    
/*
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
*/

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
    // SetupDMA_Rx();
    
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
    // UARTConfigure(HOSTuart, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    ActualHOSTBaudRate = UARTSetDataRate(HOSTuart, SYS_FREQ, 921600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);    
    
    // Set up RS485 UART   
    /*
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
    */
 
    // Set up XBEE UART at 57600 baud
    UARTConfigure(XBEEuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);    
    // UARTConfigure(XBEEuart, UART_ENABLE_PINS_TX_RX_ONLY);    
    UARTSetFifoMode(XBEEuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(XBEEuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    ActualXBEEBaudRate = UARTSetDataRate(XBEEuart, SYS_FREQ, 57600);
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
    BYTE ch;
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
    BYTE ch;
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

void ClearCopyBuffer()
{
    int i;
    for (i = 0; i < MAXBUFFER; i++)
    {
        RS485RxBuffer[i] = 0;
    }
}


BYTE processPacketData(short packetLength, BYTE *ptrPacket, short *numData, short *ptrData, BYTE *command, BYTE *subCommand, BYTE *errorCode)
{
    MConvertType dataValue;    
    short j, i = 0;  
    BYTE packetDataLength, packetBytes[MAXBUFFER];
    
    *errorCode = NO_ERROR; // Reset error flag to start with.
    
    packetDataLength = decodePacket(ptrPacket, packetBytes, errorCode);    
    if (packetDataLength == 0) return false;    
    
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
    if (!CheckCRC(packetBytes, i)) 
    {
        *errorCode = CRC_ERROR;
        return false; 
    }
    return true;
}

unsigned short decodePacket(BYTE *ptrInPacket, BYTE *ptrData, BYTE *errorCode)
{
    unsigned short i, j;
    BYTE escapeFlag = FALSE;
    BYTE startFlag = false;
    BYTE ch;

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
                else
                {
                    *errorCode = STX_ERROR;
                    return (0);
                }
            } 
            else if (ch == ETX) 
                return (j);
            else if (ch == DLE)
                escapeFlag = TRUE;
            else if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else
            {
                *errorCode = OVERRUN_ERROR;
                return (0);
            }
        } 
        // Escape flag active
        else 
        {
            escapeFlag = FALSE;
            if (ch == ETX-1) ch = ETX;            
            if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else
            {
                *errorCode = OVERRUN_ERROR;
                return(0);
            }
        }
    }
    
    if (i >= MAXBUFFER) *errorCode = OVERRUN_ERROR;
    else if (!startFlag) *errorCode = STX_ERROR;
    else *errorCode = ETX_ERROR;
    return (0);
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
    
    if (timeout)
    {
        timeout--;
        if (!timeout) flagRemoteTimeout = true;
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
        PID[i].Velocity = 0;
        PID[i].ActualPos = 0;
        PID[i].CommandPos = 0;
        PID[i].Halted = false;
        PID[i].RemoteDataValid = false;
        PID[i].Mode = DEFAULT_MODE;
        PID[i].PreviousQuad = NO_QUAD;
        PID[i].Velocity = 0;
        for (j = 0; j < FILTERSIZE; j++) PID[i].error[j] = 0;
    }
    PID[0].Mode = POT_MODE;
    PID[1].Mode = POT_MODE;
    PID[4].Mode = POT_MODE;
    errIndex = 0;
}

void InitPID()
{
    ResetPID();
    int i;
    for (i = 0; i < NUMMOTORS; i++)
    {
        PID[i].Destination = 180;
        //PID[i].previousPosition = 0;        
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
    BYTE EncoderDirection;
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
            // printf("\r>VEL: %0.1f COM: %0.0f, ERR: %ld P: %0.1f I: %0.1f D: %0.1f PWM: %d ", PID[servoID].Velocity, PID[servoID].CommandPos, Error, PCorr, ICorr, DCorr, PID[servoID].PWMvalue);            
        }
    }       
    
    return 1;
}

#define NO_QUAD 0
#define QUAD_ONE 255
#define QUAD_TWO 510
#define QUAD_THREE 1023

#define MAX_COMMAND_COUNTS 856 // 869
#define MIN_COMMAND_COUNTS 91  // 86


long POTcontrol(long servoID, struct PIDtype *PID)
{
    short Error;     
    short actualPosition;
    short commandPosition; 
    long totalDerError = 0;
    long derError;    
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    short i;
    static short displayCounter = 0;    
    unsigned short QuadReading = 0;
    
    commandPosition = PID[servoID].ADCommand;
    
    PID[servoID].ADActual = (short)(ADresult[servoID+1]);
    actualPosition = PID[servoID].ADActual;
    
    
    if (PID[servoID].ADActual < QUAD_ONE) QuadReading = QUAD_ONE;
    else if (PID[servoID].ADActual < QUAD_TWO) QuadReading = QUAD_TWO;
    else QuadReading = QUAD_THREE;
    
    if (PID[servoID].PreviousQuad == NO_QUAD) 
        PID[servoID].PreviousQuad = QuadReading;
    else if (QuadReading == QUAD_TWO)
        PID[servoID].PreviousQuad = QUAD_TWO;
    else if (PID[servoID].PreviousQuad == QUAD_TWO)
        PID[servoID].PreviousQuad = QuadReading;
    else if (PID[servoID].PreviousQuad == QUAD_ONE)
    {
        if (QuadReading == QUAD_THREE) 
            actualPosition = PID[servoID].ADActual - (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS);
    }
    else if (PID[servoID].PreviousQuad == QUAD_THREE)
    {
        if (QuadReading == QUAD_ONE) 
            actualPosition = PID[servoID].ADActual + (MAX_COMMAND_COUNTS - MIN_COMMAND_COUNTS);
    }
    
    Error = actualPosition - commandPosition;            
    PID[servoID].error[errIndex] = Error;
    
    // if (!PID[servoID].saturation) 
        PID[servoID].sumError = PID[servoID].sumError + (long)Error; 
        
    totalDerError = 0;
    for (i = 0; i < FILTERSIZE; i++)
        totalDerError = totalDerError + PID[servoID].error[i];
    derError = totalDerError / FILTERSIZE;    
    
    PCorr = ((float)Error) * -PID[servoID].kP;    
    ICorr = ((float)PID[servoID].sumError)  * -PID[servoID].kI;
    DCorr = ((float)derError) * -PID[servoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection == 0) PID[servoID].PWMvalue = 0;
    else if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) (PIDcorrection - PID[servoID].PWMoffset);            
    else PID[servoID].PWMvalue = (long) (PIDcorrection + PID[servoID].PWMoffset);                    
         
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
    
        displayCounter++;
        if (servoID == 1)
        {
            if (displayCounter >= 40 && displayFlag)
            {   
                if (PID[servoID].PreviousQuad == QUAD_ONE) printf("\rQI ");
                else if (PID[servoID].PreviousQuad == QUAD_TWO) printf("\rQII ");
                else printf("\rQIII ");
                printf("COM: %d, ROT: %d, ACT: %d ERR: %d P: %0.1f I: %0.1f PWM: %d ", PID[servoID].ADCommand, PID[servoID].ADActual, actualPosition, Error, PCorr, ICorr, PID[servoID].PWMvalue);
                displayCounter = 0;
            }
        }
    return 1;
}

// XBEE UART interrupt handler it is set at priority level 2
void __ISR(XBEE_VECTOR, ipl2) IntXBEEUartHandler(void) 
{
    BYTE ch;
    static short XBEERxIndex = 0;    
    short i;
    
                if (TEST_OUT) TEST_OUT = 0;
            else TEST_OUT = 1;                        

    
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
