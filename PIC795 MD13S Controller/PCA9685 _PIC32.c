/***************************************************************************************
 * PCA9685_PIC32.c 
 * Routines for initializing and setting PWM on Adafruit V2 Motor Shield
 * Written for PIC 32MX220F032D on Olimex PIC32-Pinguino-MX220 board
 * Compiled with Micrchip XC32 V1.30 C compiler
 * 
 * Usage: 
 * 
 ****************************************************************************************/

#include "I2C_4BUS_EEPROM_PIC32.h"
#include "PCA9685.h"
#include "Delay.h"
#include <plib.h>

#define true TRUE
#define false FALSE

unsigned char setPWM (unsigned char device, unsigned char motor, short PWMvalue, unsigned char direction){
short PWMdata;

    PWMdata = abs(PWMvalue);
    if (PWMdata > PWM_MAX) PWMdata = PWM_MAX;
    
    else if (motor == RIGHTFRONTPWM){
        if (direction == REVERSE){
            if (!setPCA9685outputs (device, AIN1, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, AIN2, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMA, 0, PWMdata)) return (FALSE);
        }
        else {
            if (!setPCA9685outputs (device, AIN1, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, AIN2, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMA, 0, PWMdata)) return (FALSE);
        }
    }        
    else if (motor == LEFTFRONTPWM){
        if (direction == REVERSE){
            if (!setPCA9685outputs (device, BIN1, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, BIN2, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMB, 0, PWMdata)) return (FALSE);
        }
        else {
            if (!setPCA9685outputs (device, BIN1, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, BIN2, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMB, 0, PWMdata)) return (FALSE);
        }
    }   
    else if (motor == LEFTREARPWM){
        if (direction == FORWARD){
            if (!setPCA9685outputs (device, CIN1, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, CIN2, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMC, 0, PWMdata)) return (FALSE);
        }
        else {
            if (!setPCA9685outputs (device, CIN1, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, CIN2, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMC, 0, PWMdata)) return (FALSE);
        }
    }   
    else {
        if (direction == FORWARD){
            if (!setPCA9685outputs (device, DIN1, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, DIN2, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMD, 0, PWMdata)) return (FALSE);
        }
        else {
            if (!setPCA9685outputs (device, DIN1, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, DIN2, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMD, 0, PWMdata)) return (FALSE);
        }
    }   
    return(TRUE);
}

#define RD 1
#define WR 0


unsigned char PCAReadByte (unsigned char device, unsigned char PCAcontrolRegister, unsigned char *ptrData)
{    
    StartTransfer(PCABUS, false);
    
    if (!TransmitOneByte(PCABUS, device | WR)) return 0; // Send I2C Device ID and WRITE Command    
    if (I2CGetACK(PCABUS)) return 0; // Get ACK from PCA
       
    if (!TransmitOneByte(PCABUS, PCAcontrolRegister)) return 0; // Send PCA register to be written to
    if (I2CGetACK(PCABUS)) return 0; // Get ACK from PCA
    
    StartTransfer(PCABUS, true); // Now send RESTART
    
    if (!TransmitOneByte(PCABUS, device | RD)) return 0; // Send ID and READ Command    
    if (I2CGetACK(PCABUS)) return 0; // Get ACK from PCA

    // Now receive data byte and send NACK::
    *ptrData = I2CReceiveByte(PCABUS, true);    
    
    StopTransfer(PCABUS); // All done

    return (1); // Return 1 to indicate successful read operation
}



unsigned char initializePCA9685(unsigned char device) 
{ 
    unsigned char dataByte;

    initI2C(PCABUS);
    
    if (!PCAWriteByte (device, ALL_LED_ON, 0)) return FALSE;
    if (!PCAWriteByte (device, ALL_LED_ON+1, 0)) return FALSE;
    if (!PCAWriteByte (device, ALL_LED_OFF, 0)) return FALSE;
    if (!PCAWriteByte (device, ALL_LED_OFF+1, 0)) return FALSE;
 
    // Configure totem pole structured output:    
    if (!PCAReadByte (device, MODE2_REG, &dataByte)) return FALSE;
    dataByte = dataByte | OUTDRV;
    if (!PCAWriteByte (device, MODE2_REG, dataByte)) return FALSE;
            
    // respond to ALL_LED register changes 
    if (!PCAReadByte (device, MODE1_REG, &dataByte)) return FALSE;
    dataByte = dataByte | ALLCALL;
    if (!PCAWriteByte (device, MODE1_REG, dataByte)) return FALSE;
    
    DelayMs(10); // wait for oscillator 

    // To set PWM frequency, first make sure PCA9685 is in SLEEP mode:
    if (!PCAReadByte (device, MODE1_REG, &dataByte)) return FALSE;    
    dataByte = dataByte |= SLEEP_BIT;
    if (!PCAWriteByte (device, MODE1_REG, dataByte)) return FALSE;
    
    // For maximum PWM frequency of 1526 Hz, set PRESCALE register to 0x03h:
    // if (!PCAWriteByte (device, PRESCALE, 0x03)) return FALSE;
     if (!PCAWriteByte (device, PRESCALE, 0xFF)) return FALSE;  // $$$$
    
    // Now clear SLEEP bit for normal operation mode:
    dataByte = dataByte &= ~SLEEP_BIT;
    if (!PCAWriteByte (device, MODE1_REG, dataByte)) return FALSE;       
 
    DelayMs(10); // wait for oscillator
    
    // Now enable restart:
    dataByte = dataByte |= RESTART;
    if (!PCAWriteByte (device, MODE1_REG, dataByte)) return FALSE;    

    return (TRUE);
}


unsigned char setPCA9685outputs (unsigned char device, unsigned short channel, unsigned short turnON, unsigned short turnOFF)
{
    unsigned char registerAddress;
    static unsigned char STARTflag = true;

    if (channel > MAX_LED_CHANNEL) return (FALSE);    
    registerAddress = LED_ON_REGISTER + (4 * channel);
    
    //printf("\rSET PCA 1, ");
    if (STARTflag)
    {
        if (!PCAWriteByte (device, registerAddress, (unsigned char)(turnON & 0xFF))) return FALSE;
        //printf("2, ");
        if (!PCAWriteByte (device, registerAddress+1, (unsigned char)((turnON & 0xFF00) >> 8))) return FALSE;        
        //printf("3, ");
    }
    if (!PCAWriteByte (device, registerAddress+2, (unsigned char)(turnOFF & 0xFF))) return FALSE;
    //printf("4, ");
    if (!PCAWriteByte (device, registerAddress+3, (unsigned char)((turnOFF & 0xFF00) >> 8))) return FALSE;    
    //printf("DONE");
    STARTflag = false;
    return (TRUE);
}


// PCAWrite4Bytes (unsigned char device, unsigned char Register, unsigned char data1, unsigned char data2, unsigned char data3, unsigned char data4);
unsigned char setPCA9685outputsFast (unsigned char device, unsigned short channel, unsigned short turnON, unsigned short turnOFF)
{
    unsigned char registerAddress;

    if (channel > MAX_LED_CHANNEL) return (FALSE);    
    registerAddress = LED_ON_REGISTER + (4 * channel);    
    
    PCAWrite4Bytes (device, registerAddress, (turnON & 0xFF), ((turnON & 0xFF00) >> 8), (turnOFF & 0xFF), ((turnOFF & 0xFF00) >> 8)  );    
    return (TRUE);
}

unsigned char PCAWriteByte(unsigned char device, unsigned char PCAcontrolRegister, unsigned char data)
{
    StartTransfer(PCABUS, false);
    if (!TransmitOneByte(PCABUS, device | WR)) return 0; // Send I2C Device ID and WRITE Command    
    if (I2CGetACK(PCABUS)) return 0; // Get ACK from PCA
    if (!TransmitOneByte(PCABUS, PCAcontrolRegister)) return 0; // Send PCA register to be written to
    if (I2CGetACK(PCABUS)) return 0; // Get ACK from PCA
    if (!TransmitOneByte(PCABUS, data)) return 0; // Send data byte
    if (I2CGetACK(PCABUS)) return 0; // Get ACK from PCA
    StopTransfer(PCABUS);
    return (1);
}

unsigned char PCAWrite4Bytes (unsigned char device, unsigned char Register, unsigned char data1, unsigned char data2, unsigned char data3, unsigned char data4)
{
    StartTransfer(PCABUS, false);
    if (!TransmitOneByte(PCABUS, device | WR)) return 0; // Send I2C Device ID and WRITE Command    
    if (I2CGetACK(PCABUS)) return 0; // Get ACK from PCA
    if (!TransmitOneByte(PCABUS, Register)) return 0; // Send PCA register to be written to
    if (I2CGetACK(PCABUS)) return 0; 
    if (!TransmitOneByte(PCABUS, data1)) return 0; 
    if (I2CGetACK(PCABUS)) return 0; 
    if (!TransmitOneByte(PCABUS, data2)) return 0; 
    if (I2CGetACK(PCABUS)) return 0; 
    if (!TransmitOneByte(PCABUS, data3)) return 0; 
    if (I2CGetACK(PCABUS)) return 0; 
    if (!TransmitOneByte(PCABUS, data4)) return 0; 
    if (I2CGetACK(PCABUS)) return 0; 
    StopTransfer(PCABUS);
    
    return (1);
}
