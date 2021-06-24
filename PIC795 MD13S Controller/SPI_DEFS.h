/* 
 * File:   SPI_DEFS.h
 * Author: Jim
 *
 * Created on August 25, 2019, 9:07 AM
 */

#ifndef SPI_DEFS_H
#define	SPI_DEFS_H

#ifdef	__cplusplus
extern "C" {
#endif

    // Description: SD-SPI Chip Select and TRIS bits
    #define SD_CS               LATEbits.LATE4 // $$$$
    #define SD_CS_TRIS          TRISEbits.TRISE4 
    // Description: SD-SPI Card Detect and TRIS bits
    #define SD_CD               PORTGbits.RG9 
    #define SD_CD_TRIS          TRISGbits.TRISG9 
    // Description: SD-SPI Write Protect input and TRIS bits
    #define SD_WE_TRIS          TRISAbits.TRISA0 
    #define SD_WE               PORTAbits.RA0
        // Registers for the SPI module you want to use
        //#define MDD_USE_SPI_1  $$$$
        #define MDD_USE_SPI_2
        #define USE_SD_INTERFACE_WITH_SPI


		//SPI Configuration
		#define SPI_START_CFG_1     (PRI_PRESCAL_64_1 | SEC_PRESCAL_8_1 | MASTER_ENABLE_ON | SPI_CKE_ON | SPI_SMP_ON)
        #define SPI_START_CFG_2     (SPI_ENABLE)

        // Define the SPI frequency
        #define SPI_FREQUENCY			(20000000)


            // Description: The main SPI control register
            #define SPICON1             SPI2CON
            // Description: The SPI status register
            #define SPISTAT             SPI2STAT
            // Description: The SPI Buffer
            #define SPIBUF              SPI2BUF
            // Description: The receive buffer full bit in the SPI status register
            #define SPISTAT_RBF         SPI2STATbits.SPIRBF
            // Description: The bitwise define for the SPI control register (i.e. _____bits)
            #define SPICON1bits         SPI2CONbits
            // Description: The bitwise define for the SPI status register (i.e. _____bits)
            #define SPISTATbits         SPI2STATbits
            // Description: The enable bit for the SPI module
            #define SPIENABLE           SPI2CONbits.ON
            // Description: The definition for the SPI baud rate generator register (PIC32)
            #define SPIBRG			    SPI2BRG

            // Tris pins for SCK/SDI/SDO lines

            // Description: The TRIS bit for the SCK pin
            #define SPICLOCK            TRISGbits.TRISG6
            // Description: The TRIS bit for the SDI pin
            #define SPIIN               TRISGbits.TRISG7
            // Description: The TRIS bit for the SDO pin
            #define SPIOUT              TRISGbits.TRISG8
            //SPI library functions
            #define putcSPI             putcSPI2
            #define getcSPI             getcSPI2
            #define OpenSPI(config1, config2)   OpenSPI2(config1, config2)



#ifdef	__cplusplus
}
#endif

#endif	/* SPI_DEFS_H */

