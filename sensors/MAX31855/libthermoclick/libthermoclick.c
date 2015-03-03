/*****************************************************************************/
/** @file   ThermoClick.c
*
*   @brief  Example code to interface BeagleBone Black to MikroElektronica Thermo Click board
*
*   @copyright (C) 2014 Premier Farnell UK Limited (trading as element14).
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy
*   of this software, code and associated documentation files (“Software”) to use,
*   copy, modify, publish, distribute, sublicense and/or sell copies of the
*   Software subject to the following conditions:
*
*   THIS SOFTWARE IS PROVIDED “AS-IS” AND NEWARK CORPORATION, ON BEHALF OF ITSELF
*   AND ITS AFFILIATES (“NEWARK”) EXPRESSLY DISCLAIMS ALL WARRANTIES OF ANY KIND,
*   EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
*   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
*
*   Newark shall not be liable (whether in contract, tort, including negligence,
*   or other grounds) for any loss or damage of any kind or nature related to,
*   arising under or in connection with the Software including for any direct,
*   indirect, special, incidental or consequential loss or damage (including, but
*   not limited to, loss of data, profits, revenue, goodwill, sales, business or
*   use) even if Newark has been advised of the possibility of such loss or damage.
*
*   The above copyright notice and this permission notice shall be included in
*   all copies or substantial portions of the Software.
******************************************************************************/

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

/*******************************************************************************
*  DEFINITIONS
*******************************************************************************/

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define TEMP_DATA_COEFF 0xFFFC0000
#define TEMP_DATA_COEFF_SHIFTED 0x1FFF
#define TEMP_SIGNED_BIT 0x2000
#define TEMP_COEFF 0.25
#define TEMP_FRACT_BITS 2
#define TEMP_OFFSET 18
#define NUMBER_OF_FRACT_BITS_TEMP 3
#define JUNCTION_OFFSET 4
#define JUNCTION_FRACT_BITS 4
#define JUNCTION_COEFF 0xFFF0
#define JUNCTION_COEFF_SHIFTED 0x7FF
#define JUNCTION_SIGN_BIT 0x800
#define BIT_0 1
#define BIT_1 2
#define BIT_2 4
#define BIT_3 8
#define BIT_16 65536

/*******************************************************************************
*  LOCAL VARIABLES
*******************************************************************************/

/**
* "how long to delay after the last bit transfer before optionally deselecting
* the device before the next transfer." From spi-dev.h
*/
static uint16_t delay = 0;
static const char *device = "/dev/spidev1.0"; ///< the path to the spi device in the dev folder
static uint8_t mode = 0; ///< The mode the spi device will operate in
static uint8_t bits = 8; ///< the number of bits per word
static uint32_t speed = 1000000; ///< Defines the frequency for the spi in Hz
static uint32_t data; ///< Holds the data received from the device

/*******************************************************************************
*  LOCAL FUNCTION PROTOTYPES
*******************************************************************************/
static float getTempV(void);
static float getJunctionTempV(void);

static uint8_t CheckError(void);

static void SetUpSpi(int*);

static void PrintData(void);


/*******************************************************************************
*  LOCAL FUNCTION PROTOTYPES
*******************************************************************************/

/*******************************************************************************
* pabort
*/
/**
* @brief Prints the error to the standard output error stream and closes all
* open streams and flushes them and terminates the program.
*
* @param[in] s String to be printed to the stderr
*
*******************************************************************************/
static void pabort(const char *s)
{
   //Print the string to the stderr
   perror(s);

   //Terminate the program
   abort();
}


/*******************************************************************************
* Read
*/
/**
* @brief * Reads the data from the termo click board in as bytes and converts it to
* a single unsigned int.
*
* @param[in] fd Holds the file descriptor.
*
*******************************************************************************/
static void Read(int fd)
{

   //variables
   int ret;
   uint8_t index;

   //This array is for the dummy writes
   uint8_t tx[] = {
      0xA1, 0xA2, 0xA3, 0xA4};

   uint8_t rx[ARRAY_SIZE(tx)] = {0, };
   struct spi_ioc_transfer tr = {
      .tx_buf = (unsigned long)tx,
      .rx_buf = (unsigned long)rx,
      .len = ARRAY_SIZE(tx),
      .delay_usecs = delay,
      .speed_hz = 0,
      .bits_per_word = 0,
   };

   //Begin the transmission
   ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   if (ret == 1)
      pabort("can't send spi message");

   //format the data
   data = 0;
   for (index = 0; index < 4; index++)
   {
      //shift the bits back to the original position
      data += rx[index] << (8 * (3 - index));
   }
}


/*******************************************************************************
* PrintData
*/
/**
* @brief Prints the data read form the Thermo Click to the terminal screen.
*
*******************************************************************************/

static void PrintData( void )
{

   //variables
   float temp_thermo, temp_junction;

   temp_junction = getJunctionTempV();
   printf("Internal junction temperature: %.4f C\n\n", temp_junction);

   //Check for errors
   if(!CheckError())
   {
      temp_thermo = getTempV();
      printf("Thermocouple measurement was:  %.2f C\n", temp_thermo);
   }

}


/*******************************************************************************
* CheckError
*/
/**
* @brief Checks to see if the error bit is toggled
* @return If there is an error this will be 1 else 0
*
* Checks the data from the termo board to see if the error bits are high and
* will print out what the error is based on the data sheet for this
* component. If there is an error this function will return 1, else it
* will return 0.
*******************************************************************************/
static uint8_t CheckError( void )
{

   //variables
   uint8_t error = 0;

   //Check for an error in the data RCVD from the board and report back
   if( (data & BIT_1) > 0)
   {
      printf("Open Circuit error\n");
      error = 1;
   }
   if( (data & BIT_2) > 0)
   {
      printf("Short to GND error\n");
      error = 1;
   }
   if( (data & BIT_3) > 0)
   {
      printf("Short to VCC error\n");
      error = 1;
   }
   //If the others are true then this one should always be true
   if( (data & BIT_16) > 0)
   {
      printf("Fault\n");
      error = 1;
   }

   return error;
}


/*******************************************************************************
* getTempV
*/
/**
* @brief Gets the temperature from the data that was read from the thermo board.
* @return Temperature in degrees C
*
*******************************************************************************/
static float getTempV( void )
{

   //variables
   uint32_t temp;
   float output = 0;

   //Keep only the data we care about
   temp = data & TEMP_DATA_COEFF;

   //Shift the data to the right
   temp >>= TEMP_OFFSET;

   //if singed convert neg
   if((temp & TEMP_SIGNED_BIT) > 0)
   {
      //The number is negative and in 2's comp

      //Remove the sign bit
      temp = temp ^ TEMP_SIGNED_BIT;
      //Subtract one to convert the number to ones compliment
      temp -= 1;
      //invert the number to bring it to an unsigned int
      temp = ~temp;
      //Removed the padded bits
      temp = temp &  TEMP_DATA_COEFF_SHIFTED;
      //Convert the fractional number to a float
      output = temp / 4.0;
      //Add the sign back
      output *= -1;
   }
   else
   {
      //The number is positive
      //Convert the fractional number to a float
      output = temp / 4.0;
   }

   return output;
}

/*******************************************************************************
* getJunctionTempV
*/
/**
* @brief Gets the junction temperature from the data that was read from the thermo board.
* @return Temperature in degrees C
*
*******************************************************************************/
static float getJunctionTempV( void )
{

   //variables
   uint32_t temp;
   float output = 0;

   //Keep only the data we care about
   temp = data & JUNCTION_COEFF;
   //Shift the data to the right
   temp >>= JUNCTION_OFFSET;

   if((temp & JUNCTION_SIGN_BIT) > 0)
   {
      //The number is in 2's compliment

      //Remove the sign bit
      temp = temp ^ JUNCTION_SIGN_BIT;
      //Subtract one to get the number to ones compliment
      temp -= 1;
      //Invert the numbers
      temp = ~temp;
      //Remove the padded bits to the number
      temp = temp & JUNCTION_COEFF_SHIFTED;
      //Convert the fractional number to a float
      output = temp / 16.0;
      //Add the sign back
      output *= -1;
   }
   else
   {
      //The number is positive
      //Convert the fractional number to a float
      output = temp / 16.0;
   }

   return output;
}


/*******************************************************************************
* SetUpSpi
*/
/**
* @brief Sets up the SPI device for the beaglebone black, for use with the thermo
* click board.
* @param[out] fd The file descriptor for the SPI device.
*
*******************************************************************************/
static void SetUpSpi(int *fd)
{
   int ret = 0;

   *fd = open(device, O_RDWR);
   if (fd < 0)
      pabort("can't open device");

   /*
    * spi mode
    */
   //Sets the mode for the spi device
   ret = ioctl(*fd, SPI_IOC_WR_MODE, &mode);
   if (ret == -1)
      pabort("can't set spi mode");

   //Checking to see if it can get the mode
   ret = ioctl(*fd, SPI_IOC_RD_MODE, &mode);
   if (ret == -1)
      pabort("can't get spi mode");

   /*
    * bits per word
    */
   //Sets the number of bits per word
   ret = ioctl(*fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
   if (ret == -1)
      pabort("can't set bits per word");

   //Checking to see if we cam get the number of bits per word
   ret = ioctl(*fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
   if (ret == -1)
      pabort("can't get bits per word");

   /*
    * max speed Hz
    */
    //Setting the speed for the spi port
   ret = ioctl(*fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
   if (ret == -1)
      pabort("can't set max speed Hz");

   //Checking to see if we can get the spi speed
   ret = ioctl(*fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
   if (ret == -1)
      pabort("can't get max speed Hz");

}
