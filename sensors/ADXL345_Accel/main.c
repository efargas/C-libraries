#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>


/*******************************************************************************
*  DEFINES
*******************************************************************************/

/**
 * "Defines"
 */
//Registers.
#define ADXL345_DEVID_REG          0x00
#define ADXL345_THRESH_TAP_REG     0x1D
#define ADXL345_OFSX_REG           0x1E
#define ADXL345_OFSY_REG           0x1F
#define ADXL345_OFSZ_REG           0x20
#define ADXL345_DUR_REG            0x21
#define ADXL345_LATENT_REG         0x22
#define ADXL345_WINDOW_REG         0x23
#define ADXL345_THRESH_ACT_REG     0x24
#define ADXL345_THRESH_INACT_REG   0x25
#define ADXL345_TIME_INACT_REG     0x26
#define ADXL345_ACT_INACT_CTL_REG  0x27
#define ADXL345_THRESH_FF_REG      0x28
#define ADXL345_TIME_FF_REG        0x29
#define ADXL345_TAP_AXES_REG       0x2A
#define ADXL345_ACT_TAP_STATUS_REG 0x2B
#define ADXL345_BW_RATE_REG        0x2C
#define ADXL345_POWER_CTL_REG      0x2D
#define ADXL345_INT_ENABLE_REG     0x2E
#define ADXL345_INT_MAP_REG        0x2F
#define ADXL345_INT_SOURCE_REG     0x30
#define ADXL345_DATA_FORMAT_REG    0x31
#define ADXL345_DATAX0_REG         0x32
#define ADXL345_DATAX1_REG         0x33
#define ADXL345_DATAY0_REG         0x34
#define ADXL345_DATAY1_REG         0x35
#define ADXL345_DATAZ0_REG         0x36
#define ADXL345_DATAZ1_REG         0x37
#define ADXL345_FIFO_CTL           0x38
#define ADXL345_FIFO_STATUS        0x39

//Data rate codes.
#define ADXL345_3200HZ      0x0F
#define ADXL345_1600HZ      0x0E
#define ADXL345_800HZ       0x0D
#define ADXL345_400HZ       0x0C
#define ADXL345_200HZ       0x0B
#define ADXL345_100HZ       0x0A
#define ADXL345_50HZ        0x09
#define ADXL345_25HZ        0x08
#define ADXL345_12HZ5       0x07
#define ADXL345_6HZ25       0x06

#define ADXL345_SPI_READ    0x80
#define ADXL345_SPI_WRITE   0x00
#define ADXL345_MULTI_BYTE  0x60

#define ADXL345_X           0x00
#define ADXL345_Y           0x01
#define ADXL345_Z           0x02

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/*******************************************************************************
*  LOCAL VARIABLES
*******************************************************************************/

/**
* "how long to delay after the last bit transfer before optionally deselecting
* the device before the next transfer." From spi-dev.h
*/
static uint16_t delay = 0;
static const char *device = "/dev/spidev1.0"; ///< the path to the spi device in the dev folder
static uint8_t mode = 3; ///< The mode the spi device will operate in
static uint8_t bits = 8; ///< the number of bits per word
static uint32_t speed = 2000000; ///< Defines the frequency for the spi in Hz
int16_t data[3]; ///< Holds the data received from the device
float dataG[3];
float dataX;
float dataY;
float dataZ;
static uint32_t scale = 1000;


/*******************************************************************************
*  LOCAL FUNCTION PROTOTYPES
*******************************************************************************/
static void SetUpSpi(int*);

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
   if (*fd < 0)
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

printf("spi mode: 0x%x\n", mode);
printf("bits per word: %d\n", bits);
printf("max speed: %d Hz (%d KHz)\n", speed, speed/scale);
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
int Get_X(int fd)
{
     int ret;
     unsigned char tx[4];
     unsigned char rx[4];

     tx[0] = (ADXL345_DATAX0_REG | ADXL345_SPI_READ);
     tx[2] = (ADXL345_DATAX1_REG | ADXL345_SPI_READ);


     //unsigned char rx[ARRAY_SIZE(tx)] = {0, };
     struct spi_ioc_transfer tr = {
         .tx_buf = (unsigned long)tx,
         .rx_buf = (unsigned long)rx,
         .len = 4,
         .delay_usecs = delay,
         .speed_hz = speed,
         .bits_per_word = bits,
     };
    data[0] = 0;
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret == 1)
       pabort("can't send spi message");

       //shift the bits back to the original position
        data[0] = rx[3] << 8 | rx[1];

        dataX = data[0]*0.001953;
        printf("X: %.2f\n",dataX);

    return 0;
}

int Get_Y(int fd)
{
     int ret;
     unsigned char tx[4];
     unsigned char rx[4];

     tx[0] = (ADXL345_DATAY0_REG | ADXL345_SPI_READ);
     tx[2] = (ADXL345_DATAY1_REG | ADXL345_SPI_READ);


     //unsigned char rx[ARRAY_SIZE(tx)] = {0, };
     struct spi_ioc_transfer tr = {
         .tx_buf = (unsigned long)tx,
         .rx_buf = (unsigned long)rx,
         .len = 4,
         .delay_usecs = delay,
         .speed_hz = speed,
         .bits_per_word = bits,
     };
    data[0] = 0;
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret == 1)
       pabort("can't send spi message");

       //shift the bits back to the original position
        data[0] = rx[3] << 8 | rx[1];

        dataY = data[0]*0.001953;
        printf("Y: %.2f\n",dataY);

    return 0;
}

int Get_Z(int fd)
{
     int ret;
     unsigned char tx[4];
     unsigned char rx[4];

     tx[0] = (ADXL345_DATAZ0_REG | ADXL345_SPI_READ);
     tx[2] = (ADXL345_DATAZ1_REG | ADXL345_SPI_READ);


     //unsigned char rx[ARRAY_SIZE(tx)] = {0, };
     struct spi_ioc_transfer tr = {
         .tx_buf = (unsigned long)tx,
         .rx_buf = (unsigned long)rx,
         .len = 4,
         .delay_usecs = delay,
         .speed_hz = speed,
         .bits_per_word = bits,
     };
    data[0] = 0;
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret == 1)
       pabort("can't send spi message");

       //shift the bits back to the original position
        data[0] = rx[3] << 8 | rx[1];

        dataZ = data[0]*0.001953;
        printf("Z: %.2f\n",dataZ);

    return 0;
}

int Get_XYZ(int fd)
{
     int ret;
     unsigned char tx[10];
     tx[0]=0xF2;
     unsigned char rx[ARRAY_SIZE(tx)] = {0, };

     struct spi_ioc_transfer tr = {
         .tx_buf = (unsigned long)tx,
         .rx_buf = (unsigned long)rx,
         .len = ARRAY_SIZE(tx),
         .delay_usecs = delay,
         .speed_hz = speed,
         .bits_per_word = bits,
     };

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret == 1)
       pabort("can't send spi message");

        data[0] = rx[2]<<8|rx[1];
        data[1] = rx[4]<<8|rx[3];
        data[2] = rx[6]<<8|rx[5];

        dataG[0] = data[0]*0.001953;
        dataG[1] = data[1]*0.001953;
        dataG[2] = data[2]*0.001953;

        printf("GX: %.2f , GY: %.2f , Z: %.2f\n",dataG[0],dataG[1],dataG[2]);

    return 0;
}

signed char Get_DEVID(int fd)
{
    int ret;
    unsigned char tx[2], rx[2];
    tx[0]= (ADXL345_SPI_READ+ADXL345_DEVID_REG);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long) tx,
        .rx_buf = (unsigned long) rx,
        .len = 2,
    };

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret == 1)
       pabort("can't send spi message");

    printf("Return value: %d\n",rx[1]);

    return 0;
}

void ADXL345_begin(int fd)
{
    int ret;
    unsigned char tx[]= {ADXL345_DATA_FORMAT_REG,0x0B,ADXL345_BW_RATE_REG,ADXL345_800HZ,ADXL345_POWER_CTL_REG,0x08};
    unsigned char rx[ARRAY_SIZE(tx)] = {0, };

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = ARRAY_SIZE(tx),
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret == 1)
       pabort("can't send spi message");
}

int main( void )
{
    int ret = 0;
    int fd;
    SetUpSpi(&fd);
    Get_DEVID(fd);
    sleep(1);
    ADXL345_begin(fd);
    sleep(1);
    Get_DEVID(fd);
    while (1) {
        Get_X(fd);
        Get_Y(fd);
        Get_Z(fd);
        Get_XYZ(fd);
        //sleep(1);
    }

    return ret;
}
