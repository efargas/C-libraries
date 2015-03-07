#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
//#include <time.h>
#include "/usr/include/time.h"
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
#define ADXL345_OFSX_REG           0x1E  //8 bits twos complement format 15.6mg/LSB.
#define ADXL345_OFSY_REG           0x1F  //8 bits twos complement format 15.6mg/LSB.
#define ADXL345_OFSZ_REG           0x20  //8 bits twos complement format 15.6mg/LSB.
#define ADXL345_DUR_REG            0x21
#define ADXL345_LATENT_REG         0x22
#define ADXL345_WINDOW_REG         0x23
#define ADXL345_THRESH_ACT_REG     0x24  //8 bits unsigned 62.5mg/LSB Threshold value for detetcting activity.
#define ADXL345_THRESH_INACT_REG   0x25  //8 bits unsigned 62.5mg/LSB Threshold value for detetcting inactivity.
#define ADXL345_TIME_INACT_REG     0x26  //8 bits unsigned amount of time that acceleration must be less than THRESH_INACT for declare inactivity. 1sec/LSB.
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
#define ADXL345_3200HZ             0x0F
#define ADXL345_1600HZ             0x0E
#define ADXL345_800HZ              0x0D
#define ADXL345_400HZ              0x0C
#define ADXL345_200HZ              0x0B
#define ADXL345_100HZ              0x0A
#define ADXL345_50HZ               0x09
#define ADXL345_25HZ               0x08
#define ADXL345_12HZ5              0x07
#define ADXL345_6HZ25              0x06

#define ADXL345_DATA_FORMAT_BASE   0x08
//Data ranges
#define ADXL345_16G                0x03
#define ADXL345_8G                 0x02
#define ADXL345_4G                 0x01
#define ADXL345_2G                 0x00

#define ADXL345_EN_MM              0x08  //Enable measurement mode
#define ADXL345_DIS_MM             0xF7  //Disable measurement mode
#define ADXL345_SPI_READ           0x80
#define ADXL345_SPI_WRITE          0x00
#define ADXL345_MULTI_BYTE         0x40
#define ADXL345_DUMMY              0x00

#define ADXL345_X                  0x00
#define ADXL345_Y                  0x01
#define ADXL345_Z                  0x02

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/*******************************************************************************
*  LOCAL VARIABLES
*******************************************************************************/

/**
* "how long to delay after the last bit transfer before optionally deselecting
* the device before the next transfer." From spi-dev.h
*/
static uint16_t delay = 0;
static const char *spi10 = "/dev/spidev1.0"; ///< the path to the SPI_1 CS0 in the dev folder
static const char *spi11 = "/dev/spidev1.1"; ///< the path to the SPI_1 CS1 device in the dev folder
static uint8_t mode = 3; ///< The mode the spi device will operate in
static uint8_t bits = 8; ///< the number of bits per word
static uint32_t speed = 2000000; ///< Defines the frequency for the spi in Hz
static unsigned char data[10]; ///< Holds the data received from the device
static unsigned char buffer[10];
static float dataG[3];
static int16_t dataRAW[3];
static float scale = 0.0039;


/*******************************************************************************
*  LOCAL FUNCTION PROTOTYPES
*******************************************************************************/
static void SetUpSpi(int*,const char*);

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
static void pabort(const char *s){
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
static void SetUpSpi(int *fd,const char *device){
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
printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
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
static void transfer(int fd, unsigned char *buffer, int len){

    int i,j,ret;
    unsigned char tx[len];

    for (i = 0; i < len; i++){
        tx[i]= *(buffer+i);
    }
    unsigned char rx[ARRAY_SIZE(tx)];

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

    for (j = 0; j < len; j++){
        data[j] = rx[j];
    }
}

static void getX(int fd){

    int i;
    for (i = 0; i < 10; i++){
        buffer[i]=0;
        data[i]=0;
    }

    buffer[0] = (ADXL345_SPI_READ | ADXL345_DATAX0_REG);
    buffer[2] = (ADXL345_SPI_READ | ADXL345_DATAX1_REG);

    transfer(fd,buffer,4);

    dataRAW[0] = *(data+3)<<8|*(data+1);
    dataG[0] = (float)dataRAW[0]*scale;
    printf("X: %.2f\n",dataG[0]);
}

static void getY(int fd){

    int i;
    for (i = 0; i < 10; i++){
        buffer[i]=0;
        data[i]=0;
    }

    buffer[0] = (ADXL345_SPI_READ | ADXL345_DATAY0_REG);
    buffer[2] = (ADXL345_SPI_READ | ADXL345_DATAY1_REG);

    transfer(fd,buffer,4);

    dataRAW[1] = *(data+3)<<8|*(data+1);
    dataG[1] = (float)dataRAW[1]*scale;
    printf("Y: %.2f\n",dataG[1]);

}

static void getZ(int fd){

    int i;
    for (i = 0; i < 10; i++){
        buffer[i]=0;
        data[i]=0;
    }

    buffer[0] = (ADXL345_SPI_READ | ADXL345_DATAZ0_REG);
    buffer[2] = (ADXL345_SPI_READ | ADXL345_DATAZ1_REG);

    transfer(fd,buffer,4);

    dataRAW[2] = *(data+3)<<8|*(data+1);
    dataG[2] = (float)dataRAW[2]*scale;
    printf("Z: %.2f\n",dataG[2]);
}

static void getXYZ(int fd){
    int i,j;
    for (i = 0; i < 7; i++){
        buffer[i]=0;
    }

    buffer[0] = (ADXL345_SPI_READ | ADXL345_DATAX0_REG | ADXL345_MULTI_BYTE);
    transfer(fd,buffer,10);

    dataRAW[0] = *(data+2)<<8|*(data+1);
    dataRAW[1] = *(data+4)<<8|*(data+3);
    dataRAW[2] = *(data+6)<<8|*(data+5);

    for (j = 0; j < 3; j++){
        dataG[j] = (float)dataRAW[j]*scale;
    }
    printf("G's X: %0.2f, Y: %0.2f, Z: %0.2f\n",dataG[0],dataG[1],dataG[2]);
}

static void getID(int fd){

    buffer[0] = (ADXL345_SPI_READ | ADXL345_DEVID_REG);
    buffer[1] = ADXL345_DUMMY;
    transfer(fd,buffer,2);
    printf("El Device ID es: %d\n",*(data+1));
}

static void ADXL345_Setup(int fd){

    buffer[0] = ADXL345_DATA_FORMAT_REG;
    buffer[1] = (ADXL345_DATA_FORMAT_BASE | ADXL345_16G);
    buffer[2] = ADXL345_BW_RATE_REG;
    buffer[3] = ADXL345_6HZ25;
    transfer(fd,buffer,4);
}

static void start_stop_measure(int fd, int startbit){

    buffer[0] = ADXL345_POWER_CTL_REG;
    if (startbit == 1){
        buffer[1] = (ADXL345_SPI_WRITE | ADXL345_EN_MM);
    } else{
        buffer[1] = (ADXL345_SPI_WRITE & ADXL345_DIS_MM);
    }
    transfer(fd,buffer,2);
}

int main( void ){
    int ret = 0;
    int fd;
    SetUpSpi(&fd,spi11);
    getID(fd);
    sleep(1);
    ADXL345_Setup(fd);
    start_stop_measure(fd,1);
    printf("START\n");
    sleep(2);

    while (1){
        int i,j,k;
        for (i = 0; i < 1000; i++){
            //getX(fd);
            //getY(fd);
            //getZ(fd);
            getXYZ(fd);
        }
        printf("STOP\n");
        sleep(10);
        start_stop_measure(fd,0);
        for (j = 0; j < 1000; j++){
            //getX(fd);
            //getY(fd);
            //getZ(fd);
            getXYZ(fd);
        }
        start_stop_measure(fd,1);
        printf("START\n");
        sleep(2);
        for (k = 0; k < 1000; k++){
            //getX(fd);
            //getY(fd);
            //getZ(fd);
            getXYZ(fd);
        }
    }
    return ret;
}
