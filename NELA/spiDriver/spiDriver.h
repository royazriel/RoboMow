#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <dirent.h>
#include <pthread.h>
#include <errno.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/serio.h>
#include <termios.h>
#include <signal.h>
#include <getopt.h>
#include <time.h>
#include <sys/time.h>
#include <execinfo.h>

#define RESULT_OK                   0
#define RESULT_ERR                  -1
#define SCREEN_RAW_LENGTH           80
#define MAX_PATH                    255

uint32 getmstime();

#define PCLS printf("\033c")

#define PINFOS(fmt, args...) \
    printf("[INFO][%s:%d][function %s]: "fmt"\r",__FILE__, __LINE__, __FUNCTION__, ##args)
#define PINFO(fmt, args...) \
    printf("[INFO][%s:%d][function %s]: "fmt"\n", __FILE__, __LINE__, __FUNCTION__, ##args)
#define PWARNING(fmt, args...) \
    printf("[WARNING][%s:%d][function %s]: "fmt"\n", __FILE__, __LINE__, __FUNCTION__, ##args)
#define PERROR(fmt, args...) \
    printf("[ERROR][%s:%d][function %s]: "fmt"\n", __FILE__, __LINE__, __FUNCTION__, ##args)



void pabort(const char *s);

void hexDump( const uint8 * buf, uint32 length );

typedef struct __SpiConfig
{
    const char *device;
    uint8_t mode;
    uint8_t bits;
    uint32_t speed;
    uint16_t delay;
    int toggle_cs;
}SpiConfig;

int getSpiHandle();
int openspi( SpiConfig * spiConfig );

int writetospi(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       bodyLength,
    const uint8 *bodyBuffer);

int readfromspi(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       readlength,
    uint8       *readBuffer);

int closespi(void);
int setspibitrate(int desiredRatekHz);

#endif //SPI_DRIVER_H
