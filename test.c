#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>

int
main(void)
{
    int fd, serial;

    fd = open("/dev/ttyUSB0", O_RDWR);

    //ioctl(fd, TIOCMGET, &serial);

    //struct termios termios_p;
    
    //tcgetattr(fd, &termios_p);
    
    //ioctl(fd, TCSBRK, 0);

    struct serial_icounter_struct ai;
    
    ioctl(fd, TIOCGICOUNT, &ai);

    //struct serial_rs485 rs485conf = {0};
    //rs485conf.flags |= SER_RS485_ENABLED;

    //ioctl(fd, TIOCSRS485, &rs485conf);
    
//    struct timeval t1, t2;
    //gettimeofday(&t1, NULL);

    //write(fd, "ppp\r", 4);
    
    close(fd);
    
    /* gettimeofday(&t2, NULL); */

    /* double elapsedTime; */
    /* elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0; */
    /* elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0; */

    /* printf("duree: %lf\n", elapsedTime);    */
}
