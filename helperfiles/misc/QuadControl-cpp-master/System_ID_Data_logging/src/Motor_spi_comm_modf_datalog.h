#ifndef SPICOMM_H_
#define SPICOMM_H_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "SimpleGPIO.h"
using namespace std;

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev1.0";
static uint8_t mode=0;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay=2000;


void init_motor_spi_ss(int gpio1,int rstgpio5){
	gpio_export(gpio1);
	gpio_export(rstgpio5);



		gpio_set_dir(gpio1, OUTPUT_PIN);
		gpio_set_dir(rstgpio5, OUTPUT_PIN);

		gpio_set_value(gpio1, HIGH);

		gpio_set_value(rstgpio5, LOW);
		usleep(100000);
		gpio_set_value(rstgpio5, HIGH);
		gpio_set_dir(rstgpio5, INPUT_PIN);
		usleep(1500000);
}

void rpm2byte(uint8_t *tx,int u){

tx[0]=u/100;
tx[1]=(u-tx[0]*100);

}
static void transfer_rpmdata(int u,int * vel,int gpio1)
{
	    int fd;
		int ret = 0;

	    uint8_t tx[2];
        tx[0]=0;
        tx[1]=0;

	    rpm2byte(tx,u);

	    //   cout<<"tx = ";
	    //   		for(int i=0;i<2;i++){
	    // 			 cout<<(int)tx[i]<<" ";
	    // 		 }
	    // cout<<endl;


		fd = open(device, O_RDWR);
		if (fd < 0)
			pabort("can't open device");

		/*
		 * spi mode
		 */
		ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
		if (ret == -1)
			pabort("can't set spi mode");

		ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
		if (ret == -1)
			pabort("can't get spi mode");

		/*
		 * bits per word
		 */
		ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
		if (ret == -1)
			pabort("can't set bits per word");

		ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
		if (ret == -1)
			pabort("can't get bits per word");

		/*
		 * max speed hz
		 */
		ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
		if (ret == -1)
			pabort("can't set max speed hz");

		ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
		if (ret == -1)
			pabort("can't get max speed hz");



	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		tr.tx_buf = (unsigned long)tx,
		tr.rx_buf = (unsigned long)rx,
		tr.len = ARRAY_SIZE(tx),
		tr.delay_usecs = delay,
		tr.speed_hz = speed,
		tr.bits_per_word = bits,
	};

	gpio_set_value(gpio1, LOW);

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	close(fd);

	gpio_set_value(gpio1, HIGH);

	//cout<<"rx = ";
	//	    		for(int i=0;i<2;i++){
	//	    			cout<<(int)rx[i]<<" ";
	//	    		 }
    // cout<<endl;
     vel[0]=(int)rx[0]*100+(int)rx[1];


}

#endif
