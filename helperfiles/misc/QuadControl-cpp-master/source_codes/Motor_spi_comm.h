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
static uint16_t delay=0;


void init_motor_spi_ss(int gpio1,int gpio2,int gpio3,int gpio4,int rstgpio5){
	gpio_export(gpio1);
	gpio_export(gpio2);
	gpio_export(gpio3);
	gpio_export(gpio4);
	gpio_export(rstgpio5);

		gpio_set_value(gpio1, HIGH);
		gpio_set_value(gpio2, HIGH);
		gpio_set_value(gpio3, HIGH);
		gpio_set_value(gpio4, HIGH);
		gpio_set_value(rstgpio5, HIGH);

		gpio_set_dir(gpio1, OUTPUT_PIN);
		gpio_set_dir(gpio2, OUTPUT_PIN);
		gpio_set_dir(gpio3, OUTPUT_PIN);
		gpio_set_dir(gpio4, OUTPUT_PIN);
		gpio_set_dir(rstgpio5, OUTPUT_PIN);

		gpio_set_value(rstgpio5, LOW);
		usleep(100000);
		gpio_set_value(rstgpio5, HIGH);
		gpio_set_dir(rstgpio5, INPUT_PIN);
}

void rpm2byte(uint8_t *tx,double *frpm){
frpm[0]=frpm[0]/60;
frpm[1]=frpm[1]/60;
frpm[2]=frpm[2]/60;
frpm[3]=frpm[3]/60;

tx[0]=frpm[0];
tx[1]=(frpm[0]-tx[0])*100;

tx[2]=frpm[1];
tx[3]=(frpm[1]-tx[2])*100;

tx[4]=frpm[2];
tx[5]=(frpm[2]-tx[4])*100;

tx[6]=frpm[3];
tx[7]=(frpm[3]-tx[6])*100;
}
static void transfer_rpmdata(double *frpm,int gpio1,int gpio2,int gpio3,int gpio4)
{
	    int fd;
		int ret = 0;
	    uint8_t tx[8]={0,0,0,0,0,0,0,0};

	    rpm2byte(tx,frpm);


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
	gpio_set_value(gpio2, LOW);
	gpio_set_value(gpio3, LOW);
	gpio_set_value(gpio4, LOW);

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	close(fd);

	gpio_set_value(gpio1, HIGH);
	gpio_set_value(gpio2, HIGH);
	gpio_set_value(gpio3, HIGH);
	gpio_set_value(gpio4, HIGH);

}

#endif
