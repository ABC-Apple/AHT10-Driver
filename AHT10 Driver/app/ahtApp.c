/*  Demo application for AHT10 sensor
 *
 *  Copyright (C) 2023 Matjaz Zibert - S59MZ
 *
 *  Linux app for device driver
 */
#include "sys/ioctl.h"
#include "fcntl.h"
#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>

#define ATH10_NAME "/dev/aht10"

//#define convert_temp(x)		((float) ((200 * (float) (x)) / 1048576) - 50)
//#define convert_humd(x)		((100 * (float) (x)) / 1048576)
#define convert_temp(x)		((float) ((25 * (float) (x)) / 131072) - 50)
#define convert_humd(x)		((25 * (float) (x)) / 262144)

int main(int argc, char *argv[])
{
	int fd;
	char *filename;
	unsigned int databuf[2];
	float temp;
	float humd;
	int ret = 0;

	filename = ATH10_NAME;
	fd = open(filename, O_RDWR);
	if(fd < 0) {
		printf("can't open file %s\r\n", filename);
		return -1;
	}

	while (1) {
		ret = read(fd, databuf, sizeof(databuf));
		if(ret == 0) { 			/* 数据读取成功 */
			temp = convert_temp(databuf[0]);
			humd = convert_humd(databuf[1]);
			printf("温度 = %.3f,湿度 = %.3f\r\n", temp, humd);
		}
		usleep(200000); /*100ms */
	}
	close(fd);	/* 关闭文件 */	
	return 0;
}
