/*
 * (C) Copyright 2012
 * Ricado Ribalda - Qtechnology A/S - ricardo.ribalda@gmail.com
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>

void use(char* name){
	fprintf(stderr,"Use: %s TTYS0_DEV\n",name);
	return;
}

int do_write(int fd, uint8_t *buffer, int len){
	int ret;
	while(len){
		ret=write(fd,buffer,len);
		if (ret<0){
			perror("do_write");
			return -1;
		}
		buffer+=ret;
		len-=ret;
	}
	return 0;
}

int do_read(int fd, uint8_t *buffer, int len){
	int ret;
	while(len){
		ret=read(fd,buffer,len);
		if (ret<0){
			perror("do_read");
			return -1;
		}
		buffer+=ret;
		len-=ret;
	}
	return 0;
}

int config_port(int fd){
	struct termios options;
	int status;

	if (!isatty(fd)){
		fprintf(stderr,"You must specify a valid serial port\n");
		return -1;
	}

	tcgetattr(fd, &options);

	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);
	options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
	options.c_oflag = 0;
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	options.c_cflag &= ~(CSIZE | PARENB | CRTSCTS);
	options.c_cflag |= CS8;
	options.c_cc[VMIN] = 1;
	options.c_cc[VTIME] = 10;
	tcsetattr(fd,TCSAFLUSH,&options);

	if (ioctl(fd, TIOCMGET, &status) == -1) {
		perror("setRTS(): TIOCMGET");
		return 0;
	}

	//DTR RTS
	status |= TIOCM_RTS;
	status |= TIOCM_DTR;
	if (ioctl(fd, TIOCMSET, &status) == -1) {
		perror("setRTS(): TIOCMSET");
		return 0;
	}

	//FLOW CONTROL
	/*tcgetattr(fd, &options);
	options.c_cflag |= CRTSCTS;
	tcsetattr(fd,TCSAFLUSH,&options);*/


	return 0;
}

int read_id(int fd){
	uint8_t buffer[]={0x02, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
	uint8_t exp_id[]={0x33, 0x37, 0x34, 0x0d};
	int i;

	do_write(fd,buffer,sizeof(buffer));
	do_read(fd,buffer,sizeof(exp_id));

	if (!(memcmp(exp_id,buffer,sizeof(exp_id))))
		return 0;

	for (i=0;i<sizeof(exp_id);i++)
		fprintf(stdout, "%.2X ",buffer[i]);
	fprintf(stdout, "\n");

	return -1;
}

#define TEMP_SIZE 33
int init_temp(int fd){
	uint8_t buffer[]={0x02, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
	uint8_t temp[TEMP_SIZE];
	do_write(fd,buffer,sizeof(buffer));
	do_read(fd,temp,sizeof(temp));
	return 0;
}

int read_temp(int fd){
	uint8_t buffer[]={0x02, 0x58, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
	uint8_t temp[TEMP_SIZE];
	int i;
	int aux=0;

	do_write(fd,buffer,sizeof(buffer));
	do_read(fd,temp,sizeof(temp));
	do_read(fd,temp,sizeof(temp));


	if ((temp[0]!=2)||(temp[TEMP_SIZE-1]!=3)){
		fprintf(stderr, "Package error:\n");
		for (i=0;i<sizeof(temp);i++)
			fprintf(stdout, "%.2X ",temp[i]);
		fprintf(stdout, "\n");
		return -1;
	}

	for (i=1;i<TEMP_SIZE-2;i++)
		aux+=temp[i];

	if ((aux&0xff)!=temp[TEMP_SIZE-2]){
		fprintf(stderr, "CRC error:\n");
		for (i=0;i<sizeof(temp);i++)
			fprintf(stdout, "%.2X ",temp[i]);
		fprintf(stdout, "\n");
		return -1;
	}


	/*for (i=0;i<sizeof(temp);i++)
		fprintf(stdout, "%.2X ",temp[i]);
	fprintf(stdout, "\n");*/

	fprintf(stdout, "HOLD:%s T1-T2(T5):%s REC:%s LIGHT:%s\n",
			(temp[1]&0x40)?" on":"off",
			(temp[1]&0x20)?" on":"off",
			(temp[1]&0x10)?" on":"off",
			(temp[2]&0x01)?"off":" on"
		);
	for (i=0;i<4;i++){
		int neg=0;

		if ((i==2)&&(temp[1]&0x20)) //T1-2
			i=4;
		aux=(temp[i*2+3])*256;
		aux+=temp[i*2+4];
		if (aux&0x8000){
			aux-=0x8000;
			neg=1;
		}
		if (aux==0x7fff)
			fprintf(stdout, "T%1d: ----- %c\n",i+1,(temp[1]&0x8)?'C':'F');
		else
			fprintf(stdout, "T%1d:%c%3d.%1d %c\n",i+1,neg?'-':' ',aux/10,aux%10,(temp[1]&0x8)?'C':'F');

		if (i==4)
			i=2;
	}
	fprintf(stdout, "\n");

	return 0;
}

int main(int argc, char *argv[]){
	int temp_fd;

	if (argc<2){
		use(argv[0]);
		return -1;
	}

	temp_fd=open(argv[1], O_RDWR);
	if (temp_fd<0){
		fprintf(stderr,"Unable to open %s\n",argv[1]);
		return -1;

	}

	if (config_port(temp_fd)<0){
		fprintf(stderr,"Unable to config serial port\n");
		return -1;
	}

	while(read_id(temp_fd)!=0){
		fprintf(stderr,"Wrong ID\n");
		sleep(1);
	}
	fprintf(stdout,"Device Recognized!\n");

	init_temp(temp_fd);
	while(1){
		read_temp(temp_fd);
		sleep(1);
	}
	return -1;
}

