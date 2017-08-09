/*
* ev3car-reconning
* Adapted ev3dead-reconning program to handle a steerable robot similar to a car.
* Changes to implementation by Jan Phillip Kretzschmar
*/

/*
 * ev3dead-reconning program
 *
 * Copyright (C) 2016 Bartosz Meglicki <meglickib@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 
 /*   
  * This program was created for EV3 with ev3dev OS 
  * 
  * ev3car-reconning:
  * -reads 1 motor position
  * -reads gyroscope angle
  * -timestamps the data
  * -sends the above data in UDP messages
  *
  * Preconditions (for EV3/ev3dev):
  * -two tacho motors connected to ports A, B (A is an medium motor)
  * -MicroInfinity CruizCore XG1300L gyroscope connected to port 3 with manually loaded I2C driver
  * 
  * See Usage() function for syntax details (or run the program without arguments)
  */

#include "shared/misc.h"
#include "shared/net_udp.h"

#include "ev3dev-lang-cpp/ev3dev.h"

#include <limits.h> //INT_MAX
#include <stdio.h>
#include <string.h> //memcpy
#include <unistd.h> //open, close, read, write
#include <fcntl.h> //O_RDONLY flag
#include <endian.h> //htobe16, htobe32, htobe64

using namespace ev3dev;

struct car_reconning_packet {

	uint64_t timestamp_us;
	int32_t position_drive;
	int16_t heading;
};

const int DEAD_RECONNING_PACKET_BYTES=14; //2 + 4 + 8

// GYRO CONSTANTS
const char *GYRO_PORT = "i2c-legoev35:i2c1";
const int GYRO_PATH_MAX = 100;
char GYRO_PATH[GYRO_PATH_MAX] = "/sys/class/lego-sensor/sensor";

void MainLoop(int socket_udp, const sockaddr_in &destination_udp, const large_motor &drive, int gyro_direct_fd, int poll_ms);

void InitMotor(motor *m);
int InitGyro(i2c_sensor *gyro);
int ReadGyroAngle(int gyro_direct_fd, int16_t *out_angle);

int EncodeCarReconningPacket(const car_reconning_packet &packet, char *buffer);
void SendCarReconningFrameUDP(int socket, const sockaddr_in &dest, const dead_reconning_packet &frame);

void Usage();
int ProcessInput(int argc, char **argv, int *out_port, int *out_poll_ms);

int main(int argc, char **argv) {
	int socket_udp, gyro_direct_fd;
	sockaddr_in destination_udp;
	int port, poll_ms;
	
	if( ProcessInput(argc, argv, &port, &poll_ms) ) {
		Usage();
		return 0;
	}
	const char *host=argv[1];
	
	//medium_motor motor_steer(OUTPUT_B);
	large_motor motor_drive(OUTPUT_A);
	i2c_sensor gyro(GYRO_PORT, {"mi-xg1300l"});

	SetStandardInputNonBlocking();	

	gyro_direct_fd=InitGyro(&gyro);

	InitNetworkUDP(&socket_udp, &destination_udp, host, port, 0);
	
	InitMotor(&motor_left);
	InitMotor(&motor_right);
		
	MainLoop(socket_udp, destination_udp, motor_drive, gyro_direct_fd, poll_ms);
	
	close(gyro_direct_fd);
	CloseNetworkUDP(socket_udp);

	printf("ev3car-reconning: bye\n");
	
	return 0;
}

void MainLoop(int socket_udp, const sockaddr_in &destination_udp, const large_motor &drive, int gyro_direct_fd, int poll_ms) {
	const int BENCHS=INT_MAX;
		
	struct car_reconning_packet frame;
	int16_t heading;
	uint64_t start=TimestampUs();
	int i, enxios=0, elapsed_us, poll_us=1000*poll_ms;
	
	for(i=0;i<BENCHS;++i) {	
		frame.timestamp_us=TimestampUs();
		frame.position_drive = drive.position();
		
		if(ReadGyroAngle(gyro_direct_fd, &heading) == -ENXIO) {
			//this is workaround for occasional ENXIO problem
			fprintf(stderr, "ev3car-reconning: got ENXIO, retrying %d\n", ++enxios);
			i--;
			continue; //we need to collect data again, this failure could be time consuming
		}
		frame.heading=heading;
		SendCarReconningFrameUDP(socket_udp, destination_udp, frame);
		enxios=0; //part of workaround for occasoinal ENXIO

		if(IsStandardInputEOF()) break;

		elapsed_us=(int)(TimestampUs()-frame.timestamp_us);
		
		if( elapsed_us < poll_us ) SleepUs(poll_us - elapsed_us);
	}
		
	uint64_t end=TimestampUs();
	
	double seconds_elapsed=(end-start)/ 1000000.0L;
	printf("ev3car-reconning: average loop %f seconds\n", seconds_elapsed/i);
}

void InitMotor(motor *m) {
	if(!m->connected()) Die("ev3car-reconning: motor not connected");
}

int InitGyro(i2c_sensor *gyro) {
	int fd;
	
	if(!gyro->connected()) Die("ev3car-reconning: unable to find gyroscope");
		
	gyro->set_poll_ms(0);
	gyro->set_command("RESET");
	
	printf("ev3car-reconning: callculating gyroscope bias drift\n");
	Sleep(1000);
	fflush(stdout);
	
	snprintf(GYRO_PATH+strlen(GYRO_PATH), GYRO_PATH_MAX, "%d/direct", gyro->device_index());
	
	if((fd=open(GYRO_PATH, O_RDONLY))==-1) DieErrno("ev3car-reconning: open(GYRO_PATH, O_RDONLY)");

	printf("ev3car-reconning: gyroscope ready\n");

	return fd;
}

int ReadGyroAngle(int gyro_direct_fd, int16_t *out_angle) {
	char temp[2];
	int result;
	
	if(lseek(gyro_direct_fd, 0x42, SEEK_SET)==-1) DieErrno("ev3car-reconning: lseek(gyro_direct_fd, 0x42, SEEK_SET)==-1");
		
	if( (result=read(gyro_direct_fd, temp, 2 )) == 2) {
		memcpy(out_angle, temp, 2);
		return 0;
	}	
		
	if( (result <= 0 && errno != ENXIO) ) DieErrno("ev3car-reconning: read Gyro failed");
		
	if( result == 1) Die("ev3car-reconning: incomplete I2C read");
	
	return -ENXIO;			
}

int EncodeCarReconningPacket(const dead_reconning_packet &p, char *data) {
	*((uint64_t*)data) = htobe64(p.timestamp_us);
	data += sizeof(p.timestamp_us);

	*((uint32_t*)data)= htobe32(p.position_drive);
	data += sizeof(p.position_drive);
	
	*((uint16_t*)data)= htobe16(p.heading);
	data += sizeof(p.heading);
	
	return DEAD_RECONNING_PACKET_BYTES;	
}
void SendCarReconningFrameUDP(int socket, const sockaddr_in &destination, const car_reconning_packet &frame) {
	static char buffer[CAR_RECONNING_PACKET_BYTES];
	EncodeCarReconningPacket(frame, buffer);
	SendToUDP(socket, destination, buffer, CAR_RECONNING_PACKET_BYTES);
}

void Usage() {
	printf("ev3car-reconning host port poll_ms\n\n");
	printf("examples:\n");
	printf("./ev3car-reconning 192.168.0.103 8005 10\n");
}

int ProcessInput(int argc, char **argv, int *out_port, int *out_poll_ms) {
	long int port, poll_ms;
		
	if(argc!=4) return -1;
		
	port=strtol(argv[2], NULL, 0);
	if(port <= 0 || port > 65535) {
		fprintf(stderr, "ev3car-reconning: the argument port has to be in range <1, 65535>\n");
		return -1;
	}
	*out_port=port;
	
	poll_ms=strtol(argv[3], NULL, 0);
	if(poll_ms <= 0 || poll_ms > 1000) {
		fprintf(stderr, "ev3car-reconning: the argument poll_ms has to be in range <1, 1000>\n");
		return -1;
	}
	*out_poll_ms=poll_ms;
	
	return 0;
}