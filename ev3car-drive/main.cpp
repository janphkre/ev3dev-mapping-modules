/*
 * ev3car-drive
 * Adapted ev3drive program to handle a steerable robot similar to a car.
 * Changes to implementation by Jan Phillip Kretzschmar
 */

/**
 * ev3drive program
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
 
 /**
  * This program was created for EV3 with ev3dev OS
  * 
  * ev3drive:
  * -initializes 2 motors
  * -reads UDP messages
  * -moves first motor to position (steering)
  * -lets second motor rotate forward or backward
  * -stops second motor on timeout
  *
  * See Usage() function for syntax details (or run the program without arguments)
  */


#include "shared/misc.h"
#include "shared/net_udp.h"

#include "ev3dev-lang-cpp/ev3dev.h"

#include <signal.h> //sigaction, sig_atomic_t
#include <endian.h> //htobe16, htobe32, htobe64
#include <stdio.h> //printf, etc

using namespace ev3dev;

// GLOBAL VARIABLES
volatile sig_atomic_t g_finish_program = 0;
//volatile sig_atomic_t g_end_turn = 0;
//volatile sig_atomic_t g_end_turn_stop = 0;
// temporary control packet, subject to change
struct car_drive_packet {
	uint64_t timestamp_us;
	int16_t command;
	int16_t param1;//forward/backward
	int16_t param2;//steering segment
};

const int CONTROL_PACKET_BYTES = 14; //8 + 3*2 = 14 bytes
enum Commands { KEEPALIVE = 0, TURN = 1, FORWARD = 2, BACKWARD = 3, STOP = 4, TURNSTOP = 5 };

const int TURN_SLEEP = 1500;
const int RAMP_UP_MS = 500;
const int INIT_STEERING_POWER = 100;

int steerLeft;
int steerForward;
int steerRight;
medium_motor steer(OUTPUT_B);
large_motor drive(OUTPUT_A);
int turnPosition;

void MainLoop(int socket_udp);
void ProcessMessage(const car_drive_packet &packet);

//void* CompleteTurn(void* v);
//void* CompleteTurnStop(void* v);

void InitMotor(motor *m);
void StopMotors();

int RecvCarDrivePacket(int socket_udp, car_drive_packet *packet);
void DecodeCarDrivePacket(car_drive_packet *packet, const char *data);

void Usage();
void ProcessArguments(int argc, char **argv, int *port, int *timeout_ms);
void Finish(int signal);


int main(int argc, char **argv) {			
	int socket_udp, port, timeout_ms;
	sockaddr_in destination_udp;
	
	ProcessArguments(argc, argv, &port, &timeout_ms);
	SetStandardInputNonBlocking();
	
	//init
	RegisterSignals(Finish);

	//init steering
	steer.reset();
	InitMotor(&steer);	
	steer.set_duty_cycle_sp(-INIT_STEERING_POWER);
	steer.run_direct();
	Sleep(TURN_SLEEP * 2);
	steer.stop();
	steerRight = steer.position();
	steer.set_duty_cycle_sp(INIT_STEERING_POWER);
	steer.run_direct();
	Sleep(TURN_SLEEP*2);
	steer.stop();
	steerLeft = steer.position();
	steerForward = (steerLeft + steerRight) / 2;
	steer.set_speed_sp(steer.max_speed());
	steer.set_position_sp(steerForward);
	steer.run_to_abs_pos();
	Sleep(TURN_SLEEP);
	printf("ev3car-drive: left %d, right %d, forward %d, duty_cycle %d, pos %d\n", steerLeft, steerRight, steerForward, steer.duty_cycle(), steer.position());
	//init drive
	drive.reset();
	InitMotor(&drive);
	drive.set_ramp_up_sp(RAMP_UP_MS);

	InitNetworkUDP(&socket_udp, &destination_udp, NULL, port, timeout_ms);

	//work
	MainLoop(socket_udp);
	
	//cleanup
	StopMotors();
	CloseNetworkUDP(socket_udp);
		
	printf("ev3car-drive: bye\n");
		
	return 0;
}

void MainLoop(int socket_udp) {
	int status;
	car_drive_packet packet;
	
	while(!g_finish_program) {
		status=RecvCarDrivePacket(socket_udp, &packet);
		if(status < 0) break;
		if(status == 0) {
			//timeout
			StopMotors();
			fprintf(stderr, "ev3car-drive: waiting for drive controller...\n");
		}
		else ProcessMessage(packet);
		if(IsStandardInputEOF()) break;
	}
}

void ProcessMessage(const car_drive_packet &packet) {	
	if (packet.command == KEEPALIVE) return;
	//g_end_turn = 0;
	//g_end_turn_stop = 0;
	if (packet.command == TURN) {
		//TURN
		fprintf(stdout, "Param2:%"+PRId16+"\n",packet.param2);
		if (packet.param2 == 0) steer.set_position_sp(steerForward);
		else if(packet.param2 > 0) steer.set_position_sp(steerLeft);
		else steer.set_position_sp(steerRight);
		steer.run_to_abs_pos();
		//FORWARD / BACKWARD
		if(packet.param1 > 0) drive.set_duty_cycle_sp(100);
		else drive.set_duty_cycle_sp(-100);
		drive.run_direct();
		turnPosition = drive.position() + packet.param2;
		/*if (packet.param2 != 0) {
			g_end_turn = 1;
			pthread_t completeThread;
			int rc = pthread_create(&completeThread, NULL, CompleteTurn, NULL);
			if (rc) {
				fprintf(stderr, "ev3car-drive: return code from pThread() was %d\n", rc);
			}
		}*/

	} else if (packet.command == FORWARD) {
		steer.set_position_sp(steerForward);
		steer.run_to_abs_pos();
		drive.set_duty_cycle_sp(100);
		drive.run_direct();
	} else if (packet.command == BACKWARD) {
		steer.set_position_sp(steerForward);
		steer.run_to_abs_pos();
		drive.set_duty_cycle_sp(-100);
		drive.run_direct();
	} else if (packet.command == STOP) {
		StopMotors();
	} else { //packet.command == TURNSTOP
		if (packet.param1 > 0) drive.set_duty_cycle_sp(100);
		else drive.set_duty_cycle_sp(-100);
		drive.run_direct();
		turnPosition = drive.position() + packet.param2;
		/*g_end_turn_stop = 1;
		pthread_t completeThread;
		int rc = pthread_create(&completeThread, NULL, CompleteTurnStop, NULL);
		if (rc) {
			fprintf(stderr, "ev3car-drive: return code from pThread() was %d\n", rc);
		}*/
	}
}

/*void* CompleteTurn(void* v) {
	while (drive.position() < turnPosition) SleepUs(TURN_SLEEP);
	if (g_end_turn) {
		steer.set_position_sp(steerForward);
		steer.run_to_abs_pos();
	}
	return NULL;
}*/

/*void* CompleteTurnStop(void* v) {
	while (drive.position() < turnPosition) SleepUs(TURN_SLEEP);
	if (g_end_turn_stop) {
		StopMotors();
		//TODO:SEND RESPONSE! (TURNSTOP)
	}
	return NULL;
}*/

void InitMotor(motor *m) {
	if(!m->connected())
		Die("ev3car-drive: motor not connected");
	m->set_stop_action(m->stop_action_brake);
}

void StopMotors() {
	steer.stop();
	drive.stop();
}


// returns CONTROL_PACKET_BYTES on success  0 on timeout, -1 on error
int RecvCarDrivePacket(int socket_udp, car_drive_packet *packet) {
	static char buffer[CONTROL_PACKET_BYTES];	
	int recv_len;
	
	if((recv_len = recvfrom(socket_udp, buffer, CONTROL_PACKET_BYTES, 0, NULL, NULL)) == -1) {
		if(errno==EAGAIN || errno==EWOULDBLOCK || errno==EINPROGRESS) return 0; //timeout!
		perror("ev3car-drive: error while receiving control packet");
		return -1;
	}
	if(recv_len < CONTROL_PACKET_BYTES) {
		fprintf(stderr, "ev3car-drive: received incomplete datagram\n");
		return -1; 
	}
	DecodeCarDrivePacket(packet, buffer);	

	return recv_len;	       
}
 
void DecodeCarDrivePacket(car_drive_packet *packet, const char *data) {
	packet->timestamp_us=be64toh(*((uint64_t*)data));
	packet->command=be16toh(*((int16_t*)(data+8)));
	packet->param1=be16toh(*((int16_t*)(data+10)));
	packet->param2=be32toh(*((int32_t*)(data+12)));	
}

void Usage() {
	printf("ev3car-drive udp_port timeout_ms\n\n");
	printf("examples:\n");
	printf("./ev3car-drive 8003 500\n");
}

void ProcessArguments(int argc, char **argv, int *port, int *timeout_ms) {
	if(argc!=3)
	{
		Usage();
		exit(EXIT_SUCCESS);		
	}
	
	long temp;
	
	temp=strtol(argv[1], NULL, 0);
	
	if(temp <= 0 || temp > 65535)
	{
		fprintf(stderr, "ev3car-drive: the argument port has to be in range <1, 65535>\n");
		exit(EXIT_SUCCESS);
	}

	*port=temp;
	temp=strtol(argv[2], NULL, 0);
	if(temp <= 0 || temp > 10000)
	{
		fprintf(stderr, "ev3car-drive: the argument timeout_ms has to be in range <1, 10000>\n");
		exit(EXIT_SUCCESS);
	}
	
	*timeout_ms=temp;
}

void Finish(int signal) {
	g_finish_program=1;
	//g_end_turn = 0;
	//g_end_turn_stop = 0;
}
