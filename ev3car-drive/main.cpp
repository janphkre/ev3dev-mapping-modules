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
volatile sig_atomic_t g_end_turn = 0;
volatile sig_atomic_t g_end_turn_stop = 0;
// temporary control packet, subject to change
struct car_drive_packet {
	uint64_t timestamp_us;
	int16_t command;
	int16_t param1;//forward/backward
	int16_t param2;//steering segment
};

const int CONTROL_PACKET_BYTES = 14; //8 + 3*2 = 14 bytes
enum Commands { KEEPALIVE = 0, TURN = 1, FORWARD = 2, BACKWARD = 3, STOP = 4, TURNSTOP = 5 };

const int RAMP_UP_MS = 500;
const int INIT_STEERING_POWER = 20;
const mode_type STALLED = "stalled";

int steerLeft;
int steerForward;
int steerRight;
medium_motor *steer;
large_motor *drive;
int turnPosition;

void MainLoop(int socket_udp);
void ProcessMessage(const drive_packet &packet);

void CompleteTurn();
void CompleteTurnStop();

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
	motor_steer(OUTPUT_B);
	motor_drive(OUTPUT_A);
	
	ProcessArguments(argc, argv, &port, &timeout_ms);
	SetStandardInputNonBlocking();
	
	//init
	RegisterSignals(Finish);

	//init steering
	InitMotor(&motor_steer);
	motor_steer->set_duty_cycle_sp(INIT_STEERING_POWER);
	motor_steer->run_direct();
	while (!motor_steer->state().Contains(STALLED)) Thread.Yield();//TODO:sleep or something.
	steerLeft = motor_steer->position_sp();
	motor_steer->set_duty_cycle_sp(-INIT_STEERING_POWER);
	while (!motor_steer->state().Contains(STALLED)) Thread.Yield();//TODO:sleep or something.
	steerRight = motor_steer->position_sp();
	motor_steer->stop();
	steerForward = ((steerLeft + steerRight) / 2) + steerLeft;
	motor_steer->run_to_abs_pos(steerForward);

	//init drive
	InitMotor(&motor_drive);
	motor_drive->set_ramp_up_sp(RAMP_UP_MS);

	InitNetworkUDP(&socket_udp, &destination_udp, NULL, port, timeout_ms);

	//work
	MainLoop(socket_udp, &motor_steer, &motor_drive);
	
	//cleanup
	StopMotors(&motor_steer, &motor_drive);
	CloseNetworkUDP(socket_udp);
		
	printf("ev3cardrive: bye\n");
		
	return 0;
}

void MainLoop(int socket_udp) {
	int status;
	drive_packet packet;
	
	while(!g_finish_program) {
		status=RecvDrivePacket(socket_udp, &packet);
		if(status < 0) break;
		if(status == 0) {
			//timeout
			StopMotors(steer, drive);
			fprintf(stderr, "ev3drive: waiting for drive controller...\n");
		}
		else ProcessMessage(packet, steer, drive);
		if(IsStandardInputEOF()) break;
	}
}

void ProcessMessage(const drive_packet &packet) {	
	if (packet.command == KEEPALIVE) return;
	g_end_turn = 0;
	g_end_turn_stop = 0;
	if (packet.command == TURN) {
		//TURN
		if (packet.param2 == 0) steer->set_position_sp(steerForward);
		else if(packet.param2 > 0) steer->set_position_sp(steerLeft);
		else steer->set_position_sp(steerRight);
		steer->run_to_abs_pos();
		//FORWARD / BACKWARD
		if(packet.param1 > 0) drive->set_duty_cycle_sp(100);
		else drive->set_duty_cycle_sp(-100);
		drive->run_direct();
		turnPosition = drive->position() + packet.param2;
		if (packet.param2 != 0) {
			g_end_turn = 1;
			//TODO: start CompleteTurn() als 2.Thread oder fork() mit g_end_turn als shared memory
		}

	} else if (packet.command == FORWARD) {
		drive->set_duty_cycle_sp(100);
		drive->run_direct();
	} else if (packet.command == BACKWARD) {
		drive->set_duty_cycle_sp(-100);
		drive->run_direct();
	} else if (packet.command == STOP) {
		StopMotors(steer, drive);
	} else { //packet.command == TURNSTOP
		if (packet.param1 > 0) drive->set_duty_cycle_sp(100);
		else drive->set_duty_cycle_sp(-100);
		drive->run_direct();
		turnPosition = drive->position() + packet.param2;
		g_end_turn_stop = 1;
		//TODO: start CompleteTurnStop() als 2.Thread oder fork() mit g_end_turn als shared memory
	}
}

void CompleteTurn() {
	while (drive->position() < turnPosition) Thread.Yield();//TODO
	if (g_end_turn) {
		steer->set_position_sp(steerForward);
		steer->run_to_abs_pos();
	}
}

void CompleteTurnStop() {
	while (drive->position() < turnPosition) Thread.Yield();//TODO
	if (g_end_turn_stop) {
		StopMotors();
		//TODO:SEND RESPONSE! (TURNSTOP)
	}
}

void InitMotor(motor *m) {
	if(!m->connected())
		Die("ev3drive: motor not connected");
	m->set_stop_action(m->stop_action_brake);
}

void StopMotors() {
	steer->stop();
	drive->stop();
}


// returns CONTROL_PACKET_BYTES on success  0 on timeout, -1 on error
int RecvCarDrivePacket(int socket_udp, car_drive_packet *packet) {
	static char buffer[CONTROL_PACKET_BYTES];	
	int recv_len;
	
	if((recv_len = recvfrom(socket_udp, buffer, CONTROL_PACKET_BYTES, 0, NULL, NULL)) == -1) {
		if(errno==EAGAIN || errno==EWOULDBLOCK || errno==EINPROGRESS) return 0; //timeout!
		perror("ev3drive: error while receiving control packet");
		return -1;
	}
	if(recv_len < CONTROL_PACKET_BYTES) {
		fprintf(stderr, "ev3drive: received incomplete datagram\n");
		return -1; 
	}
	DecodeCarDrivePacket(packet, buffer);	

	return recv_len;	       
}
 
void DecodeCarDrivePacket(car_drive_packet *packet, const char *data) {
	packet->timestamp_us=be64toh(*((uint64_t*)data));
	packet->command=be16toh(*((int16_t*)(data+8)));
	packet->param1=be8toh(*((int16_t*)(data+10)));
	packet->param2=be32toh(*((int32_t*)(data+12)));	
}

void Usage() {
	printf("ev3drive udp_port timeout_ms\n\n");
	printf("examples:\n");
	printf("./ev3drive 8003 500\n");
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
		fprintf(stderr, "ev3drive: the argument port has to be in range <1, 65535>\n");
		exit(EXIT_SUCCESS);
	}

	*port=temp;
	temp=strtol(argv[2], NULL, 0);
	if(temp <= 0 || temp > 10000)
	{
		fprintf(stderr, "ev3drive: the argument timeout_ms has to be in range <1, 10000>\n");
		exit(EXIT_SUCCESS);
	}
	
	*timeout_ms=temp;
}

void Finish(int signal) {
	g_finish_program=1;
	g_end_turn = 0;
	g_end_turn_stop = 0;
}
