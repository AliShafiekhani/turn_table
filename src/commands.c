/*
 *  Copyright (C) 2015 Vision-Guided and Intelligent Robotics Lab
 *  Written by Ali Shafiekhani <Ashafiekhani@mail.missouri.edu>
 *             Guilherme Nelson DeSouza <DeSouzaG@missouri.edu>
 *  Based on the sample code provided in the RaspiCam Package
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation. Meaning:
 *          keep this copyright notice,
 *          do  not try to make money out of it,
 *          it's distributed WITHOUT ANY WARRANTY,
 *          yada yada yada...
 *
 *  You can get a copy of the GNU General Public License by writing to
 *  the Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 *  Boston, MA  02111-1307  USA
 *
 *
 */
#include <stdio.h>
#include <string.h>
#include <turn_table/ArcusPerformaxDriver.h>
//#include "commands.h"

AR_HANDLE motorInitialization(AR_HANDLE Handle){
	if(!fnPerformaxComOpen(0,&Handle))
	{
	  printf( "Error opening device\n");
	  return 1;
	}

	if(!fnPerformaxComSetTimeouts(5000,5000))
	{
	  printf("Error setting timeouts\n");
	  return 1;
	}
	if(!fnPerformaxComFlush(Handle))
	{
	  printf("Error flushing the coms\n");
	  return 1;
	}
	return Handle;
}

void readPos(Handle, respond)
{

	char command[64];
	strcpy(command, "PX"); // read the current position by counting steps
	if(!fnPerformaxComSendRecv(Handle, command, 64, 64, respond))
	{
		printf("Could not send Position\n");
	}
}
void stop(Handle, respond)
{
	char command[64];
	strcpy(command, "STOP");
	if (!fnPerformaxComSendRecv(Handle, command, 64, 64, respond))
	{
		printf("Could not STOP\n");
	}
}
void forceStop(Handle, respond)
{
	char command[64];
	strcpy(command, "ABORT");
	if (!fnPerformaxComSendRecv(Handle, command, 64, 64, respond))
	{
		printf("Could not Force STOP\n");
	}
}
void setMoPosAs(Handle, respond, angle) // sets motor position
{
	char command[64];
	double moSteps = 10000 * angle;
	sprintf(command,"%s=%.0f", "PX", moSteps);
	printf("%s\n", command);
	if (!fnPerformaxComSendRecv(Handle, command, 64, 64, respond))
	{
		printf("Could not set Motor Position\n");
	}
}
void setEnPosAs(Handle, respond, angle) // sets motor position
{
	double enCounts = 800 * angle;
	char command[64];
	sprintf(command,"%s=%.0f", "EX", enCounts);
	if (!fnPerformaxComSendRecv(Handle, command, 64, 64, respond))
	{
		printf("Could not Encoder Position\n");
	}
}

float readEnPos(Handle)
{
	char command[64], respond[64];

	strcpy(command, "EX");
	if (!fnPerformaxComSendRecv(Handle, command, 64, 64, respond))
		printf("Could not send/receive Encoder Position");
	float angle = (float) atoi(respond)/800;
	return angle;
}

void moveHomeCW(Handle)
{
	char command[64], respond[64];
	strcpy(command, "ZH+");
	if (!fnPerformaxComSendRecv(Handle, command, 64, 64, respond))
	{
		printf("Could not move to home CW\n");
	}
	sleep(1);
	while(getMotorStatus(Handle) == 7)
	{
		sleep(1);
	}
	printf("moved to Home CW\n");
}

void moveHomeCCW(Handle)
{
	char command[64], respond[64];
	strcpy(command, "ZH-");
	if (!fnPerformaxComSendRecv(Handle, command, 64, 64, respond))
	{
		printf("Could not move to home CCW\n");
	}
	sleep(1);
	while(getMotorStatus(Handle) == 7)
	{
		sleep(1);
	}
	printf("moved to Home CCW\n");

}

void setSpeed(Handle, LSPD, HSPD)
{
	char respond[64], lspd[64], hspd[64];
	double L = LSPD;
	double H = HSPD;
	sprintf(lspd,"LSPD=%.0f",L);
	sprintf(hspd,"HSPD=%.0f",H);
	if(!fnPerformaxComSendRecv(Handle, lspd, 64, 64, respond))
		printf("Could not send\n");
	if(!fnPerformaxComSendRecv(Handle, hspd, 64, 64, respond))
		printf("Could not send\n");
}

void enableCloseLoopPosControl(Handle, LSPD, HSPD)
{
	char slr[64], slt[64], sle[64], sla[64];
	char respond[64], lspd[64], hspd[64];
	double L = LSPD;
	double H = HSPD;

	sprintf(lspd,"LSPD=%.0f",L);
	sprintf(hspd,"HSPD=%.0f",H);
	strcpy(slr,"SLR=12.5");
	strcpy(slt,"SLT=100");
	strcpy(sle,"SLE=500");
	strcpy(sla,"SLA=20");

	if(!fnPerformaxComSendRecv(Handle, lspd, 64, 64, respond))
		printf("Could not send\n");
	if(!fnPerformaxComSendRecv(Handle, hspd, 64, 64, respond))
		printf("Could not send\n");

	if(!fnPerformaxComSendRecv(Handle, "SL=1", 64, 64, respond))
		printf("Could not send\n");

	if(!fnPerformaxComSendRecv(Handle, slr, 64, 64, respond))
		printf("Could not set SLR\n");
	if(!fnPerformaxComSendRecv(Handle, slt, 64, 64, respond))
		printf("Could not set SLT\n");
	if(!fnPerformaxComSendRecv(Handle, sle, 64, 64, respond))
		printf("Could not set SLE\n");
	if(!fnPerformaxComSendRecv(Handle, sla, 64, 64, respond))
		printf("Could not set SLA\n");
}
int getMotorStatus(Handle)
{
	char command[64];
	char respond[64];
	strcpy(command,"SLS");
	if(!fnPerformaxComSendRecv(Handle, command, 64, 64, respond))
			printf("Could not set SLT\n");
	// i
	// 0	:	Idle
	// 1	:	Moving
	// 2	:	Correcting
	// 3	:	Stopping
	// 4	:	Aborting
	// 5	:	Jogging
	// 6	:	Homing
	// 7	:	Z-Homing
	// 8&9	:	Correction range error
	// 10	:	Stall Error. DX value has exceeded
//	printf("Motor Status = %s\n",respond);
	int i = atoi(respond);
	if (i == 10)
	{
		fnPerformaxComSendRecv(Handle, "DX", 64, 64, respond);
		printf("DX value is : %s\n",respond);
	}
	return i;

}

void moveAndWait(Handle, angle) // Should not be used with closed loop positioning
{
	double enCounts = 800 * angle;
	char command[64], respond[64];
	sprintf(command,"X%.0f",enCounts);

	do
	{
//		printf("tries to send moving command\n");
		if(!fnPerformaxComSendRecv(Handle, command, 64, 64, respond))
			printf("Could not send Positioning Command\n");
		sleep(1);
	} while(respond[0]!='O');

	while(getMotorStatus(Handle) == 1)
	{
		sleep(1);
	}
}
void moveToPos(Handle, angle) // Should not be used with closed loop positioning
{
	char command[64], respond[64];
	double moSteps = 10000 * angle;
	sprintf(command,"%X=%.0f", moSteps);
	if (!fnPerformaxComSendRecv(Handle, command, 64, 64, respond))
	{
		printf("Could not move to home CCW\n");
	}
}
