/*
 *  Copyright (C) 2017 Vision-Guided and Intelligent Robotics Lab
 *  Written by Ali Shafiekhani <Ashafiekhani@mail.missouri.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation. Meaning:
 *          keep this copyright notice,
 *          do  not try to make money out of it,
 *          it's distributed WITHOUT ANY WARRANTY,
 *
 *  You can get a copy of the GNU General Public License by writing to
 *  the Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 *  Boston, MA  02111-1307  USA
 *
 *
 */


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

#include "turn_table/MoveToAngle.h"

#include <stdio.h>
#include <string.h>

extern "C" {
#include <turn_table/ArcusPerformaxDriver.h>
};

class TurnTableDriver{
private:
	AR_HANDLE Handle_;
	ros::Publisher encoder_pub_;
	ros::Publisher status_pub_;
	ros::ServiceServer positioning_service_;
	double loop_hz_;
	double initial_angle_;
	bool set_init_angle_;
	bool enable_control_loop_;

protected:
  	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

public:
	TurnTableDriver(ros::NodeHandle&);
	~TurnTableDriver(void);
	bool initTurnTable(void);
	bool moveToAngle(turn_table::MoveToAngle::Request &req,
				turn_table::MoveToAngle::Response &res);
	void publishAngle(void);
	void publishStatus(void);
	void stop(void);
	void forceStop(void);
	void setMoPosAs(double);
	void setEnPosAs(double);
	float readEnPos(void);
	void moveHomeCW(void);
	void moveHomeCCW(void);
	void setSpeed(double, double);
	void enableCloseLoopPosControl(double, double);
	int getMotorStatus(void);
	int moveAndWait(double);
	void moveToPos(double);

};

TurnTableDriver::TurnTableDriver(ros::NodeHandle& nh)
	: nh_(nh),
	priv_nh_("~")
	
{
	priv_nh_.param<double>("loop_hz", loop_hz_, 10.0);
	priv_nh_.param<double>("initial_angle", initial_angle_, 0.0);
	priv_nh_.param<bool>("set_init_angle", set_init_angle_, false);
	priv_nh_.param<bool>("enable_control_loop",enable_control_loop_, true);

	encoder_pub_ = nh_.advertise<std_msgs::Float64>("/turn_table/current_angle",1);
	status_pub_ = nh_.advertise<std_msgs::Int16>("/turn_table/motor_status",1);
	positioning_service_ = nh_.advertiseService("move_to_angle", &TurnTableDriver::moveToAngle, this);

	if(!initTurnTable()){
		ROS_ERROR("Failed Initialization");
		nh_.shutdown();
	}
	else
		ROS_INFO("Initialization Done Successfully!");

	if(enable_control_loop_)
		enableCloseLoopPosControl(100, 5000);
	
	if(set_init_angle_){
		setMoPosAs(initial_angle_);
		setEnPosAs(initial_angle_);
	}

}
TurnTableDriver::~TurnTableDriver(){
	if(!fnPerformaxComClose(Handle_))
    {
	ROS_ERROR("Error Closing");
    }
}

bool TurnTableDriver::initTurnTable(){

	if(!fnPerformaxComOpen(0,&Handle_))
	{
	  ROS_ERROR("Error opening device");
	  return 0;
	}

	if(!fnPerformaxComSetTimeouts(5000,5000))
	{
	  ROS_ERROR("Error setting timeouts");
	  return 0;
	}
	if(!fnPerformaxComFlush(Handle_))
	{
	  ROS_ERROR("Error flushing the coms");
	  return 0;
	}
	return 1;
}

bool TurnTableDriver::moveToAngle(turn_table::MoveToAngle::Request &req,
								  turn_table::MoveToAngle::Response &res){
	res.Status = moveAndWait(req.Angle);
}

void TurnTableDriver::publishAngle(){
	std_msgs::Float64 angle;
	angle.data = readEnPos();
	encoder_pub_.publish(angle);
}
void TurnTableDriver::publishStatus(){
	std_msgs::Int16 status;
	status.data = getMotorStatus();
	status_pub_.publish(status);
}
//TODO: make a service call for stop
void TurnTableDriver::stop(){
	char command[64];
	char respond[64];
	strcpy(command, "STOP");
	if (!fnPerformaxComSendRecv(Handle_, command, 64, 64, respond))
	{
		ROS_ERROR("Could not STOP");
	}
}
//TODO: make a service call for force stop
void TurnTableDriver::forceStop(){
	char command[64];
	char respond[64];
	strcpy(command, "ABORT");
	if (!fnPerformaxComSendRecv(Handle_, command, 64, 64, respond))
	{
		ROS_ERROR("Could not Force STOP");
	}
}

void TurnTableDriver::setMoPosAs(double angle) // sets motor position
{
	char command[64];
	char respond[64];
	double moSteps = 10000 * angle;
	sprintf(command,"%s=%.0f", "PX", moSteps);
	ROS_INFO("%s", command);
	
	if (!fnPerformaxComSendRecv(Handle_, command, 64, 64, respond))
	{
		ROS_ERROR("Could not set Motor Position");
	}
	else
		ROS_INFO("Motor Initialized To %.0f Degree", angle);
}

void TurnTableDriver::setEnPosAs(double angle) // sets motor position
{
	double enCounts = 800 * angle;
	char command[64];
	char respond[64];
	sprintf(command,"%s=%.0f", "EX", enCounts);
	ROS_INFO("%s", command);

	if (!fnPerformaxComSendRecv(Handle_, command, 64, 64, respond))
	{
		ROS_ERROR("Could not Encoder Position");
	}
	else
		ROS_INFO("Encoder Initialized To %.0f Degree", angle);
}

float TurnTableDriver::readEnPos()
{
	char command[64], respond[64];

	strcpy(command, "EX");
	if (!fnPerformaxComSendRecv(Handle_, command, 64, 64, respond))
		ROS_ERROR("Could not send/receive Encoder Position");
	float angle = (float) atoi(respond)/800;
	return angle;
}
// TODO: make a service call for Homing
void TurnTableDriver::moveHomeCW()
{
	char command[64], respond[64];
	strcpy(command, "ZH+");
	if (!fnPerformaxComSendRecv(Handle_, command, 64, 64, respond))
	{
		ROS_ERROR("Could not move to home CW");
	}
	sleep(1);
	while(getMotorStatus() == 7)
	{
		sleep(1);
	}
	ROS_INFO("Turn table moved to Home CW");
}
// TODO: make a service call for Homing
void TurnTableDriver::moveHomeCCW()
{
	char command[64], respond[64];
	strcpy(command, "ZH-");
	if (!fnPerformaxComSendRecv(Handle_, command, 64, 64, respond))
	{
		ROS_ERROR("Could not move to home CCW");
	}
	sleep(1);
	while(getMotorStatus() == 7)
	{
		sleep(1);
	}
	ROS_INFO("Turn table moved to Home CCW");
}


void TurnTableDriver::setSpeed(double LSPD, double HSPD)
{
	char respond[64], lspd[64], hspd[64];
	double L = LSPD;
	double H = HSPD;
	sprintf(lspd,"LSPD=%.0f",L);
	sprintf(hspd,"HSPD=%.0f",H);
	if(!fnPerformaxComSendRecv(Handle_, lspd, 64, 64, respond))
		ROS_ERROR("Could not send");
	if(!fnPerformaxComSendRecv(Handle_, hspd, 64, 64, respond))
		ROS_ERROR("Could not send");
}

void TurnTableDriver::enableCloseLoopPosControl(double LSPD, double HSPD)
{
	char slr[64], slt[64], sle[64], sla[64];
	char respond[64],lspd[64], hspd[64], sl[64];
	double L = LSPD;
	double H = HSPD;

	sprintf(lspd,"LSPD=%.0f",L);
	sprintf(hspd,"HSPD=%.0f",H);
	strcpy(sl,  "SL=1");
	strcpy(slr,"SLR=12.5");
	strcpy(slt,"SLT=100");
	strcpy(sle,"SLE=500");
	strcpy(sla,"SLA=20");

	if(!fnPerformaxComSendRecv(Handle_, lspd, 64, 64, respond))
		ROS_ERROR("Could not send");
	if(!fnPerformaxComSendRecv(Handle_, hspd, 64, 64, respond))
		ROS_ERROR("Could not send");

	if(!fnPerformaxComSendRecv(Handle_, sl, 64, 64,respond))
		ROS_ERROR("Could not send");

	if(!fnPerformaxComSendRecv(Handle_, slr, 64, 64, respond))
		ROS_ERROR("Could not set SLR");
	if(!fnPerformaxComSendRecv(Handle_, slt, 64, 64, respond))
		ROS_ERROR("Could not set SLT");
	if(!fnPerformaxComSendRecv(Handle_, sle, 64, 64, respond))
		ROS_ERROR("Could not set SLE");
	if(!fnPerformaxComSendRecv(Handle_, sla, 64, 64, respond))
		ROS_ERROR("Could not set SLA");
}

int TurnTableDriver::getMotorStatus()
{
	char command[64];
	char respond[64];
	strcpy(command,"SLS");
	if(!fnPerformaxComSendRecv(Handle_, command, 64, 64, respond))
			ROS_ERROR("Could not set SLT");
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

	int i = atoi(respond);
	if (i == 10)
	{
		char dx[64];
		strcpy(dx, "DX");
		fnPerformaxComSendRecv(Handle_,dx , 64, 64, respond);
		ROS_INFO("DX value is : %s",respond);
	}
	return i;
}

int TurnTableDriver::moveAndWait(double angle) // Should not be used with closed loop positioning
{
	double enCounts = 800 * angle;
	char command[64], respond[64];
	sprintf(command,"X%.0f",enCounts);

	do
	{

		ROS_DEBUG("tries to send moving command");
		if(!fnPerformaxComSendRecv(Handle_, command, 64, 64, respond)){
			ROS_ERROR("Could not send Positioning Command");
			return getMotorStatus();
		}
		sleep(1);
	} while(respond[0]!='O');
	ros::Rate loop_rate(loop_hz_);
	while(getMotorStatus() == 1)
	{
		publishAngle();
		loop_rate.sleep();
	}
	return getMotorStatus();
}

void TurnTableDriver::moveToPos(double angle) // Should not be used with closed loop positioning
{
	char command[64], respond[64];
	double moSteps = 10000 * angle;
	sprintf(command,"X=%.0f", moSteps);
	if (!fnPerformaxComSendRecv(Handle_, command, 64, 64, respond))
	{
		ROS_ERROR("Could not move to home CCW");

	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turn_table_driver");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  TurnTableDriver turn_table(nh);

  while(nh.ok()){
  	ros::spinOnce();
  	turn_table.publishAngle();
  	turn_table.publishStatus();
  	loop_rate.sleep();
  }
 
  return 0;
}