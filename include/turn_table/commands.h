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

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <libusb-1.0/libusb.h>

typedef libusb_device_handle*	AR_HANDLE;

AR_HANDLE motorInitialization(AR_HANDLE Handle);
void readPos(AR_HANDLE Handle, char respond[64]);
void stop(AR_HANDLE Handle, char respond[64]);
void forceStop(AR_HANDLE Handle, char respond[64]);
void setMoPosAs(AR_HANDLE Handle, char respond[64], int angle);
void setEnPosAs(AR_HANDLE Handle, char respond[64], int angle);
float readEnPos(AR_HANDLE Handle);
void moveHomeCW(AR_HANDLE Handle);
void moveHomeCCW(AR_HANDLE Handle);
void enableCloseLoopPosControl(AR_HANDLE Handle, unsigned int LSPD, unsigned int HSPD);
int getMotorStatus(AR_HANDLE Handle);
void moveAndWait(AR_HANDLE Handle, int angle);
void setSpeed(AR_HANDLE Handle, unsigned int LSPD, unsigned int HSPD);
void moveToPos(AR_HANDLE Handle, int angle);
#endif /* COMMANDS_H_ */
