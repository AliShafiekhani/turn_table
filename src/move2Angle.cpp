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

/*------------Right Raspberry Pi---------------*/

#include <iostream>
#include <sstream>
#include <iomanip>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <algorithm>    // std::find
#include <sys/stat.h>

extern "C" {
#include "ArcusPerformaxDriver.h"
#include "commands.h"
};

#ifndef True
#define True 1
#endif
#ifndef False
#define False 0
#endif

using namespace std;

void usage(void) {
  cout << "Usage is: Move2Angle [Direction] [Angle]" << endl;
  cout << "Direction:" << endl; 
  cout << "\t -cw\t Move turn table CW" << endl; 
  cout << "\t -ccw\tMove turn table CCW" << endl;
  }


int main(int argc, char** argv)
{
  /* ----------------------------------------------------------------- */
  /* ------------------ TurnTable Initialization---------------------- */
  /* ----------------------------------------------------------------- */
  char 		lpDeviceString[PERFORMAX_MAX_DEVICE_STRLEN];
  AR_HANDLE	Handle; //usb handle
  AR_DWORD	num;
  
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
  cout << "enable close loop control"<<endl;
  enableCloseLoopPosControl(Handle, 100, 5000);
  char respond[64];
  setMoPosAs(Handle, respond, 0);
  setEnPosAs(Handle, respond, 0);
  int Angle = 0;
  if ((argc > 1) && (argv[1][0] == '-') && (argv[1][1] == 'c'))
  {
    if ((argv[1][2]=='W') || (argv[1][2]=='w')){
      Angle = atoi(&argv[2][0]);
    }
    else if((argv[1][2]=='C') || (argv[1][2]=='c')) {
      Angle = -atoi(&argv[2][0]);
    }
    else {
      cout << "Unknown Option " << argv[1] << endl;
      usage();
      return -1;
    }
    moveAndWait(Handle, Angle);
    if(!fnPerformaxComClose(Handle))
    {
	printf( "Error Closing\n");
	return 1;
    }
  }
  else {
    cout << "Unknown Option " << argv[1] << endl;
    usage();
    return -1;
    
  }

  return 0;
}