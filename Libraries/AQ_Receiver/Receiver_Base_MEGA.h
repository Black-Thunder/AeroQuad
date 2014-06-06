/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AEROQUAD_RECEIVER_BASE_MEGA_H_
#define _AEROQUAD_RECEIVER_BASE_MEGA_H_

#include "Arduino.h"
#include "Receiver_Base.h"


int receiverCommand[MAX_NB_CHANNEL] = {1500,1500,1500,1000,1000,1000,1000,1000};
float receiverMinValue[MAX_NB_CHANNEL] = {1000,1000,1000,1000,1000,1000,1000,1000};
float receiverMaxValue[MAX_NB_CHANNEL] = {2000,2000,2000,2000,2000,2000,2000,2000};
byte receiverChannelMap[MAX_NB_CHANNEL] = {XAXIS,YAXIS,ZAXIS,THROTTLE,MODE,AUX1,AUX2,AUX3};

#if defined(GraupnerFailsafe)
	int lastGoodReceiverCommand[MAX_NB_CHANNEL] = {0,0,0,0,0,0,0,0};
	boolean isFailsafeActive = false;
	int failsafeCounter = 0;
#endif

void initializeReceiverPPM();
void initializeReceiverPWM();
void initializeReceiverSBUS();

void readReceiver();

functionPtr initializeReceiver[] = {initializeReceiverPPM,initializeReceiverPWM,initializeReceiverSBUS};

int getRawChannelValuePPM(byte channel);
int getRawChannelValuePWM(byte channel);
int getRawChannelValueSBUS(byte channel);

intFunctionPtrByte getRawChannelValue[] = {getRawChannelValuePPM,getRawChannelValuePWM,getRawChannelValueSBUS};

#if defined(GraupnerFailsafe)
boolean areOldAndNewChannelValuesTheSame() {
	bool areSame = true;

	for (byte channel = XAXIS; channel < LAST_CHANNEL; channel++) {
		if(lastGoodReceiverCommand[channel] != (map(((*getRawChannelValue[receiverTypeUsed])(channel)),receiverMinValue[channel],receiverMaxValue[channel],1000,2000))) {
			areSame = false;
		}
	}

	return areSame;
}

boolean checkFailsafeStatus() {

	if (areOldAndNewChannelValuesTheSame())	{
		failsafeCounter++;
	}
	else {
		failsafeCounter = 0;
		isFailsafeActive = false;
	}

	if(failsafeCounter > 40) {
		isFailsafeActive = true;
		return true;
	}
	return false;
}

void overrideChannelValuesWithFailsafe() {
	receiverCommand[receiverChannelMap[XAXIS]] = 1500;
	receiverCommand[receiverChannelMap[YAXIS]] = 1500;
	receiverCommand[receiverChannelMap[ZAXIS]] = 1500;
	receiverCommand[receiverChannelMap[THROTTLE]] = 1400;
	receiverCommand[receiverChannelMap[MODE]] = 2000;
	receiverCommand[receiverChannelMap[AUX1]] = 2000;

	if (LAST_CHANNEL == 8) {
		receiverCommand[receiverChannelMap[AUX2]] = 1000;
		receiverCommand[receiverChannelMap[AUX3]] = 1000;
	}
}
#endif

void readReceiver()
{
  for(byte channel = XAXIS; channel < LAST_CHANNEL; channel++) {
    #if defined(GraupnerFailsafe)
		if (checkFailsafeStatus())	{
			overrideChannelValuesWithFailsafe();
		} 
		else
	#endif
		{
			// Apply receiver calibration adjustment
			receiverCommand[channel] = map(((*getRawChannelValue[receiverTypeUsed])(channel)),receiverMinValue[channel],receiverMaxValue[channel],1000,2000);
		
			#if defined(GraupnerFailsafe)
				lastGoodReceiverCommand[channel] = receiverCommand[channel];
			#endif
		}
  }
}
  
/*  
const float getReceiverSIData(byte channel) {
  return ((receiverCommand[receiverChannelMap[channel]] - 1500) * (2.5 * PWM2RAD));  // +/- 2.5RPS 50% of full rate
}
*/
  
#endif



