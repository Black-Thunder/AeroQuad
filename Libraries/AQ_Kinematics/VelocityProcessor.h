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

#ifndef _AQ_VELOCITY_PROCESSOR_
#define _AQ_VELOCITY_PROCESSOR_

#include "GlobalDefined.h"
#include "AP_Buffer.h"

float timeConstantZ = 0.1;    
float k1_z = 3 / timeConstantZ;
float k2_z = 3 / (timeConstantZ*timeConstantZ);
float k3_z = 1 / (timeConstantZ*timeConstantZ*timeConstantZ);

float computedZVelicity = 0.0;
float zErrorPosition = 0.0;
float zPositionCorrection = 0.0;
float accelZCorrection = 0.0;
float baseZPosition = 0.0;
AP_BufferFloat_Size15 zBasePositionHistoryBuffer;

void initVelocityProcessor();
void updateVelocityProcessorGains();
void computeVelocity(float filteredAccelZ, float dt);
void computerVelocityErrorFromBaroAltitude(float baroAltitude);


void computerVelocityErrorFromBaroAltitude(float baroAltitude)
{
	float historySum = zBasePositionHistoryBuffer.peek(14);
    zErrorPosition = baroAltitude - (historySum + zPositionCorrection);
}


void computeVelocity(float filteredAccelZ, float dt)
{
	filteredAccelZ = constrain(filteredAccelZ, -10.0,10.0); // Sercurity to prevent overflow
	
	accelZCorrection += zErrorPosition * k3_z  * dt;
	computedZVelicity += zErrorPosition * k2_z  * dt;
	zPositionCorrection += zErrorPosition * k1_z  * dt;
	float velocity_increase = (filteredAccelZ + accelZCorrection) * dt;
	baseZPosition += (computedZVelicity + velocity_increase*0.5) * dt;
	computedZVelicity += velocity_increase;
	zBasePositionHistoryBuffer.add(baseZPosition);
}

#endif
