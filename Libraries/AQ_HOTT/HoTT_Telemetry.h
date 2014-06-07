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

#ifndef _AEROQUAD_HOTT_TELEMETRY_H_
#define _AEROQUAD_HOTT_TELEMETRY_H_

#include "HoTT.h"
#include <stdio.h>

HardwareSerial *hottV4Serial;

static uint8_t minutes = 0;
static uint16_t milliseconds = 0;
const unsigned int varioSoundNeutral = 30000;
static signed int maxAltitude = 500;
static signed int minAltitude = 500;
volatile unsigned int CountMilliseconds = 0;
bool isAHOff, isAHOn, isNavOn, isHoldOn, isGPSOff;

/* ##################################################################### *
 *                HoTTv4 Common Serial                                   *
 * ##################################################################### */

/**
 * Writes out given data to data register.
 */
static void hottV4SerialWrite(uint8_t data) {
  hottV4Serial->write(data);
}

/**
 * Clears input buffer
 */
void hottV4SerialClearInput() {
	while(hottV4Serial->available() > 0) {
		hottV4Serial->read();
	}
}

/**
 * Reads last byte from input buffer
 */
int hottV4SerialRead() {
	while(hottV4Serial->available() > 1) {
		hottV4Serial->read();
	}
    if(hottV4Serial->available()) {
    	return hottV4Serial->read();
    } else {
    	return 0xff;
    }
}

#define hottV4TelemetryBufferSize 45
static uint8_t  hottV4TelemetryBuffer[hottV4TelemetryBufferSize];
static byte  hottV4TelemetryBufferIndex;

/**
 * Buffer telemetry data
 */
static void hottV4SendBinary(uint8_t *data) {
  uint16_t crc = 0;

  for (uint8_t index = 0; index < hottV4TelemetryBufferSize-1; index++) {
    crc = crc + data[index];
    hottV4TelemetryBuffer[index] = data[index];
   }

  uint8_t crcVal = crc & 0xFF;
  hottV4TelemetryBuffer[hottV4TelemetryBufferSize-1] = crcVal;

  hottV4TelemetryBufferIndex = 0;
}

/* ##################################################################### *
 *                HoTTv4 Module specific Update functions                *
 * ##################################################################### */

#if defined(HOTTV4DIR)
/**
 * Updates current direction related on compass information.
 */
static int hottV4UpdateDirection() {
	return (((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360) >> 1;
	}
#endif

static void HoTTInvertDisplay(uint8_t *data) {
    data[4] = 0x80; // Inverts MikroKopter Telemetry Display for Voltage
}

#if defined(HOTTV4BATT)
/**
 * Updates battery voltage telemetry data with given value.
 * Resolution is in 0,1V, e.g. 0x7E == 12,6V.
 * If value is below speakBatteryWarning, telemetry alarm is triggered
 */
static short hottv4UpdateBattery(uint8_t *data) {
	short voltage = batteryData[0].voltage/10;

	if (speakBatteryWarning) {
		HoTTInvertDisplay(data);
	}

	return voltage;
}

static short hottv4UpdateCurrent() {
	if (batteryData[0].cPin != BM_NOPIN) return batteryData[0].current/100;
	return 0;
}

static long hottv4UpdateCapacity() {
	if (batteryData[0].cPin != BM_NOPIN) return batteryData[0].usedCapacity/1000;
	return 0;
}
#endif


#if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
/**
 * Current relative altitude based on baro or ultrasonic values. 
 * Result is displayed in meter.
 */
static int32_t hottv4UpdateAlt() {
  int32_t alt = 0;
  
#if defined(AltitudeHoldBaro)
  alt = (int)getBaroAltitude() + 500;  // 500 == 0m
#endif 

#if defined(AltitudeHoldRangeFinder)
  if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX])) {
	alt = (int)rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] + 500;
  }
#endif
  
  if(alt > maxAltitude) maxAltitude = alt;
  else if(alt < minAltitude) minAltitude = alt;

  return alt;
}

static unsigned int hottv4UpdateAltVario() {
	unsigned int varioSound = varioSoundNeutral;

	if(altitudeHoldState == VELOCITY_HOLD_STATE || altitudeHoldState == ALTITUDE_HOLD_STATE)	{
		if((receiverCommand[receiverChannelMap[THROTTLE]] > (altitudeHoldThrottle + altitudeHoldBump))) varioSound += 300;
		else if((receiverCommand[receiverChannelMap[THROTTLE]] < (altitudeHoldThrottle - altitudeHoldBump))) varioSound -= 300;
	}

	return varioSound;
}
#endif

unsigned int SetDelay (unsigned int t) {
	return(CountMilliseconds + t + 1);
}

char CheckDelay(unsigned int t)	{
	return(((t - CountMilliseconds) & 0x8000) >> 9);
}

static unsigned char hottVoiceOutput() {
	unsigned char status = 0;
	static char oldStatus = 0;
	static int repeat;

#if defined(GraupnerFailsafe)
	if (isFailsafeActive) {
		status = HoTTv4NotificationErrorReceiver;
	}
#endif

#if defined(HOTTV4BATT)
	if (speakBatteryWarning) {
		status = HoTTv4NotificationUndervoltage;
	}
#endif

	if(!SpeakHoTT) {
#if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
		if(altitudeHoldState == VELOCITY_HOLD_STATE || altitudeHoldState == ALTITUDE_HOLD_STATE)  isAHOn = true;
		else if(altitudeHoldState == ALTPANIC || altitudeHoldState == OFF) isAHOff = true;

		 if(isAHOn) {
			 if (altitudeHoldState == ALTPANIC || altitudeHoldState == OFF) {
				isAHOn = false;
				SpeakHoTT = HoTTv4NotificationAltitudeOff;
			 }
		 }
		 if(isAHOff) {
			 if(altitudeHoldState == VELOCITY_HOLD_STATE || altitudeHoldState == ALTITUDE_HOLD_STATE) {
				 isAHOff = false;
				 SpeakHoTT = HoTTv4NotificationAltitudeOn;
			 }
		 }
#endif

#if defined(HOTTV4NAV)
		 if(navigationState == ON)  isNavOn = true;
		 else if(positionHoldState == ON) isHoldOn = true;
		 else if(positionHoldState == OFF && navigationState == OFF) isGPSOff = true;

		 if(isNavOn) {
			 if (positionHoldState == ON) {
				 isNavOn = false;
				 SpeakHoTT = HoTTv4NotificationGPSHold;
			 }
			 if (positionHoldState == OFF && navigationState == OFF) {
				 isNavOn = false;
				 SpeakHoTT = HoTTv4NotificationGPSOff;
			 }
		 }
		 if(isHoldOn) {
			 if(navigationState == ON) {
				 isHoldOn = false;
				 SpeakHoTT = HoTTv4NotificationGPSHome;
				 }
			 if (positionHoldState == OFF && navigationState == OFF) {
				 isHoldOn = false;
				 SpeakHoTT = HoTTv4NotificationGPSOff;
				 }
		 }
		 if(isGPSOff) {
			 if (positionHoldState == ON) {
				 isGPSOff = false;
				 SpeakHoTT = HoTTv4NotificationGPSHold;
			 }
			 if(navigationState == ON) {
				 isGPSOff = false;
				 SpeakHoTT = HoTTv4NotificationGPSHome;
			}
		 }
#endif
	}
	
	if(!status) {
		status = SpeakHoTT; 
	}

	if(oldStatus == status) {
		if(!CheckDelay(repeat)) return 0;
		repeat = SetDelay(5000);
	}
	else repeat = SetDelay(2000);

	if(status) 	{
		if(status == SpeakHoTT) SpeakHoTT = 0;
	}   

	oldStatus = status;

	return status;
}


/**
 * Updates current flight time (with motors armed) by counting the seconds 
 * from the moment power was applied.
 */
static void hottv4UpdateFlightTime(uint8_t *data) {
  static uint32_t previousEAMUpdate = 0;
  
  uint16_t timeDiff = millis() - previousEAMUpdate;
  previousEAMUpdate += timeDiff;
  
  CountMilliseconds += timeDiff;

  if (motorArmed) {
    milliseconds += timeDiff;
	
    if (milliseconds >= 60000) {
      milliseconds -= 60000;
      minutes += 1;
    }
  }
  
  data[39] = minutes;
  // Enough accuracy and faster than divide by 1000
  data[40] = (milliseconds >> 10) ;
}

/**
 * Call to initialize HOTTV4
 */
void hottv4Init(HardwareSerial *serial) {
  hottV4Serial = serial;
  hottV4Serial->begin(19200);
}

/* ##################################################################### *
 *                HoTTv4 General Module                                      *
 * ##################################################################### */

/**
 * Main method to send General telemetry data
 */
static void FillGeneralTelemetryPackage() {

 uint8_t telemetry_data[] = { 
              0x7C, /* 0 */
              HOTTV4_GENERAL_MODULE, /* 1 */
              0x00, /* 2 Alarm */
              HOTTV4_GENERAL_SENSOR_ID, /* 3 */
              0x00, /* 4 InverseStatus1 */
              0x00, /* 5 InverseStatus2 */
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 6-11 Voltage Cell 1-6 208 = 4,16V  (Voltage * 50 = Value) */
              0x00, 0x00, /* 12-13 Battetry 1 LSB/MSB in 100mv steps, 50 == 5V */
              0x00, 0x00, /* 14-15 Battetry 2 LSB/MSB in 100mv steps, 50 == 5V */
              0x14, /* 16 Temp 1, Offset of 20. 20 == 0C */ 
              0x14, /* 17 Temp 2, Offset of 20. 20 == 0C */
			  0x00, /* 18 FuelPercent */
			  0x00, 0x00, /* 19-20 FuelCapacity */
			  0x00, 0x00, /* 21-22 Rpm */
              0xF4, 0x01, /* 23-24 Altitude Offset -500. 500 == 0 */
			  0x48, 0x00, /* 25-26 m_s 30000 = 0 */ 
              0x78, /* 27 m_3s 120 = 0*/
              0x00, 0x00, /* 28-29 Current LSB, MSB 1 = 0.1A */
              0x00, 0x00, /* 30-31 Drive Voltage 66 = 6.6V*/
              0x00, 0x00,  /* 32-33 Capacity 1 = 10mAh */
              0x00, 0x00, /* 34-35 Speed */
			  0x00, /* 36  LowestCellVoltage*/
			  0x00, /* 37 LowestCellNumber */
              0x00, 0x00, /* 38-39 RPM2 10er steps, 300 == 3000rpm */
              0x00, /* 40 ErrorNumber */
              0x00, /* 41 Pressure in 0,1bar 20 = 2,0bar */
              0x00, /* 42 Version Number */
              0x7D, /* 43 End sign */
              0x00 /* 44 Checksum */
 };
 
   #if defined(HOTTV4ALTITUDE)
    int32_t altitude = hottv4UpdateAlt();
	telemetry_data[23] = altitude;
	telemetry_data[24] = (altitude >> 8) & 0xFF;

	unsigned int varioSound = hottv4UpdateAltVario();
	telemetry_data[25] = varioSound;
    telemetry_data[26] = (varioSound >> 8) & 0xFF;
	
	telemetry_data[27] = 120;
  #endif

  #if defined(HOTTV4BATT)
    short voltage = hottv4UpdateBattery(telemetry_data);
	telemetry_data[30] = telemetry_data[12] = telemetry_data[14] = voltage;
	telemetry_data[31] = telemetry_data[13] = telemetry_data[15] = (voltage >> 8) & 0xFF;

	short current = hottv4UpdateCurrent();
	telemetry_data[28] = current;
	telemetry_data[29] = (current >> 8) & 0xFF;

	long capacity = hottv4UpdateCapacity();
	telemetry_data[32] = capacity;
	telemetry_data[33] = (capacity >> 8) & 0xFF;
  #endif
  
   // Write out telemetry data as General Module to serial           
  hottV4SendBinary(telemetry_data);
}


/* ##################################################################### *
 *                HoTTv4 EAM Module                                      *
 * ##################################################################### */

/**
 * Main method to send EAM telemetry data
 */
static void FillEAMTelemetryPackage() {  
  uint8_t telemetry_data[] = { 
              0x7C, /* 0 */
              HOTTV4_ELECTRICAL_AIR_MODULE, /* 1 */
              0x00, /* 2 Alarm */
              HOTTV4_ELECTRICAL_AIR_SENSOR_ID, /* 3 */
              0x00, 0x00, /* 4-5 Alarm Value 1 and 2 */
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 6-12 Low Voltage Cell 1-7 in 2mV steps */
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 13-19 High Voltage Cell 1-7 in 2mV steps */
              0x00, 0x00, /* 20-21 Battetry 1 LSB/MSB in 100mv steps, 50 == 5V */
              0x00, 0x00, /* 22-23 Battetry 2 LSB/MSB in 100mv steps, 50 == 5V */
              0x14, /* 24 Temp 1, Offset of 20. 20 == 0C */ 
              0x14, /* 25 Temp 2, Offset of 20. 20 == 0C */
              0xF4, 0x01, /* 26-27 Height. Offset -500. 500 == 0 */
              0x00, 0x00, /* 28-29 Current LSB, MSB 1 = 0.1A */
              0x00, 0x00, /* 30-31 Drive Voltage */
              0x00, 0x00,  /* 32-33 Capacity mAh */
              0x48, 0x00, /* 34-35 m_s */ 
              0x78, /* 36 m_3s 120 = 0*/
              0x00, 0x00, /* 37-38 RPM. 10er steps, 300 == 3000rpm */
              0x00, /* 39 Electric minutes */
              0x00, /* 40 Electric seconds */
              0x00, /* 41 Speed */
              0x00, /* 42 Version Number */
              0x7D, /* 43 End sign */
              0x00 /* 44 Checksum */
            };
  
  #if defined(HOTTV4BATT)
    short voltage = hottv4UpdateBattery(telemetry_data);
	telemetry_data[20] = telemetry_data[22] = telemetry_data[30] = voltage;
	telemetry_data[21] = telemetry_data[23] = telemetry_data[31] = (voltage >> 8) & 0xFF;

	short current = hottv4UpdateCurrent();
	telemetry_data[28] = current;
	telemetry_data[29] = (current >> 8) & 0xFF;

	long capacity = hottv4UpdateCapacity();
	telemetry_data[32] = capacity;
	telemetry_data[33] = (capacity >> 8) & 0xFF;
  #endif
  
  #if defined(HOTTV4ALTITUDE)
    int32_t altitude = hottv4UpdateAlt();
	telemetry_data[26] = altitude;
	telemetry_data[27] = (altitude >> 8) & 0xFF;

	unsigned int varioSound = hottv4UpdateAltVario();
	telemetry_data[34] = varioSound;
    telemetry_data[35] = (varioSound >> 8) & 0xFF;
	telemetry_data[36] = 120;
  #endif

  hottv4UpdateFlightTime(telemetry_data);

  // Write out telemetry data as Electric Air Module to serial           
  hottV4SendBinary(telemetry_data);
}

/* ##################################################################### *
 *                HoTTv4 GPS Module                                      *
 * ##################################################################### */

#if defined(HOTTV4NAV)
/**
 * Converts unsigned long representation of GPS coordinate back to
 * N Deg MM.SSSS representation and puts it into GPS data frame.
 */
static void updatePosition(uint8_t *data, uint32_t value, uint8_t index) {
  data[index] = (value < 0);
  
  value /= 10;
  uint16_t first = value / 10000;
  uint16_t second = value % 10000;
  
  data[index+1] = first;
  data[index+2] = first >> 8; 
  data[index+3] = second; 
  data[index+4] = second >> 8;
} 
#endif

/**
 * Main method to send GPS telemetry data
 */
static void FillGPSTelemetryPackage() {
  uint8_t telemetry_data[] = { 
              0x7C, /* 0 */
              HOTTV4_GPS_MODULE, /* 1 */ 
              0x00, /* 2 Alarm */
              HOTTV4_GPS_SENSOR_ID, /* 3 */
              0x00, 0x00, /* 4-5 Alarm Value 1 and 2 */
              0x00, /* 6 Flight direction */ 
              0x00, 0x00, /* 7-8 Velocity in km/h*/ 
              0x00, 0x00, 0x00, 0x00, 0x00, /* 9-13 Latitude */
              0x00, 0x00, 0x00, 0x00, 0x00, /* 14-18 Longitude */
              0x00, 0x00, /* 19-20 Distance */
              0xF4, 0x01, /* 21-22 Altitude, 500 = 0m */
              0x78, 0x00, /* 23-24 m_s, 1 = 0.01m/s */ 
              0x78, /* 25 m_3s, 120 = 0 */
              0x00, /* 26 Number of satelites */ 
              0x00, /* 27 GPS fix character */
              0x00, /* 28 Home direction */
              0x00, /* 29 angle x-direction */
              0x00, /* 30 angle y-direction */
              0x00, /* 31 angle z-direction */
              0x00, 0x00,  /* 32-33 gyro x */
              0x00, 0x00, /* 34-35 gyro y */ 
              0x00, 0x00, /* 36-37 gyro z */
              0x00, /* 38 Vibrations */
              0x00, /* 39 ASCII Free Character 4 */
              0x00, /* 40 ASCII Free Character 5 */
              0x00, /* 41 ASCII Free Character 6 */
              0x00, /* 42 Version Number */
              0x7D, /* 43 End sign */
              0x00 /* 44 Checksum */
            };

#if defined(HOTTV4NAV)

  telemetry_data[26] = gpsData.sats;
	  
  if (gpsData.state > GPS_NOFIX)  {
      if(gpsData.state == GPS_FIX2D) telemetry_data[27] = telemetry_data[41] = HoTTGPS2DFix;
	  else if(gpsData.state == GPS_FIX3D) telemetry_data[27] = telemetry_data[41] = HoTTGPS3DFix;
	  else if(gpsData.state == GPS_FIX3DD) telemetry_data[27] = telemetry_data[41] = HoTTGPS3DDFix;

      updatePosition(telemetry_data, currentPosition.latitude, 9);
      updatePosition(telemetry_data, currentPosition.longitude, 14);

      /** GPS Speed in km/h */
	  uint8_t gpsSpeed = (int)getGpsSpeed()*36/1000;
      telemetry_data[7] = gpsSpeed;
	  telemetry_data[8] = (gpsSpeed >> 8) & 0xFF;

      /** Distance to home */
	  if(isHomeBaseInitialized()) {
		  computeDistanceAndBearing(currentPosition, homePosition);
		  telemetry_data[19] = (int)getDistanceMeter();
		  telemetry_data[20] = (int)getDistanceMeter() >> 8;
		  telemetry_data[28] = (gpsBearing - (int)(trueNorthHeading * RAD2DEG));
		  }

	  if(navigationState == ON) { 
		  telemetry_data[39] = HoTTGPSWaypoint;
		  }
	  else if(positionHoldState == ON) {
		  telemetry_data[39] = HoTTGPSPositionHold;
		  }
	  else {
		  telemetry_data[39] = HoTTGPSFree;
		  }
  }
  else if(gpsData.state == GPS_NOFIX)  {
	  telemetry_data[27] = telemetry_data[41] = HoTTGPSNoFix;
  }
  else  {
	  telemetry_data[27] = telemetry_data[41] = HoTTGPSDetecting;
  }
#endif
          
#if defined(HOTTV4ALTITUDE)
	int32_t altitude = hottv4UpdateAlt();
	telemetry_data[21] = altitude;
	telemetry_data[22] = (altitude >> 8) & 0xFF;

	unsigned int varioSound = hottv4UpdateAltVario();
	telemetry_data[23]  = varioSound;
    telemetry_data[24] = (varioSound >> 8) & 0xFF;
	telemetry_data[25] = 120;
#endif


#if defined(HOTTV4DIR) 
    telemetry_data[6] = hottV4UpdateDirection();
#endif

  // Triggers voice alarm if necessary
  telemetry_data[2] = hottVoiceOutput();
  
  // Write out telemetry data as GPS Module to serial           
  hottV4SendBinary(telemetry_data);
}

/* ##################################################################### *
 *                HoTTv4 Vario Module                                    *
 * ##################################################################### */

/**
 * Main method to send Vario telemetry data
 */
static void FillVarioTelemetryPackage() {
  uint8_t telemetry_data[] = { 
              0x7C, /* 0 */
              HOTTV4_VARIO_MODULE, /* 1 */ 
              0x00, /* 2 Alarm */
              HOTTV4_VARIO_SENSOR_ID, /* 3 */
              0x00, /* 4 Inverse status */
              0xF4, 0x01, /* 5-6 Current altitude 500 = 0m */ 
              0xF4, 0x01, /* 7-8 Max. altitude 500 = 0m */ 
              0xF4, 0x01, /* 9-10 Min. altitude 500 = 0m */
              0x30, 0x75, /* 11-12 m_s 30000 = 0 */
              0x30, 0x75, /* 13-14 m_3s  */
              0x30, 0x75, /* 15-16 m_10s */
              0x00, 0x00, 0x00, 0x00, /* 17-20 ASCII */
              0x00, 0x00, 0x00, 0x00, /* 21-24 ASCII */
              0x00, 0x00, 0x00, 0x00, /* 25-28 ASCII */
              0x00, 0x00, 0x00, 0x00, /* 29-32 ASCII */
              0x00, 0x00, 0x00, 0x00, /* 33-36 ASCII */
              0x00,                   /* 37 ASCII */
              0x00, 0x00, 0x00, 0x00, /* 38-41 free */
              0x00, /* 42 Version Number */
              0x7D, /* 43 End sign */
              0x00  /* 44 Checksum */
            };

#if defined(HOTTV4ALTITUDE)
  int32_t altitude = hottv4UpdateAlt();
  telemetry_data[5] = altitude;
  telemetry_data[6] = (altitude >> 8) & 0xFF;

  telemetry_data[7] = maxAltitude;
  telemetry_data[8] = (maxAltitude >> 8)  & 0xFF;
  
  telemetry_data[9] = minAltitude;
  telemetry_data[10] = (minAltitude >> 8) & 0xFF;

  unsigned int varioSound = hottv4UpdateAltVario();
	
  telemetry_data[11] = telemetry_data[13] = telemetry_data[15] = varioSound;
  telemetry_data[12] = telemetry_data[14] = telemetry_data[16] = (varioSound >> 8) & 0xFF;

  if(altitudeHoldState == VELOCITY_HOLD_STATE || altitudeHoldState == ALTITUDE_HOLD_STATE) {
	  if((receiverCommand[receiverChannelMap[THROTTLE]] > (altitudeHoldThrottle + altitudeHoldBump))) telemetry_data[38] = '+';
	  else if((receiverCommand[receiverChannelMap[THROTTLE]] < (altitudeHoldThrottle - altitudeHoldBump))) telemetry_data[38] = '-';
	  else telemetry_data[38] = '=';
  }
  else if (altitudeHoldState == ALTPANIC) {
	  telemetry_data[38] = '!';
  }
#endif

  // Buffer for the available 21 ASCII + \0 chars
  char text[VARIO_ASCIIS+1];
  
  if (isFailsafeActive) strcpy(text, HOTTV4_VARIO_FAILSAFE);
  else if (speakBatteryWarning) strcpy(text, HOTTV4_VARIO_LOWVOLTAGE);
  else {
    if (flightMode == ATTITUDE_FLIGHT_MODE) strcpy(text, HOTTV4_VARIO_ATTITUDE);
    else if (flightMode == HORIZON_FLIGHT_MODE) strcpy(text, HOTTV4_VARIO_HORIZON);
	else if (flightMode == RATE_FLIGHT_MODE) strcpy(text, HOTTV4_VARIO_RATE);
	
    if(altitudeHoldState == VELOCITY_HOLD_STATE) strcat(text, HOTTV4_VARIO_VELOCITY);
    else if (altitudeHoldState == ALTITUDE_HOLD_STATE) strcat(text, HOTTV4_VARIO_ALTITUDE);
	else if (altitudeHoldState == OFF || altitudeHoldState == ALTPANIC) strcat(text, HOTTV4_VARIO_OFF);
  }
 
  uint8_t offset = (VARIO_ASCIIS - strlen(text)) / 2;

  for(uint8_t index = 0; (index + offset) < VARIO_ASCIIS; index++) {
    if (text[index] != 0x0) {
      // 17 == start byte for ASCII
      telemetry_data[17+index+offset] = text[index];
    } else {
      break;
    }
  }  
            
  // Write out telemetry data as Vario Module to serial           
  hottV4SendBinary(telemetry_data);
}

/**
 * Main entry point for HoTTv4 telemetry
 */
bool hottV4Hook(uint8_t serialData) {
  switch (serialData) {
    case HOTTV4_GPS_MODULE:
    case '1':
      FillGPSTelemetryPackage();
      return true;
      break;
    
    case HOTTV4_ELECTRICAL_AIR_MODULE:
    case '2':
      FillEAMTelemetryPackage();
      return true;
      break;
         
    case HOTTV4_VARIO_MODULE:
    case '3':
      FillVarioTelemetryPackage();
      return true;
      break;
	  
	case HOTTV4_GENERAL_MODULE:
    case '4':
      FillGeneralTelemetryPackage();
      return true;
      break;
  }

  return false;
}

enum HottStateMachine {
	eHottReadCmd,
	eHottSendStartDelay,
	eHottSendByte,
	eHottCleanUp
} ;

HottStateMachine hottState = eHottReadCmd;
uint32 hottTime;
void hottHandler()
{
	switch(hottState) {
	case eHottReadCmd:
		{
		  byte cmd = hottV4SerialRead();
	      if(cmd != 0xff && hottV4Hook(cmd)) {
  	        hottTime = micros();
  	        hottState = eHottSendStartDelay;
	      }
		}
		break;

	case eHottSendStartDelay:
		if(micros() - hottTime > 2000) {
		    hottTime = micros();
		    hottState = eHottSendByte;
		}
		break;

	case eHottSendByte:
		if(micros() - hottTime > 1000) {
			hottV4SerialWrite(hottV4TelemetryBuffer[hottV4TelemetryBufferIndex++]);
			hottV4SerialClearInput();

			hottTime = micros();
		    if(hottV4TelemetryBufferIndex >= hottV4TelemetryBufferSize) {
		    	hottState = eHottCleanUp;
		    }
		}
		break;

	case eHottCleanUp:
		hottV4SerialClearInput();

		if(micros() - hottTime > 5000) {
		    hottTime = micros();
		    hottState = eHottReadCmd;
		}
		break;
	}
}
#endif 
