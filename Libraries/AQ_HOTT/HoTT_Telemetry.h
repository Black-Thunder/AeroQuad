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
 * If value is below batteryWarning, telemetry alarm is triggered
 */
static short hottv4UpdateBattery(uint8_t *data) {
	short voltage = batteryData[0].voltage/10;

	if (batteryWarning || batteryAlarm) {
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
 *
 * @param data Pointer to telemetry data frame
 * @param lowByteIndex Index for the low byte that represents the altitude in telemetry data frame
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
	unsigned int varioSound = 30000;

	if(altitudeHoldState == ON)
	{
		if((receiverCommand[THROTTLE] > (altitudeHoldThrottle + altitudeHoldBump))) varioSound = 30100;
		else if((receiverCommand[THROTTLE] < (altitudeHoldThrottle - altitudeHoldBump))) varioSound = 29900;
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

static unsigned char hottVoiceOutput()
	{
	unsigned char status = 0;
	static char oldStatus = 0;
	static int repeat;


#if defined(HOTTV4BATT)
	if (batteryAlarm) {
		status = HoTTv4NotificationUndervoltage;
	}
#endif

	if(!SpeakHoTT) {
#if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
		if(altitudeHoldState == ON)  isAHOn = true;
		else if(altitudeHoldState == ALTPANIC || altitudeHoldState == OFF) isAHOff = true;

		 if(isAHOn) {
			 if (altitudeHoldState == ALTPANIC || altitudeHoldState == OFF) {
				isAHOn = false;
				SpeakHoTT = HoTTv4NotificationAltitudeOff;
			 }
		 }
		 if(isAHOff) {
			 if(altitudeHoldState == ON) {
				 isAHOff = false;
				 SpeakHoTT = HoTTv4NotificationAltitudeOn;
			 }
		 }
#endif

#if defined(UseGPSNavigator)
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
    
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    /* Enable PullUps on RX3
     * without signal is to weak to be recognized
     */
    DDRJ &= ~(1 << 0);
    PORTJ |= (1 << 0);
  #endif
  hottV4Serial->begin(19200);
}

/* ##################################################################### *
 *                HoTTv4 EAM Module                                      *
 * ##################################################################### */

/**
 * Main method to send EAM telemetry data
 */
static void hottV4SendEAMTelemetry() {  
  uint8_t telemetry_data[] = { 
              0x7C,
              HOTTV4_ELECTRICAL_AIR_MODULE, 
              0x00, /* Alarm */
              HOTTV4_ELECTRICAL_AIR_SENSOR_ID,
              0x00, 0x00, /* Alarm Value 1 and 2 */
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Low Voltage Cell 1-7 in 2mV steps */
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* High Voltage Cell 1-7 in 2mV steps */
              0x00, 0x00, /* Battetry 1 LSB/MSB in 100mv steps, 50 == 5V */
              0x00, 0x00, /* Battetry 2 LSB/MSB in 100mv steps, 50 == 5V */
              0x14, /* Temp 1, Offset of 20. 20 == 0C */ 
              0x14, /* Temp 2, Offset of 20. 20 == 0C */
              0xF4, 0x01, /* Height. Offset -500. 500 == 0 */
              0x00, 0x00, /* Current LSB, MSB 1 = 0.1A */
              0x00, 0x00, /* Drive Voltage */
              0x00, 0x00,  /* mAh */
              0x48, 0x00, /* m2s */ 
              0x78, /* m3s */
              0x00, 0x00, /* RPM. 10er steps, 300 == 3000rpm */
              0x00, /* Electric minutes */
              0x00, /* Electric seconds */
              0x00, /* Speed */
              0x00, /* Version Number */
              0x7D, /* End sign */
              0x00 /* Checksum */
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
    telemetry_data[11] = telemetry_data[13] = varioSound;
    telemetry_data[12] = telemetry_data[14] = (varioSound >> 8) & 0xFF;
  #endif

  hottv4UpdateFlightTime(telemetry_data);

  // Write out telemetry data as Electric Air Module to serial           
  hottV4SendBinary(telemetry_data);
}

/* ##################################################################### *
 *                HoTTv4 GPS Module                                      *
 * ##################################################################### */

#if defined(UseGPS)
/**
 * Converts unsigned long representation of GPS coordinate back to
 * N Deg MM.SSSS representation and puts it into GPS data frame.
 */
static void updatePosition(uint8_t *data, uint32_t value, uint8_t index) {
  data[index] = (value < 0); 

  uint8_t deg = value / 100000;
  uint32_t sec = (value - (deg * 100000)) * 6;
  uint8_t min = sec / 10000;
  sec = sec % 10000;
  
  uint16_t degMin = (deg * 100) + min;

  data[index+1] = degMin;
  data[index+2] = degMin >> 8; 
  data[index+3] = sec; 
  data[index+4] = sec >> 8;
}
#endif

/**
 * Main method to send GPS telemetry data
 */
static void hottV4SendGPSTelemetry() {
  uint8_t telemetry_data[] = { 
              0x7C,
              HOTTV4_GPS_MODULE, 
              0x00, /* Alarm */
              HOTTV4_GPS_SENSOR_ID,
              0x00, 0x00, /* Alarm Value 1 and 2 */
              0x00, /* Flight direction */ 
              0x00, 0x00, /* Velocity */ 
              0x00, 0x00, 0x00, 0x00, 0x00, /* Latitude */
              0x00, 0x00, 0x00, 0x00, 0x00, /* Longitude */
              0x00, 0x00, /* Distance */
              0xF4, 0x01, /* Altitude, 500 = 0m */
              0x78, 0x00, /* m/s, 1 = 0.01m/s */ 
              0x78, /* m/3s, 120 = 0 */
              0x00, /* Number of satelites */ 
              0x00, /* GPS fix character */
              0x00, /* Home direction */
              0x00, /* angle x-direction */
              0x00, /* angle y-direction */
              0x00, /* angle z-direction */
              0x00, 0x00,  /* gyro x */
              0x00, 0x00, /* gyro y */ 
              0x00, 0x00, /* gyro z */
              0x00, /* Vibrations */
              0x00, /* ASCII Free Character 4 */
              0x00, /* ASCII Free Character 5 */
              0x00, /* ASCII Free Character 6 */
              0x00, /* Version Number */
              0x7D, /* End sign */
              0x00 /* Checksum */
            };



#if defined(UseGPS)

  telemetry_data[26] = nbSatelitesInUse;

    if (haveAGpsLock()) {
      updatePosition(telemetry_data, currentPosition.latitude, 9);
      updatePosition(telemetry_data, currentPosition.longitude, 14);

      telemetry_data[27] = telemetry_data[41] = 'f'; // Displays a 'f' for fix

      /** GPS Speed in km/h */
      telemetry_data[7] = getGpsSpeed()*36/1000;

      /** Distance to home */
	  if(isHomeBaseInitialized()) {
		  computeDistanceAndBearing(currentPosition, homePosition);
		  telemetry_data[19] = (int)getDistanceMeter();
		  telemetry_data[20] = (int)getDistanceMeter() >> 8;
		  telemetry_data[28] = (gpsBearing - (int)(trueNorthHeading * RAD2DEG)) * 50;
		  }

	  if (navigationState == ON) { 
		  telemetry_data[39] = HoTTGPSWaypoint; // Displays a 'W' for Waypoint
		  }
	  else if(positionHoldState == ON) {
		  telemetry_data[39] = HoTTGPSPositionHold; //Displays a 'P' for Position Hold
		  }
	  else {
		  telemetry_data[39] = HoTTGPSFree; //Displays a '/' for GPS Mode off 
		  }
    }
#endif
          
#if defined(HOTTV4ALTITUDE)
	int32_t altitude = hottv4UpdateAlt();
	telemetry_data[21] = altitude;
	telemetry_data[22] = (altitude >> 8) & 0xFF;

	unsigned int varioSound = hottv4UpdateAltVario();
	telemetry_data[11] = varioSound;
	telemetry_data[13] = 120;
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
static void hottV4SendVarioTelemetry() {
  uint8_t telemetry_data[] = { 
              0x7C,
              HOTTV4_VARIO_MODULE, 
              0x00, /* Alarm */
              HOTTV4_VARIO_SENSOR_ID,
              0x00, /* Inverse status */
              0xF4, 0x01, /* Current altitude */ 
              0xF4, 0x01, /* Max. altitude */ 
              0xF4, 0x01, /* Min. altitude */
              0x30, 0x75, /* m/s */
              0x30, 0x75, /* m/3s  */
              0x30, 0x75, /* m/10s */
              0x00, 0x00, 0x00, 0x00, /* ASCII */
              0x00, 0x00, 0x00, 0x00, /* ASCII */
              0x00, 0x00, 0x00, 0x00, /* ASCII */
              0x00, 0x00, 0x00, 0x00, /* ASCII */
              0x00, 0x00, 0x00, 0x00, /* ASCII */
              0x00,                   /* ASCII */
              0x00, 0x00, 0x00, 0x00, /* free */
              0x00, /* Version Number */
              0x7D, /* End sign */
              0x00  /* Checksum */
            };

#if defined(HOTTV4ALTITUDE)
  int32_t altitude = hottv4UpdateAlt();
  telemetry_data[5] = altitude;
  telemetry_data[6] = (altitude >> 8) & 0xFF;

  telemetry_data[7] = maxAltitude;
  telemetry_data[9] = minAltitude;

  unsigned int varioSound = hottv4UpdateAltVario();
  telemetry_data[11] = telemetry_data[13] = telemetry_data[15] = varioSound;
  telemetry_data[12] = telemetry_data[14] = telemetry_data[16] = (varioSound >> 8) & 0xFF;

  if(altitudeHoldState == ON) {
	  if((receiverCommand[THROTTLE] > (altitudeHoldThrottle + altitudeHoldBump))) telemetry_data[38] = '+';
	  else if((receiverCommand[THROTTLE] < (altitudeHoldThrottle - altitudeHoldBump))) telemetry_data[38] = '-';
	  else telemetry_data[38] = '=';
	  }
  else if (altitudeHoldState == ALTPANIC) {
	  telemetry_data[38] = '!';
	  }
#endif

  // Buffer for the available 21 ASCII + \0 chars
  char text[VARIO_ASCIIS+1];
  
  if(flightMode == ATTITUDE_FLIGHT_MODE) snprintf(text, VARIO_ASCIIS+1, HOTTV4_VARIO_ATTITUDE);
  else snprintf(text, VARIO_ASCIIS+1, HOTTV4_VARIO_RATE);

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

/* ##################################################################### *
 *                HoTTv4 Text Mode                                       *
 * ##################################################################### */

/**
 * Main entry point for HoTTv4 telemetry
 */
bool hottV4Hook(uint8_t serialData) {
  switch (serialData) {
    case HOTTV4_GPS_MODULE:
    case '1':
      hottV4SendGPSTelemetry();
      return true;
      break;
    
    case HOTTV4_ELECTRICAL_AIR_MODULE:
    case '2':
      hottV4SendEAMTelemetry();
      return true;
      break;
         
    case HOTTV4_VARIO_MODULE:
    case '3':
      hottV4SendVarioTelemetry();
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
