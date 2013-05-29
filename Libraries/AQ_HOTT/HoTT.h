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

#ifndef _AEROQUAD_HOTT_H_
#define _AEROQUAD_HOTT_H_

#define HOTTV4_UPDATE_INTERVAL 2000

#if defined(BattMonitor)
#define HOTTV4BATT
#endif

#if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
#define HOTTV4ALTITUDE
#endif

#if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
#define HOTTV4DIR
#endif

#if defined(UseGPS) && defined(UseGPSNavigator)
#define HOTTV4NAV
#endif

/** ###### HoTT module specific settings ###### */

#define HOTTV4_GENERAL_MODULE 0x8D
#define HOTTV4_GENERAL_SENSOR_ID 0xD0

#define HOTTV4_ELECTRICAL_AIR_SENSOR_ID 0xE0 // Electric Air Sensor ID
#define HOTTV4_ELECTRICAL_AIR_MODULE 0x8E // Electric Air Module ID
#define HOTTV4_ELECTRICAL_AIR_TEXTMODE 0x7F // Electrical Air Module Text Mode ID

#define HOTTV4_GPS_SENSOR_ID 0xA0 // GPS Sensor ID
#define HOTTV4_GPS_MODULE  0x8A  // GPS Module ID

#define HOTTV4_VARIO_SENSOR_ID 0x90 // Vario Sensor ID
#define HOTTV4_VARIO_MODULE 0x89 // Vario Sensor Module ID

#if !defined (HOTTV4_TX_DELAY) 
#define HOTTV4_TX_DELAY 600
#endif

/** ###### VARIO Text ###### */

#define HOTTV4_VARIO_LOWVOLTAGE "!! LOW VOLTAGE !!"
#define HOTTV4_VARIO_ATTITUDE "AeroQuad (Attitude)"
#define HOTTV4_VARIO_RATE     "AeroQuad (Rate)"

/** ###### Common settings ###### */

#define VARIO_ASCIIS 21

typedef enum {
	HoTTGPSWaypoint       = 'W',
	HoTTGPSComingHome     = 'H',
	HoTTGPSPositionHold   = 'P',
	HoTTGPSFree           = '/'
	} HoTTGPSChar;

typedef enum {
	HoTTGPSDetecting	= '?',
	HoTTGPSNoFix		= ' ',
	HoTTGPS2DFix		= '2',
	HoTTGPS3DFix		= '3',
	HoTTGPS3DDFix       = 'D'
} HoTTGPSFixChar;

#endif
