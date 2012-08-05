/*
  AeroQuad v3.x - July 2012
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
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

#ifndef _AQ_MAVLINK_H_
#define _AQ_MAVLINK_H_

#define MAV_COMPONENT_ID MAV_COMP_ID_IMU

// MavLink 1.0 DKP
#include "../mavlink/include/mavlink/v1.0/common/mavlink.h" 

#include "Aeroquad.h"

enum parameterTypeIndicator 
{
  P,
  I,
  D,
  windUpGuard,
  NONE
};

// Variables for sending parameters
int indexCounter = 0;
int paramListPartIndicator = -1;

// Variables for writing parameters
int parameterChangeIndicator = -1;
int parameterMatch = 0;
mavlink_param_set_t set;
char* key;


int systemType = MAV_TYPE_QUADROTOR;
int autopilotType = MAV_AUTOPILOT_GENERIC;
uint16_t len;
int systemMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
int systemStatus = MAV_STATE_UNINIT;

int parameterType = MAVLINK_TYPE_FLOAT;
int parameterListSize;
String parameterNameRateRollP = "Rate Roll_P";
String parameterNameRateRollI = "Rate Roll_I";
String parameterNameRateRollD = "Rate Roll_D";
String parameterNameRatePitchP = "Rate Pitch_P";
String parameterNameRatePitchI = "Rate Pitch_I";
String parameterNameRatePitchD = "Rate Pitch_D";
String parameterNameAttitudeRollP = "Att Roll_P";
String parameterNameAttitudeRollI = "Att Roll_I";
String parameterNameAttitudeRollD = "Att Roll_D";
String parameterNameAttitudePitchP = "Att Pitch_P";
String parameterNameAttitudePitchI = "Att Pitch_I";
String parameterNameAttitudePitchD = "Att Pitch_D";
String parameterNameAttitudeGyroRollP = "AttGyroRoll_P";
String parameterNameAttitudeGyroRollI = "AttGyroRoll_I";
String parameterNameAttitudeGyroRollD = "AttGyroRoll_D";
String parameterNameAttitudeGyroPitchP = "AttGyroPitc_P";
String parameterNameAttitudeGyroPitchI = "AttGyroPitc_I";
String parameterNameAttitudeGyroPitchD = "AttGyroPitc_D";
String parameterNameYawP = "Yaw_P";
String parameterNameYawI = "Yaw_I";
String parameterNameYawD = "Yaw_D";
String parameterNameHeadingP = "Heading_P";
String parameterNameHeadingI = "Heading_I";
String parameterNameHeadingD = "Heading_D";
String parameterNameHeadingConfig = "Heading_Conf";
String parameterNameGyroSmooth = "Misc_GyroSmoo";
String parameterNameAREF = "Misc_AREF";
String parameterNameMinThrottle = "Misc_MinThr";
String parameterNameTxFactor = "TX_TX Factor";
String parameterNameTxRollSmooth = "TX_RollSmooth";
String parameterNameTxPitchSmooth = "TX_PitcSmooth";
String parameterNameTxYawSmooth = "TX_YawSmooth";
String parameterNameTxThrottleSmooth = "TX_ThrSmooth";
String parameterNameTxModeSmooth = "TX_ModeSmooth";
String parameterNameTxAUX1Smooth = "TX_AUX1Smooth";
String parameterNameTxAUX2Smooth = "TX_AUX2Smooth";
String parameterNameTxAUX3Smooth = "TX_AUX3Smooth";
#if defined BattMonitor
  String parameterNameBattMonAlarmVoltage = "BatMo_AlarmVo";
  String parameterNameBattMonThrottleTarget = "BatMo_ThrTarg";
  String parameterNameBattMonGoingDownTime = "BatMo_DownTim";
#endif
#ifdef CameraControl
  String parameterNameCamMode = "Cam_Mode";
  String parameterNameCamPitchMiddle = "Cam_PitchMid";
  String parameterNameCamRollMiddle = "Cam_RollMid";
  String parameterNameCamYawMiddle = "Cam_YawMid";
  String parameterNameCamRollServoMiddle = "Cam_ServoPitM";
  String parameterNameCamPitchServoMiddle = "Cam_ServoRolM";
  String parameterNameCamYawServoMiddle = "Cam_ServoYawM";
  String parameterNameCamPitchServoMin = "Cam_SerMinPit";
  String parameterNameCamRollServoMin = "Cam_SerMinRol";
  String parameterNameCamYawServoMin = "Cam_SerMinYaw";
  String parameterNameCamPitchServoMax = "Cam_SerMaxPit";
  String parameterNameCamRollServoMax = "Cam_SerMaxRol";
  String parameterNameCamYawServoMax = "Cam_SerMaxYaw";
#endif
#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
  String parameterNameAHminThrottleAdjust = "AH_Min Adjust";
  String parameterNameAHmaxThrottleAdjust = "AH_Max Adjust";
  String parameterNameAHBumpValue = "AH_Bump Value";
  String parameterNameAHPanicValue = "AH_PanicValue";
#endif
#if defined AltitudeHoldBaro
  String parameterNameAHBaroSmooth = "AH_SmoothFact";
  String parameterNameBaroP = "Baro_P";
  String parameterNameBaroI = "Baro_I";
  String parameterNameBaroD = "Baro_D";
  String parameterNameBaroWindUpGuard = "Baro_WindUp";
  String parameterNameZDampeningP = "Z Dampening_P";
  String parameterNameZDampeningI = "Z Dampening_I";
  String parameterNameZDampeningD = "Z Dampening_D";
#endif
#if defined AltitudeHoldRangeFinder
  String parameterNameRangeFinderP = "Range_P";
  String parameterNameRangeFinderI = "Range_I";
  String parameterNameRangeFinderD = "Range_D";
  String parameterNameRangeFinderWindUpGuard = "Range_WindUp";
#endif
#if defined UseGPSNavigator
  String parameterNameGPSRollP = "GPS Roll_P";
  String parameterNameGPSRollI = "GPS Roll_I";
  String parameterNameGPSRollD = "GPS Roll_D";
  String parameterNameGPSPitchP = "GPS Pitch_P";
  String parameterNameGPSPitchI = "GPS Pitch_I";
  String parameterNameGPSPitchD = "GPS Pitch_D";
  String parameterNameGPSYawP = "GPS Yaw_P";
  String parameterNameGPSYawI = "GPS Yaw_I";
  String parameterNameGPSYawD = "GPS Yaw_D";
#endif

parameterTypeIndicator paramIndicator = NONE;
float *parameterToBeChangedFloat;
byte *parameterToBeChangedByte;
int *parameterToBeChangedInt;
unsigned long *parameterToBeChangedULong;

static uint16_t millisecondsSinceBoot = 0;
long system_dropped_packets = 0;

mavlink_message_t msg; 
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_status_t status;


void evaluateParameterListSize() {
	parameterListSize = 35;

  #if defined AltitudeHoldBaro && defined AltitudeHoldRangeFinder && defined UseGPSNavigator
    parameterListSize += 25;
  #endif
  
  #if defined AltitudeHoldBaro && defined AltitudeHoldRangeFinder && !defined UseGPSNavigator
    parameterListSize += 16;
  #endif
  
  #if defined AltitudeHoldBaro && !defined AltitudeHoldRangeFinder && !defined UseGPSNavigator
    parameterListSize += 12;
  #endif
  
  #if !defined AltitudeHoldBaro && defined AltitudeHoldRangeFinder && !defined UseGPSNavigator
    parameterListSize += 8;
  #endif
  
  #if defined AltitudeHoldBaro && !defined AltitudeHoldRangeFinder && defined UseGPSNavigator
    parameterListSize += 21;
  #endif
  
  #if defined BattMonitor
    parameterListSize += 3;
  #endif
  
  #ifdef CameraControl
    parameterListSize += 13;
  #endif

  if (LASTCHANNEL == 8) {
    parameterListSize += 2;
  }
}


void initCommunication() {
  evaluateParameterListSize();
}

uint32_t previousFlightTimeUpdate = 0;
void updateFlightTime() {

  uint16_t timeDiff = millis() - previousFlightTimeUpdate;
  previousFlightTimeUpdate += timeDiff;

  if (motorArmed) {
    millisecondsSinceBoot += timeDiff;
  }
}

void sendSerialHeartbeat() {
  systemMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

  if (flightMode == ATTITUDE_FLIGHT_MODE) {
    systemMode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
  }

  #ifdef UseGPSNavigator
    if (navigationState == ON || positionHoldState == ON) {
      systemMode |= MAV_MODE_FLAG_GUIDED_ENABLED;
    }
  #endif

  systemMode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

  if (motorArmed) {
    systemMode |= MAV_MODE_FLAG_SAFETY_ARMED;
    systemStatus = MAV_STATE_ACTIVE;
  }
  else {
    systemStatus = MAV_STATE_STANDBY;
  }

  mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, systemType, autopilotType, systemMode, 0, systemStatus);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}


void sendSerialRawIMU() {
  #ifdef HeadingMagHold
    mavlink_msg_raw_imu_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, meterPerSecSec[XAXIS], meterPerSecSec[YAXIS], meterPerSecSec[ZAXIS], gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], getMagnetometerRawData(XAXIS), getMagnetometerRawData(YAXIS), getMagnetometerRawData(ZAXIS));
  #else
    mavlink_msg_raw_imu_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0, meterPerSecSec[XAXIS], meterPerSecSec[YAXIS], meterPerSecSec[ZAXIS], gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], 0, 0, 0);
  #endif
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}


void sendSerialAttitude() {
  mavlink_msg_attitude_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], kinematicsAngle[ZAXIS], 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}

void sendSerialHudData() {
  #if defined HeadingMagHold
    #if defined AltitudeHoldBaro
      mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverData[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
    #else
      mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360, (receiverData[THROTTLE]-1000)/10, 0, 0.0);
    #endif
  #else
    #if defined AltitudeHoldBaro
      mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, 0, (receiverData[THROTTLE]-1000)/10, getBaroAltitude(), 0.0);
    #else
      mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, 0.0, 0.0, 0, (receiverData[THROTTLE]-1000)/10, 0, 0.0);
    #endif
  #endif
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);   
}

void sendSerialGpsPostion() {
  #ifdef UseGPS
    if (haveAGpsLock())
    {
      #if defined AltitudeHoldBaro
        mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, currentPosition.latitude, currentPosition.longitude, getGpsAltitude() * 10, (getGpsAltitude() - baroGroundAltitude * 100) * 10 , 0, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
      #else
        mavlink_msg_global_position_int_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, currentPosition.latitude, currentPosition.longitude, getGpsAltitude() * 10, getGpsAltitude() * 10 , 0, 0, 0, ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360);
      #endif
      len = mavlink_msg_to_send_buffer(buf, &msg);
      SERIAL_PORT.write(buf, len);
    }
  #endif
}

void sendSerialRawPressure() {
  #ifdef AltitudeHoldBaro
    mavlink_msg_raw_pressure_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, readRawPressure(), 0,0, readRawTemperature());
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_PORT.write(buf, len);
  #endif
}

void sendSerialRcRaw() {
  #if defined UseRSSIFaileSafe
    mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, 0, receiverCommand[THROTTLE], receiverCommand[XAXIS], receiverCommand[YAXIS], receiverCommand[ZAXIS], receiverCommand[MODE], receiverCommand[AUX1], receiverCommand[AUX2], receiverCommand[AUX3], rssiRawValue * 2.55);
  #else 
    mavlink_msg_rc_channels_raw_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millisecondsSinceBoot, 0, receiverCommand[THROTTLE], receiverCommand[XAXIS], receiverCommand[YAXIS], receiverCommand[ZAXIS], receiverCommand[MODE], receiverCommand[AUX1], receiverCommand[AUX2], receiverCommand[AUX3], 0);
  #endif
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}

void sendSerialSysStatus() {
  uint32_t controlSensorsPresent = 0;
  uint32_t controlSensorEnabled;
  uint32_t controlSensorsHealthy;

  // first what sensors/controllers we have
  if (GYRO_DETECTED) {
    controlSensorsPresent |= (1<<0); // 3D gyro present
  }
  if (ACCEL_DETECTED) {
    controlSensorsPresent |= (1<<1); // 3D accelerometer present
  }
  #if defined HeadingMagHold
    if (MAG_DETECTED) {
      controlSensorsPresent |= (1<<2); // compass present
    }
  #endif
  
  #if defined AltitudeHoldBaro
    if (BARO_DETECTED) {
      controlSensorsPresent |= (1<<3); // absolute pressure sensor present
    }
  #endif
  #if defined UseGPS
    if (gps->valid_read) {
      controlSensorsPresent |= (1<<5); // GPS present
    }
  #endif
  controlSensorsPresent |= (1<<10); // 3D angular rate control
  controlSensorsPresent |= (1<<11); // attitude stabilisation
  controlSensorsPresent |= (1<<12); // yaw position
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    controlSensorsPresent |= (1<<13); // altitude control
  #endif
  #if defined UseGPSNavigator
    controlSensorsPresent |= (1<<14); // X/Y position control
  #endif
  controlSensorsPresent |= (1<<15); // motor control

  // now what sensors/controllers are enabled
  // first the sensors
  controlSensorEnabled = controlSensorsPresent & 0x1FF;

  // now the controllers
  controlSensorEnabled = controlSensorsPresent & 0x1FF;

  controlSensorEnabled |= (1<<10); // 3D angular rate control
  if (flightMode == ATTITUDE_FLIGHT_MODE) {
    controlSensorEnabled |= (1<<11); // attitude stabilisation
  }
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    if (altitudeHoldState == ON) {
      controlSensorEnabled |= (1<<13); // altitude control
    }
  #endif
  controlSensorEnabled |= (1<<15); // motor control
  if (headingHoldConfig == ON) {
    controlSensorEnabled |= (1<<12); // yaw position
  }
  #if defined UseGPSNavigator
    if (positionHoldState == ON || navigationState == ON) {
      controlSensorEnabled |= (1<<14); // X/Y position control
    }
  #endif

  // at the moment all sensors/controllers are assumed healthy
  controlSensorsHealthy = controlSensorsPresent;

  #if defined BattMonitor
    mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, controlSensorsPresent, controlSensorEnabled, controlSensorsHealthy, 0, batteryData[0].voltage * 10, (int)(batteryData[0].current*1000), -1, system_dropped_packets, 0, 0, 0, 0, 0);
  #else
    mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, controlSensorsPresent, controlSensorEnabled, controlSensorsHealthy, 0, 0, 0, 0, system_dropped_packets, 0, 0, 0, 0, 0);  // system_dropped_packets
  #endif

  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}





void sendSerialPID(int IDPid, int8_t id_p[], int8_t id_i[], int8_t id_d[], int8_t id_windUp[], int listsize, int index) {

  int counter = 0;
  if (id_p != 0) {	
    mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)id_p, PID[IDPid].P, parameterType, listsize, index);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_PORT.write(buf, len);
    counter++;
  }

  if (id_i != 0) {
    mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)id_i, PID[IDPid].I, parameterType, listsize, index + counter);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_PORT.write(buf, len);
    counter++;
  }

  if (id_d != 0) {
    mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)id_d, PID[IDPid].D, parameterType, listsize, index + counter);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_PORT.write(buf, len);
    counter++;
  }

  if (id_windUp != 0) {
    mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)id_windUp, PID[IDPid].windupGuard, parameterType, listsize, index + counter);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SERIAL_PORT.write(buf, len);
  }
}

void sendSerialParameter(float parameterID, int8_t parameterName[], int listsize, int index) {
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterName, parameterID, parameterType, listsize, index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}	

void sendSerialParameter(int parameterID, int8_t parameterName[], int listsize, int index) {
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterName, parameterID, parameterType, listsize, index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}	

void sendSerialParameter(byte parameterID, int8_t parameterName[], int listsize, int index) {
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterName, parameterID, parameterType, listsize, index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}	

void sendSerialParameter(unsigned long parameterID, int8_t parameterName[], int listsize, int index) {
  mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, (char*)parameterName, parameterID, parameterType, listsize, index);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_PORT.write(buf, len);
}	

void sendParameterListPart1() {
  int8_t rateRoll_P[14] = "Rate Roll_P";
  int8_t rateRoll_I[14] = "Rate Roll_I";
  int8_t rateRoll_D[14] = "Rate Roll_D";
  sendSerialPID(RATE_XAXIS_PID_IDX, rateRoll_P, rateRoll_I, rateRoll_D, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  int8_t ratePitch_P[14] = "Rate Pitch_P";
  int8_t ratePitch_I[14] = "Rate Pitch_I";
  int8_t ratePitch_D[14] = "Rate Pitch_D";
  sendSerialPID(RATE_YAXIS_PID_IDX, ratePitch_P, ratePitch_I, ratePitch_D, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  int8_t attitudeRoll_P[14] = "Att Roll_P";
  int8_t attitudeRoll_I[14] = "Att Roll_I";
  int8_t attitudeRoll_D[14] = "Att Roll_D";
  sendSerialPID(ATTITUDE_XAXIS_PID_IDX, attitudeRoll_P, attitudeRoll_I, attitudeRoll_D, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  int8_t attitudePitch_P[14] = "Att Pitch_P";
  int8_t attitudePitch_i[14] = "Att Pitch_I";
  int8_t attitudePitch_d[14] = "Att Pitch_D";
  sendSerialPID(ATTITUDE_YAXIS_PID_IDX, attitudePitch_P, attitudePitch_i, attitudePitch_d, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  int8_t attitudeGyroRoll_P[14] = "AttGyroRoll_P";
  int8_t attitudeGyroRoll_I[14] = "AttGyroRoll_I";
  int8_t attitudeGyroRoll_D[14] = "AttGyroRoll_D";
  sendSerialPID(ATTITUDE_GYRO_XAXIS_PID_IDX, attitudeGyroRoll_P, attitudeGyroRoll_I, attitudeGyroRoll_D, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  int8_t attitudeGyroPitch_P[14] = "AttGyroPitc_P";
  int8_t attitudeGyroPitch_I[14] = "AttGyroPitc_I";
  int8_t attitudeGyroPitch_D[14] = "AttGyroPitc_D";
  sendSerialPID(ATTITUDE_GYRO_YAXIS_PID_IDX, attitudeGyroPitch_P, attitudeGyroPitch_I, attitudeGyroPitch_D, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  int8_t yaw_p[14] = "Yaw_P";
  int8_t yaw_i[14] = "Yaw_I";
  int8_t yaw_d[14] = "Yaw_D";
  sendSerialPID(ZAXIS_PID_IDX, yaw_p, yaw_i, yaw_d, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  int8_t heading_p[14] = "Heading_P";
  int8_t heading_i[14] = "Heading_I";
  int8_t heading_d[14] = "Heading_D";
  sendSerialPID(HEADING_HOLD_PID_IDX, heading_p, heading_i, heading_d, 0, parameterListSize, indexCounter);
  indexCounter += 3;

  int8_t heading_config[14] = "Heading_Conf";
  sendSerialParameter(headingHoldConfig, heading_config, parameterListSize, indexCounter);
  indexCounter++;

  int8_t gyro_smooth_factor[14] = "Misc_GyroSmoo";
  sendSerialParameter(gyroSmoothFactor, gyro_smooth_factor, parameterListSize, indexCounter);
  indexCounter++;

  int8_t a_ref[14] = "Misc_AREF";
  sendSerialParameter(aref, a_ref, parameterListSize, indexCounter);
  indexCounter++;
}

void sendParameterListPart2() {
  int8_t min_armed_throttle[14] = "Misc_MinThr";
  sendSerialParameter(minArmedThrottle, min_armed_throttle, parameterListSize, indexCounter);
  indexCounter++;

  int8_t receiver_xmit_factor[14] = "TX_TX Factor";
  sendSerialParameter(receiverXmitFactor, receiver_xmit_factor, parameterListSize, indexCounter);
  indexCounter++;

  int8_t receiver_smooth_factor_roll[14] = "TX_RollSmooth";
  sendSerialParameter(receiverSmoothFactor[XAXIS], receiver_smooth_factor_roll, parameterListSize, indexCounter);
  indexCounter++;

  int8_t receiver_smooth_factor_pitch[14] = "TX_PitcSmooth";
  sendSerialParameter(receiverSmoothFactor[YAXIS], receiver_smooth_factor_pitch, parameterListSize, indexCounter);
  indexCounter++;

  int8_t receiver_smooth_factor_yaw[14] = "TX_YawSmooth";
  sendSerialParameter(receiverSmoothFactor[ZAXIS], receiver_smooth_factor_yaw, parameterListSize, indexCounter);
  indexCounter++;

  int8_t receiver_smooth_factor_throttle[14] = "TX_ThrSmooth";
  sendSerialParameter(receiverSmoothFactor[THROTTLE], receiver_smooth_factor_throttle, parameterListSize, indexCounter);
  indexCounter++;

  int8_t receiver_smooth_factor_mode[14] = "TX_ModeSmooth";
  sendSerialParameter(receiverSmoothFactor[MODE], receiver_smooth_factor_mode, parameterListSize, indexCounter);
  indexCounter++;

  int8_t receiver_smooth_factor_aux1[14] = "TX_AUX1Smooth";
  sendSerialParameter(receiverSmoothFactor[AUX1], receiver_smooth_factor_aux1, parameterListSize, indexCounter);
  indexCounter++;

  if (LASTCHANNEL == 8) {
    int8_t receiver_smooth_factor_aux2[14] = "TX_AUX2Smooth";
    sendSerialParameter(receiverSmoothFactor[AUX2], receiver_smooth_factor_aux2, parameterListSize, indexCounter);
    indexCounter++;

    int8_t receiver_smooth_factor_aux3[14] = "TX_AUX3Smooth";
    sendSerialParameter(receiverSmoothFactor[AUX3], receiver_smooth_factor_aux3, parameterListSize, indexCounter);
    indexCounter++;
  }

  #if defined BattMonitor
    int8_t battery_monitor_alarm_voltage[14] = "BatMo_AlarmVo";
    sendSerialParameter(batteryMonitorAlarmVoltage, battery_monitor_alarm_voltage, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t battery_monitor_throttle_target[14] = "BatMo_ThrTarg";
    sendSerialParameter(batteryMonitorThrottleTarget, battery_monitor_throttle_target, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t battery_monitor_going_down_time[14] = "BatMo_DownTim";
    sendSerialParameter(batteryMonitorGoingDownTime, battery_monitor_going_down_time, parameterListSize, indexCounter);
    indexCounter++;
  #endif
}
void sendParameterListPart3() {
  #if defined CameraControl
    int8_t camera_mode[14] = "Cam_Mode";
    sendSerialParameter(cameraMode, camera_mode, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t m_camera_pitch[14] = "Cam_PitchMid";
    sendSerialParameter(mCameraPitch, m_camera_pitch, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t m_camera_roll[14] = "Cam_RollMid";
    sendSerialParameter(mCameraRoll, m_camera_roll, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t m_camera_yaw[14] = "Cam_YawMid";
    sendSerialParameter(mCameraYaw, m_camera_yaw, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t m_servo_pitch[14] = "Cam_ServoPitM";
    sendSerialParameter(servoCenterPitch, m_servo_pitch, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t m_servo_roll[14] = "Cam_ServoRolM";
    sendSerialParameter(servoCenterRoll, m_servo_roll, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t m_servo_yaw[14] = "Cam_ServoYawM";
    sendSerialParameter(servoCenterYaw, m_servo_yaw, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t servo_min_pitch[14] = "Cam_SerMinPit";
    sendSerialParameter(servoMinPitch, servo_min_pitch, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t servo_min_roll[14] = "Cam_SerMinRol";
    sendSerialParameter(servoMinRoll, servo_min_roll, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t servo_min_yaw[14] = "Cam_SerMinYaw";
    sendSerialParameter(servoMinYaw, servo_min_yaw, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t servo_max_pitch[14] = "Cam_SerMaxPit";
    sendSerialParameter(servoMaxPitch, servo_max_pitch, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t servo_max_roll[14] = "Cam_SerMaxRol";
    sendSerialParameter(servoMaxRoll, servo_max_roll, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t servo_max_yaw[14] = "Cam_SerMaxYaw";
    sendSerialParameter(servoMaxYaw, servo_max_yaw, parameterListSize, indexCounter);
    indexCounter++;
  #endif

  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    int8_t min_throttle_adjust[14] = "AH_Min Adjust";
    sendSerialParameter(minThrottleAdjust, min_throttle_adjust, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t max_throttle_adjust[14] = "AH_Max Adjust";
    sendSerialParameter(maxThrottleAdjust, max_throttle_adjust, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t altitude_hold_bump[14] = "AH_Bump Value";
    sendSerialParameter(altitudeHoldBump, altitude_hold_bump, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t altitude_hold_panic_stick_movement[14] = "AH_PanicValue";
    sendSerialParameter(altitudeHoldPanicStickMovement, altitude_hold_panic_stick_movement, parameterListSize, indexCounter);
    indexCounter++;
  #endif 
}

void sendParameterListPart4() {
  #if defined AltitudeHoldBaro  && !defined AltitudeHoldRangeFinder
    int8_t baro_smooth_factor[14] = "AH_SmoothFact";
    sendSerialParameter(baroSmoothFactor, baro_smooth_factor, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t baro_p[14] = "Baro_P";
    int8_t baro_i[14] = "Baro_I";
    int8_t baro_d[14] = "Baro_D";
    sendSerialPID(BARO_ALTITUDE_HOLD_PID_IDX, baro_p, baro_i, baro_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  
    int8_t baro_windUpGuard[14] = "Baro_WindUp";
    sendSerialPID(BARO_ALTITUDE_HOLD_PID_IDX, 0, 0, 0, baro_windUpGuard, parameterListSize, indexCounter);
    indexCounter ++;
  
    int8_t zDampening_p[14] = "Z Dampening_P";
    int8_t zDampening_i[14] = "Z Dampening_I";
    int8_t zDampening_d[14] = "Z Dampening_D";
    sendSerialPID(ZDAMPENING_PID_IDX, zDampening_p, zDampening_i, zDampening_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  #endif

  #if defined AltitudeHoldRangeFinder && defined AltitudeHoldBaro
    int8_t baro_smooth_factor[14] = "AH_SmoothFact";
    sendSerialParameter(baroSmoothFactor, baro_smooth_factor, parameterListSize, indexCounter);
    indexCounter++;
  
    int8_t baro_p[14] = "Baro_P";
    int8_t baro_i[14] = "Baro_I";
    int8_t baro_d[14] = "Baro_D";
    sendSerialPID(BARO_ALTITUDE_HOLD_PID_IDX, baro_p, baro_i, baro_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  
    int8_t baro_windUpGuard[14] = "Baro_WindUp";
    sendSerialPID(BARO_ALTITUDE_HOLD_PID_IDX, 0, 0, 0, baro_windUpGuard, parameterListSize, indexCounter);
    indexCounter ++;
  
    int8_t zDampening_p[14] = "Z Dampening_P";
    int8_t zDampening_i[14] = "Z Dampening_I";
    int8_t zDampening_d[14] = "Z Dampening_D";
    sendSerialPID(ZDAMPENING_PID_IDX, zDampening_p, zDampening_i, zDampening_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  
    int8_t range_p[14] = "Range_P";
    int8_t range_i[14] = "Range_I";
    int8_t range_d[14] = "Range_D";
    sendSerialPID(SONAR_ALTITUDE_HOLD_PID_IDX, range_p, range_i, range_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  
    int8_t range_windUpGuard[14] = "Range_WindUp";
    sendSerialPID(SONAR_ALTITUDE_HOLD_PID_IDX, 0, 0, 0, range_windUpGuard, parameterListSize, indexCounter);
    indexCounter ++;
  #endif
}

void sendParameterListPart5() {
  #if defined UseGPSNavigator && defined AltitudeHoldRangeFinder && defined AltitudeHoldBaro
    int8_t gps_roll_p[14] = "GPS Roll_P";
    int8_t gps_roll_i[14] = "GPS Roll_I";
    int8_t gps_roll_d[14] = "GPS Roll_D";
    sendSerialPID(GPSROLL_PID_IDX, gps_roll_p, gps_roll_i, gps_roll_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  
    int8_t gps_pitch_p[14] = "GPS Pitch_P";
    int8_t gps_pitch_i[14] = "GPS Pitch_I";
    int8_t gps_pitch_d[14] = "GPS Pitch_D";
    sendSerialPID(GPSPITCH_PID_IDX, gps_pitch_p, gps_pitch_i, gps_pitch_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  
    int8_t gps_yaw_p[14] = "GPS Yaw_P";
    int8_t gps_yaw_i[14] = "GPS Yaw_I";
    int8_t gps_yaw_d[14] = "GPS Yaw_D";
    sendSerialPID(GPSYAW_PID_IDX, gps_yaw_p, gps_yaw_i, gps_yaw_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  #endif

  #if defined UseGPSNavigator && !defined AltitudeHoldRangeFinder && defined AltitudeHoldBaro
    int8_t gps_roll_p[14] = "GPS Roll_P";
    int8_t gps_roll_i[14] = "GPS Roll_I";
    int8_t gps_roll_d[14] = "GPS Roll_D";
    sendSerialPID(GPSROLL_PID_IDX, gps_roll_p, gps_roll_i, gps_roll_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  
    int8_t gps_pitch_p[14] = "GPS Pitch_P";
    int8_t gps_pitch_i[14] = "GPS Pitch_I";
    int8_t gps_pitch_d[14] = "GPS Pitch_D";
    sendSerialPID(GPSPITCH_PID_IDX, gps_pitch_p, gps_pitch_i, gps_pitch_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  
    int8_t gps_yaw_p[14] = "GPS Yaw_P";
    int8_t gps_yaw_i[14] = "GPS Yaw_I";
    int8_t gps_yaw_d[14] = "GPS Yaw_D";
    sendSerialPID(GPSYAW_PID_IDX, gps_yaw_p, gps_yaw_i, gps_yaw_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  #endif

  #if defined AltitudeHoldRangeFinder && !defined AltitudeHoldBaro
    int8_t range_p[14] = "Range_P";
    int8_t range_i[14] = "Range_I";
    int8_t range_d[14] = "Range_D";
    sendSerialPID(SONAR_ALTITUDE_HOLD_PID_IDX, range_p, range_i, range_d, 0, parameterListSize, indexCounter);
    indexCounter += 3;
  
    int8_t range_windUpGuard[14] = "Range_WindUp";
    sendSerialPID(SONAR_ALTITUDE_HOLD_PID_IDX, 0, 0, 0, range_windUpGuard, parameterListSize, indexCounter);
    indexCounter ++;
  #endif 
}




bool checkParameterMatch(String parameterName, char* key) {
  
  for (uint16_t j = 0; j < parameterName.length(); j++) {
    if (((char) (parameterName[j])) != (char) (key[j]))	{
      return false;
    }
  }
  return true;
}

int findParameter(char* key) {
  paramIndicator = NONE;
  parameterToBeChangedFloat = NULL;
  parameterToBeChangedByte = NULL;
  parameterToBeChangedInt = NULL;
  parameterToBeChangedULong = NULL;

  if (checkParameterMatch(parameterNameRateRollP, key)) {
    paramIndicator = P;
    return RATE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameRateRollI, key)) {
    paramIndicator = I;
    return RATE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameRateRollD, key)) {
    paramIndicator = D;
    return RATE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameRatePitchP, key)) {
    paramIndicator = P;
    return RATE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameRatePitchI, key)) {
    paramIndicator = I;
    return RATE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameRatePitchP, key)) {
    paramIndicator = D;
    return RATE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeRollP, key)) {
    paramIndicator = P;
    return ATTITUDE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeRollI, key)) {
    paramIndicator = I;
    return ATTITUDE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeRollD, key)) {
    paramIndicator = D;
    return ATTITUDE_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudePitchP, key)) {
    paramIndicator = P;
    return ATTITUDE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudePitchI, key)) {
    paramIndicator = I;
    return ATTITUDE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudePitchD, key)) {
    paramIndicator = D;
    return ATTITUDE_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroRollP, key)) {
    paramIndicator = P;
    return ATTITUDE_GYRO_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroRollI, key)) {
    paramIndicator = I;
    return ATTITUDE_GYRO_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroRollD, key)) {
    paramIndicator = D;
    return ATTITUDE_GYRO_XAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroPitchP, key)) {
    paramIndicator = P;
    return ATTITUDE_GYRO_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroPitchI, key)) {
    paramIndicator = I;
    return ATTITUDE_GYRO_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameAttitudeGyroPitchD, key)) {
    paramIndicator = D;
    return ATTITUDE_GYRO_YAXIS_PID_IDX;
  }
  if (checkParameterMatch(parameterNameHeadingP, key)) {
    paramIndicator = P;
    return HEADING_HOLD_PID_IDX;
  }
  if (checkParameterMatch(parameterNameHeadingI, key)) {
    paramIndicator = I;
    return HEADING_HOLD_PID_IDX;
  }
  if (checkParameterMatch(parameterNameHeadingD, key)) {
    paramIndicator = D;
    return HEADING_HOLD_PID_IDX;
  }
  if (checkParameterMatch(parameterNameHeadingConfig, key)) {
    paramIndicator = NONE;
    parameterToBeChangedByte = &headingHoldConfig;
    return -1;
  }
  if (checkParameterMatch(parameterNameGyroSmooth, key)) {
    paramIndicator = NONE;
    parameterToBeChangedFloat = &gyroSmoothFactor;
    return -1;
  }
  if (checkParameterMatch(parameterNameGyroSmooth, key)) {
    paramIndicator = NONE;
    parameterToBeChangedFloat = &gyroSmoothFactor;
    return -1;
  }
  if (checkParameterMatch(parameterNameAREF, key)) {
    paramIndicator = NONE;
    parameterToBeChangedFloat = &aref;
    return -1;
  }
  if (checkParameterMatch(parameterNameMinThrottle, key)) {
    paramIndicator = NONE;
    parameterToBeChangedInt = &minArmedThrottle;
    return -1;
  }
  if (checkParameterMatch(parameterNameTxFactor, key)) {
    paramIndicator = NONE;
    parameterToBeChangedFloat = &receiverXmitFactor;
    return -1;
  }
  if (checkParameterMatch(parameterNameTxRollSmooth, key)) {
    paramIndicator = NONE;
    parameterToBeChangedFloat = &receiverSmoothFactor[XAXIS];
    return -1;
  }
  if (checkParameterMatch(parameterNameTxPitchSmooth, key)) {
    paramIndicator = NONE;
    parameterToBeChangedFloat = &receiverSmoothFactor[YAXIS];
    return -1;
  }
  if (checkParameterMatch(parameterNameTxYawSmooth, key)) {
    paramIndicator = NONE;
    parameterToBeChangedFloat = &receiverSmoothFactor[ZAXIS];
    return -1;
  }
  if (checkParameterMatch(parameterNameTxThrottleSmooth, key)) {
    paramIndicator = NONE;
    parameterToBeChangedFloat = &receiverSmoothFactor[THROTTLE];
    return -1;
  }
  if (checkParameterMatch(parameterNameTxModeSmooth, key)) {
    paramIndicator = NONE;
    parameterToBeChangedFloat = &receiverSmoothFactor[MODE];
    return -1;
  }
  if (checkParameterMatch(parameterNameTxAUX1Smooth, key)) {
    paramIndicator = NONE;
    parameterToBeChangedFloat = &receiverSmoothFactor[AUX1];
    return -1;
  }
  if (LASTCHANNEL == 8) {
    if (checkParameterMatch(parameterNameTxAUX2Smooth, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &receiverSmoothFactor[AUX2];
      return -1;
    }
    if (checkParameterMatch(parameterNameTxAUX3Smooth, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &receiverSmoothFactor[AUX3];
      return -1;
    }
  }

  #if defined BattMonitor
    if (checkParameterMatch(parameterNameBattMonAlarmVoltage, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &batteryMonitorAlarmVoltage;
      return -1;
    }
    if (checkParameterMatch(parameterNameBattMonThrottleTarget, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &batteryMonitorThrottleTarget;
      return -1;
    }
    if (checkParameterMatch(parameterNameBattMonGoingDownTime, key)) {
      paramIndicator = NONE;
      parameterToBeChangedULong = &batteryMonitorGoingDownTime;
      return -1;
    }
  #endif

  #ifdef CameraControl
    if (checkParameterMatch(parameterNameCamMode, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &cameraMode;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamPitchMiddle, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &mCameraPitch;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamRollMiddle, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &mCameraRoll;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamYawMiddle, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &mCameraYaw;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamPitchServoMin, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMinPitch;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamRollServoMin, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMinRoll;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamYawServoMin, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMinYaw;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamPitchServoMax, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMaxPitch;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamRollServoMax, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMaxRoll;
      return -1;
    }
    if (checkParameterMatch(parameterNameCamYawServoMax, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &servoMaxYaw;
      return -1;
    }
  #endif

  #if defined (AltitudeHoldBaro) || defined AltitudeHoldRangeFinder
    if (checkParameterMatch(parameterNameAHminThrottleAdjust, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &minThrottleAdjust;
      return -1;
    }
    if (checkParameterMatch(parameterNameAHmaxThrottleAdjust, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &maxThrottleAdjust;
      return -1;
    }
    if (checkParameterMatch(parameterNameAHBumpValue, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &altitudeHoldBump;
      return -1;
    }
    if (checkParameterMatch(parameterNameAHPanicValue, key)) {
      paramIndicator = NONE;
      parameterToBeChangedInt = &altitudeHoldPanicStickMovement;
      return -1;
    }
  #endif

  #if defined AltitudeHoldBaro
    if (checkParameterMatch(parameterNameAHBaroSmooth, key)) {
      paramIndicator = NONE;
      parameterToBeChangedFloat = &baroSmoothFactor;
      return -1;
    }
    if (checkParameterMatch(parameterNameBaroP, key)) {
      paramIndicator = P;
      return BARO_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameBaroI, key)) {
      paramIndicator = I;
      return BARO_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameBaroD, key)) {
      paramIndicator = D;
      return BARO_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameBaroWindUpGuard, key)) {
      paramIndicator = windUpGuard;
      return BARO_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameZDampeningP, key)) {
      paramIndicator = P;
      return ZDAMPENING_PID_IDX;
    }
    if (checkParameterMatch(parameterNameZDampeningI, key)) {
      paramIndicator = I;
      return ZDAMPENING_PID_IDX;
    }
    if (checkParameterMatch(parameterNameZDampeningD, key)) {
      paramIndicator = D;
      return ZDAMPENING_PID_IDX;
    }
  #endif

  #if defined AltitudeHoldRangeFinder
    if (checkParameterMatch(parameterNameRangeFinderP, key)) {
      paramIndicator = P;
      return SONAR_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameRangeFinderI, key)) {
      paramIndicator = I;
      return SONAR_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameRangeFinderD, key)) {
      paramIndicator = D;
      return SONAR_ALTITUDE_HOLD_PID_IDX;
    }
    if (checkParameterMatch(parameterNameRangeFinderWindUpGuard, key)) {
      paramIndicator = windUpGuard;
      return SONAR_ALTITUDE_HOLD_PID_IDX;
    }
  #endif

  #if defined UseGPSNavigator
    if (checkParameterMatch(parameterNameGPSRollP, key)) {
      paramIndicator = P;
      return GPSROLL_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSRollI, key)) {
      paramIndicator = I;
      return GPSROLL_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSRollD, key)) {
      paramIndicator = D;
      return GPSROLL_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSPitchP, key)) {
      paramIndicator = P;
      return GPSPITCH_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSPitchI, key)) {
      paramIndicator = I;
      return GPSPITCH_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSPitchD, key)) {
      paramIndicator = D;
      return GPSPITCH_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSYawP, key)) {
      paramIndicator = P;
      return GPSYAW_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSYawI, key)) {
      paramIndicator = I;
      return GPSYAW_PID_IDX;
    }
    if (checkParameterMatch(parameterNameGPSYawD, key)) {
      paramIndicator = D;
      return GPSYAW_PID_IDX;
    }
  #endif

  return 0;
}

void changeAndSendParameter() {
  if(parameterChangeIndicator == 0) {
    // Only write and emit changes if there is actually a difference AND only write if new value is NOT "not-a-number" AND is NOT infinit

    if(parameterMatch != 0) {
      if (paramIndicator == P) {
        if (PID[parameterMatch].P != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
          PID[parameterMatch].P = set.param_value;
          writeEEPROM();
          // Report back new value
          mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].P, parameterType, parameterListSize, -1);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          SERIAL_PORT.write(buf, len);
        }
      }

      else if (paramIndicator == I) {
        if (PID[parameterMatch].I != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
          PID[parameterMatch].I = set.param_value;
          writeEEPROM();
          // Report back new value
          mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].I, parameterType, parameterListSize, -1);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          SERIAL_PORT.write(buf, len);
        }
      }

      else if (paramIndicator == D) {
        if (PID[parameterMatch].D != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
          PID[parameterMatch].D = set.param_value;
          writeEEPROM();
          // Report back new value
          mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].D, parameterType, parameterListSize, -1);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          SERIAL_PORT.write(buf, len);
        }
      }

      else if (paramIndicator == NONE) {
        if (parameterToBeChangedFloat != NULL) {
          if (*parameterToBeChangedFloat != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
            *parameterToBeChangedFloat = set.param_value;
            writeEEPROM();
            // Report back new value
            mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, *parameterToBeChangedFloat, parameterType, parameterListSize, -1);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            SERIAL_PORT.write(buf, len);
          }
        }
        else if (parameterToBeChangedByte != NULL) {
          if (*parameterToBeChangedByte != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
            *parameterToBeChangedByte = set.param_value;
            writeEEPROM();
            // Report back new value
            mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, *parameterToBeChangedByte, parameterType, parameterListSize, -1);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            SERIAL_PORT.write(buf, len);
          }
        }
        else if (parameterToBeChangedInt != NULL) {
          if (*parameterToBeChangedInt != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
            *parameterToBeChangedInt = set.param_value;
            writeEEPROM();
            // Report back new value
            mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, *parameterToBeChangedInt, parameterType, parameterListSize, -1);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            SERIAL_PORT.write(buf, len);
          }
        }
        else if (parameterToBeChangedULong != NULL) {
          if (*parameterToBeChangedULong != set.param_value && !isnan(set.param_value) && !isinf(set.param_value)) {
            *parameterToBeChangedULong = set.param_value;
            writeEEPROM();
            // Report back new value
            mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, *parameterToBeChangedULong, parameterType, parameterListSize, -1);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            SERIAL_PORT.write(buf, len);
          }
        }
        parameterChangeIndicator = -1;
      }
    }
  }
}

void readSerialCommand() {
  while(SERIAL_PORT.available() > 0) { 

    uint8_t c = SERIAL_PORT.read();
    //try to get a new message 
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
      // Handle message
      switch(msg.msgid) {

        // 					case MAVLINK_MSG_ID_SET_MODE: { // setting the system mode makes no sense for now
        // 						systemMode = mavlink_msg_set_mode_get_base_mode(&msg);
        // 					}
        // 					break;

        case MAVLINK_MSG_ID_COMMAND_LONG:  {
          uint8_t result = 0;
          uint8_t command = mavlink_msg_command_long_get_command(&msg);

          // 						if (command == 	MAV_CMD_COMPONENT_ARM_DISARM) { // needs some security checks to prevent accidential arming/disarming
          // 							if (mavlink_msg_command_long_get_param1(&msg) == 1.0) motorArmed = ON;
          // 							else if (mavlink_msg_command_long_get_param1(&msg) == 0.0) motorArmed = OFF;
          // 							result = MAV_RESULT_ACCEPTED;
          // 						}

          // 						if (command == MAV_CMD_DO_SET_MODE) { // setting the system mode makes no sense for now
          // 							systemMode = mavlink_msg_command_long_get_param1(&msg);
          // 							result = MAV_RESULT_ACCEPTED;
          // 						}

          if (command == MAV_CMD_NAV_RETURN_TO_LAUNCH) {
            #if defined UseGPSNavigator 
              //TODO	add coming home
              //result = MAV_RESULT_ACCEPTED;
            #else
              result = MAV_RESULT_UNSUPPORTED;
            #endif
          }
          else if (command == MAV_CMD_NAV_TAKEOFF) {
            #if defined UseGPSNavigator 
              //TODO	add gps takeoff
              //result = MAV_RESULT_ACCEPTED;
            #else
              result = MAV_RESULT_UNSUPPORTED;	
            #endif
          }
          else if (command == MAV_CMD_DO_SET_HOME) {
            #if defined UseGPS
              if (mavlink_msg_command_long_get_param1(&msg) == 1.0) {
                homePosition = currentPosition;
              }
              else {
                homePosition.latitude = mavlink_msg_command_long_get_param5(&msg);
                homePosition.longitude = mavlink_msg_command_long_get_param6(&msg);
                homePosition.altitude = mavlink_msg_command_long_get_param7(&msg);
              }
              result = 	MAV_RESULT_ACCEPTED;
            #else
              result = 	MAV_RESULT_UNSUPPORTED;
            #endif
          }
          else if (command ==	MAV_CMD_PREFLIGHT_CALIBRATION) {
            if (!motorArmed) {
              if (mavlink_msg_command_long_get_param1(&msg) == 1.0f) {
                calibrateGyro();
                storeSensorsZeroToEEPROM();
                result = MAV_RESULT_ACCEPTED;
              }
              if (mavlink_msg_command_long_get_param2(&msg) == 1.0f) {
                computeAccelBias();
                storeSensorsZeroToEEPROM();
                calibrateKinematics();
                zeroIntegralError();
                result = MAV_RESULT_ACCEPTED;
              }			
            }
            else result = 	MAV_RESULT_TEMPORARILY_REJECTED;
          }

          mavlink_msg_command_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, command, result);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          SERIAL_PORT.write(buf, len);
        }
        break;

      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
          paramListPartIndicator = indexCounter = 0;
        }
        break;

      case MAVLINK_MSG_ID_PARAM_REQUEST_READ: { 
          mavlink_param_request_read_t read;
          mavlink_msg_param_request_read_decode(&msg, &read);

          key = (char*) read.param_id;

          int parameterMatch = findParameter(key);

          if(parameterMatch != 0) {
            if (paramIndicator == P) {
              mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].P, parameterType, parameterListSize, -1);
              len = mavlink_msg_to_send_buffer(buf, &msg);
              SERIAL_PORT.write(buf, len);
            }
            else if (paramIndicator == I) {
              mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].I, parameterType, parameterListSize, -1);
              len = mavlink_msg_to_send_buffer(buf, &msg);
              SERIAL_PORT.write(buf, len);
            }
            else if (paramIndicator == D) {
              mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, PID[parameterMatch].D, parameterType, parameterListSize, -1);
              len = mavlink_msg_to_send_buffer(buf, &msg);
              SERIAL_PORT.write(buf, len);
            }
            else if (paramIndicator == NONE) {
              mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, key, parameterMatch, parameterType, parameterListSize, -1);
              len = mavlink_msg_to_send_buffer(buf, &msg);
              SERIAL_PORT.write(buf, len);
            }
          }
        }
        break;

      case MAVLINK_MSG_ID_PARAM_SET: 
        {
          if(!motorArmed) { // added for security reason, as the software is shortly blocked by this command (maybe this can be avoided?)
            mavlink_msg_param_set_decode(&msg, &set);
            key = (char*) set.param_id;
            parameterMatch = findParameter(key);
            parameterChangeIndicator = 0;
          }
        }
        break;

      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: { //TODO needs to be tested
        #if defined UseGPSNavigator
          mavlink_msg_mission_count_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, MAX_WAYPOINTS);
          len = mavlink_msg_to_send_buffer(buf, &msg);
          SERIAL_PORT.write(buf, len);

          for (byte index = 0; index < MAX_WAYPOINTS; index++) {
            if (index != missionNbPoint) mavlink_msg_mission_item_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, index, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 0, 1, 0, MIN_DISTANCE_TO_REACHED, 0, 0, waypoint[index].longitude, waypoint[index].latitude, waypoint[index].altitude);
            else mavlink_msg_mission_item_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_SYSTEM_ID, MAV_COMPONENT_ID, index, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, 1, 1, 0, MIN_DISTANCE_TO_REACHED, 0, 0, waypoint[index].longitude, waypoint[index].latitude, waypoint[index].altitude);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            SERIAL_PORT.write(buf, len);
          }			
        #endif
        }
        break;

      default:
        break;
      }
    } 
  } 
  system_dropped_packets += status.packet_rx_drop_count;
}


void sendQueuedParameters() {
  if(paramListPartIndicator >= 0 && paramListPartIndicator <= 4) {
    if(paramListPartIndicator == 0) {
      sendParameterListPart1();
    }
    else if(paramListPartIndicator == 1) {
      sendParameterListPart2();
    }
    else if(paramListPartIndicator == 2) {
      sendParameterListPart3();
    }
    else if(paramListPartIndicator == 3) {
      sendParameterListPart4();
    }
    else if(paramListPartIndicator == 4) {
      sendParameterListPart5();
    }
    paramListPartIndicator++;
  }
  else {
    paramListPartIndicator = -1;
  }
}

void sendSerialVehicleData() {
  sendSerialHudData();
  sendSerialAttitude();
  sendSerialRcRaw();
  sendSerialRawPressure();
  sendSerialRawIMU();
  sendSerialGpsPostion();
  sendSerialSysStatus();
}


void sendSerialTelemetry() {
  sendSerialVehicleData();
  updateFlightTime();
  sendQueuedParameters();
  changeAndSendParameter();
}

#endif //#define _AQ_MAVLINK_H_

