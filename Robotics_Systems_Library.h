#ifndef ROBOTICS_SYSTEMS_LIBRARY_H
#define ROBOTICS_SYSTEMS_LIBRARY_H

#include <Arduino.h>
#include <SMS_STS.h>
#include <math.h>


// SMS_STS st;                                       // Motor Object used to send & receive motor data
#define MOTOR_BAUD_RATE 1000000                       // BaudRate used between Arduino and Motors
#define USB_BAUD_RATE 115200                          // BaudRate used between MATLAB and Arduino
#define CONNECTION_TIMEOUT 10000                       // Timeout for connecting to MATLAB
#define MAX_ID 8                                      // MAX Number of motors
#define POSITION 0
#define VELOCITY 1
#define ERROR -1
#define RAD_2_SMS 4096/(2*PI)
#define RAD_2_SMS_VEL 4096/(2*PI)
#define ACK_DATA_RECEIVED "d"


// State variable
static enum {
    IDLE,
    READ_FB,
    DRIVE_MOTOR,
    UPDATE_DRIVE_MODE,
    UPDATE_MOTOR
} state = IDLE;



class UOM_RS_Robot
{
public:
  SMS_STS motors;                                       // Motor Object used to send & receive motor data
  u8 num_ID;                                            // Number of connected motors
  int connection_status;
  u8 ID[MAX_ID];
  u8 connected_ID[MAX_ID];
  s16 q_FB_STS[MAX_ID];
  s16 q_STS[MAX_ID];
  s16 q_dot_STS[MAX_ID];
  bool control_mode[MAX_ID];
  
  

  UOM_RS_Robot();
  int EstablishUSBConnection();
  int EstablishMotorSerialConnection();
  int getID();
  int connectToMATLAB();

  void SetMotorControlMode(bool *new_control_mode);
  void SerialMonitorMotorControl();
  
  void DriveMotors();
  void ReadFeedback();
  void InitMotorFeedback();
  int getState();
  void SendFB2MATLAB();
  void getReference();
  void sendMotorIDs();
  void getControlMode();
  

private:
  
  void UpdateMotorControlMode();
  void sendDataSerial(int *data, int dataSize);
  void getDataSerial(double *serialData);
  void sendAck(char ack);
  
};




#endif