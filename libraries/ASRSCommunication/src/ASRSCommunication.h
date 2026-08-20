#ifndef ASRS_COMMUNICATION_H
#define ASRS_COMMUNICATION_H

#include "ASRS_Protocol.h"
#include "ASRS_Comm_Base.h"
#include "ASRS_FrameCodec.h"
#include "ASRS_RackProtocol.h"
#include "ASRS_Comm_UART.h"
#include "ASRS_Comm_SoftwareSerial.h"
#include "ASRS_Comm_ESPNow.h"
#include "ASRS_Master.h"
#include "ASRS_Slave.h"
#include "session/ASRS_ESPNow_MasterSession.h"
#include "session/ASRS_ESPNow_SlaveSession.h"
#include "VL53L1X_Manager.h"
//1.to link the sensor to the slave, the send to master.
//2. master can send some location (using serial)

#endif
