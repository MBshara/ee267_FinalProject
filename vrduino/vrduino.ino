/**
 * EE267 Virtual Reality
 * Homework 5
 * Orientation Tracking with IMUs Arduino Programming
 * Modified for Serial Data Streaming and File Recording
 *
 * Instructor: Gordon Wetzstein <gordon.wetzstein@stanford.edu>
 *
 * @copyright The Board of Trustees of the Leland Stanford Junior University
 * @version 2020/04/01
 */

#include <Wire.h>
#include "OrientationTracker.h"
#include "TestOrientation.h"

//complementary filter value [0,1].
//1: ignore acc tilt, 0: use all acc tilt
double alphaImuFilter = 0.95;

//if true, get imu values from recorded data (replay mode)
//if false, get imu values from live sampling.
bool simulateImu = false;

//if test is true, then run tests in TestOrientation.cpp and exit
bool test = false;

//if measureImuBias is true, measure imu bias and variance
bool measureImuBias = true;

//if measureBias is false, set the imu bias to the following:
double gyrBiasSet[3] = {0.17408, -0.19935, 0.44581};

// DATA STREAMING VARIABLES
bool isStreaming = false;
unsigned long streamingStartTime = 0;

// REPLAY VARIABLES
bool isReplaying = false;
bool waitingForReplayData = false;

//initialize orientation tracker
OrientationTracker tracker(alphaImuFilter, simulateImu);

//stream mode
//To change what the Teensy is printing out, set streamMode
//to one of the following values.

//bias values, read frequency
const int INFO   = 0;
//flatland roll
const int FLAT   = 1;
//full 3D orientation in quaternion (gyro), and euler angles (acc), quaternion (comp filter)
const int THREED = 2;
//gyro values after bias subtraction
const int GYR    = 3;
//acc values
const int ACC    = 4;
//quaternion from comp filter
const int QC     = 5;
const int MG     = 6;
const int EK     = 7;
const int MH     = 8;
//raw data streaming for file recording
const int STREAM = 9;

//chose which values you want to stream
int streamMode = QC;

//variables to measure read frequency
int nReads = 0;
unsigned long prevTime = 0;

// Replay data buffer
struct ReplayIMUData {
  double gyr[3];
  double acc[3];
  bool valid;
};
/* *************************
 ALL REPLAY STUFF was made for the video I presented in the poster session
*******************/
ReplayIMUData currentReplayData = {{0,0,0}, {0,0,0}, false};

void startStreaming() {
  isStreaming = true;
  isReplaying = false;
  simulateImu = false;
  streamingStartTime = micros();
  Serial.println("STREAM_START");
  Serial.println("# Timestamp(us), AccX, AccY, AccZ, GyrX, GyrY, GyrZ");
}

void stopStreaming() {
  isStreaming = false;
  Serial.println("STREAM_END");
}

void startReplay() {
  isReplaying = true;
  isStreaming = false;
  simulateImu = true;
  waitingForReplayData = true;
  tracker.resetOrientation();
  Serial.println("REPLAY_START");
  Serial.println("# Ready for replay data. Send data in format: gyrX,gyrY,gyrZ,accX,accY,accZ");
}

void stopReplay() {
  isReplaying = false;
  simulateImu = false;
  waitingForReplayData = false;
  tracker.resetOrientation();
  Serial.println("REPLAY_END");
}

void parseReplayData(String dataLine) {
  int commaIndex[5];
  int commaCount = 0;
  
  for (int i = 0; i < dataLine.length() && commaCount < 5; i++) {
    if (dataLine.charAt(i) == ',') {
      commaIndex[commaCount] = i;
      commaCount++;
    }
  }
  
  if (commaCount == 5) {
    currentReplayData.gyr[0] = dataLine.substring(0, commaIndex[0]).toFloat();
    currentReplayData.gyr[1] = dataLine.substring(commaIndex[0] + 1, commaIndex[1]).toFloat();
    currentReplayData.gyr[2] = dataLine.substring(commaIndex[1] + 1, commaIndex[2]).toFloat();
    currentReplayData.acc[0] = dataLine.substring(commaIndex[2] + 1, commaIndex[3]).toFloat();
    currentReplayData.acc[1] = dataLine.substring(commaIndex[3] + 1, commaIndex[4]).toFloat();
    currentReplayData.acc[2] = dataLine.substring(commaIndex[4] + 1).toFloat();
    currentReplayData.valid = true;
  }
}

//runs when the Teensy is powered on
void setup() {
  Serial.begin(115200);
  
  if (test) {
    delay(1000);
    testMain();
    return;
  }

  tracker.initImu();

  if (measureImuBias) {
    Serial.println("Measuring bias...");
    tracker.measureImuBiasVariance();
  } else {
    tracker.setImuBias(gyrBiasSet);
  }

  prevTime = micros();
  
  // Print available commands
  Serial.println("=== IMU DATA RECORDING SYSTEM ===");
  Serial.println("Commands:");
  Serial.println("  'd' - Start/Stop data streaming for recording");
  Serial.println("  'x' - Start replay mode");
  Serial.println("  'z' - Stop replay mode");
  Serial.println("  'r' - Reset orientation");
  Serial.println("  'b' - Remeasure bias");
  Serial.println("  '0-9' - Set stream mode");
  Serial.println("==================================");
}

void loop() {
  // Handle serial commands and replay data
  if (Serial.available()) {
    if (isReplaying && waitingForReplayData) {
      // Read replay data line
      String dataLine = Serial.readStringUntil('\n');
      dataLine.trim();
      
      if (dataLine == "REPLAY_DATA_END") {
        stopReplay();
      } else if (dataLine.length() > 0 && !dataLine.startsWith("#")) {
        parseReplayData(dataLine);
      }
    } else {
      // Read command
      int read = Serial.read();
      
      // Clear the rest of the line
      while (Serial.available()) {
        Serial.read();
      }
      
      //check for streamMode
      int modeRead = read - 48;
      
      if (modeRead >= 0 && modeRead <= 9) {
        streamMode = modeRead;
        Serial.print("Stream mode set to: ");
        Serial.println(streamMode);
        
      } else if (read == 'r') {
        //reset orientation estimate to 0
        tracker.resetOrientation();
        Serial.println("Orientation reset");
        
      } else if (read == 'b') {
        //measure imu bias
        Serial.println("Measuring bias...");
        tracker.measureImuBiasVariance();
        
      } else if (read == 'd') {
        // Toggle data streaming
        if (isStreaming) {
          stopStreaming();
        } else {
          startStreaming();
        }
        
      } else if (read == 'x') {
        // Start replay mode
        startReplay();
        
      } else if (read == 'z') {
        // Stop replay mode
        stopReplay();
      }
    }
  }

  if (test) {
    return;
  }

  if (streamMode == INFO) {
    //print out number of reads / sec
    unsigned long now = micros();
    if ((now - prevTime) > 1000000) {
      Serial.print("nReads/sec: ");
      Serial.println(nReads);
      nReads = 0;
      prevTime = now;

      //print out bias/variance
      const double* gyrBias = tracker.getGyrBias();
      const double* gyrVariance = tracker.getGyrVariance();
      Serial.printf("GYR_BIAS: %.5f %.5f %.5f\n", gyrBias[0], gyrBias[1], gyrBias[2]);
      Serial.printf("GYR_VAR: %.5f %.5f %.5f\n", gyrVariance[0], gyrVariance[1], gyrVariance[2]);

      const double* accBias = tracker.getAccBias();
      const double* accVariance = tracker.getAccVariance();
      Serial.printf("ACC_BIAS: %.3f %.3f %.3f\n", accBias[0], accBias[1], accBias[2]);
      Serial.printf("ACC_VAR: %.3f %.3f %.3f\n", accVariance[0], accVariance[1], accVariance[2]);
    }
  }

  bool imuTrack = tracker.processImu();

  //return if there's no new values
  if (!imuTrack) {
    return;
  }

  nReads++;

  //get relevant values from the tracker class
  double flatlandRollGyr = tracker.getFlatLandRollGyr();
  double flatlandRollAcc = tracker.getFlatLandRollAcc();
  double flatlandRollComp = tracker.getFlatLandRollComp();
  const double* acc = tracker.getAcc();
  const double* gyr = tracker.getGyr();
  const Quaternion& qGyr = tracker.getQuaternionGyr();
  const double* eulerAcc = tracker.getEulerAcc();
  const Quaternion& qComp = tracker.getQuaternionComp();
  const Quaternion& qMadg = tracker.getQuaternionMadgwick();
  const Quaternion& qEKF = tracker.getQuaternionEKF();
  const Quaternion& qMah = tracker.getQuaternionMah();

  // Handle data streaming for file recording
  if (isStreaming) {
    // Output in CSV format: gyrX,gyrY,gyrZ,accX,accY,accZ (no timestamp)
    Serial.print(gyr[0], 3);
    Serial.print(",");
    Serial.print(gyr[1], 3);
    Serial.print(",");
    Serial.print(gyr[2], 3);
    Serial.print(",");
    Serial.print(acc[0], 3);
    Serial.print(",");
    Serial.print(acc[1], 3);
    Serial.print(",");
    Serial.print(acc[2], 3);
    Serial.println();
  }

  // Regular stream modes (only if not in data streaming mode)
  if (!isStreaming) {
    if (streamMode == FLAT) {
      //print out flatland roll
      Serial.printf("FLAT %.3f %.3f %.3f\n",
        flatlandRollGyr, flatlandRollAcc, flatlandRollComp);

    } else if (streamMode == THREED) {
      //quat values from gyro
      Serial.printf("QG %.3f %.3f %.3f %.3f\n",
        qGyr.q[0], qGyr.q[1], qGyr.q[2], qGyr.q[3]);
      //euler values from acc
      Serial.printf("EA %.3f %.3f %.3f\n",
        eulerAcc[0], eulerAcc[1], eulerAcc[2]);
      //quat values from comp filter
      Serial.printf("QC %.3f %.3f %.3f %.3f\n",
        qComp.q[0], qComp.q[1], qComp.q[2], qComp.q[3]);

    } else if (streamMode == GYR) {
      //print out gyr values
      Serial.printf("GYR: %.3f %.3f %.3f\n", gyr[0], gyr[1], gyr[2]);

    } else if (streamMode == ACC) {
      //print out acc values
      Serial.printf("ACC: %.3f %.3f %.3f\n", acc[0], acc[1], acc[2]);

    } else if (streamMode == QC) {
      //just print out comp filter
      Serial.printf("QC %.3f %.3f %.3f %.3f\n",
        qComp.q[0], qComp.q[1], qComp.q[2], qComp.q[3]);

    } else if (streamMode == MG) { // printing out madg
      Serial.printf("QC %.3f %.3f %.3f %.3f\n",
        qMadg.q[0], qMadg.q[1], qMadg.q[2], qMadg.q[3]);
    } else if (streamMode == EK) { // printing out EKF
      Serial.printf("QC %.3f %.3f %.3f %.3f\n",
        qEKF.q[0], qEKF.q[1], qEKF.q[2], qEKF.q[3]);
    } else if (streamMode == MH) { // printing out mahony
      Serial.printf("QC %.3f %.3f %.3f %.3f\n",
        qMah.q[0], qMah.q[1], qMah.q[2], qMah.q[3]);
    }
  }

  delay(10);
}