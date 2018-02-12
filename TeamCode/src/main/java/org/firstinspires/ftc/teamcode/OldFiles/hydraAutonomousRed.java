/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode.OldFiles;

import android.media.MediaPlayer;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous mode
 * enable movement based on sensors and preset code
 */
public class hydraAutonomousRed extends LinearOpMode {
  //creates motors
  DcMotor motorBL; //motors for movement
  DcMotor motorBR;
  DcMotor motorFL;
  DcMotor motorFR;
  //    DcMotor lift;
  DcMotor rightClaw; //motors to control claws
  DcMotor leftClaw;
  //    DcMotor manipulator;
//    Servo basket;
  Servo rightBar; //servos to control side bars
  Servo leftBar;
  Servo climberBar; //servo to control climber dumping
  MediaPlayer song;
  DeviceInterfaceModule cdim;
  ColorSensor color;
  AdafruitIMU gyro;
  static final int LED_CHANNEL = 5;
  volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
  private static final String LOG_TAG = hydraAutonomousBlue.class.getSimpleName();
  //    ColorSensor color;
  //  OpticalDistanceSensor distance
  ElapsedTime elapsedTime;
  public void startMotors(double power1, double power2, double power3, double power4) {
    motorBR.setPower(power1);
    motorBL.setPower(power2);
    motorFL.setPower(power3);
    motorFR.setPower(power4);
  }
  public void first() {
    motorBL = hardwareMap.dcMotor.get("motorBL");
    motorBR = hardwareMap.dcMotor.get("motorBR");
    motorFR = hardwareMap.dcMotor.get("motorFR");
    motorFL = hardwareMap.dcMotor.get("motorFL");
    rightClaw = hardwareMap.dcMotor.get("rightClaw");
    leftClaw = hardwareMap.dcMotor.get("leftClaw");
    rightBar = hardwareMap.servo.get("rightBar");
    leftBar = hardwareMap.servo.get("leftBar");
    climberBar = hardwareMap.servo.get("climberBar");
    color = hardwareMap.colorSensor.get("color");
    climberBar.setPosition(Servo.MAX_POSITION);
    rightBar.setPosition(Servo.MIN_POSITION);
    leftBar.setPosition(Servo.MAX_POSITION);
    cdim = hardwareMap.deviceInterfaceModule.get("dim");
    try {
      gyro = new AdafruitIMU(hardwareMap, "hydro"

              //The following was required when the definition of the "I2cDevice" class was incomplete.
              //, "cdim", 5

              , (byte) (AdafruitIMUAccel.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
              //addressing
              , (byte) AdafruitIMUAccel.OPERATION_MODE_IMU);
    } catch (RobotCoreException e) {
      Log.e(LOG_TAG, "ROBOT CORE EXCEPTION", e);
    }


  }
  public int getEncoderAvg() {
    return (Math.abs(motorBL.getCurrentPosition()) + Math.abs(motorBR.getCurrentPosition()) + Math.abs(motorFL.getCurrentPosition()) + Math.abs(motorFR.getCurrentPosition())) / 4;
  }
  public void stopMotors() {
    motorBR.setPower(0);
    motorBL.setPower(0);
    motorFL.setPower(0);
    motorFR.setPower(0);
  }
  public void resetEncoders() {
//    motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//    motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//    motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//    motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
  }
  public void getEncoderValues() {
    telemetry.addData("motorBL", motorBL.getCurrentPosition());

    telemetry.addData("motorFR", motorFR.getCurrentPosition());

    telemetry.addData("motorBR", motorBR.getCurrentPosition());

    telemetry.addData("motorFL", motorFL.getCurrentPosition());
  }
  public void getTime() {
    telemetry.addData("time", elapsedTime.time());
  }
  @Override
  public void runOpMode() {
    first();
    cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
    cdim.setDigitalChannelState(LED_CHANNEL, false);
    try {
      waitOneFullHardwareCycle();
    } catch (InterruptedException e) {
      Log.i(LOG_TAG, e.toString());
    }

    waitForStart();

    int distance1 = 5000; // TODO: 11/19/2015 CORRECT VALUES
    int distance2 = 7075; // TODO: 11/19/2015 CORRECT VALUES
    int distance3 = 12750; // TODO: 11/19/2015 CORRECT VALUES
    int currentEncoder = 0;
    int nullEncoder = 0;
    double currentAngle = yawAngle[0];
    gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
    while (currentEncoder < distance1) {
      startMotors(-1, 1, 1, -1);
      if(currentAngle > 5) {
        startMotors(1, 1, 1, 1);
        nullEncoder = getEncoderAvg() - nullEncoder;
      }
      if(currentAngle < -5) {
        startMotors(-1, -1, -1, -1);
        nullEncoder = getEncoderAvg() - currentEncoder;
      }
      else
        currentAngle = getEncoderAvg() - nullEncoder;
      getEncoderValues();
    }
    gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
    currentAngle = yawAngle[0];
    while (currentAngle < 90.0) {
      startMotors(1, 1, 1, 1);
      gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
      currentAngle = yawAngle[0];
      telemetry.addData("yaw", currentAngle);
      getEncoderValues();
    }
    resetEncoders();
    while (currentAngle > -90.0) {
      startMotors(1, 1, 1, 1);
      gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
      currentAngle = yawAngle[0];
      telemetry.addData("yaw", currentAngle);
      getEncoderValues();
    }
    resetEncoders();
    while (currentAngle < 90.0) {
      startMotors(-1, -1, -1, -1);
      gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
      currentAngle = yawAngle[0];
      telemetry.addData("yaw", currentAngle);
      getEncoderValues();
    }
    stopMotors();
    resetEncoders();
    nullEncoder = 0;
    while (currentEncoder < distance2) {
      startMotors(-1, 1, 1, -1);
      if(currentAngle < 85) {
        startMotors(1, 1, 1, 1);
        nullEncoder = getEncoderAvg() - currentEncoder;
      }
      if(currentAngle > 95) {
        startMotors(-1, -1, -1, -1);
        nullEncoder = getEncoderAvg() - currentEncoder;
      }
      else
        currentAngle = getEncoderAvg() - nullEncoder;
      getEncoderValues();
    }
    stopMotors();
    elapsedTime.reset();
    double red = 0.0;
    double blue= 0.0;
    while (elapsedTime.time() < 1.0) {
      red = color.red();
      blue = color.blue();
      startMotors(-.2, .2, .2, -.2);
    }
    if(blue > red + 50) {
      climberBar.setPosition(1); // TODO: 11/19/2015 MAKE SURE THAT THIS IS CORRECT POSITION
    }
    else {
      elapsedTime.reset();
      while(elapsedTime.time() < 1.0) { // TODO: 11/19/2015 ADD MOVE SLIGHTLY BACKWARDS WITH ENCODERS
        startMotors(.5, -.5, -.5, .5);
      }
      gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
      currentAngle = yawAngle[0];
      while(currentAngle > 0) {
        startMotors(.5, .5, .5, .5);
      }
      resetEncoders();
      elapsedTime.reset();
      while(elapsedTime.time() < 1.0) { // TODO: 11/19/2015 ADD MOVE SLIGHTLY FORWARD WITH ENCODERS
        startMotors(-.5, .5, .5, -.5);
      }
      while(currentAngle < 90) {
        startMotors(-.5, -.5, -.5, -.5);
      }
      if(blue > red + 50) {
        climberBar.setPosition(1); // TODO: 11/19/2015 MAKE SURE THAT THIS IS CORRECT POSITION
      }

    }
    try {
      wait(1500);
    } catch(InterruptedException e) {
      Log.i(LOG_TAG, e.toString());
    }
    climberBar.setPosition(0); // TODO: 11/19/2015 MAKE SURE THIS IS CORRECT POSITION
    try {
      wait(1500);
    } catch(InterruptedException e) {
      Log.i(LOG_TAG, e.toString());
    }
    climberBar.setPosition(1); // TODO: 11/19/2015 MAKE SURE THIS IS CORRECT POSITION
    while (motorBL.getCurrentPosition() < distance3) {
      startMotors(1, -1, -1, 1);
      getEncoderValues();
    }
    while (currentAngle < 135.0) {
      startMotors(-1, -1, -1, -1);
      gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
      currentAngle = yawAngle[0];
      telemetry.addData("yaw", currentAngle);
      getEncoderValues();
    }
    elapsedTime.reset();
    stopMotors();
    motorBL.close();
    motorFL.close();
    motorBR.close();
    motorFR.close();
    song.stop();
  }
}