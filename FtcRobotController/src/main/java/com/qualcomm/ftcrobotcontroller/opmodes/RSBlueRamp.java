/* Copyright (c) 2015 Qualcomm Technologies Inc

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

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;

import RobotSquad.RSRobot;

/**
 * A simple example of a linear op mode that will approach an IR beacon
 */
public class RSBlueRamp extends LinearOpMode {
  RSRobot robot;

  DcMotor motor1;

  DcMotorController motorController1;
  GyroSensor gyro;

  //Gyrothread gyrothread;

  @Override
  public void runOpMode() throws InterruptedException {

    //initialize motors
    motor1 = hardwareMap.dcMotor.get("motor_1");
    motor1.setDirection(DcMotor.Direction.FORWARD);

    motorController1 = hardwareMap.dcMotorController.get("drive");
    //gyro = hardwareMap.gyroSensor.get("gyro");

    robot = new RSRobot(null);

    //This lets the robot know what way to spin based on alliance
    robot.setMyAlliance(RSRobot.Alliance.BLUE);

    // pass motor objects to robot
    robot.SetFrontRightMotor(null);
    robot.SetFrontLeftMotor(null);
    robot.SetBackRightMotor(null);
    robot.SetBackLeftMotor(motor1);
    robot.setMotorControllerFrontDrive(null);
    robot.setMotorControllerRearDrive(motorController1);

    robot.setOpMode(this);

    // calibrate gyro etc.
    robot.Initialize();

    waitForStart();


    //robot.SpinRight(.8, 360);
    robot.DriveForward(.5, 64);
  }
}
