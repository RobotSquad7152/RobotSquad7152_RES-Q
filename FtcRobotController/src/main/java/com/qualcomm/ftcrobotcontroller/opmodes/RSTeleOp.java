/* Copyright (c) 2014 Qualcomm Technologies Inc

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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import RobotSquad.BucketThread;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class RSTeleOp extends OpMode {

	/*
	 * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
	// TETRIX VALUES.

	DcMotor motorFrontRight;
	DcMotor motorFrontLeft;
	DcMotor motorBackRight;
	DcMotor motorBackLeft;
	DcMotor motorArm;
	DcMotor motorSlide;
	DcMotor motorBucket;
	DcMotor motorSpinner;
	Servo servoDoor;
	Servo servoHopper;
	Servo servoChurro;

	BucketThread bucketThread;
	double leftPower = 0;
	double rightPower = 0;
	double armPower = 0;
	double deadZone = .1;
	double servoDoorOpen = .8;
	double servoDoorClose = 0;
	double servoHopperPos = 0.5;//stopped continuous
	int bucketTarget;
	int bucketPos = 0;
	boolean continuousSpin = false;
	boolean j1aPressed = false;
	boolean j1bPressed = false;
	double servoChurroUp = .5;
	double servoChurroDown = .3;
	/**
	 * Constructor
	 */
	public RSTeleOp() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		/*Controller		Config Name		Robot
		Front Drive Port 1	motor_1			Front Right
		Front Drive Port 2	motor_2			Front Left
		Rear Drive Port 1	motor_3			Back Right
		Rear Drive Port 2	motor_4			Back Left
		Extend/hang			motor_5			Slide motor
		Harvester port 1	motor_8			Harvester bucket
		Harvester port 2	motor_7			Harvester spinner*/

		motorFrontLeft = hardwareMap.dcMotor.get("motor_2");
		motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
		motorFrontRight = hardwareMap.dcMotor.get("motor_1");
		motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
		motorBackLeft = hardwareMap.dcMotor.get("motor_4");
		motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
		motorBackRight = hardwareMap.dcMotor.get("motor_3");
		motorBackRight.setDirection(DcMotor.Direction.REVERSE);

		motorSlide = hardwareMap.dcMotor.get("motor_5");
		motorSlide.setDirection(DcMotor.Direction.FORWARD);

		motorBucket = hardwareMap.dcMotor.get("motor_8");
		motorBucket.setDirection(DcMotor.Direction.FORWARD);

		motorSpinner = hardwareMap.dcMotor.get("motor_7");
		motorSpinner.setDirection(DcMotor.Direction.FORWARD);

		servoDoor = hardwareMap.servo.get("servo_door");
		servoDoor.setPosition(servoDoorClose);

		servoHopper = hardwareMap.servo.get("servo_hopper");
		servoHopper.setPosition(servoHopperPos);

		servoChurro = hardwareMap.servo.get("servo_churro");
		servoChurro.setPosition(servoChurroUp);
4
		bucketThread = new BucketThread(motorBucket);

		bucketThread.InitalizeBucket();

		bucketTarget = motorBucket.getCurrentPosition();




	//	motorBucket.getController().setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

	//	motorBucket.setMode(DcMotorController.RunMode.RESET_ENCODERS);

	}

	@Override
	public void start() {
		motorBucket.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

		bucketThread.start();

	}


	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {



		double j1y1 = -gamepad1.left_stick_y;
		double j1x2 = gamepad1.right_stick_x;

		double j2y1 = gamepad2.left_stick_y;




	//	telemetry.addData("y1 ", j1y1);
	//	telemetry.addData("x2 ", j1x2);
		telemetry.addData("servo", servoDoor.getPosition());
		telemetry.addData("servo", servoChurro.getPosition());

		if (j1y1 >= deadZone)
		{
			if (j1x2 >= deadZone)
			{
				leftPower = j1y1;
				rightPower = j1y1 - (j1y1 * j1x2);
			} else if (j1x2 < -deadZone)
			{
				leftPower = j1y1 - (j1y1 * -j1x2);
				rightPower = j1y1;
			} else
			{
				leftPower = j1y1;
				rightPower = j1y1;
			}

		} else if (j1y1 <= -deadZone)
		{
			if (j1x2 >= deadZone)
			{
				leftPower = j1y1;
				rightPower = j1y1 + (j1y1 * -j1x2);
			} else if (j1x2 < -deadZone)
			{
				leftPower = j1y1 + (j1y1 * j1x2);
				rightPower = j1y1;
			} else
			{
				leftPower = j1y1;
				rightPower = j1y1;
			}
		} else //y1 is equal to 0 for spins
		{
			if ((j1x2 < -deadZone) || (j1x2 > deadZone))
			{
				leftPower = j1x2;
				rightPower = -j1x2;
			} else
			{
				leftPower = 0;

				rightPower = 0;
			}
		}

		telemetry.addData("left power", "LeftPow " + leftPower);
		telemetry.addData("right power", "RightPow " + rightPower);

		motorFrontRight.setPower(Range.clip(rightPower, -1, 1));
		motorBackRight.setPower(Range.clip(rightPower, -1, 1));
		motorFrontLeft.setPower(Range.clip(leftPower, -1, 1));
		motorBackLeft.setPower(Range.clip(leftPower, -1, 1));


		if (gamepad2.dpad_up)
		{
			motorSlide.setPower(1);
		}
		else if(gamepad2.dpad_down)
		{
			motorSlide.setPower(-1);
		}
		else
		{
			motorSlide.setPower(0);

		}

		if (gamepad2.dpad_left)
		{
			//hopper left
			servoHopperPos = 0;
		}
		else if (gamepad2.dpad_right)
		{
			//hopper  right
			servoHopperPos = 1;
		}
		else
		{
			//hopper stopped
			servoHopperPos = 0.5;
		}

		servoHopper.setPosition(servoHopperPos);

		if (gamepad2.x)
		{
			servoDoor.setPosition(Range.clip(servoDoorOpen, 0, 1));


		}
		if (gamepad2.y)
		{
			servoDoor.setPosition(Range.clip(servoDoorClose, 0, 1));
		}

		if (gamepad1.x)
		{
			servoChurro.setPosition(Range.clip(servoChurroUp, 0, 1));


		}
		if (gamepad1.y)
		{
			servoChurro.setPosition(Range.clip(servoChurroDown, 0, 1));
		}

		//telemetry.addData("ServoDoor pos " , "servoDoorpos;" + servoDoorPos);

		if ((j2y1 < -deadZone) || (j2y1 > deadZone))
		{
			if (j2y1 < 0)
			{
				//harvester up
			bucketTarget -= 10;
				//motorBucket.setPower(-0.3);
			}
			else if (j2y1 > 0)
			{
				//harvester down
				bucketTarget += 10;
				//motorBucket.setPower(0.3);
			}

		}
		else
		{
			//motorBucket.setPower(0);
			bucketTarget = motorBucket.getCurrentPosition();//stop moving after dpad is released
		}

		bucketThread.targetPos = bucketTarget;

		//bucketThread.targetPos = bucketTarget;
		bucketPos = motorBucket.getCurrentPosition();


	//	telemetry.addData("bucket target " , "bucketTarget;" + bucketTarget);
		telemetry.addData("bucket position " , "bucketpos;" + bucketPos);

		if (gamepad1.a)
		{
			if (!j1aPressed)
			{
				j1aPressed = true;
				//motorSpinner
				if (continuousSpin)
				{
					continuousSpin = false;
					motorSpinner.setPower(0);
				}
				else
				{
					continuousSpin = true;
					motorSpinner.setPower(1);
				}
			}
		}
		else
		{
			j1aPressed = false;
		}

		if (gamepad1.b)
		{
			if (!j1bPressed)
			{
				j1bPressed = true;
				//motorSpinner
				if (continuousSpin)
				{
					continuousSpin = false;
					motorSpinner.setPower(0);
				}
				else
				{
					continuousSpin = true;
					motorSpinner.setPower(-1);
				}
			}
		}
		else
		{
			j1bPressed = false;
		}
		if (gamepad1.left_trigger > 0.1) {
			//motor spinner
			continuousSpin =false;
			motorSpinner.setPower(-gamepad1.left_trigger);
		}else if (gamepad1.right_trigger > 0.1) {
			//motor spinner
			continuousSpin = false;
			motorSpinner.setPower(gamepad1.right_trigger);
		}
		else
		{
			if (!continuousSpin)
				motorSpinner.setPower(0);
		}






//		if( gamepad1.dpad_down )
//			motorBackLeft.setPower(1);
//		else
//			motorBackLeft.setPower(0);
//
//		if( gamepad1.dpad_right )
//			motorBackRight.setPower(1);
//		else
//			motorBackRight.setPower(0);
//
//		if( gamepad1.dpad_left )
//			motorFrontLeft.setPower(1);
//		else
//			motorFrontLeft.setPower(0);
//
//		if( gamepad1.dpad_up )
//			motorFrontRight.setPower(1);
//		else
//			motorFrontRight.setPower(0);
	}


	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

    	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
		
		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		
		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
