package RobotSquad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.Object;
import java.lang.Math;
//import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Tony on 10/4/2015.
 */

public class RSRobot {


    LinearOpMode opMode;
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotorController motorControllerFrontDrive;
    DcMotorController motorControllerRearDrive;
    private GyroThread gyrothread;
    private GyroSensor gyro;
    double minSpinRampUpPow = .2;
    double minSpinRampDownPow = .1;
    public enum Alliance{
        BLUE  (1),
        RED  (-1);
        private final double alliance;

        private Alliance(double alliance) {
            this.alliance = alliance;
        }
    }



    Alliance myAlliance;
    public void setMyAlliance(Alliance myAlliance) {
        this.myAlliance = myAlliance;
    }

    //defines drive wheel diameter
    final double wheeldiacm = 4 * 2.54;
    //defines drive wheel circumference
    final double wheelcirccm = wheeldiacm * 3.141592653589793;
    //defines how many teeth on gear attached to motor
    final double motorgearteeth = 24;
    //defines how many teeth on gear attached to wheel
    final double wheelgearteeth = 24;
    //matrix encoder counts per rotation of the motor
    final double motorclicksperrotation = 1120;
    //calculates how far the robot will drive for each motor encoder click
    final double onemotorclick = ((motorgearteeth / wheelgearteeth) * wheelcirccm) / motorclicksperrotation;

    public void Initialize() {
        if (gyro != null) {
            gyrothread = new GyroThread(gyro);

            gyrothread.calibrategyro();

            gyrothread.start();
        }
        if (motorBackRight != null)
            motorBackRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        if (motorBackLeft != null)
            motorBackLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    public RSRobot(GyroSensor gyro) {
        this.gyro = gyro;
    }


    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void SetFrontRightMotor(DcMotor motor) {
        motorFrontRight = motor;
    }

    public void SetFrontLeftMotor(DcMotor motor) {
        motorFrontLeft = motor;
    }

    public void SetBackRightMotor(DcMotor motor) {
        motorBackRight = motor;
    }

    public void SetBackLeftMotor(DcMotor motor) {
        motorBackLeft = motor;
    }

    public void setMotorControllerFrontDrive(DcMotorController motorControllerFrontDrive) {
        this.motorControllerFrontDrive = motorControllerFrontDrive;
    }

    public void setMotorControllerRearDrive(DcMotorController motorControllerRearDrive) {
        this.motorControllerRearDrive = motorControllerRearDrive;
    }

    public long DriveForwardLegacy(double power, long distance) throws InterruptedException {
        double encoderTarget;
        double calculatedPow = 0;
        int frontRightStartPosition = 0;
        int currentPosition;
        //set current heading to zero
         gyrothread.setCurrentHeading(0);
        //use a while loop to keep motors going until desired heading reached

        encoderTarget = distance / onemotorclick;

        motorBackLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        while (motorControllerRearDrive.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY) {
            opMode.telemetry.addData("WAITING FOR READ_ONLY ", 1234);
            opMode.waitForNextHardwareCycle();

        }

        //gets front rights position to be able to zero out the current position
        frontRightStartPosition = motorBackRight.getCurrentPosition();

      //  opMode.telemetry.addData("Current Encoder Position ", frontRightStartPosition);

        currentPosition = frontRightStartPosition;



        while (currentPosition - frontRightStartPosition < encoderTarget) {

            //calculatedPow = calculateTurnPow(degrees, gyrothread.getCurrentheading(), power);
            calculatedPow = power;
            motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

            while (motorControllerRearDrive.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY) {
                opMode.telemetry.addData("WAITING FOR WRITE_ONLY", 1);
                opMode.waitForNextHardwareCycle();
            }


            if (motorControllerRearDrive.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.WRITE_ONLY) {
                motorBackRight.setPower(calculatedPow);


                //motorBackRight.setPower(calculatedPow);
                motorBackLeft.setPower(calculatedPow);

              //  motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);

            }

            //motorBackLeft.setPower(calculatedPow);

            motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
            while (motorControllerRearDrive.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY) {
                opMode.telemetry.addData("WAITING FOR READ_ONLY", 2);
                opMode.waitForNextHardwareCycle();
            }

            currentPosition = motorBackLeft.getCurrentPosition();


            opMode.telemetry.addData("Current Encoder Position ", currentPosition);
            opMode.telemetry.addData("Current Power ", motorBackLeft.getPower());
        }

        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        while (motorControllerRearDrive.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY) {
            opMode.telemetry.addData("WAITING FOR WRITE_ONLY", 1);
            opMode.waitForNextHardwareCycle();
        }


        motorBackRight.setPower(0);


        //motorBackRight.setPower(calculatedPow);
        motorBackLeft.setPower(0);


        return (distance);
    }

    private long Spin(double power, long degrees, double direction) throws InterruptedException {
        double calculatedPow = 0;
        //set current heading to zero
        gyrothread.setCurrentHeading(0);
        //use a while loop to keep motors going until desired heading reached
        while (java.lang.Math.abs(gyrothread.getCurrentHeading()) < degrees) {
            calculatedPow = (calculateTurnPow(degrees, gyrothread.getCurrentHeading(), power))*direction*myAlliance.alliance;

            motorFrontRight.setPower(-calculatedPow);
            motorBackRight.setPower(-calculatedPow);
            motorFrontLeft.setPower(calculatedPow);
            motorBackLeft.setPower(calculatedPow);

            opMode.telemetry.addData("curr heading ", gyrothread.getCurrentHeading());
            opMode.telemetry.addData("pow ", calculatedPow);

            opMode.waitForNextHardwareCycle();

        }
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        return (long)java.lang.Math.abs(gyrothread.getCurrentHeading());
    }

    public long SpinRight(double power, long degrees) throws InterruptedException {
        //Calling spin function and direction 1 is right
        return(Spin(power, degrees, 1));
    }
    public long SpinLeft(double power, long degrees) throws InterruptedException {
        //Calling spin function and direction -1 is left
        return(Spin(power, degrees, -1));

    }

    double calculateTurnPow(double totalTurn, double currentHeading, double maxPow) {
        double calculatedPow = 0;
        double rampUpCalcPow = 0;
        double minPow = .2;
        double rampDownCalcPow = 0;
        //number of degrees for speeding up
        double rampUpDegrees = 30;
        //number of degrees for slowing down
        double rampDownDegrees = 30;

        rampUpCalcPow = minPow + (((maxPow - minPow) / rampUpDegrees) * currentHeading);
        rampDownCalcPow = minPow + (((minPow - maxPow) / rampDownDegrees) * (currentHeading - totalTurn));

        calculatedPow = Math.min(maxPow, rampUpCalcPow);
        calculatedPow = Math.min(calculatedPow, rampDownCalcPow);

        if (calculatedPow < minPow) {
            calculatedPow = minPow;

        }
        return Range.clip(calculatedPow, -1, 1);
    }

    double calculateDrivePow(double totalDistance, double currentDistance, double maxPow) {
        double calculatedPow = 0;
        double rampUpCalcPow = 0;
        double minPow = .2;
        double rampDownCalcPow = 0;
        //distance in cm for speeding up
        double rampUpDistance = 20;
        //distance in cm for slowing down
        double rampDownDistance = 20;

        rampUpCalcPow = minPow + (((maxPow - minPow) / rampUpDistance) * currentDistance);
        rampDownCalcPow = minPow + (((minPow - maxPow) / rampDownDistance) * (currentDistance - totalDistance));

        calculatedPow = Math.min(maxPow, rampUpCalcPow);
        calculatedPow = Math.min(calculatedPow, rampDownCalcPow);

        if (calculatedPow < minPow) {
            calculatedPow = minPow;

        }
        return Range.clip(calculatedPow, -1, 1);
    }

    private long Drive(double power, long distance, double direction) throws InterruptedException {
        double encoderTarget;
        double calculatedPow = 0;

        //set current heading to zero
        //gyrothread.setCurrentheading(0);
        //use a while loop to keep motors going until desired heading reached

        motorControllerRearDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorBackLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorBackRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        while(motorBackLeft.getCurrentPosition() != 0 || motorBackRight.getCurrentPosition() != 0)
        {
            opMode.waitForNextHardwareCycle();
        }
        motorBackLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorBackRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        opMode.waitForNextHardwareCycle();

        encoderTarget = distance / onemotorclick;

        while (java.lang.Math.abs(motorBackLeft.getCurrentPosition()) < encoderTarget) {

            calculatedPow = calculateDrivePow(distance, motorBackLeft.getCurrentPosition()*onemotorclick, power)*direction;

            motorBackLeft.setPower(calculatedPow);
            motorBackRight.setPower(calculatedPow);
            motorFrontLeft.setPower(calculatedPow);
            motorFrontRight.setPower(calculatedPow);

            opMode.telemetry.addData("Current Encoder Position ", motorBackLeft.getCurrentPosition());

            opMode.waitForNextHardwareCycle();

        }

        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);

        return (distance);
    }

    public long DriveForward(double power, long distance) throws InterruptedException {
        //Calling drive function and 1 is forward
        return(Drive(power, distance, 1));
    }
    public long DriveBackward(double power, long distance) throws InterruptedException {
        //Calling drive function and -1 is backward
        return(Drive(power, distance, -1));
    }
}