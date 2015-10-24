package RobotSquad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

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
    private Gyrothread gyrothread;
    private GyroSensor gyro;
    double minSpinRampUpPow = .2;
    double minSpinRampDownPow = .1;

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
        gyrothread = new Gyrothread(gyro);

        gyrothread.calibrategyro();

        gyrothread.start();
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

    public long DriveForward(double power, long distance) throws InterruptedException {
        double encoderTarget;
        double calculatedPow = 0;
        int frontRightStartPosition = 0;
        //set current heading to zero
        gyrothread.setCurrentheading(0);
        //use a while loop to keep motors going until desired heading reached

        encoderTarget = distance / onemotorclick;

     //   motorFrontRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
     //   while(motorFrontRight.getCurrentPosition() != 0 ){
      //      opMode.waitOneHardwareCycle();
      //  }

        //gets front rights position to be able to zero out the current position
        frontRightStartPosition = motorFrontRight.getCurrentPosition();

        opMode.telemetry.addData("ENC TGT ", encoderTarget);

        motorControllerFrontDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        while(motorControllerFrontDrive.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY)
            opMode.waitForNextHardwareCycle();

        while (motorFrontRight.getCurrentPosition() - frontRightStartPosition < encoderTarget) {

            //calculatedPow = calculateTurnPow(degrees, gyrothread.getCurrentheading(), power);
            calculatedPow = power;
            motorControllerFrontDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
            while(motorControllerFrontDrive.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.WRITE_ONLY)
                opMode.waitForNextHardwareCycle();

            motorFrontRight.setPower(calculatedPow);
            motorBackRight.setPower(calculatedPow);
            motorFrontLeft.setPower(calculatedPow);
            motorBackLeft.setPower(calculatedPow);

            motorControllerFrontDrive.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
            while(motorControllerFrontDrive.getMotorControllerDeviceMode() != DcMotorController.DeviceMode.READ_ONLY)
                opMode.waitForNextHardwareCycle();


        }
        return(distance);
    }

    public long SpinRight(double power, long degrees)  throws InterruptedException {
        double calculatedPow = 0;
        //set current heading to zero
        gyrothread.setCurrentheading(0);
        //use a while loop to keep motors going until desired heading reached
        while (gyrothread.getCurrentheading() < degrees) {
            calculatedPow = calculateTurnPow(degrees, gyrothread.getCurrentheading(), power);

            motorFrontRight.setPower(-calculatedPow);
            motorBackRight.setPower(-calculatedPow);
            motorFrontLeft.setPower(calculatedPow);
            motorBackLeft.setPower(calculatedPow);

            opMode.telemetry.addData("curr heading ", gyrothread.getCurrentheading());
            opMode.telemetry.addData("pow ", calculatedPow);

            opMode.waitForNextHardwareCycle();

        }
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        return degrees;
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
}
