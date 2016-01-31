package RobotSquad;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Tony on 1/30/2016.
 */
public class BucketThread  extends Thread{

    DcMotor motorBucket;
    int currentPos = 0;
    volatile public int targetPos = 0;

    public BucketThread(DcMotor motor) {

        motorBucket = motor;


    }
//    public void SetTargetPos(int pos){
//
//        targetPos = pos;
//
//    }

    public void run() {

        while (true) {

            currentPos = motorBucket.getCurrentPosition();

            if ((currentPos >= (targetPos - 20)) && (currentPos <= (targetPos + 20)))//was 20
                motorBucket.setPower(0);
            else if(currentPos > targetPos)
                motorBucket.setPower(-0.2);
            else if (currentPos < targetPos)
                motorBucket.setPower(0.2);
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void InitalizeBucket(){

        motorBucket.getController().setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorBucket.setMode(DcMotorController.RunMode.RESET_ENCODERS);

    }
}
