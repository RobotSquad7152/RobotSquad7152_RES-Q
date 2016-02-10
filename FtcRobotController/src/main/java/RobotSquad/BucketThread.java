package RobotSquad;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Tony on 1/30/2016.
 */
public class BucketThread extends Thread
{

    DcMotor motorHarvester;
    int currentPos = 0;
    volatile public static int targetPos = 0;
    volatile boolean stopped = false;
    static int threadcounter = 0;

    public BucketThread(DcMotor motor)
    {

        motorHarvester = motor;
        if (threadcounter == 0)
        {
            threadcounter++;
    }

        else

        {
            stopped = true;
        }


    }

    public void stopBucketThread()
    {

        stopped = true;
        Log.d("DEBUG", "Stop Bucket Thread");
    }

    public void run()
    {

        while (!stopped)
        {

            currentPos = motorHarvester.getCurrentPosition();

            if ((currentPos >= (targetPos - 25)) && (currentPos <= (targetPos + 25)))
                motorHarvester.setPower(0);
            else if ((currentPos >= (targetPos - 50)) && (currentPos <= targetPos))
                motorHarvester.setPower(.02);
            else if ((currentPos <= (targetPos + 50)) && (currentPos >= targetPos))
                motorHarvester.setPower(-0.02);
            else if (currentPos > targetPos)
                motorHarvester.setPower(-0.2);
            else if (currentPos < targetPos)
                motorHarvester.setPower(0.2);
            try
            {
                sleep(50);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        motorHarvester.setPower(0);
        Log.d("DEBUG", "BucketThread.RunFinished");
    }

    public void InitializeBucket()
    {

        motorHarvester.getController().setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_WRITE);

        motorHarvester.setMode(DcMotorController.RunMode.RESET_ENCODERS);

    }
}
