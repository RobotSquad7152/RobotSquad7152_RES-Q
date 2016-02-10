package RobotSquad;

import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Tony on 9/20/2015.
 */
public class GyroThread extends Thread
{


    double currentHeading;
    double calibratedGyroRotation;
    GyroSensor gyro;

    public GyroThread(GyroSensor g)
    {
        currentHeading = 0;
        gyro = g;
        calibratedGyroRotation = 0;


    }


    public double getCurrentHeading()
    {
        return currentHeading;
    }

    public void setCurrentHeading(double currentHeading)
    {
        this.currentHeading = currentHeading;
    }


    public void calibrategyro()
    {
        double totalGyroReadings = 0;

        //set the millisecond timer
        //    long millisStart = Calendar.getInstance().getTimeInMillis();


        // Take 1000 readings and average them out
        for (int i = 0; i < 500; i++)
        {
            // Wait until 3ms has passed
            //   while (Calendar.getInstance().getTimeInMillis() < millisStart + 3)
            //       try {
            try
            {
                sleep(5);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            //     } catch (InterruptedException e) {
            //          e.printStackTrace();
            //      }

            // millisStart = Calendar.getInstance().getTimeInMillis();
            totalGyroReadings += gyro.getRotation();


        }
        calibratedGyroRotation = totalGyroReadings / 500;
    }

    public void run()
    {
        Stopwatch stopwatch = new Stopwatch();
        do
        {


            while (stopwatch.elapsedTime() < 5)
                try
                {
                    sleep(1);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }

            // Reset the timer
            stopwatch.Resetstopwatch();

            // calculate turn based on gyro reading divided by the time taken since the last reading (6ms)
            currentHeading = currentHeading + (gyro.getRotation() - calibratedGyroRotation) * .005;
        } while (true);


    }
}
