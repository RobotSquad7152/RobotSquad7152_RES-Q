package RobotSquad;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import com.qualcomm.robotcore.robocol.Telemetry;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;

import java.io.ByteArrayOutputStream;

/**
 * Created by Tony on 12/21/2015.
 */
public class RSCamera {
    private Camera camera;
    public CameraPreview preview;
    public Bitmap image;
    private int width;
    private int height;
    private YuvImage yuvImage = null;
    private int looped = 0;
    private String data;
    private Context context;
    private Telemetry telemetry;

    public RSCamera(Context context, Telemetry telemetry) {
        this.context = context;
        this.telemetry = telemetry;
    }


    public void init() {
        camera = ((FtcRobotControllerActivity) context).camera;

        camera.setPreviewCallback(previewCallback);


        Camera.Parameters parameters = camera.getParameters();

        data = parameters.flatten();


        ((FtcRobotControllerActivity) context).initPreviewLinear(camera, this, previewCallback);
    }



    private int red(int pixel) {
        return (pixel >> 16) & 0xff;
    }

    private int green(int pixel) {
        return (pixel >> 8) & 0xff;
    }

    private int blue(int pixel) {
        return pixel & 0xff;
    }

    private Camera.PreviewCallback previewCallback = new Camera.PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera)
        {
            Camera.Parameters parameters = camera.getParameters();
            width = parameters.getPreviewSize().width;
            height = parameters.getPreviewSize().height;
            yuvImage = new YuvImage(data, ImageFormat.NV21, width, height, null);
            looped += 1;
        }
    };

    private void convertImage() {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, width, height), 0, out);
        byte[] imageBytes = out.toByteArray();
        image = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
    }



    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    private int highestColor(int red, int green, int blue) {
        int[] color = {red,green,blue};
        int value = 0;
        for (int i = 1; i < 3; i++) {
            if (color[value] < color[i]) {
                value = i;
            }
        }
        return value;
    }


    public int calculateMiddle()
    {
        int middle = 0;
        if (yuvImage != null) {
            int redValue = 0;
            int blueValue = 0;
            int greenValue = 0;

            convertImage();

            int blueCounter = 0;
            int redCounter = 0;
            int blueEndCounter = 0;
            int redEndCounter = 0;
            int blueTarget = 100;
            int redTarget = 170;
            int blueStart = 0;
            int redStart = 0;
            int blueEnd = 0;
            int redEnd = 0;
            int y = height/2;
            for (int x = 0 ; x < width; x++) {
                int pixel = image.getPixel(x, y);
                redValue = red(pixel);
                blueValue = blue(pixel);

                if(blueCounter <= 5 && redCounter <= 5)
                {
                    if(redValue >= redTarget)
                    {
                        blueCounter = 0;
                        redCounter++;
                        if(redCounter == 5)
                        {
                            redStart = x - 4;
                        }
                    }
                    else if(blueValue >= blueTarget)
                    {
                        redCounter = 0;
                        blueCounter++;
                        if(blueCounter == 5)
                        {
                            blueStart = x - 4;
                        }
                    }
                }
                if(blueCounter >= 5)
                {
                    if(blueValue < blueTarget)
                    {
                        blueEndCounter++;
                        if(blueEndCounter == 5)
                        {
                            blueCounter = 0;
                            blueEnd = x - 5;
                        }
                    }
                    else
                    {
                        blueEndCounter = 0;
                    }
                    if(x == width - 1)
                    {
                        blueEnd = x;
                    }
                }
                if(redCounter >= 5)
                {
                    if(redValue < redTarget)
                    {
                        redEndCounter++;
                        if(redEndCounter == 5) {
                            redCounter = 0;
                            redEnd = x - 5;
                        }
                    }
                    else
                    {
                        redEndCounter = 0;
                    }
                    if(x == width - 1)
                    {
                        redEnd = x;
                    }
                }


            }

            if((redStart < blueStart) && (redEnd > 0))
            {
                middle = redEnd + ((blueStart - redEnd)/2);
            }
            else if((blueStart < redStart) && (blueEnd > 0))
            {
                middle = blueEnd + ((redStart - blueEnd)/2);
            }
//            int color = highestColor(redValue, greenValue, blueValue);
//            String colorString = "";
//            switch (color) {
//                case 0:
//                    colorString = "RED";
//                    break;
//                case 1:
//                    colorString = "GREEN";
//                    break;
//                case 2:
//                    colorString = "BLUE";
//            }
            telemetry.addData("1Red Start:", " Red Start: " + redStart);
            telemetry.addData("2Red End:", " Red End: " + redEnd);
            telemetry.addData("3Blue Start", "Blue Start: " + blueStart);
            telemetry.addData("4Blue End", "Blue End: " + blueEnd);
            telemetry.addData("Middle", "Middle: " + middle);


        }
//        telemetry.addData("Looped","Looped " + Integer.toString(looped) + " times");

//        Log.d("Looped:", "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Looped " + looped);
//        Log.d("DEBUG:",data);
        return middle;
    }

}
