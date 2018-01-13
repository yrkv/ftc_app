package org.firstinspires.ftc.teamcode.testing;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.drawable.BitmapDrawable;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.*;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.OpMode8696;

import java.nio.ByteBuffer;

/**
 * Created by USER on 10/4/2017.
 */

//@Autonomous(name="Testing Vision", group ="Test")
public class TestBlockVision extends OpMode8696 {


    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();

        telemetry.addData("bits per pixel", Vuforia.getBitsPerPixel(PIXEL_FORMAT.RGB565));
        telemetry.update();

        waitForStart();

//        relicTrackables.activate();

        int a = 0;

        while (opModeIsActive()) {
            idle();
            while (vuforia.getFrameQueue().size() > 0) {
                telemetry.addData(">", a);
                VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
                Image capture = frame.getImage(0);

                if (capture.getFormat() != PIXEL_FORMAT.RGB565) {
                    frame.close();
                    continue;
                }

                telemetry.addData(">", capture.getWidth() + ", " + capture.getHeight() + ", " + capture.getPixels().capacity());
                telemetry.addData(">", capture.getFormat());


                a++;

                capture.getPixels().rewind();
                ByteBuffer bb = capture.getPixels().duplicate();

                Bitmap bitmap = Bitmap.createBitmap(capture.getWidth(), capture.getHeight(), Bitmap.Config.RGB_565);

                bitmap.copyPixelsFromBuffer(bb);


                Bitmap resized = getResizedBitmap(bitmap, bitmap.getWidth() / 10, bitmap.getHeight() / 10);

                float[] topRight = new float[3];
                Color.colorToHSV(resized.getPixel(0, 0), topRight);

                telemetry.addData(">", topRight[0] + ", " + topRight[1] + ", " + topRight[2]);

                int[][] containsCube = new int[resized.getWidth()][resized.getHeight()];

                int couldContainCube = 0;

                int regions = 0;

                for (int i = 3; i < resized.getWidth() - 3; i++) {
                    for (int j = 3; j < resized.getHeight() - 2 - 3; j++) {
                        float[] top = new float[3];
                        Color.colorToHSV(resized.getPixel(i, j), top);
                        float[] bot = new float[3];
                        Color.colorToHSV(resized.getPixel(i, j+2), bot);

                        if (top[2] >= 1.6 * bot[2] && top[2] >= 0.3 && Math.abs(top[1] - bot[1]) <= 0.1) {
                            couldContainCube++;
                            int newRegion = 0;
                            loop: for (int x = i - 3; x < i + 3; x++) {
                                for (int y = j - 3; y < j + 3; y++) {
                                    if (containsCube[x][y] > 0) {
                                        newRegion = containsCube[x][y];
                                        break loop;
                                    }
                                }
                            }
                            if (newRegion == 0) {
                                regions++;
                                for (int x = i - 3; x < i + 3; x++) {
                                    for (int y = j - 3; y < j + 3; y++) {
                                        containsCube[x][y] = regions;
                                    }
                                }
                            } else {
                                for (int x = i - 3; x < i + 3; x++) {
                                    for (int y = j - 3; y < j + 3; y++) {
                                        containsCube[x][y] = newRegion;
                                    }
                                }
                            }

                        }
                    }
                }

                for (int i = 0; i < containsCube.length; i++) {
                    for (int j = 0; j < containsCube[0].length; j++) {

                    }
                }

                telemetry.addData(">", couldContainCube);
                telemetry.addData(">", regions);

                capture = null;
                bitmap = null;
                resized = null;
                containsCube = null;

                telemetry.update();

                frame.close();

                break;
            }
        }

    }


    private Bitmap getResizedBitmap(Bitmap bm, int newWidth, int newHeight) {
        int width = bm.getWidth();
        int height = bm.getHeight();
        float scaleWidth = ((float) newWidth) / width;
        float scaleHeight = ((float) newHeight) / height;

        Matrix matrix = new Matrix();

        matrix.postScale(scaleWidth, scaleHeight);

        return Bitmap.createBitmap(
                bm, 0, 0, width, height, matrix, false);
    }
}
