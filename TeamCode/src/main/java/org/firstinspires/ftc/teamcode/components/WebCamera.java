/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.components;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

public class WebCamera extends BotComponent {

    public String cameraName;

    private static final String VUFORIA_KEY = "AYb6Z43/////AAABmUDTPvzNUErvv+V5mxyyLy5xIbXbTnWaz/luHdMrGjmXWTa49gQSiDxm1hnzzQVmlkAh/5PCeNEicf28nm7T31+td8OKFeU4C4iu/aQ7HXEv74/NRf38ixE2iYmLLPrPApWBKRrUnuz7v4wsZdXZwIZzgHI0S0t4T4cX34ppylT72P+GXG9U48f7qr5x0KZpn+WgkiSMVQ2r0KvSGTAvU7Sx5y69teWPt+NdHwkes7vpnOQyOXn9NvVSuDgByMcGKbTEScLa9L4zyyRLrBIK9fSIxrRFDNbVGojzcu8+70TuZuyjx+2u/9OzuK4mMDdpqL/46aXinDXqNuSj/BZsPcDCaPsG7R5oxpp9zdfhIwiO";
    public VuforiaLocalizer vuforia;
    public VuforiaLocalizer.Parameters parameters;

    /**
     * @see #captureFrameToFile()
     */
    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;


    /* Constructor */
    public WebCamera() {

    }

    public WebCamera(Logger aLogger, OpMode aOpMode, String aCameraName) {
        super(aLogger, aOpMode);

        try {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            cameraName = aCameraName;
            parameters.cameraName = aOpMode.hardwareMap.get(WebcamName.class, cameraName);

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            isAvailable = true;
            logger.logDebug("WebCamera","isAvailable:%b", isAvailable);

        } catch (VuforiaException | NullPointerException err) {
            logger.logErr("WebCamera","Error Starting WebCamera:", err);
            opMode.telemetry.addData("Error Starting WebCamera", err.getMessage());
        }


    }


    public void initForNavigation() {

        try {
            logger.logDebug("initForNavigation", cameraName);

            int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            /*
             * Retrieve the webCamera we are to use.
             */
            WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, cameraName);
            parameters.cameraName = webcamName;


            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            logger.logDebug("initForNavigation", "debug #5");
        } catch (VuforiaException err) {
            isAvailable = false;
            logger.logErr("initForNavigation", "Error initializing", err.getMessage());
        }

        if (isAvailable) { enableCaptureFrameToFile(); }

    }


    void enableCaptureFrameToFile() {
        /**
         * Because this opmode processes frames in order to write them to a file, we tell Vuforia
         * that we want to ensure that certain frame formats are available in the {@link Frame}s we
         * see.
         */
        vuforia.enableConvertFrameToBitmap();

        /** @see #captureFrameToFile() */
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
    }

    /**
     * Sample one frame from the Vuforia stream and write it to a .PNG image file on the robot
     * controller in the /sdcard/FIRST/data directory. The images can be downloaded using Android
     * Studio's Device File Explorer, ADB, or the Media Transfer Protocol (MTP) integration into
     * Windows Explorer, among other means. The images can be useful during robot design and calibration
     * in order to get a sense of what the webCamera is actually seeing and so assist in webCamera
     * aiming and alignment.
     */
    void captureFrameToFile() {
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
                    try {
                        FileOutputStream outputStream = new FileOutputStream(file);
                        try {
                            bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                        } finally {
                            outputStream.close();
                            opMode.telemetry.log().add("captured %s", file.getName());
                        }
                    } catch (IOException e) {
                        logger.logErr("captureFrameToFile", "error",e);
                    }
                }
            }
        }));
    }


}



