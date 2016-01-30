
package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.CameraPreview;
import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.ByteArrayOutputStream;


enum nrState {
    rrState1,
    rrState2,
    rrState3,
    rrState4
}





/**
 * Created by johnson on 1/4/16.
 * first comment
 */
public class NoRobot extends OpMode {

    // ??
    private boolean v_warning_generated = false;

    // ??
    private String v_warning_message = "Can't map";

    private nrState mystate;
    private boolean rrNewState;

    // Counter

        public NoRobot() {

        }


    public Camera camera;
    public CameraPreview preview;

    public int width;
    public int height;
    public YuvImage yuvImage = null;

    volatile private boolean imageReady = false;

    private int looped = 0;
    private String data;
    private int ds = 1; // downsampling parameter

    public Camera.PreviewCallback previewCallback = new Camera.PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera) {
            try {
                Camera.Parameters parameters = camera.getParameters();
                width = parameters.getPreviewSize().width;
                height = parameters.getPreviewSize().height;
                yuvImage = new YuvImage(data, ImageFormat.NV21, width, height, null);
                imageReady = true;
                looped += 1;
            } catch (Exception e) {

            }
        }
    };

    public void setCameraDownsampling(int downSampling) {
        ds = downSampling;
    }

    public boolean imageReady() {
        return imageReady;
    }

    public boolean isCameraAvailable() {
        int cameraId = -1;
        Camera cam = null;
        int numberOfCameras = Camera.getNumberOfCameras();
        for (int i = 0; i < numberOfCameras; i++) {
            Camera.CameraInfo info = new Camera.CameraInfo();
            Camera.getCameraInfo(i, info);
            if (info.facing == Camera.CameraInfo.CAMERA_FACING_BACK) { // Camera.CameraInfo.CAMERA_FACING_FRONT or BACK
                cameraId = i;
                break;
            }
        }
        try {
            cam = Camera.open(cameraId);
        } catch (Exception e) {
            Log.e("Error", "Camera Not Available!");
            return false;
        }
        if(cam != null) {
            cam.release();
        }
        cam = null;
        return true;
    }

    public Camera openCamera(int cameraInfoType) {
        int cameraId = -1;
        Camera cam = null;
        int numberOfCameras = Camera.getNumberOfCameras();
        for (int i = 0; i < numberOfCameras; i++) {
            Camera.CameraInfo info = new Camera.CameraInfo();
            Camera.getCameraInfo(i, info);
            if (info.facing == cameraInfoType) { // Camera.CameraInfo.CAMERA_FACING_FRONT or BACK
                cameraId = i;
                break;
            }
        }
        try {
            Log.d("Debug", "Trying Camera" + cameraId);
//            cam = Camera.open(cameraId);
            cam = android.hardware.Camera.open(cameraId);
        } catch (Exception e) {
            Log.e("Error", "Can't Open Camera");
        }
        return cam;
    }

    public int red(int pixel) {
        return (pixel >> 16) & 0xff;
    }

    public int green(int pixel) {
        return (pixel >> 8) & 0xff;
    }

    public int blue(int pixel) {
        return pixel & 0xff;
    }

    public int gray(int pixel) {
        return (red(pixel) + green(pixel) + blue(pixel));
    }

    public int highestColor(int red, int green, int blue) {
        int[] color = {red, green, blue};
        int value = 0;
        for (int i = 1; i < 3; i++) {
            if (color[value] < color[i]) {
                value = i;
            }
        }
        return value;
    }

    public Bitmap convertYuvImageToRgb(YuvImage yuvImage, int width, int height, int downSample) {
        Bitmap rgbImage;
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, width, height), 0, out);
        byte[] imageBytes = out.toByteArray();

        BitmapFactory.Options opt;
        opt = new BitmapFactory.Options();
        opt.inSampleSize = downSample;

        rgbImage = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length, opt);
        return rgbImage;
    }

    public void startCamera() {
        camera = openCamera(Camera.CameraInfo.CAMERA_FACING_BACK);

        camera.setPreviewCallback(previewCallback);

        Camera.Parameters parameters = camera.getParameters();

        width = parameters.getPreviewSize().width / ds;
        height = parameters.getPreviewSize().height / ds;
        parameters.setPreviewSize(width, height);

        camera.setParameters(parameters);

        data = parameters.flatten();

        if (preview == null) {
            ((FtcRobotControllerActivity) hardwareMap.appContext).NRinitPreview(camera, this, previewCallback);
        }
    }

    public void stopCamera() {
        if (camera != null) {
            if (preview != null) {
                ((FtcRobotControllerActivity) hardwareMap.appContext).NRremovePreview(this);
                preview = null;
            }
            camera.stopPreview();
            camera.setPreviewCallback(null);
            if(camera != null) {
                camera.release();
            }
            camera = null;
        }
    }


        /**
         * Perform any actions that are necessary when the OpMode is enabled.
         * The system calls this member once when the OpMode is enabled.
         */
        @Override
        public void init() {

            mystate = nrState.rrState1;
            rrNewState = true;

            v_warning_generated = false;
            v_warning_message = "Can't map";
            mystate = nrState.rrState1;
            startCamera();
        }

        @Override
        public void loop()
        {

            switch (mystate) {

                case rrState1:
                    if(rrNewState){
                        rrNewState = false;
                    }
                    if(gamepad2.x){
                        mystate = nrState.rrState2;
                        rrNewState = true;
                        while(gamepad2.x)
                            ;
                    }
                    break;
                case rrState2:
                    if(rrNewState){
                        rrNewState = false;
                    }
                    if(gamepad2.x){
                        mystate = nrState.rrState3;
                        rrNewState = true;
                        while(gamepad2.x)
                            ;
                    }
                    break;
                case rrState3:
                    if(rrNewState){
                        rrNewState = false;
                    }
                    if(gamepad2.x){
                        mystate = nrState.rrState4;
                        rrNewState = true;
                        while(gamepad2.x)
                            ;
                    }
                    break;
                case rrState4:
                    if(rrNewState){
                        rrNewState = false;
                    }
                    if(gamepad2.x){
                        mystate = nrState.rrState1;
                        rrNewState = true;
                        while(gamepad2.x)
                            ;
                    }
                    break;

            }
            //
            // Send telemetry data to the driver station.
            //
            update_telemetry(); // Update common telemetry
            update_gamepad_telemetry();

        } // end of loop


    @Override
    public void stop() {
        stopCamera();

    }

        //--------------------------------------------------------------------------
        //
        // update_telemetry
        //
        /**
         * Update the telemetry with current values from the base class.
         */
        public void update_telemetry ()

        {
            if (a_warning_generated ())
            {
                set_first_message (a_warning_message ());
            }
            switch (mystate) { 
                case rrState1:
                    telemetry.addData ( "t08" , "nrState: " + "rrState1");
                    break;
                case rrState2:
                    telemetry.addData ( "t08" , "nrState: " + "rrState2");
                    break;
                case rrState3:
                    telemetry.addData ( "t08" , "nrState: " + "rrState3");
                    break;
                case rrState4:
                    telemetry.addData ( "t08" , "nrState: " + "rrState4");
                    break;
            }


        } // update_telemetry

        //--------------------------------------------------------------------------
        //
        // update_gamepad_telemetry
        //
        /**
         * Update the telemetry with current gamepad readings.
         */
        public void update_gamepad_telemetry ()

        {
            //
            // Send telemetry data concerning gamepads to the driver station.
            //
            telemetry.addData ("05", "GP1 Left: " + -gamepad1.left_stick_y);
            telemetry.addData ("06", "GP1 Right: " + -gamepad1.right_stick_y);
            telemetry.addData ("07", "GP2 Left: " + -gamepad2.left_stick_y);
            telemetry.addData ("07", "GP2 Right: " + -gamepad2.right_stick_y);
            telemetry.addData ("08", "GP2 X: " + gamepad2.x);
            telemetry.addData ("09", "GP2 Y: " + gamepad2.y);
            telemetry.addData ("10", "GP1 LT: " + gamepad1.left_trigger);
            telemetry.addData ("11", "GP1 RT: " + gamepad1.right_trigger);

        } // update_gamepad_telemetry

        //--------------------------------------------------------------------------
        //
        // set_first_message
        //
        /**
         * Update the telemetry's first message with the specified message.
         */
        public void set_first_message (String p_message)

        {
            telemetry.addData ( "00", p_message);

        } // set_first_message



        //--------------------------------------------------------------------------
        //
        // a_warning_generated
        //
        /**
         * Access whether a warning has been generated.
         */
        boolean a_warning_generated () {
            return v_warning_generated;
        } // a_warning_generated

        //--------------------------------------------------------------------------
        //
        // a_warning_message
        //
        /**
         * Access the warning message.
         */
        String a_warning_message () {
            return v_warning_message;
        } // a_warning_message

        //--------------------------------------------------------------------------
        //
        // m_warning_message
        //
        /**
         * Mutate the warning message by ADDING the specified message to the current
         * message; set the warning indicator to true.
         *
         * A comma will be added before the specified message if the message isn't
         * empty.
         */
        void m_warning_message (String p_exception_message) {
            if (v_warning_generated)
            {
                v_warning_message += ", ";
            }
            v_warning_generated = true;
            v_warning_message += p_exception_message;
        } // m_warning_message
}
