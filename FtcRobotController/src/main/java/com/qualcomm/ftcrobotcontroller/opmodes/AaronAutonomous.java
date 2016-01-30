package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;


enum rrState {
    rrState1,
    rrState2,
    rrState3,
    rrState4
}

/**
 * Created by johnson on 1/4/16.
 * Edited RoboRaptors 1/22/2016
 * first comment
 */
public class AaronAutonomous extends OpMode {

    // ??
    private boolean v_warning_generated = false;

    // ??
    private String v_warning_message = "Can't map";

    private double LDPosition = 0.0; // Left drive position
    private double LDDelta = 0.0;    // Left drive delta
    private double LSPosition = 0.0; // Left Shoulder position
    private double LEOrigin = 0.0;   // Right Elbow Origin
    private double LSDelta = 0.0;    // Left Shoulder delta
    private double LSPower = 0.0;    // Left Shoulder delta
    private double LEPosition = 0.0; // Left Elbow position
    private double LEDelta = 0.0;    // Left Elbow Delta
    private double LWPosition = 0.0; // Left Wrist position
//    private double LWDelta = 0.0;    // Left Wrist Delta

    private double RDPosition = 0.0; // Right drive position
    private double RDDelta = 0.0;    // Right drive delta
    private double RSPosition = 0.0; // Right Shoulder position
    private double REOrigin = 0.0;   // Right Elbow Origin
    private double RSDelta = 0.0;    // Right Shoulder delta
    private double RSPower = 0.0;    // Left Shoulder delta
    private double REPosition = 0.0; // Right Elbow Position
    private double REDelta = 0.0;    // Right Elbow delta
    private double RWPosition = 0.0; // Right Wrist Position
//    private double RWDelta = 0.0;    // Right Wrist delta

    private rrState mystate;
    private boolean rrNewState;

    private DcMotor v_motor_left_drive;
    private DcMotor v_motor_right_drive;

    private DcMotor v_motor_right_shoulder;
    private DcMotor v_motor_right_elbow;
    private DcMotor v_motor_right_wrist;

    private DcMotor v_motor_left_shoulder;
    private DcMotor v_motor_left_elbow;
    private DcMotor v_motor_left_wrist;

    // Counter
    private int counter = 0;

        public AaronAutonomous() {

        }

        /**
         * Perform any actions that are necessary when the OpMode is enabled.
         * The system calls this member once when the OpMode is enabled.
         */
        @Override
        public void init() {

            mystate = rrState.rrState1;

            rrNewState = true;

            // Drive Positions
            LDPosition = LDDelta = 0.0;
            RDPosition = RDDelta = 0.0;

            // Shoulder Positions
            LSPosition = LSDelta = 0.0;
            RSPosition = RSDelta = 0.0;

            // Elbow Positions
            LEPosition = LEDelta = 0.0;
            REPosition = REDelta = 0.0;

            // Wrist Positions
            LWPosition = 0.0;
            RWPosition = 0.0;

            //
            // Use the hardwareMap to associate class members to hardware ports.
            //
            // Note that the names of the devices (i.e. arguments to the get method)
            // must match the names specified in the configuration file created by
            // the FTC Robot Controller (Settings-->Configure Robot).
            //
            // The variable below is used to provide telemetry data to a class user.
            //
            v_warning_generated = false;
            v_warning_message = "Can't map";


// Wheels
// Left
            try {
                v_motor_left_drive = hardwareMap.dcMotor.get("motorLD");
            }
            catch(Exception p_exeception) {
                m_warning_message("motorLD");
                DbgLog.msg(p_exeception.getLocalizedMessage());
                v_motor_left_drive = null;
            }

            if (v_motor_left_drive != null) {
                v_motor_left_drive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            }



// Right
            try {
                v_motor_right_drive = hardwareMap.dcMotor.get("motorRD");
                v_motor_right_drive.setDirection (DcMotor.Direction.REVERSE);
            }
            catch(Exception p_exeception) {
                m_warning_message("motorRD");
                DbgLog.msg(p_exeception.getLocalizedMessage());
                v_motor_right_drive = null;
            }

            if(v_motor_right_drive != null) {
                v_motor_right_drive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            }

// Shoulder Right
            try {
                v_motor_right_shoulder = hardwareMap.dcMotor.get("motorRS");
            }
            catch(Exception p_exeception) {
                m_warning_message("motorRS");
                DbgLog.msg(p_exeception.getLocalizedMessage());
                v_motor_right_shoulder = null;
            }

            if (v_motor_right_shoulder != null) {
                v_motor_right_shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                v_motor_right_shoulder.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            }


// Elbow Right
            try {
                v_motor_right_elbow = hardwareMap.dcMotor.get("motorRE");
            }
            catch(Exception p_exeception) {
                m_warning_message("motorRE");
                DbgLog.msg(p_exeception.getLocalizedMessage());
                v_motor_right_elbow = null;
            }

            if (v_motor_right_elbow != null) {
                v_motor_right_elbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                v_motor_right_elbow.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            }


// Wrist Right
            try {
                v_motor_right_wrist = hardwareMap.dcMotor.get("motorRW");
            }
            catch(Exception p_exeception) {
                m_warning_message("motorRW");
                DbgLog.msg(p_exeception.getLocalizedMessage());
                v_motor_right_wrist = null;
            }

            if (v_motor_right_wrist != null) {
                v_motor_right_wrist.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                v_motor_right_wrist.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//                v_motor_right_wrist.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            }

// Shoulder Left
            try {
                v_motor_left_shoulder = hardwareMap.dcMotor.get("motorLS");
            }
            catch(Exception p_exeception) {
                m_warning_message("motorLS");
                DbgLog.msg(p_exeception.getLocalizedMessage());
                v_motor_left_drive = null;
            }

            if (v_motor_left_shoulder != null) {
                v_motor_left_shoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                v_motor_left_shoulder.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            }

// Elbow Left
            try {
                v_motor_left_elbow = hardwareMap.dcMotor.get("motorLE");
            }
            catch(Exception p_exeception) {
                m_warning_message("motorLE");
                DbgLog.msg(p_exeception.getLocalizedMessage());
                v_motor_left_drive = null;
            }

            if (v_motor_left_elbow != null) {
                v_motor_left_elbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                v_motor_left_elbow.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            }



// Wrist Left
            try {
                v_motor_left_wrist = hardwareMap.dcMotor.get("motorLW");
            }
            catch(Exception p_exeception) {
                m_warning_message("motorLW");
                DbgLog.msg(p_exeception.getLocalizedMessage());
                v_motor_left_wrist = null;
            }

            if (v_motor_left_wrist != null) {
                v_motor_left_wrist.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                v_motor_left_wrist.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
//                v_motor_left_wrist.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

            }

            /**
             * Implement a state machine that controls the robot during
             * manual-operation.  The state machine uses gamepad input to transition
             * between states.
             *
             * The system calls this member repeatedly while the OpMode is running.
             */
            RSPosition = v_motor_right_shoulder.getCurrentPosition();
            REPosition = v_motor_right_elbow.getCurrentPosition();
            RWPosition = v_motor_right_wrist.getCurrentPosition();
            LSPosition = v_motor_left_shoulder.getCurrentPosition();
            LEPosition = v_motor_left_elbow.getCurrentPosition();
            LWPosition = v_motor_left_wrist.getCurrentPosition();

            REOrigin = REPosition;
            LEOrigin = REPosition;
            mystate = rrState.rrState1;
        }

        @Override
        public void loop() {

            counter = counter + 1;
            switch (mystate) {

                case rrState1:
                    if(rrNewState){
                        rrNewState = false;
                    }
                    if(gamepad2.x){
                        mystate = rrState.rrState2;
                        rrNewState = true;
                    }
                    break;
                case rrState2:
                    if(rrNewState){
                        rrNewState = false;
                    }
                    if(gamepad2.x){
                        mystate = rrState.rrState3;
                        rrNewState = true;
                    }
                    break;
                case rrState3:
                    if(rrNewState){
                        rrNewState = false;
                    }
                    if(gamepad2.x){
                        mystate = rrState.rrState4;
                        rrNewState = true;
                    }
                    break;
                case rrState4:
                    if(rrNewState){
                        rrNewState = false;
                    }
                    if(gamepad2.x){
                        mystate = rrState.rrState1;
                        rrNewState = true;if(gamepad2.x){
                            mystate = rrState.rrState2;
                            rrNewState = true;
                        }
                    }
                    break;
            }

// Wheel Power
// Left
            if (v_motor_left_drive != null) {
                v_motor_left_drive.setPower(scale_0_to_1(-gamepad1.left_stick_y));
            }
// Right
            if (v_motor_right_drive != null) {
                v_motor_right_drive.setPower(scale_0_to_1(-gamepad1.right_stick_y));
            }

// Shoulder Position
// Left
            /*  This code uses the FTC app PID
            LSDelta = 1.0 * -gamepad2.left_stick_y;
            LSPosition += LSDelta;
            if (v_motor_left_shoulder != null) {
                LSPower = 0.01 * (LSPosition - v_motor_left_shoulder.getCurrentPosition());
                if(LSPower < -1.0)
                {
                    LSPower = -1.0;
                }
                if(LSPower > 1.0)
                {
                    LSPower = 1.0;
                }
                v_motor_left_shoulder.setPower(LSPower);
            }
            */



            LSDelta = 25.0 * scale_0_to_1(gamepad2.left_stick_y);
            if (!v_motor_left_shoulder.isBusy()) {
                LSPosition += LSDelta;

            }

            if (v_motor_left_shoulder != null) {
                v_motor_left_shoulder.setTargetPosition((int)LSPosition);
                v_motor_left_shoulder.setPower(0.4);
            }
// Right
            /*  This code uses the custom PID
            RSDelta = 1.0 * -gamepad2.right_stick_y;
            RSPosition += RSDelta;
            if (v_motor_right_shoulder != null) {
                RSPower = 0.01 * (RSPosition - v_motor_right_shoulder.getCurrentPosition());
                if(RSPower < -1.0)
                {
                    RSPower = -1.0;
                }
                if(RSPower > 1.0)
                {
                    RSPower = 1.0;
                }
                v_motor_right_shoulder.setPower(RSPower);
            }
            */
            RSDelta = 25.0 * scale_0_to_1(gamepad2.right_stick_y);
            if (!v_motor_right_shoulder.isBusy()) {
                RSPosition += RSDelta;
            }
            if (v_motor_right_shoulder != null) {
                v_motor_right_shoulder.setTargetPosition((int)RSPosition);
                v_motor_right_shoulder.setPower(0.4);
            }


// Elbow Position
// Left
            LEDelta = 70.0 * scale_0_to_1(-gamepad2.left_trigger);
            if (!v_motor_left_elbow.isBusy()) {
                if(gamepad2.left_bumper) {
                    LEPosition -= LEDelta;
                }
                else {
                    LEPosition += LEDelta;
                }
            }
            if(LEPosition > LEOrigin) {
                LEPosition = LEOrigin;
            }

            if (v_motor_left_elbow != null) {
                v_motor_left_elbow.setTargetPosition((int)LEPosition);
                v_motor_left_elbow.setPower(0.3);
            }
// Right
            REDelta = 70.0 * scale_0_to_1(gamepad2.right_trigger);
            if (!v_motor_right_elbow.isBusy()) {
                if(gamepad2.right_bumper) {
                    REPosition -= REDelta;
                }
                else {
                    REPosition += REDelta;
                }
            }
            if(REPosition < REOrigin) {
                REPosition = REOrigin;
            }
            if (v_motor_right_elbow != null) {
                v_motor_right_elbow.setTargetPosition((int)REPosition);
                v_motor_right_elbow.setPower(0.3);
            }



// Wrist Power
// Left
//            LWDelta = 5.0 * -gamepad2.left_trigger;
            if(gamepad2.y) {
                LWPosition += 10.0;
            }

            if(gamepad2.b) {
                LWPosition -= 10.0;
            }

            if (v_motor_left_wrist != null) {
            v_motor_left_wrist.setTargetPosition((int)LWPosition);
            v_motor_left_wrist.setPower(0.1);
        }

// Right
//            RWDelta = 5.0 * -gamepad2.right_trigger;
            if(gamepad2.x) {
                RWPosition += 10.0;
            }

            if(gamepad2.a) {
                RWPosition -= 10.0;
            }

            if (v_motor_right_wrist != null) {
            v_motor_right_wrist.setTargetPosition((int)RWPosition);
            v_motor_right_wrist.setPower(0.1);
        }


            update_telemetry(); // Update common telemetry
            update_gamepad_telemetry();

        } // end of loop

//--------------------------------------------------------------------------
        //
        // scale_motor_power
        //
        /**
         * Scale the joystick input using a nonlinear algorithm.
         */
        private double scale_0_to_1(double input ) {

            double speed;
            boolean negative = false;
            int i;
            double[][] sArray;
            sArray = new double[][]{
                    {0.0,0.0},
                    {0.1,0.01},
                    {0.2,0.03},
                    {0.3,0.06},
                    {0.4,0.10},
                    {0.5,0.15},
                    {0.6,0.23},
                    {0.7,0.35},
                    {0.8,0.55},
                    {0.9,0.78},
                    {1.0,1.0}};

            if(input < 0.0) {
                negative = true;
                input = -input;
            }

            i = 0;
            while(i < 10) {
                if(sArray[i][0]>input) {
                    break;
                }
                i++;
            }

            speed=(((input - sArray[i-1][0]) / (0.1)) * ((sArray[i][1] - sArray[i-1][1]) + sArray[i-1][1]));

            if(negative) {
                return -speed;
            }
            else {
                return speed;
            }
        }


        //--------------------------------------------------------------------------
        //
        // update_telemetry
        //
        /**
         * Update the telemetry with current values from the base class.
         */
        public void update_telemetry () {

            if (a_warning_generated ()) {
                set_first_message (a_warning_message ());
            }

            //
            // Send telemetry data to the driver station.
            //
            telemetry.addData
                    ( "t01"
                            , "Left Drive: "
                                    + get_left_drive_power ()
                                    + ", "
                                    + get_left_encoder_count ()
                    );
            telemetry.addData
                    ( "t02"
                            , "LD Position: "
                                    + LDPosition
                    );
            telemetry.addData
                    ( "t03"
                            , "Right Drive: "
                                    + get_right_drive_power ()
                                    + ", "
                                    + get_right_encoder_count ()
                    );
            telemetry.addData
                    ( "t04"
                            , "RD Position: "
                                    + RDPosition
                    );
//        telemetry.addData
//                ( "t05"
//                        , "Run time: "
//                                + this.getRuntime()
//                );
//        telemetry.addData
//                ( "t06"
//                        , "Run time Delta: "
//                                + LDRuntimeDelta
//                );
//
//
//        telemetry.addData
//                ( "t07"
//                        , "LDPower: "
//                                + LDPower
//
//
//
// //
//
//
// //                );
            switch (mystate) {

                case rrState1:
                    telemetry.addData ( "t08" , "rrState: " + "rrState1");
                    break;
                case rrState2:
                    telemetry.addData ( "t08" , "rrState: " + "rrState2"); break;
                case rrState3:
                    telemetry.addData ( "t08" , "rrState: " + "rrState3");
                    break;
                case rrState4:
                    telemetry.addData ( "t08" , "rrState: " + "rrState4");
                    break;
            }


            telemetry.addData
                    ( "RS"
                            , "Right Shoulder: "
                                    + get_right_shoulder_encoder_count ()
                    );
            telemetry.addData
                    ( "RD"
                            , "RS Delta: "
                                    + RSDelta
                    );
            telemetry.addData
                    ( "RP"
                            , "RS Pos: "
                                    + RSPosition
                    );

            telemetry.addData
                    ( "RE"
                            , "Right Elbow: "
                                    + get_right_elbow_encoder_count ()
                    );
            telemetry.addData
                    ( "RW"
                            , "Right Wrist: "
                                    + get_right_wrist_encoder_count ()
                    );

            telemetry.addData
                    ( "LS"
                            , "Left Shoulder: "
                                    + get_left_shoulder_encoder_count ()
                    );
            telemetry.addData
                    ( "LD"
                            , "LS Delta: "
                                    + LSDelta
                    );
            telemetry.addData
                    ( "LP"
                            , "LS Pos: "
                                    + LSPosition
                    );

            telemetry.addData
                    ( "LE"
                            , "Left Elbow: "
                                    + get_left_elbow_encoder_count ()
                    );
            telemetry.addData
                    ( "LW"
                            , "Left Wrist: "
                                    + get_left_wrist_encoder_count ()
                    );

        } // update_telemetry

        //--------------------------------------------------------------------------
        //
        // update_gamepad_telemetry
        //
        /**
         * Update the telemetry with current gamepad readings.
         */
        public void update_gamepad_telemetry () {

            //
            // Send telemetry data concerning gamepads to the driver station.
            //
            telemetry.addData ("05", "GP1 Left: " + -gamepad1.left_stick_y);
            telemetry.addData ("06", "GP1 Right: " + -gamepad1.right_stick_y);
            telemetry.addData ("07", "GP2 Left: " + -gamepad2.left_stick_y);
            telemetry.addData ("08", "GP2 Right: " + -gamepad2.right_stick_y);
            telemetry.addData ("09", "GP2 X: " + gamepad2.x);
            telemetry.addData ("10", "GP2 Y: " + gamepad2.y);
            telemetry.addData ("11", "GP1 LT: " + gamepad1.left_trigger);
            telemetry.addData ("12", "GP1 RT: " + gamepad1.right_trigger);

        } // update_gamepad_telemetry

        //--------------------------------------------------------------------------
        //
        // set_first_message
        //
        /**
         * Update the telemetry's first message with the specified message.
         */
        public void set_first_message (String p_message) {
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

            if (v_warning_generated) {
                v_warning_message += ", ";
            }
            v_warning_generated = true;
            v_warning_message += p_exception_message;
        } // m_warning_message

        /**
         * Access the right encoder's count.
         */
        int get_right_encoder_count () {

            int l_return = 0;
            if (v_motor_right_drive != null) {
                l_return = v_motor_right_drive.getCurrentPosition ();
            }
            return l_return;
        } // get_right_encoder_count

        int get_right_shoulder_encoder_count () {

            int l_return = 0;

            if (v_motor_right_shoulder != null) {
                l_return = v_motor_right_shoulder.getCurrentPosition ();
            }

            return l_return;

        } // get_right_shoulder_encoder_count

        int get_right_elbow_encoder_count () {

            int l_return = 0;

            if (v_motor_right_elbow != null) {
                l_return = v_motor_right_elbow.getCurrentPosition ();
            }

            return l_return;

        } // get_right_elbow_encoder_count

        int get_right_wrist_encoder_count () {
            int l_return = 0;

            if (v_motor_right_wrist != null) {
                l_return = v_motor_right_wrist.getCurrentPosition ();
            }

            return l_return;

        } // get_right_wrist_encoder_count


//--------------------------------------------------------------------------
        //
        // get_left_drive_power
        //

        /**
         * Access the left encoder's count.
         */
        int get_left_encoder_count () {

            int l_return = 0;

            if (v_motor_left_drive != null) {
                l_return = v_motor_left_drive.getCurrentPosition ();
            }

            return l_return;

        } // get_left_encoder_count

        /**
         * Access the left drive motor's power level.
         */
        double get_left_drive_power () {

            double l_return = 0.0;

            if (v_motor_left_drive != null) {
                l_return = v_motor_left_drive.getPower();
            }

            return l_return;
        } // get_left_drive_power


        int get_left_shoulder_encoder_count () {

            int l_return = 0;

            if (v_motor_left_shoulder != null) {
                l_return = v_motor_left_shoulder.getCurrentPosition ();
            }

            return l_return;
        } // get_left_shoulder_encoder_count

        int get_left_elbow_encoder_count () {

            int l_return = 0;

            if (v_motor_left_elbow != null) {
                l_return = v_motor_left_elbow.getCurrentPosition ();
            }

            return l_return;
        } // get_left_elbow_encoder_count

        int get_left_wrist_encoder_count () {

            int l_return = 0;

            if (v_motor_left_wrist != null) {
                l_return = v_motor_left_wrist.getCurrentPosition ();
            }

            return l_return;
        } // get_left_wrist_encoder_count

//--------------------------------------------------------------------------
        //
        // get_right_drive_power
        //
        /**
         * Access the right drive motor's power level.
         */
        double get_right_drive_power () {
            double l_return = 0.0;
            if (v_motor_right_drive != null) {
                l_return = v_motor_right_drive.getPower();
            }
            return l_return;
        } // get_right_drive_power

}
