package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;



/**
 * Created by johnson on 12/22/15.
 */

public class rrEncoder extends OpMode {


    /**
     * Constructor
     */
    public rrEncoder() {

    }

    //--------------------------------------------------------------------------
    //
    // init
    //

    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     * The system calls this member once when the OpMode is enabled.
     */
    @Override
    public void init()
    {

        counter = 0;
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
        v_warning_message = "Can't map; ";

        //
        // Connect the drive wheel motors.
        //
        // The direction of the right motor is reversed, so joystick inputs can
        // be more generically applied.
        //
        try
        {
            v_motor_left_drive = hardwareMap.dcMotor.get("motorLD");
        }
        catch(Exception p_exeception)
        {
            m_warning_message("motorLD");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            v_motor_left_drive = null;
        }

        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }



        try
        {
            v_motor_right_drive = hardwareMap.dcMotor.get("motorRD");
            v_motor_right_drive.setDirection (DcMotor.Direction.REVERSE);
        }
        catch(Exception p_exeception)
        {
            m_warning_message("motorRD");
            DbgLog.msg(p_exeception.getLocalizedMessage());
            v_motor_right_drive = null;
        }

        if(v_motor_right_drive != null)
        {
            v_motor_right_drive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }



        //
        // Connect the arm motor.
        //
/*
        try
        {
            v_motor_left_arm = hardwareMap.dcMotor.get ("left_arm");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_arm");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_left_arm = null;
        }
*/
        //
        // Connect the servo motors.
        //
        // Indicate the initial position of both the left and right servos.  The
        // hand should be halfway opened/closed.
        //

/*        double l_hand_position = 0.5;

        try
        {
            v_servo_left_hand = hardwareMap.servo.get ("left_hand");
            v_servo_left_hand.setPosition (l_hand_position);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_hand");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_left_hand = null;
        }
*/


        /**
         * Implement a state machine that controls the robot during
         * manual-operation.  The state machine uses gamepad input to transition
         * between states.
         *
         * The system calls this member repeatedly while the OpMode is running.
         */
    }

    @Override
    public void loop()
    {

        counter = counter + 1;
        //----------------------------------------------------------------------
        //
        // DC Motors
        //
        // Obtain the current values of the joystick controllers.
        //
        // Note that x and y equal -1 when the joystick is pushed all of the way
        // forward (i.e. away from the human holder's body).
        //
        // The clip method guarantees the value never exceeds the range +-1.
        //
        // The DC motors are scaled to make it easier to control them at slower
        // speeds.
        //
        // The setPower methods write the motor power values to the DcMotor
        // class, but the power levels aren't applied until this method ends.
        //

        //
        // Manage the drive wheel motors.
        //
        double l_left_drive_power = scale_0_to_1(-gamepad1.left_stick_y);
        double l_right_drive_power = scale_0_to_1(-gamepad1.right_stick_y);

        if (v_motor_left_drive != null) {
            v_motor_left_drive.setPower(l_left_drive_power);
        }
        if (v_motor_right_drive != null) {
            v_motor_right_drive.setPower(l_right_drive_power);
        }

        /*
        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setTargetPosition(l_left_drive_power);
        }
        */

        //
        // Manage the arm motor.
        //

        /*
        float l_left_arm_power = scale_motor_power(-gamepad2.left_stick_y);
        m_left_arm_power(l_left_arm_power);
        */
        //----------------------------------------------------------------------
        //
        // Servo Motors
        //
        // Obtain the current values of the gamepad 'x' and 'b' buttons.
        //
        // Note that x and b buttons have boolean values of true and false.
        //
        // The clip method guarantees the value never exceeds the allowable range of
        // [0,1].
        //
        // The setPosition methods write the motor power values to the Servo
        // class, but the positions aren't applied until this method ends.
        //
        /*
        if (gamepad2.x) {
            m_hand_position(a_hand_position() + 0.05);
        } else if (gamepad2.b) {
            m_hand_position(a_hand_position() - 0.05);
        }
        */

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry(); // Update common telemetry
        update_gamepad_telemetry();

    } // loop

//--------------------------------------------------------------------------
    //
    // scale_motor_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
	private double scale_0_to_1(double input )
	{
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

        if(input < 0.0)
        {
            negative = true;
            input = -input;
        }


        i = 0;
        while(i < 10)
		{
			if(sArray[i][0]>input)
			{
				break;
			}
            i++;
		}
//        for(i=0;i<11;i++)
//		{
//			if(sArray[i][0]>input)
//			{
//				break;
//			}
//
//		}
		speed=(((input - sArray[i-1][0]) / (0.1)) * ((sArray[i][1] - sArray[i-1][1]) + sArray[i-1][1]));



        if(negative == true)
        {
            return -speed;
        }
        else
        {
            return speed;
        }
	}


//    float scale_motor_power (float p_power)
//    {
//        //
//        // Assume no scaling.
//        //
//        float l_scale;
//
//        //
//        // Ensure the values are legal.
//        //
//        float l_power = Range.clip(p_power, -1, 1);
//
//        float[] l_array =
//                { 0.00f, 0.05f, 0.09f, 0.10f, 0.12f
//                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
//                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
//                        , 1.00f, 1.00f
//                };
//
//        //
//        // Get the corresponding index for the specified argument/parameter.
//        //
//        int l_index = (int)(l_power * 16.0);
//        if (l_index < 0)
//        {
//            l_index = -l_index;
//        }
//        else if (l_index > 16)
//        {
//            l_index = 16;
//        }
//
//        if (l_power < 0)
//        {
//            l_scale = -l_array[l_index];
//        }
//        else
//        {
//            l_scale = l_array[l_index];
//        }
//
//        return l_scale;
//
//    } // scale_motor_power


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
        //
        // Send telemetry data to the driver station.
        //
        telemetry.addData
                ( "01"
                        , "Left Drive: "
                                + get_left_drive_power ()
                                + ", "
                                + get_left_encoder_count ()
                );
        telemetry.addData
                ( "02"
                        , "Right Drive: "
                                + get_right_drive_power ()
                                + ", "
                                + get_right_encoder_count ()
                );
        telemetry.addData
                ( "03"
                        , "Loop Counter: "
                                + counter
                );


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
    boolean a_warning_generated ()

    {
        return v_warning_generated;

    } // a_warning_generated

    //--------------------------------------------------------------------------
    //
    // a_warning_message
    //
    /**
     * Access the warning message.
     */
    String a_warning_message ()

    {
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
    void m_warning_message (String p_exception_message)
    {
        if (v_warning_generated)
        {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    } // m_warning_message




    /**
     * Access the left encoder's count.
     */
    int get_left_encoder_count ()
    {
        int l_return = 0;

        if (v_motor_left_drive != null)
        {
            l_return = v_motor_left_drive.getCurrentPosition ();
        }

        return l_return;

    } // get_left_encoder_count

//--------------------------------------------------------------------------
    //
    // get_left_drive_power
    //
    /**
     * Access the left drive motor's power level.
     */
    double get_left_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_left_drive != null)
        {
            l_return = v_motor_left_drive.getPower();
        }

        return l_return;

    } // get_left_drive_power


    /**
     * Access the right encoder's count.
     */
    int get_right_encoder_count ()
    {
        int l_return = 0;

        if (v_motor_right_drive != null)
        {
            l_return = v_motor_right_drive.getCurrentPosition ();
        }

        return l_return;

    } // get_right_encoder_count

//--------------------------------------------------------------------------
    //
    // get_right_drive_power
    //
    /**
     * Access the right drive motor's power level.
     */
    double get_right_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_right_drive != null)
        {
            l_return = v_motor_right_drive.getPower();
        }

        return l_return;

    } // get_right_drive_power







    private DcMotor v_motor_left_drive;
    private DcMotor v_motor_right_drive;

    //--------------------------------------------------------------------------
    //
    // v_warning_generated
    //
    /**
     * Indicate whether a message is a available to the class user.
     */
    private boolean v_warning_generated = false;

    //--------------------------------------------------------------------------
    //
    // v_warning_message
    //
    /**
     * Store a message to the user if one has been generated.
     */
    private String v_warning_message;

    private long counter;

}
