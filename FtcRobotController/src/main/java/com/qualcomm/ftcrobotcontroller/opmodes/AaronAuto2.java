package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotAuto
//

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Provide a basic autonomous operational mode that uses the left and right
 * drive motors and associated encoders implemented using a state machine for
 * the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */
public class AaronAuto2 extends PushBotTelemetry {

    //--------------------------------------------------------------------------
    //
    // v_state
    //
    /**
     * This class member remembers which state is currently active.  When the
     * start method is called, the state will be initialized (0).  When the loop
     * starts, the state will change from initialize to state_1.  When state_1
     * actions are complete, the state will change to state_2.  This implements
     * a state machine for the loop method.
     */
    private int v_state = 0;

    private DcMotor v_motor_left_drive;
    private DcMotor v_motor_right_drive;

    private double LDPosition = 0.0; // Left drive position
    private double LDDelta = 0.0; // Left drive delta
    private double RDPosition = 0.0; // Right drive position
    private double RDDelta = 0.0; // Right drive delta


    //--------------------------------------------------------------------------
    //
    // PushBotAuto
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public AaronAuto2 () {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotAuto

    //--------------------------------------------------------------------------
    //
    // start
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override
    public void init () {

        LDPosition = LDDelta = 0.0;
        RDPosition = RDDelta = 0.0;

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

        try {
            // Motor right drive
            v_motor_right_drive = hardwareMap.dcMotor.get("motorRD");

            // Set the motor right drive to reverse
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

    } // start

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during auto-operation.
     * The state machine uses a class member and encoder input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override
    public void loop () {

        v_motor_left_drive.setPower(-1);
        v_motor_right_drive.setPower(-1);


    } // loop



} // PushBotAuto

