package frc.robot.constants;

import frc.robot.XboxMappings;

public class RobotContainerConstants {
    public static final int DRIVER_CONTROLLER_PORT      = 0;
    public static final int OPERATOR_CONTROLLER_PORT    = 1;

    public static final int ELEVATOR_PRIMARY_MOTOR_ID   = 0;
    public static final int ELEVATOR_SECONDARY_MOTOR_ID = 0;

    public static final int MAILBOX_OUTPUT_BUTTON = XboxMappings.Button.A;
    public static final int MAILBOX_INPUT_BUTTON  = XboxMappings.Button.B;
    public static final int MAILBOX_STOP_BUTTON   = XboxMappings.Button.X;

    public static final double CONTROLLER_MOVEMENT_DEADBAND = 0.1;
    public static final double CONTROLLER_ROTATION_DEADBAND = 0.3;
    
    public static final int SOLENOID_EXTEND_BUTTON  = XboxMappings.Button.RightBumper;
    public static final int SOLENOID_RETRACT_BUTTON = XboxMappings.Button.LeftBumper;

    public static final int GYRO_ZERO_BUTON = XboxMappings.Button.X;

    public static final int SNAP_FORWARDS_DIRECTION  = XboxMappings.DPad.Up;
    public static final int SNAP_LEFT_DIRECTION      = XboxMappings.DPad.Left;
    public static final int SNAP_RIGHT_DIRECTION     = XboxMappings.DPad.Right;
    public static final int SNAP_BACKWARDS_DIRECTION = XboxMappings.DPad.Down;
    public static final int SNAP_STOP_BUTTON         = XboxMappings.Button.B;

    public static final int INTAKE_RUN_BUTTON = XboxMappings.Button.LeftStick;
    public static final int INTAKE_LAUNCH_BUTTON = XboxMappings.Button.Y;
    public static final int INTAKE_STOP_BUTTON = XboxMappings.Button.RightStick;
}
