package frc.robot.constants;
import frc.robot.XboxMappings;

public class RobotContainerConstants {
    public static final int DRIVER_CONTROLLER_PORT      = 0;
    public static final int OPERATOR_CONTROLLER_PORT    = 1;

    public static final int MAILBOX_SHOOT_BUTTON  = XboxMappings.Button.A;
    public static final int MAILBOX_FEED_BUTTON   = XboxMappings.Button.Y;
    public static final int MAILBOX_REWIND_BUTTON = XboxMappings.Button.Back;

    public static final double CONTROLLER_MOVEMENT_DEADBAND = 0.15;
    public static final double CONTROLLER_ROTATION_DEADBAND = 0.3;

    public static final int GYRO_ZERO_BUTON = XboxMappings.Button.X;

    public static final int SNAP_FORWARDS_DIRECTION  = XboxMappings.DPad.Up;
    public static final int SNAP_LEFT_DIRECTION      = XboxMappings.DPad.Left;
    public static final int SNAP_RIGHT_DIRECTION     = XboxMappings.DPad.Right;
    public static final int SNAP_BACKWARDS_DIRECTION = XboxMappings.DPad.Down;
    public static final int SNAP_STOP_BUTTON         = XboxMappings.Button.B;

    public static final int ELEVATOR_UP_BUTTON     = XboxMappings.DPad.Up;
    public static final int ELEVATOR_DOWN_BUTTON   = XboxMappings.DPad.Down;
    public static final int ELEVATOR_BOTTOM_BUTTON = XboxMappings.Button.B;
    public static final int ELEVATOR_FEED_BUTTON   = XboxMappings.Button.X;
    public static final int ELEVATOR_L1_BUTTON     = XboxMappings.DPad.Up;
    public static final int ELEVATOR_L2_BUTTON     = XboxMappings.DPad.Right;
    public static final int ELEVATOR_L3_BUTTON     = XboxMappings.DPad.Down;
    public static final int ELEVATOR_L4_BUTTON     = XboxMappings.DPad.Left;

    public static final int FLOPPER_UP_BUTTON   = XboxMappings.Button.LeftBumper;
    public static final int FLOPPER_DOWN_BUTTON = XboxMappings.Button.RightBumper;

    public static final int RIP_CONTROL_BUTTON = XboxMappings.Button.Back;

    public static final double TRACK_LEFT_TRIGGER_POINT  = 0.5;
    public static final double TRACK_RIGHT_TRIGGER_POINT = 0.5;

    public static final int BOT_RELATIVE_BUTTON   = XboxMappings.Button.A;
    public static final int SLOW_DOWN_BUTTON      = XboxMappings.Button.B;

    public static final int LED_BUTTON = XboxMappings.Button.Back;
}
