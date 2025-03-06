package frc.robot.constants;

public class ElevatorConstants {
    public static final boolean MANUAL_ELEVATOR_CONTROL = true;

    public static final int MOTOR_ONE_ID    = 20;
    public static final int LIMIT_SWITCH_ID = 8;

    public static final double ELEVATOR_UP_SPEED    = 0.5;
    public static final double ELEVATOR_DOWN_SPEED  = 0.4;
    public static final double ELEVATOR_SLOW_SPEED  = 0.1;
    public static final double ELEVATOR_RESET_SPEED = 0.2;

    public static final double ELEVATOR_SLOW_LOW_POS  = 0.15;
    public static final double ELEVATOR_SLOW_HIGH_POS = 4.15;

    // No unit
    public static final double FEED_POSITION = 0.336602;
    public static final double L1_POSITION   = 0.738770;
    public static final double L2_POSITION   = 1.304644;
    public static final double L3_POSITION   = 2.350679;
    public static final double L4_POSITION   = 4.304453;

    public static final double PID_KP          = 0.25;
    public static final double PID_KI          = 0.1;
    public static final double PID_KD          = 0;
    public static final double PID_TAU         = 0;
    public static final double PID_LIM_MIN     = -0.7;
    public static final double PID_LIM_MAX     = 0.7;
    public static final double PID_LIM_MIN_INT = -0.2;
    public static final double PID_LIM_MAX_INT = 0.2;
    public static final double PID_SAMPLE_TIME = 1;

    public static final double MAX_ACCEPTABLE_ERROR = 0.005;

    public static final int    ENCODER_DIO_A              = 0;
    public static final int    ENCODER_DIO_B              = 1;
    public static final double ENCODER_PULSE_PER_ROTATION = 2048;
}