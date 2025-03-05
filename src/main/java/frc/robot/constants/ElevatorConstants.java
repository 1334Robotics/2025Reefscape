package frc.robot.constants;

public class ElevatorConstants {
    public static final int MOTOR_ONE_ID    = 20;
    public static final int LIMIT_SWITCH_ID = 8;

    public static final double ELEVATOR_UP_SPEED    = 0.5;
    public static final double ELEVATOR_DOWN_SPEED  = 0.4;
    public static final double ELEVATOR_SLOW_SPEED  = 0.1;
    public static final double ELEVATOR_RESET_SPEED = 0.2;

    public static final double ELEVATOR_SLOW_LOW_POS  = 0.5;
    public static final double ELEVATOR_SLOW_HIGH_POS = 11;

    public static final double FEED_POSITION = 1.589387;
    public static final double L1_POSITION   = 2.789952;
    public static final double L2_POSITION   = 4.513580;
    public static final double L3_POSITION   = 7.915724;
    public static final double L4_POSITION   = 0;

    public static final double PID_KP          = 0.1;
    public static final double PID_KI          = 0.1;
    public static final double PID_KD          = 0;
    public static final double PID_TAU         = 0;
    public static final double PID_LIM_MIN     = -0.5;
    public static final double PID_LIM_MAX     = 0.5;
    public static final double PID_LIM_MIN_INT = -0.2;
    public static final double PID_LIM_MAX_INT = 0.2;
    public static final double PID_SAMPLE_TIME = 1;

    public static final double MAX_ACCEPTABLE_ERROR = 0.05;
}