package frc.robot.constants;

public class ElevatorConstants {
    public static final boolean MANUAL_ELEVATOR_CONTROL = false;

    public static final int MOTOR_ONE_ID    = 20;
    public static final int LIMIT_SWITCH_ID = 8;

    public static final double ELEVATOR_UP_SPEED    = 0.5;
    public static final double ELEVATOR_DOWN_SPEED  = 0.4;
    public static final double ELEVATOR_SLOW_SPEED  = 0.1;
    public static final double ELEVATOR_RESET_SPEED = 0.3;

    // No unit
    public static final double ELEVATOR_SLOW_LOW_POS  = 0.15;
    public static final double ELEVATOR_SLOW_HIGH_POS = 5.00;

    // No unit
    public static final double FEED_POSITION = 0.936236;
    public static final double L1_POSITION   = 1.314453;
    public static final double L2_POSITION   = 1.938770;
    public static final double L3_POSITION   = 3.024644;
    public static final double L4_POSITION   = 5.155067;

    // Set a limit where KI takes effect
    // D term reduces oscillation
    // I term pretty much inverses the D term
    // Put units for setpoints
    public static final double PID_KP          = 0.9;
    public static final double PID_KI          = 0.07;
    public static final double PID_KD          = 0;
    public static final double PID_TAU         = 0;
    public static final double PID_LIM_MIN     = -0.9;
    public static final double PID_LIM_MAX     = 0.9;
    public static final double PID_LIM_MIN_INT = -0.2;
    public static final double PID_LIM_MAX_INT = 0.2;
    public static final double PID_SAMPLE_TIME = 1;
    public static final double PID_ISTART      = 0.5;

    public static final double MAX_ACCEPTABLE_ERROR = 0.005;

    public static final int    ENCODER_DIO_A              = 0;
    public static final int    ENCODER_DIO_B              = 1;
    public static final double ENCODER_PULSE_PER_ROTATION = 2048;
}