package frc.robot.constants;

public class ElevatorConstants {
    // Constants from the previous ElevatorSubsystem
    public static final int MOTOR_1 = 20;
    public static final double GEAR_RATIO = 10;
    public static final double MAX_HEIGHT_INCHES = 30;
    public static final double MIN_HEIGHT_INCHES = 0;
    public static final double POSITION_TOLERANCE = 0.5;
    public static final double DRUM_RADIUS_INCHES = 0.75;
    public static final double POSITION_TOLERANCE_INCHES = 0.5;
    
    public static final double PRIMARY_MAX_HEIGHT_INCHES = 48;
    public static final double PRIMARY_MIN_HEIGHT_INCHES = 0;
    public static final double SECONDARY_MAX_HEIGHT_INCHES = 24;
    public static final double SECONDARY_MIN_HEIGHT_INCHES = 0;
    
    public static final double PRIMARY_DRUM_RADIUS_INCHES = 1;
    public static final double SECONDARY_DRUM_RADIUS_INCHES = 0.75;

    public static final double GROUND_PRIMARY_HEIGHT = 0;
    public static final double GROUND_SECONDARY_HEIGHT = 0;
    public static final double MIDDLE_PRIMARY_HEIGHT = 24;
    public static final double MIDDLE_SECONDARY_HEIGHT = 12;
    public static final double HIGH_PRIMARY_HEIGHT = 48;
    public static final double HIGH_SECONDARY_HEIGHT = 24;

    // Constants for the new ElevatorSubsystem
    public static final int MOTOR_ONE_ID    = 20;
    public static final int LIMIT_SWITCH_ID = 8;

    public static final double ELEVATOR_UP_SPEED    = 0.5;
    public static final double ELEVATOR_DOWN_SPEED  = 0.4;
    public static final double ELEVATOR_SLOW_SPEED  = 0.1;
    public static final double ELEVATOR_RESET_SPEED = 0.2;

    public static final double ELEVATOR_SLOW_LOW_POS  = 0.5;
    public static final double ELEVATOR_SLOW_HIGH_POS = 11;

    public static final double L1_POSITION = 2.789952;
    public static final double L2_POSITION = 4.463580;
    public static final double L3_POSITION = 7.915724;
    public static final double L4_POSITION = 0;

    public static final double PID_KP          = 0.1;
    public static final double PID_KI          = 0.1;
    public static final double PID_KD          = 0;
    public static final double PID_TAU         = 0;
    public static final double PID_LIM_MIN     = -0.2;
    public static final double PID_LIM_MAX     = 0.2;
    public static final double PID_LIM_MIN_INT = -0.1;
    public static final double PID_LIM_MAX_INT = 0.1;
    public static final double PID_SAMPLE_TIME = 1;

    public static final double MAX_ACCEPTABLE_ERROR = 0.1;
}