package frc.robot.constants;

public class DirectionSnapConstants {
    public static final double MAX_ACCEPTABLE_YAW_ERROR = 1;

    // PID config
    public static final double PID_KP          = 0.1;
    public static final double PID_KI          = 0.01;
    public static final double PID_KD          = 0;
    public static final double PID_TAU         = 0;
    public static final double PID_LIM_MIN     = -1;
    public static final double PID_LIM_MAX     = 1;
    public static final double PID_LIM_MIN_INT = -0.5;
    public static final double PID_LIM_MAX_INT = 0.5;
    public static final double PID_SAMPLE_TIME = 0.1;
}
