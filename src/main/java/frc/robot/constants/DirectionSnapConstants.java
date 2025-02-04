package frc.robot.constants;

public class DirectionSnapConstants {
    public static final double MAX_ACCEPTABLE_YAW_ERROR = 5;

    // PID config
    public static final double PID_KP          = 0.0025;
    public static final double PID_KI          = 0.0001;
    public static final double PID_KD          = 0;
    public static final double PID_TAU         = 0.1;
    public static final double PID_LIM_MIN     = -0.5;
    public static final double PID_LIM_MAX     = 0.5;
    public static final double PID_LIM_MIN_INT = -0.17;
    public static final double PID_LIM_MAX_INT = 0.17;
    public static final double PID_SAMPLE_TIME = 1;
}
