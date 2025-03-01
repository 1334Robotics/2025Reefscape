package frc.robot.constants;

/**
 * Constants for PathPlanner auto paths and path following
 */
public final class PathPlannerConstants {
    // PID Controllers for path following
    public static final double TRANSLATION_KP = 5.0;
    public static final double TRANSLATION_KI = 0.0;
    public static final double TRANSLATION_KD = 0.0;
    
    public static final double ROTATION_KP = 5.0;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.0;
    
    // Constraints for path following
    public static final double MAX_MODULE_SPEED_MPS = 4.5; // Meters per second
    public static final double MAX_ACCELERATION_MPS2 = 3.0; // Meters per second squared
    
    // Maximum velocity the robot can travel at
    public static final double MAX_VELOCITY = 4.0; // m/s
    
    // Maximum acceleration the robot can travel at
    public static final double MAX_ACCELERATION = 2.0; // m/s^2
    
    // Path names - create these paths in the PathPlanner tool
    public static final String TEST_PATH = "TestPath";
    
    // Auto path names - create these autos in the PathPlanner tool
    public static final String EXAMPLE_AUTO = "ExampleAuto";
} 