package frc.robot.constants;

public final class OIConstants {
    // The port number for the driver's Xbox controller
    public static final int DRIVER_CONTROLLER_PORT = 0;

    // Deadband for controller joysticks to avoid drift
    public static final double CONTROLLER_DEADBAND = 0.05;

    // Axis IDs for Xbox controller (WPILib standards)
    public static final int LEFT_X_AXIS = 0;
    public static final int LEFT_Y_AXIS = 1;
    public static final int RIGHT_X_AXIS = 4;
    public static final int RIGHT_Y_AXIS = 5;

    // Button IDs for Xbox controller (WPILib standards)
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;

    // Trigger axes
    public static final int LEFT_TRIGGER_AXIS = 2;
    public static final int RIGHT_TRIGGER_AXIS = 3;

    // Maximum output scaling for swerve drive (can be adjusted in runtime)
    public static final double MAX_SPEED = 1.0;
    public static final double SLOW_MODE_SPEED = 0.5;

    // Field-relative or robot-relative drive default
    public static final boolean FIELD_RELATIVE_DEFAULT = true;

    private OIConstants() {
        // Prevent instantiation
    }
}
