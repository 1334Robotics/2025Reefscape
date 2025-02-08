package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class VisionConstants {
    // Camera mounting position relative to robot center THIS NEEDS TO BE UPDATED FOR THE ACTUAL ROBOT
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.5, 0, 0.5),  // Camera is 50cm forward, 50cm up
        new Rotation3d(0, Math.toRadians(-30), 0)  // Pitched up 30 degrees
    );

    // Camera configuration
    public static final String CAMERA_NAME = "Arducam_OV9782_USB_Camera";
    public static final int CAMERA_RESOLUTION_WIDTH = 640;
    public static final int CAMERA_RESOLUTION_HEIGHT = 480;
    public static final double CAMERA_FOV_DEGREES = 100;
    
    // Vision processing constants
    public static final double MAX_AMBIGUITY = 0.2;  // Maximum allowed pose ambiguity
    public static final double MIN_TARGET_AREA = 10.0;  // Minimum target area in pixels²
    
    // Simulation constants
    public static final double SIM_CAMERA_FPS = 30;
    public static final double SIM_AVERAGE_LATENCY_MS = 35;
    public static final double SIM_LATENCY_STD_DEV_MS = 5;
    public static final double SIM_ERROR_MEAN = 0.1;  // Reduced for better simulation
    public static final double SIM_ERROR_STD_DEV = 0.05;  // Reduced for better simulation

    // Field layout constants
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // AprilTag tracking constants
    public static final int TARGET_TAG_ID = 3;  // Target AprilTag ID
    public static final double TARGET_FOLLOW_DISTANCE = 100;  // Centimeters from tag
    public static final double AREA_10_DISTANCE = 41.5; // In Centimeters

    // Rotation PID constants
    public static final double ROTATION_KP          = 0.05;
    public static final double ROTATION_KI          = 0.0001;
    public static final double ROTATION_KD          = 0;
    public static final double ROTATION_TAU         = 0;
    public static final double ROTATION_LIM_MIN_INT = -0.3;
    public static final double ROTATION_LIM_MAX_INT = 0.3;

    // Drive PID constants
    public static final double DRIVE_KP          = 0.01;
    public static final double DRIVE_KI          = 0.0001;
    public static final double DRIVE_KD          = 0;
    public static final double DRIVE_TAU         = 0;
    public static final double DRIVE_LIM_MIN_INT = -0.3;
    public static final double DRIVE_LIM_MAX_INT = 0.3;
}