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
    public static final int CAMERA_RESOLUTION_WIDTH = 800;
    public static final int CAMERA_RESOLUTION_HEIGHT = 600;
    public static final double CAMERA_FOV_DEGREES = 100;  // Verify this value matches your camera
    public static final int CAMERA_FPS = 90;  // Added to document actual frame rate
    
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
}