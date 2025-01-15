package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class VisionConstants {
    // Camera mounting position relative to robot center
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.1, 0, 0.5),  // Camera is 10cm forward, 50cm up
        new Rotation3d(0, Math.toRadians(-15), 0)  // Pitched up 15 degrees
    );

    // Camera configuration
    public static final String CAMERA_NAME = "OV981";  // Update this to match your camera
    public static final int CAMERA_RESOLUTION_WIDTH = 640;
    public static final int CAMERA_RESOLUTION_HEIGHT = 480;
    public static final double CAMERA_FOV_DEGREES = 100;
    
    // Simulation constants
    public static final double SIM_CAMERA_FPS = 30;
    public static final double SIM_AVERAGE_LATENCY_MS = 35;
    public static final double SIM_LATENCY_STD_DEV_MS = 5;
    public static final double SIM_ERROR_MEAN = 0.25;
    public static final double SIM_ERROR_STD_DEV = 0.08;
}