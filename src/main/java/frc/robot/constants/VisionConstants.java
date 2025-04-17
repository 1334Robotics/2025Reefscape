package frc.robot.constants;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public final class VisionConstants {
    // Camera mounting position relative to robot center
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.75, 0.5, 0),  // Camera is 75cm forward, 50cm to side
        new Rotation3d(0, 0, 0)  // No rotation
    );

    // Camera configuration
    public static final String LEFT_CAMERA_NAME  = "Cam1";
    public static final String RIGHT_CAMERA_NAME = "Cam2";
    
    // Vision processing constants
    public static final double MAX_AMBIGUITY = 0.2;  // Maximum allowed pose ambiguity
    public static final double MIN_TARGET_AREA = 10.0;  // Minimum target area in pixels²
    public static final double MAX_ACCEPTABLE_DELAY = 50; // In milliseconds
    public static final double MIN_TARGET_QUALITY = 0.2;  // Lower threshold for better target detection
    public static final double SMOOTHING_FACTOR = 0.2;    // Factor for angle smoothing

    // Simulation constants
    public static final double SIM_CAMERA_FPS = 30;
    public static final double SIM_AVERAGE_LATENCY_MS = 35;
    public static final double SIM_LATENCY_STD_DEV_MS = 5;
    public static final double SIM_ERROR_MEAN = 0.1;  // Reduced for better simulation
    public static final double SIM_ERROR_STD_DEV = 0.05;  // Reduced for better simulation

    // Field layout constants
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Rotation PID constants
    public static final double ROTATION_KP          = 0.02;
    public static final double ROTATION_KI          = 0.000;
    public static final double ROTATION_KD          = 0.000;
    public static final double ROTATION_TAU         = 0;
    public static final double ROTATION_LIM_MIN_INT = -0.3;
    public static final double ROTATION_LIM_MAX_INT = 0.3;

    // Drive PID constants
    public static final double FORWARDS_KP          = 0.08;
    public static final double FORWARDS_KI          = 0.00;
    public static final double FORWARDS_KD          = 0.000;
    public static final double FORWARDS_TAU         = 0;
    public static final double FORWARDS_LIM_MIN_INT = -0.3;
    public static final double FORWARDS_LIM_MAX_INT = 0.3;
    
    public static final double HORIZONTAL_KP          = 0.0;
    public static final double HORIZONTAL_KI          = 0.000;
    public static final double HORIZONTAL_KD          = 0.000;
    public static final double HORIZONTAL_TAU         = 0;
    public static final double HORIZONTAL_LIM_MIN_INT = -0.3;
    public static final double HORIZONTAL_LIM_MAX_INT = 0.3;

    // Camera mounting position relative to robot center (centimeters)
    public static final Transform3d CAMERA_POSITION = new Transform3d(Distance.ofBaseUnits(75, Millimeters), // Forwards
                                                                      Distance.ofBaseUnits(50, Millimeters), // Off centerline
                                                                      Distance.ofBaseUnits(0, Millimeters),  // Vertical offset
                                                                      new Rotation3d(0,
                                                                                     0,
                                                                                     0));
    public static final double CAMERA_POSITION_X = 75.0;  // 75cm forward from midpoint
    public static final double CAMERA_POSITION_Y = 50.0;  // 50cm off centerline
    public static final double CAMERA_POSITION_Z = 0.0;   // No vertical offset

    // Camera mounting angles (radians)
    public static final double CAMERA_PITCH_RADIANS = 0.0;   // No upward tilt
    public static final double CAMERA_YAW_RADIANS = 0.0;     // No left/right rotation
    public static final double CAMERA_ROLL_RADIANS = 0.0;    // No roll
    
    // Vision measurement accuracy constants
    public static final double BASE_VISION_STD_DEV = 0.5;  // Base std dev (Standard deviation) in meters
    public static final double MAX_VISION_STD_DEV = 3.0;   // Max std dev in meters
    public static final double MIN_VISION_STD_DEV = 0.1;   // Min std dev in meters
    public static final double AREA_TO_STD_DEV_FACTOR = 20.0; // Higher area = lower std dev

    public static final int MAX_ALLOWED_BLANK_FRAMES = 5;

    public static final String APRILTAG_LAYOUT = AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile;

    public static final Vector<N3> STATE_STD_DEVS  = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    public static final Vector<N3> VISION_STD_DEVS = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
}