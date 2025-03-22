package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Constants for autonomous operation and path following
 */
public final class AutoConstants {
    public static final double TRANSLATION_KP = 0.0020645;
    public static final double TRANSLATION_KI = 0;
    public static final double TRANSLATION_KD = 0;

    public static final double ROTATION_KP = 20;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0;

    // PID constants for path following
    public static final double PATH_PLANNER_KP = 5.0;
    public static final double PATH_PLANNER_KI = 0.0;
    public static final double PATH_PLANNER_KD = 0.0;

    // Maximum speeds for path following
    public static final double MAX_MODULE_SPEED = 4.5; // meters per second

    public static final Pose2d BLUE_LEFT_STARTING_POSE = new Pose2d(7.6, 1, Rotation2d.fromDegrees(180));    // Blue Bottom
    public static final Pose2d BLUE_CENTER_STARTING_POSE = new Pose2d(7.6, 4, Rotation2d.fromDegrees(180));  // Blue Middle
    public static final Pose2d BLUE_RIGHT_STARTING_POSE = new Pose2d(7.6, 7, Rotation2d.fromDegrees(180));   // Blue Top
    public static final Pose2d RED_LEFT_STARTING_POSE = new Pose2d(10, 7.036, Rotation2d.fromDegrees(180));  // Red Top
    public static final Pose2d RED_CENTER_STARTING_POSE = new Pose2d(10, 4, Rotation2d.fromDegrees(180));    // Red Middle
    public static final Pose2d RED_RIGHT_STARTING_POSE = new Pose2d(10, 1.599, Rotation2d.fromDegrees(180)); // Red Bottom

    private AutoConstants() {
        throw new AssertionError("utility class");
    }
}
