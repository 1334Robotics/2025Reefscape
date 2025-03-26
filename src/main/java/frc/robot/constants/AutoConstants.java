package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoConstants {
    public static final double TRANSLATION_KP = 0.0020645;
    public static final double TRANSLATION_KI = 0;
    public static final double TRANSLATION_KD = 0;

    public static final double ROTATION_KP = 20;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0;

    // Path Planner PID Constants
    public static final double PATH_PLANNER_TRANSLATION_KP = 4.0;
    public static final double PATH_PLANNER_TRANSLATION_KI = 0.5;
    public static final double PATH_PLANNER_TRANSLATION_KD = 5.0;

    public static final double PATH_PLANNER_ROTATION_KP = 5.0;
    public static final double PATH_PLANNER_ROTATION_KI = 0.0;
    public static final double PATH_PLANNER_ROTATION_KD = 0.0;

    // Blue alliance starting poses for 2025 Reefscape
    public static final Pose2d BLUE_LEFT_STARTING_POSE = new Pose2d(7.6, 7.0, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_CENTER_STARTING_POSE = new Pose2d(7.6, 4.0, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE_RIGHT_STARTING_POSE = new Pose2d(7.6, 1.0, Rotation2d.fromDegrees(180));
    
    // Red alliance starting poses for 2025 Reefscape (mirrored)
    public static final Pose2d RED_LEFT_STARTING_POSE = new Pose2d(16.2, 7.0, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_CENTER_STARTING_POSE = new Pose2d(16.2, 4.0, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_RIGHT_STARTING_POSE = new Pose2d(16.2, 1.0, Rotation2d.fromDegrees(0));
}
