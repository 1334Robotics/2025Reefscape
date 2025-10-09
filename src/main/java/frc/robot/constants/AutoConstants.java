package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoConstants {
    public static final double TRANSLATION_KP = 4;
    public static final double TRANSLATION_KI = 5;
    public static final double TRANSLATION_KD = 0.5;
    //this is a text
    public static final double ROTATION_KP = 5;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0;

    // The rotation of 180 is definitiely a suspect for the bot starting 180
    public static final Pose2d BLUE_LEFT_STARTING_POSE   = new Pose2d(7.6, 1, Rotation2d.fromDegrees(180)); // Blue Bottom
    public static final Pose2d BLUE_CENTER_STARTING_POSE = new Pose2d(7.6, 4, Rotation2d.fromDegrees(180)); // Blue Middle
    public static final Pose2d BLUE_RIGHT_STARTING_POSE  = new Pose2d(7.6, 7, Rotation2d.fromDegrees(180)); // Blue Top
    public static final Pose2d RED_LEFT_STARTING_POSE    = new Pose2d(10, 7, Rotation2d.fromDegrees(180));  // Red Top
    public static final Pose2d RED_CENTER_STARTING_POSE  = new Pose2d(10, 4, Rotation2d.fromDegrees(180));  // Red Middle
    public static final Pose2d RED_RIGHT_STARTING_POSE   = new Pose2d(10, 1, Rotation2d.fromDegrees(180));  // Red Bottom
}
