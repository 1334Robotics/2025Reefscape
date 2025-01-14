package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    // Physical properties
    public static final double WHEEL_BASE = Units.inchesToMeters(21.73); // Distance between front and back wheels
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.73); // Distance between left and right wheels
    
    // Swerve Module Positions (relative to center of robot)
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
    
    // Kinematics
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        FRONT_LEFT_LOCATION,
        FRONT_RIGHT_LOCATION,
        BACK_LEFT_LOCATION,
        BACK_RIGHT_LOCATION
    );
    
    // Speed limits
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // One rotation per second
    
    // Controller constants
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double DEADBAND = 0.1;
    
    // YAGSL JSON config file path
    public static final String SWERVE_CONFIG_FILE = "swerve/kraken.json";
}
