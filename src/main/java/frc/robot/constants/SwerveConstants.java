package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    public static final String SWERVE_DRIVE_DIRECTORY = "swerve";
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    public static final double SLOW_DRIVE_SPEED   = 0.1;
    public static final double NORMAL_DRIVE_SPEED = 0.6;
    public static final double FAST_DRIVE_SPEED   = 0.8;
    public static final int NUM_DRIVE_MOTORS = 4;
    public static final int NUM_STEER_MOTORS = 4;
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 21.429;
}