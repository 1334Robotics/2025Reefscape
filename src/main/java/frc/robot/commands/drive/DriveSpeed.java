package frc.robot.commands.drive;

import frc.robot.constants.SwerveConstants;

public enum DriveSpeed {
    SLOW(SwerveConstants.SLOW_DRIVE_SPEED),
    NORMAL(SwerveConstants.NORMAL_DRIVE_SPEED),
    FAST(SwerveConstants.FAST_DRIVE_SPEED);

    public final double speed;
    private DriveSpeed(double speed) {
        this.speed = speed;
    }
}
