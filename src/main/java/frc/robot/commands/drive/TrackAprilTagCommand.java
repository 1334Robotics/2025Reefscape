package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import swervelib.SwerveController;

public class DriveCommand extends Command {
    private final DoubleSupplier   vX;
    private final DoubleSupplier   vY;
    private final DoubleSupplier   omega;
    private final SwerveController controller;

    public DriveCommand(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega) {
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.controller = RobotContainer.swerveSubsystem.getSwerveController();

        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void execute() {
        // This math is from previous years
        double xVelocity = Math.pow(this.vX.getAsDouble(), 3) * SwerveConstants.DRIVE_SPEED;
        double yVelocity = Math.pow(this.vY.getAsDouble(), 3) * SwerveConstants.DRIVE_SPEED;
        double angularVelocity = Math.pow(this.omega.getAsDouble(), 3);

        // Update the values within SmartDashboard
        SmartDashboard.putNumber("[SWERVE] X Velocity", xVelocity);
        SmartDashboard.putNumber("[SWERVE] Y Velocity", yVelocity);
        SmartDashboard.putNumber("[SWERVE] Angular Velocity", angularVelocity);

        // Drive
        RobotContainer.swerveSubsystem.drive(new Translation2d(xVelocity * SwerveConstants.MAX_SPEED,
                                                               yVelocity * SwerveConstants.MAX_SPEED),
                                            angularVelocity * controller.config.maxAngularVelocity);
    }

    @Override
    public boolean isFinished() {
        // The drive command will never finish
        return false;
    }
}

public class TrackAprilTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    public TrackAprilTagCommand(VisionSubsystem vision, SwerveSubsystem swerve) {
        this.visionSubsystem = vision;
        this.swerveSubsystem = swerve;
        addRequirements(vision, swerve);
    }

    @Override
    public void execute() {
        // Get vision data
        boolean hasTarget = visionSubsystem.isTargetVisible();
        SmartDashboard.putBoolean("[TRACKING] Has Target", hasTarget);

        if (hasTarget) {
            double yaw = visionSubsystem.getTargetYaw();
            double pitch = visionSubsystem.getTargetPitch();
            double area = visionSubsystem.getTargetArea();
            
            // Log tracking data
            SmartDashboard.putNumber("[TRACKING] Target Yaw", yaw);
            SmartDashboard.putNumber("[TRACKING] Target Pitch", pitch);
            SmartDashboard.putNumber("[TRACKING] Target Area", area);
            
            // Get current robot pose for reference
            SmartDashboard.putString("[TRACKING] Robot Pose", 
                swerveSubsystem.getPose().toString());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
