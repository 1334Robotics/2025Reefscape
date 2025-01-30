package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;

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
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);

        // Add swerve drive state debugging
        var swerveSpeeds = swerveSubsystem.getChassisSpeeds();
        SmartDashboard.putNumber("Swerve/VxMetersPerSec", swerveSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/VyMetersPerSec", swerveSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/OmegaRadPerSec", swerveSpeeds.omegaRadiansPerSecond);

        if (hasTarget) {
            // Get target information
            double yaw = visionSubsystem.getTargetYaw();
            double pitch = visionSubsystem.getTargetPitch();
            
            // Log data to SmartDashboard
            SmartDashboard.putNumber("Vision/TargetYaw", yaw);
            SmartDashboard.putNumber("Vision/TargetPitch", pitch);
            
            // Enhanced pose debugging
            var robotPose = swerveSubsystem.getPose();
            var odometryPose = swerveSubsystem.getOdometryPose();
            
            SmartDashboard.putString("Vision/RobotPose", 
                String.format("X: %.2f, Y: %.2f, Rot: %.2f", 
                    robotPose.getX(), 
                    robotPose.getY(), 
                    robotPose.getRotation().getDegrees()));
                    
            SmartDashboard.putString("Vision/OdometryPose",
                String.format("X: %.2f, Y: %.2f, Rot: %.2f",
                    odometryPose.getX(),
                    odometryPose.getY(),
                    odometryPose.getRotation().getDegrees()));
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Run continuously
    }
}