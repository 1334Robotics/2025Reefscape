package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import swervelib.SwerveController;


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
        boolean hasTarget = visionSubsystem.isTargetVisible();
        
        if (hasTarget && visionSubsystem.getTargetId() == VisionConstants.TARGET_TAG_ID) {
            double yaw = visionSubsystem.getTargetYaw();
            double area = visionSubsystem.getTargetArea();
            // Cap the area at 2%
            if(area > 2) area = 2;
            
            // Calculate drive commands
            double rotationSpeed = yaw * VisionConstants.ROTATION_P;
            double forwardSpeed = (VisionConstants.TARGET_FOLLOW_DISTANCE / (area/2)) * VisionConstants.DISTANCE_P;
            if(rotationSpeed > SwerveConstants.DRIVE_SPEED) rotationSpeed = SwerveConstants.DRIVE_SPEED;
            if(forwardSpeed > SwerveConstants.DRIVE_SPEED) forwardSpeed = SwerveConstants.DRIVE_SPEED;
            if(rotationSpeed < -SwerveConstants.DRIVE_SPEED) rotationSpeed = -SwerveConstants.DRIVE_SPEED;
            if(forwardSpeed < -SwerveConstants.DRIVE_SPEED) forwardSpeed = -SwerveConstants.DRIVE_SPEED;

            // Publish the speeds 
            
            // Drive robot
            swerveSubsystem.drive(
                new Translation2d(forwardSpeed, 0),
                rotationSpeed
            );
            
            // Log status
            SmartDashboard.putString("Vision/Status", "FOLLOWING TAG " + VisionConstants.TARGET_TAG_ID);
        } else {
            // Stop if wrong tag or no target
            swerveSubsystem.drive(new Translation2d(0, 0), 0);
            SmartDashboard.putString("Vision/Status", "NO TARGET");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
