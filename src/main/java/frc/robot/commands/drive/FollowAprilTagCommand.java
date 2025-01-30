package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class FollowAprilTagCommand extends Command {
    private final VisionSubsystem vision;
    private final SwerveSubsystem swerve;
    private final int targetTagId;
    
    private static final double TARGET_DISTANCE_INCHES = 6.0;
    private static final double ANGLE_P = 0.1;  // Tune this
    private static final double DISTANCE_P = 0.1; // Tune this
    
    public FollowAprilTagCommand(VisionSubsystem vision, SwerveSubsystem swerve, int tagId) {
        this.vision = vision;
        this.swerve = swerve;
        this.targetTagId = tagId;
        addRequirements(vision, swerve);
    }
    
    @Override
    public void execute() {
        // Vision Status
        boolean hasTargetTag = vision.isSpecificTargetVisible(targetTagId);
        SmartDashboard.putBoolean("Follow/HasTarget", hasTargetTag);
        SmartDashboard.putNumber("Follow/TargetId", targetTagId);
        SmartDashboard.putNumber("Follow/CurrentId", vision.getBestTargetId());
        
        if (!hasTargetTag) {
            swerve.drive(0, 0, 0, true);
            SmartDashboard.putString("Follow/Status", "WRONG_TAG");
            return;
        }

        // Target Measurements
        double yaw = vision.getTargetYaw();
        double pitch = vision.getTargetPitch();
        SmartDashboard.putNumber("Follow/Target/Yaw", yaw);
        SmartDashboard.putNumber("Follow/Target/Pitch", pitch);
        
        // Distance Calculation
        double currentDistance = estimateDistance(pitch);
        SmartDashboard.putNumber("Follow/Distance/Current", currentDistance);
        SmartDashboard.putNumber("Follow/Distance/Target", TARGET_DISTANCE_INCHES);
        SmartDashboard.putNumber("Follow/Distance/Error", TARGET_DISTANCE_INCHES - currentDistance);
        
        // Control Outputs
        double rotationSpeed = -yaw * ANGLE_P;
        double forwardSpeed = (TARGET_DISTANCE_INCHES - currentDistance) * DISTANCE_P;
        SmartDashboard.putNumber("Follow/Control/RotationSpeed", rotationSpeed);
        SmartDashboard.putNumber("Follow/Control/ForwardSpeed", forwardSpeed);
        
        // Drive Commands
        swerve.drive(forwardSpeed, 0, rotationSpeed, true);
        SmartDashboard.putString("Follow/Status", "FOLLOWING");
    }
    
    private double estimateDistance(double pitchDegrees) {
        // This is a simplified calculation - needs calibration
        double cameraHeightInches = 24.0; // Update with actual camera height
        double targetHeightInches = 18.0; // Update with actual target height
        double cameraPitchDegrees = 0.0;  // Update with actual camera mounting angle
        
        double totalAngle = cameraPitchDegrees + pitchDegrees;
        return (targetHeightInches - cameraHeightInches) / Math.tan(Math.toRadians(totalAngle));
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}