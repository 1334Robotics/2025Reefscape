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
        
        if (hasTarget) {
            // Get vision data
            double yaw = visionSubsystem.getTargetYaw();
            double area = visionSubsystem.getTargetArea();
            
            // Calculate drive commands
            double rotationSpeed = -yaw * 0.1; // Adjust constant for response
            double forwardSpeed = (2.0 - area) * 0.5; // Adjust for desired distance
            
            // Drive robot
            swerveSubsystem.drive(
                new Translation2d(forwardSpeed, 0),
                rotationSpeed
            );
        } else {
            // Stop if no target
            swerveSubsystem.drive(new Translation2d(0, 0), 0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
