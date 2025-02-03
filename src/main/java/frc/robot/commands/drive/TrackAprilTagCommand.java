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
