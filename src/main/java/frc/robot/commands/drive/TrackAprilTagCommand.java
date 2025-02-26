package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystemBase;
import frc.robot.subsystems.drive.SwerveSubsystem;


public class TrackAprilTagCommand extends Command {
    private final VisionSubsystemBase visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    public TrackAprilTagCommand(VisionSubsystemBase vision, SwerveSubsystem swerve) {
        this.visionSubsystem = vision;
        this.swerveSubsystem = swerve;
        addRequirements(vision, swerve);
    }

    @Override
    public void execute() {
        boolean hasTarget = visionSubsystem.isTargetVisible();
        double yaw = visionSubsystem.getTargetYaw();
        double area = visionSubsystem.getTargetArea();

        if (hasTarget && visionSubsystem.getTargetId() == VisionConstants.TARGET_TAG_ID) {
            double rotationSpeed = -yaw * VisionConstants.ROTATION_P;
            double forwardSpeed = (VisionConstants.TARGET_FOLLOW_DISTANCE - area) * VisionConstants.DISTANCE_P;

            swerveSubsystem.drive(new Translation2d(forwardSpeed, 0), rotationSpeed);
            SmartDashboard.putString("Vision/Status", "FOLLOWING TAG " + VisionConstants.TARGET_TAG_ID);
        } else {
            swerveSubsystem.drive(new Translation2d(0, 0), 0);
            SmartDashboard.putString("Vision/Status", "NO TARGET");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
