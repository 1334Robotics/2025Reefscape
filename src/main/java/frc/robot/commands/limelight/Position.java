package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import edu.wpi.first.wpilibj.Timer;





public class Position extends Command{
    private final SwerveSubsystem swerve;
    private final LimelightSubsystem limelight;
    private final double disstanceFromTarget;
    private Pose2d targetPose;

    public Position(SwerveSubsystem swerve, LimelightSubsystem limelight, double distanceFromTarget) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.disstanceFromTarget = distanceFromTarget;
        addRequirements(swerve, limelight);
    }

    @Override
    public void initialize() {
        var tagPose = limelight.getBotPose();
        if (tagPose == null) {
            tagPose = null;
            return;
        }

        double tagX = tagPose[0];
        double tagY = tagPose[1];
        double tagYaw = tagPose[5];
        double tagYawRadians = Math.toRadians(tagYaw);

        double robotX = tagX - (disstanceFromTarget * Math.cos(tagYawRadians));
        double robotY = tagY - (disstanceFromTarget * Math.sin(tagYawRadians));
        double robotYaw = tagYaw + 180.0; // Adjust yaw to face the target
        if (robotYaw > 180.0) {
            robotYaw -= 360.0; // Normalize yaw to [-180, 180]
        }

        targetPose = new Pose2d(robotX, robotY, Rotation2d.fromDegrees(robotYaw));

    }

    @Override
    public void execute() {
        if (targetPose != null) {
            return; //add drive to pose later
        }
    }

    @Override
    public boolean isFinished() {
        if (targetPose == null) {
            return true; // If target pose is not set, finish the command
        }
        return swerve.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.1; // 10 cm tolerance
    }   

}
