package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystemBase;
import frc.robot.subsystems.drive.PID;
import frc.robot.subsystems.drive.SwerveSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.littletonrobotics.junction.Logger;


public class TrackAprilTagCommand extends Command {
    private final VisionSubsystemBase visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    // Custom PID controllers for rotation and distance
    private final PID rotationController;
    private final PID distanceController;

    public TrackAprilTagCommand(VisionSubsystemBase vision, SwerveSubsystem swerve) {
        this.visionSubsystem = vision;
        this.swerveSubsystem = swerve;
        addRequirements(vision, swerve);

                // Initialize PID controllers
        // Tune these values for your robot
        this.rotationController = new PID(
            0.2,  // kP (Proportional gain for rotation)
            0.001, // kI (Integral gain for rotation)
            0.05,  // kD (Derivative gain for rotation)
            0.02,  // tau (Filter time constant)
            -1.0,  // limMin (Minimum output for rotation)
            1.0,   // limMax (Maximum output for rotation)
            -0.5,  // limMinInt (Minimum integral term for rotation)
            0.5,   // limMaxInt (Maximum integral term for rotation)
            0.02   // sampleTime (Update period in seconds)
        );

        this.distanceController = new PID(
            0.8,  // kP (Proportional gain for distance)
            0.005, // kI (Integral gain for distance)
            0.1,   // kD (Derivative gain for distance)
            0.02,  // tau (Filter time constant)
            -1.0,  // limMin (Minimum output for distance)
            1.0,   // limMax (Maximum output for distance)
            -0.5,  // limMinInt (Minimum integral term for distance)
            0.5,   // limMaxInt (Maximum integral term for distance)
            0.02   // sampleTime (Update period in seconds)
        );
    }

    @Override
    public void initialize() {
        // Reset PID controllers when the command starts
        rotationController.zero();
        distanceController.zero();
    }

    @Override
    public void execute() {
        if (visionSubsystem.isTargetVisible() && visionSubsystem.getTargetId() == VisionConstants.TARGET_TAG_ID) {
            // Get the target's pose relative to the robot
            PhotonTrackedTarget target = visionSubsystem.getTarget();
            double yaw = target.getYaw(); // Yaw angle to the target
            double distance = target.getBestCameraToTarget().getTranslation().getNorm(); // Distance to the target

            // Update PID controllers
            rotationController.update(0, yaw); // Target yaw is 0 (facing the tag)
            distanceController.update(VisionConstants.TARGET_FOLLOW_DISTANCE, distance); // Target distance

            // Get PID outputs
            double rotationSpeed = rotationController.getSteer();
            double forwardSpeed = distanceController.getSteer();

            // Drive the robot
            swerveSubsystem.drive(new Translation2d(forwardSpeed, 0), rotationSpeed);

            // Log data for AdvantageScope
            Logger.recordOutput("TrackAprilTag/RotationError", yaw);
            Logger.recordOutput("TrackAprilTag/DistanceError", distance - VisionConstants.TARGET_FOLLOW_DISTANCE);
            Logger.recordOutput("TrackAprilTag/RotationSpeed", rotationSpeed);
            Logger.recordOutput("TrackAprilTag/ForwardSpeed", forwardSpeed);
            SmartDashboard.putString("Vision/Status", "FOLLOWING TAG " + VisionConstants.TARGET_TAG_ID);
        } else {
            // Stop the robot if no target is visible
            swerveSubsystem.drive(new Translation2d(0, 0), 0);

            // Log that no target is visible
            Logger.recordOutput("TrackAprilTag/RotationError", 0.0);
            Logger.recordOutput("TrackAprilTag/DistanceError", 0.0);
            Logger.recordOutput("TrackAprilTag/RotationSpeed", 0.0);
            Logger.recordOutput("TrackAprilTag/ForwardSpeed", 0.0);
            SmartDashboard.putString("Vision/Status", "NO TARGET");
        }
    }

    @Override
    public boolean isFinished() {
        // End the command when the robot is within tolerance of the target pose
        return Math.abs(rotationController.getSteer()) < 0.1 && // Rotation tolerance
               Math.abs(distanceController.getSteer()) < 0.1;  // Distance tolerance
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        swerveSubsystem.drive(new Translation2d(0, 0), 0);
        SmartDashboard.putString("Vision/Status", "COMMAND ENDED");
    }
}
