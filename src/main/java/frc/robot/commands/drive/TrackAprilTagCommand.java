package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.drive.PIDController;
import frc.robot.subsystems.drive.SwerveSubsystem;


public class TrackAprilTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController rotationController;
    private final PIDController speedController;

    public TrackAprilTagCommand(VisionSubsystem vision, SwerveSubsystem swerve) {
        this.visionSubsystem = vision;
        this.swerveSubsystem = swerve;

        // Initialize the PID controllers
        rotationController = new PIDController(VisionConstants.ROTATION_KP,
                                               VisionConstants.ROTATION_KI,
                                               VisionConstants.ROTATION_KD,
                                               VisionConstants.ROTATION_TAU,
                                               // These are the limMin and limMax terms
                                               // Which should allow us to simply multiply the output of the PID controller
                                               // by the SwerveConstants.DRIVE_SPEED and the SwerveConstants.
                                               -1,
                                               1, 
                                               VisionConstants.ROTATION_LIM_MIN_INT,
                                               VisionConstants.ROTATION_LIM_MAX_INT,
                                               1 /* The sampleTime which should never need to be changed (it doesn't do much) */);
        speedController   = new PIDController(VisionConstants.DRIVE_KP,
                                              VisionConstants.DRIVE_KI,
                                              VisionConstants.DRIVE_KD,
                                              VisionConstants.DRIVE_TAU,
                                              -1,
                                              1,
                                              VisionConstants.DRIVE_LIM_MIN_INT,
                                              VisionConstants.DRIVE_LIM_MAX_INT,
                                              1);

        addRequirements(vision, swerve);
    }

    @Override
    public void execute() {
        boolean hasTarget = visionSubsystem.isTargetVisible();
        
        if (hasTarget && visionSubsystem.getTargetId() == VisionConstants.TARGET_TAG_ID) {
            double yaw = visionSubsystem.getTargetYaw();
            double area = visionSubsystem.getTargetArea();
            double distance = DistanceCalculator.getDistance(area);
            
            // Calculate drive commands
            rotationController.update(0, yaw);
            speedController.update(VisionConstants.TARGET_FOLLOW_DISTANCE, distance);
            double rotationSpeed = rotationController.getOutput() * SwerveConstants.DRIVE_SPEED * SwerveConstants.MAX_SPEED;
            double forwardSpeed = -speedController.getOutput() * SwerveConstants.DRIVE_SPEED * SwerveConstants.MAX_SPEED;

            // Publish the speeds
            SmartDashboard.putNumber("[VISION] Rotation Speed", rotationSpeed);
            SmartDashboard.putNumber("[VISION] Forward Speed", forwardSpeed);
            SmartDashboard.putNumber("[VISION] Predicted Distance From Tag", distance);
            
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
            rotationController.zero();
            speedController.zero();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}