package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.PIDController;

public class TrackAprilTagCommand extends Command {
    private final PIDController rotationController;
    private final PIDController forwardController;
    private final PIDController horizontalController;
    private int targetTag;
    private double targetX;
    private double targetY;

    public TrackAprilTagCommand(int targetTag, Distance relativeDistance) {
        this.targetTag = targetTag;
        this.targetX = relativeDistance.x;
        this.targetY = relativeDistance.y;

        // Initialize the PID controllers
        rotationController   = new PIDController(VisionConstants.ROTATION_KP,
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
        forwardController   = new PIDController(VisionConstants.DRIVE_KP,
                                                VisionConstants.DRIVE_KI,
                                                VisionConstants.DRIVE_KD,
                                                VisionConstants.DRIVE_TAU,
                                                -1,
                                                1,
                                                VisionConstants.DRIVE_LIM_MIN_INT,
                                                VisionConstants.DRIVE_LIM_MAX_INT,
                                                1);
        horizontalController = new PIDController(VisionConstants.DRIVE_KP,
                                                 VisionConstants.DRIVE_KI,
                                                 VisionConstants.DRIVE_KD,
                                                 VisionConstants.DRIVE_TAU,
                                                 -1,
                                                 1,
                                                 VisionConstants.DRIVE_LIM_MIN_INT,
                                                 VisionConstants.DRIVE_LIM_MAX_INT,
                                                 1);

        addRequirements(RobotContainer.swerveSubsystem, RobotContainer.visionSubsystem);
    }

    public void setRelativeDistance(Distance relativeDistance) {
        // Treat as if the target was lost
        RobotContainer.swerveSubsystem.drive(new Translation2d(0, 0), 0);
        SmartDashboard.putString("Vision/Status", "NO TARGET");
        rotationController.zero();
        forwardController.zero();
        horizontalController.zero();

        this.targetX = relativeDistance.x;
        this.targetY = relativeDistance.y;
    }

    public void setTargetTag(int targetTag) {
        RobotContainer.swerveSubsystem.drive(new Translation2d(0, 0), 0);
        SmartDashboard.putString("Vision/Status", "NO TARGET");
        rotationController.zero();
        forwardController.zero();
        horizontalController.zero();

        this.targetTag = targetTag;
    }

    @Override
    public void execute() {
        boolean hasTarget = RobotContainer.visionSubsystem.isTargetVisible();
        
        if(hasTarget
        && RobotContainer.visionSubsystem.getTargetId() == this.targetTag
        && RobotContainer.visionSubsystem.getImageAge() <= VisionConstants.MAX_ACCEPTABLE_DELAY) {
            double yaw        = RobotContainer.visionSubsystem.getTargetYaw();
            double pitch      = RobotContainer.visionSubsystem.getTargetPitch();
            double area       = RobotContainer.visionSubsystem.getTargetArea();
            Distance distance = DistanceCalculator.getDistance(yaw, pitch, area);

            // Publish the distance
            SmartDashboard.putNumber("[VISION] Distance X", distance.x);
            SmartDashboard.putNumber("[VISION] Distance Y", distance.y);
            
            // Calculate drive commands
            rotationController.update(0, yaw);
            forwardController.update(this.targetY, distance.y);
            horizontalController.update(this.targetX, distance.x);
            double rotationSpeed   = rotationController.getOutput() * VisionConstants.ROTATION_SPEED * SwerveConstants.MAX_SPEED;
            double forwardSpeed    = -forwardController.getOutput() * VisionConstants.DRIVE_SPEED * SwerveConstants.MAX_SPEED;
            double horizontalSpeed = horizontalController.getOutput() * VisionConstants.DRIVE_SPEED * SwerveConstants.MAX_SPEED;

            // Magic numbers (bad)
            if(Math.abs(distance.x - this.targetX) < 2
            && Math.abs(distance.y - this.targetY) < 1) return;

            // Publish the speeds
            SmartDashboard.putNumber("[VISION] Rotation Speed", rotationSpeed);
            SmartDashboard.putNumber("[VISION] Forward Speed", forwardSpeed);
            
            // Drive robot
            RobotContainer.swerveSubsystem.drive(
                new Translation2d(forwardSpeed, horizontalSpeed),
                rotationSpeed
            );
            
            // Log status
            SmartDashboard.putString("Vision/Status", "FOLLOWING TAG " + this.targetTag);
        } else {
            // Stop if wrong tag or no target
            RobotContainer.swerveSubsystem.drive(new Translation2d(0, 0), 0);
            SmartDashboard.putString("Vision/Status", "NO TARGET");
            rotationController.zero();
            forwardController.zero();
            horizontalController.zero();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}