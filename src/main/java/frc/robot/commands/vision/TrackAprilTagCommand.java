package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveController.Controller;
import frc.robot.subsystems.drive.PIDController;

public class TrackAprilTagCommand extends Command {
    private final PIDController rotationController;
    private final PIDController forwardController;
    private final PIDController horizontalController;
    private int targetTag;
    private double targetX;
    private double targetY;
    private boolean enabled;

    // Explain this later
    private double targetYaw;

    public TrackAprilTagCommand(int targetTag, Distance relativeDistance) {
        this.targetTag = targetTag;
        this.enabled = false;

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

        this.setRelativeDistance(relativeDistance);

        addRequirements(RobotContainer.swerveSubsystem, RobotContainer.visionSubsystem);
    }

    public void setRelativeDistance(Distance relativeDistance) {
        // Treat as if the target was lost
        RobotContainer.driveController.drive(Controller.TAGTRACKING, new Translation2d(0, 0), 0);
        SmartDashboard.putString("Vision/Status", "NO TARGET");
        rotationController.zero();
        forwardController.zero();
        horizontalController.zero();

        this.targetX   = relativeDistance.x;
        this.targetY   = relativeDistance.y;
        this.targetYaw = Math.atan2(this.targetX, this.targetY) * (180.0 / Math.PI);
        SmartDashboard.putNumber("[VISION] Target Yaw", this.targetYaw);
    }

    public void setTargetTag(int targetTag) {
        RobotContainer.driveController.drive(Controller.TAGTRACKING, new Translation2d(0, 0), 0);
        SmartDashboard.putString("Vision/Status", "NO TARGET");
        rotationController.zero();
        forwardController.zero();
        horizontalController.zero();

        this.targetTag = targetTag;
    }

    public void enable() {
        this.enabled = true;
    }

    public void disable() {
        this.enabled = false;
    }

    @Override
    public void execute() {
        if(!this.enabled) return;

        boolean hasTarget = RobotContainer.visionSubsystem.isTargetVisible();
        
        if(hasTarget
        && RobotContainer.visionSubsystem.getTargetId() == this.targetTag
        && RobotContainer.visionSubsystem.getImageAge() <= VisionConstants.MAX_ACCEPTABLE_DELAY) {
            RobotContainer.driveController.requestControl(Controller.TAGTRACKING);
            double angle      = RobotContainer.visionSubsystem.getTargetAngle();
            Distance distance = RobotContainer.visionSubsystem.getDistanceAway();

            // Change angle from (-180 to 180) to (0 to 360) ensuring that -180 and 180 are the same
            if(angle < 0) angle = 180 - (-180 - angle);

            // Publish the distance
            SmartDashboard.putNumber("[VISION] Distance X", distance.x);
            SmartDashboard.putNumber("[VISION] Distance Y", distance.y);
            SmartDashboard.putNumber("[VISION] Target Angle", angle);
            
            // Calculate drive commands
            rotationController.update(180, angle);
            forwardController.update(this.targetY, distance.y);
            horizontalController.update(this.targetX, distance.x);
            double rotationSpeed   = -rotationController.getOutput() * VisionConstants.ROTATION_SPEED * SwerveConstants.MAX_SPEED;
            double forwardSpeed    = -forwardController.getOutput() * VisionConstants.DRIVE_SPEED * SwerveConstants.MAX_SPEED;
            double horizontalSpeed = -horizontalController.getOutput() * VisionConstants.DRIVE_SPEED * SwerveConstants.MAX_SPEED;

            // Magic numbers (bad)
            if(Math.abs(180 - angle) < 1)               rotationSpeed = 0;
            if(Math.abs(distance.y - this.targetY) < 1) forwardSpeed = 0;
            if(Math.abs(distance.x - this.targetX) < 1) horizontalSpeed = 0;

            // Publish the speeds
            SmartDashboard.putNumber("[VISION] Rotation Speed", rotationSpeed);
            SmartDashboard.putNumber("[VISION] Forward Speed", forwardSpeed);
            
            // Drive robot
            RobotContainer.driveController.drive(Controller.TAGTRACKING,
                new Translation2d(forwardSpeed, horizontalSpeed),
                rotationSpeed
            );
            
            // Log status
            SmartDashboard.putString("Vision/Status", "FOLLOWING TAG " + this.targetTag);
        } else {
            // Stop if wrong tag or no target
            RobotContainer.driveController.drive(Controller.TAGTRACKING, new Translation2d(0, 0), 0);
            RobotContainer.driveController.relinquishControl(Controller.TAGTRACKING);
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