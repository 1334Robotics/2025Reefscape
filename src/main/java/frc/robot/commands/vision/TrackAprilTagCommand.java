package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveController.Controller;
import frc.robot.subsystems.controller.ControllerVibration;
import frc.robot.subsystems.drive.PIDController;

public class TrackAprilTagCommand extends Command {
    // private final PIDController rotationController;
    // private final PIDController forwardController;
    // private final PIDController horizontalController;
    // private int targetTag;
    // private double targetX;
    // private double targetY;
    // private boolean enabled;
    // private int framesSinceLastSeen;

    // public TrackAprilTagCommand() {
    //     this.targetTag = -1;
    //     this.enabled = false;
    //     this.framesSinceLastSeen = 0;

    //     // Initialize the PID controllers
    //     rotationController   = new PIDController(VisionConstants.ROTATION_KP,
    //                                              VisionConstants.ROTATION_KI,
    //                                              VisionConstants.ROTATION_KD,
    //                                              VisionConstants.ROTATION_TAU,
    //                                              // These are the limMin and limMax terms
    //                                              // Which should allow us to simply multiply the output of the PID controller
    //                                              // by the SwerveConstants.DRIVE_SPEED and the SwerveConstants.
    //                                              -1,
    //                                              1, 
    //                                              VisionConstants.ROTATION_LIM_MIN_INT,
    //                                              VisionConstants.ROTATION_LIM_MAX_INT,
    //                                              1 /* The sampleTime which should never need to be changed (it doesn't do much) */);
    //     forwardController   = new PIDController(VisionConstants.FORWARDS_KP,
    //                                             VisionConstants.FORWARDS_KI,
    //                                             VisionConstants.FORWARDS_KD,
    //                                             VisionConstants.FORWARDS_TAU,
    //                                             -1,
    //                                             1,
    //                                             VisionConstants.FORWARDS_LIM_MIN_INT,
    //                                             VisionConstants.FORWARDS_LIM_MAX_INT,
    //                                             1);
    //     horizontalController = new PIDController(VisionConstants.HORIZONTAL_KP,
    //                                              VisionConstants.HORIZONTAL_KI,
    //                                              VisionConstants.HORIZONTAL_KD,
    //                                              VisionConstants.HORIZONTAL_TAU,
    //                                              -1,
    //                                              1,
    //                                              VisionConstants.HORIZONTAL_LIM_MIN_INT,
    //                                              VisionConstants.HORIZONTAL_LIM_MAX_INT,
    //                                              1);

    //     this.setRelativeDistance(new Distance(-1, -1));

    //     addRequirements(RobotContainer.swerveSubsystem, RobotContainer.visionSubsystem);
    // }

    // public TrackAprilTagCommand(int targetTag, Distance relativeDistance) {
    //     this.targetTag = targetTag;
    //     this.enabled = false;
    //     this.framesSinceLastSeen = 0;

    //     // Initialize the PID controllers
    //     rotationController   = new PIDController(VisionConstants.ROTATION_KP,
    //                                              VisionConstants.ROTATION_KI,
    //                                              VisionConstants.ROTATION_KD,
    //                                              VisionConstants.ROTATION_TAU,
    //                                              // These are the limMin and limMax terms
    //                                              // Which should allow us to simply multiply the output of the PID controller
    //                                              // by the SwerveConstants.DRIVE_SPEED and the SwerveConstants.
    //                                              -1,
    //                                              1, 
    //                                              VisionConstants.ROTATION_LIM_MIN_INT,
    //                                              VisionConstants.ROTATION_LIM_MAX_INT,
    //                                              1 /* The sampleTime which should never need to be changed (it doesn't do much) */);
    //     forwardController   = new PIDController(VisionConstants.FORWARDS_KP,
    //                                             VisionConstants.FORWARDS_KI,
    //                                             VisionConstants.FORWARDS_KD,
    //                                             VisionConstants.FORWARDS_TAU,
    //                                             -1,
    //                                             1,
    //                                             VisionConstants.FORWARDS_LIM_MIN_INT,
    //                                             VisionConstants.FORWARDS_LIM_MAX_INT,
    //                                             1);
    //     horizontalController = new PIDController(VisionConstants.HORIZONTAL_KP,
    //                                              VisionConstants.HORIZONTAL_KI,
    //                                              VisionConstants.HORIZONTAL_KD,
    //                                              VisionConstants.HORIZONTAL_TAU,
    //                                              -1,
    //                                              1,
    //                                              VisionConstants.HORIZONTAL_LIM_MIN_INT,
    //                                              VisionConstants.HORIZONTAL_LIM_MAX_INT,
    //                                              1);

    //     this.setRelativeDistance(relativeDistance);

    //     addRequirements(RobotContainer.swerveSubsystem, RobotContainer.visionSubsystem);
    // }

    // public void setRelativeDistance(Distance relativeDistance) {
    //     // Treat as if the target was lost
    //     RobotContainer.driveController.drive(Controller.TAGTRACKING, new Translation2d(0, 0), 0);
    //     SmartDashboard.putString("Vision/Status", "NO TARGET");
    //     rotationController.zero();
    //     forwardController.zero();
    //     horizontalController.zero();

    //     this.targetX   = relativeDistance.x;
    //     this.targetY   = relativeDistance.y;
    // }

    // public void setTargetTag(int targetTag) {
    //     RobotContainer.driveController.drive(Controller.TAGTRACKING, new Translation2d(0, 0), 0);
    //     SmartDashboard.putString("Vision/Status", "NO TARGET");
    //     rotationController.zero();
    //     forwardController.zero();
    //     horizontalController.zero();

    //     this.targetTag = targetTag;
    // }

    // public void enable() {
    //     SmartDashboard.putString("[VISION] Good Stop", "Working");
    //     RobotContainer.driveController.requestControl(Controller.TAGTRACKING);
    //     this.enabled = true;
    // }

    // public void disable() {
    //     RobotContainer.driveController.drive(Controller.TAGTRACKING, new Translation2d(0, 0), 0);
    //     RobotContainer.driveController.relinquishControl(Controller.TAGTRACKING);
    //     SmartDashboard.putString("Vision/Status", "NO TARGET");
    //     rotationController.zero();
    //     forwardController.zero();
    //     horizontalController.zero();
    //     this.enabled = false;
    //     RobotContainer.tagTrackingHandler.setTarget(null);
    // }

    // private boolean withinAcceptableError(double angle, Distance distance) {
    //     if(Math.abs(180 - angle)               >= VisionConstants.MAX_ALLOWED_ROTATION_ERROR)   return false;
    //     if(Math.abs(distance.y - this.targetY) >= VisionConstants.MAX_ALLOWED_FORWARDS_ERROR)   return false;
    //     if(Math.abs(distance.x - this.targetX) >= VisionConstants.MAX_ALLOWED_HORIZONTAL_ERROR) return false;
    //     return true;
    // }

    // public boolean isAligned() {
    //     if (!enabled || !RobotContainer.visionSubsystem.isTargetVisible()) {
    //         return false;
    //     }
        
    //     double angle = RobotContainer.visionSubsystem.getTargetAngle();
    //     if (angle < 0) angle = 180 - (-180 - angle);
        
    //     Distance distance = RobotContainer.visionSubsystem.getDistanceAway();
        
    //     return withinAcceptableError(angle, distance);
    // }

    // @Override
    // public void execute() {
    //     // Ensure that it is initialized
    //     if(this.targetTag == -1 || (this.targetX == -1 && this.targetY == -1)) return;

    //     SmartDashboard.putBoolean("[VISION] Tracking Enabled", this.enabled);
    //     if(!this.enabled) return;

    //     // Disable if PhotonVision has disconnected
    //     if(RobotContainer.visionSubsystem.getImageAge() > VisionConstants.MAX_ACCEPTABLE_DELAY) {
    //         this.disable();
    //         SmartDashboard.putString("[VISION] Good Stop", "No");
    //     }

    //     boolean hasTarget = RobotContainer.visionSubsystem.isTargetVisible();
    //     if(!hasTarget) {
    //         if(this.framesSinceLastSeen++ > VisionConstants.MAX_ALLOWED_BLANK_FRAMES) {
    //             this.disable();
    //             SmartDashboard.putString("[VISION] Good Stop", "No");
    //         }
    //     } else this.framesSinceLastSeen = 0;
        
    //     if(RobotContainer.visionSubsystem.getTargetId() == this.targetTag) {
    //         double angle      = RobotContainer.visionSubsystem.getTargetAngle();
    //         Distance distance = RobotContainer.visionSubsystem.getDistanceAway();

    //         // Change angle from (-180 to 180) to (0 to 360) ensuring that -180 and 180 are the same
    //         if(angle < 0) angle = 180 - (-180 - angle);

    //         // Publish the distance
    //         SmartDashboard.putNumber("[VISION] Target X", this.targetX);
    //         SmartDashboard.putNumber("[VISION] Target Y", this.targetY);
    //         SmartDashboard.putNumber("[VISION] Distance X", distance.x);
    //         SmartDashboard.putNumber("[VISION] Distance Y", distance.y);
    //         SmartDashboard.putNumber("[VISION] Error X", distance.x - this.targetX);
    //         SmartDashboard.putNumber("[VISION] Error Y", distance.y - this.targetY);
    //         SmartDashboard.putNumber("[VISION] Target Angle", angle);

    //         // Check if the error is low enough and stop it
    //         if(this.withinAcceptableError(angle, distance)) {
    //             this.disable();
    //             SmartDashboard.putString("[VISION] Good Stop", "Yes");
    //             RobotContainer.driverControllerSubsystem.vibrate(ControllerVibration.LIGHT);
    //         }
            
    //         // Calculate drive commands
    //         rotationController.update(180, angle);
    //         forwardController.update(this.targetY, distance.y);
    //         horizontalController.update(this.targetX, distance.x);
    //         double rotationSpeed   = -rotationController.getOutput() * VisionConstants.ROTATION_SPEED * SwerveConstants.MAX_SPEED;
    //         double forwardSpeed    = -forwardController.getOutput() * VisionConstants.DRIVE_SPEED * SwerveConstants.MAX_SPEED;
    //         double horizontalSpeed = -horizontalController.getOutput() * VisionConstants.DRIVE_SPEED * SwerveConstants.MAX_SPEED;

    //         // Cancel speeds if the robot is within acceptable error
    //         if(Math.abs(180 - angle)               < VisionConstants.MAX_ALLOWED_ROTATION_ERROR)   rotationSpeed   = 0;
    //         if(Math.abs(distance.y - this.targetY) < VisionConstants.MAX_ALLOWED_FORWARDS_ERROR)   forwardSpeed    = 0;
    //         if(Math.abs(distance.x - this.targetX) < VisionConstants.MAX_ALLOWED_HORIZONTAL_ERROR) horizontalSpeed = 0;

    //         // Publish the speeds
    //         SmartDashboard.putNumber("[VISION] Rotation Speed", rotationSpeed);
    //         SmartDashboard.putNumber("[VISION] Forward Speed", forwardSpeed);
            
    //         // Drive robot
    //         RobotContainer.driveController.drive(Controller.TAGTRACKING,
    //             new Translation2d(forwardSpeed, horizontalSpeed),
    //             rotationSpeed
    //         );
            
    //         // Log status
    //         SmartDashboard.putString("Vision/Status", "FOLLOWING TAG " + this.targetTag);
    //     } else {
    //         // Stop if wrong tag or no target
    //         this.disable();
    //         SmartDashboard.putString("[VISION] Good Stop", "No");
    //     }
    // }

    // @Override
    // public boolean isFinished() {
    //     return false;
    // }
}