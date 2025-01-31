package frc.robot.commands.vision;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Command that continuously prints target information to the console.
 */
public class PrintTargetInfo extends Command {

    private final VisionSubsystem visionSystem;

    public PrintTargetInfo(VisionSubsystem visionSystem) {
        this.visionSystem = visionSystem;
        // Declare subsystem dependencies
        addRequirements(visionSystem);
    }

    @Override
    public void execute() {
        if (visionSystem.isTargetVisible()) {
            double yaw = visionSystem.getTargetYaw();
            System.out.println("Target visible! Yaw: " + yaw + " degrees");
            double pitch = visionSystem.getTargetPitch();
            System.out.println("Target visible! Pitch: " + pitch + " degrees");
            double poseAmbiguity = visionSystem.getTargetPoseAmbiguity();
            System.out.println("Target visible! Pose Ambiguity: " + poseAmbiguity + " percent");
            double area = visionSystem.getTargetArea();
            System.out.println("Target visible! Area: " + area + " percent");
            double skew = visionSystem.getTargetSkew();
            System.out.println("Target visible! Skew: " + skew + " degrees");
            Integer id = visionSystem.getTargetId();
            System.out.println("Target visible! ID is : " + id );
            
        } else {
            System.out.println("No target visible.");
        }
    }

    @Override
    public boolean isFinished() {
        // This command never ends on its own; runs until interrupted
        return false;
    }
    
}
