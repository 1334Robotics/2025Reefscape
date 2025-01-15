package frc.robot;

import frc.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    // The robot's subsystems - only keep what we're using
    private final VisionSubsystem m_visionSubsystem;

    public RobotContainer() {
        System.out.println("Initializing RobotContainer");
        m_visionSubsystem = new VisionSubsystem();
        configureBindings();
    }

    private void configureBindings() {
        // We'll add keyboard bindings later if needed
    }

    public Command getAutonomousCommand() {
        // No auto command for now
        return null;
    }
}