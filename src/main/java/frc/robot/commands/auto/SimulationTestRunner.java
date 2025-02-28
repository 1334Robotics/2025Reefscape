package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/**
 * A command that runs a predefined sequence of paths in simulation for testing.
 * This allows for automated testing without requiring joystick input.
 */
public class SimulationTestRunner extends SequentialCommandGroup {

    /**
     * Creates a new command to run a test sequence in simulation
     */
    public SimulationTestRunner() {
        addCommands(
            // Print test sequence start message
            Commands.runOnce(() -> {
                System.out.println("=== STARTING SIMULATION TEST SEQUENCE ===");
                System.out.println("This sequence will run multiple paths in succession");
                System.out.println("No controller input is required");
                System.out.println("========================================");
                
                // Reset robot state
                if (RobotBase.isSimulation()) {
                    RobotContainer.gyroSubsystem.zero();
                    RobotContainer.swerveSubsystem.resetPose(
                        RobotContainer.swerveSubsystem.getPose());
                }
            }),
            
            // Wait a bit before starting
            Commands.waitSeconds(1.0),
            
            // Run the first path
            new PathFollowerCommand("AutoDrive20"),
            
            // Pause between paths
            Commands.waitSeconds(1.0),
            
            // Run the second path
            new PathFollowerCommand("AutoDrive22"),
            
            // Print completion message
            Commands.runOnce(() -> {
                System.out.println("=== SIMULATION TEST SEQUENCE COMPLETE ===");
                System.out.println("All paths have been executed");
                System.out.println("Final pose: " + RobotContainer.swerveSubsystem.getPose());
                System.out.println("=========================================");
            })
        );
    }
} 