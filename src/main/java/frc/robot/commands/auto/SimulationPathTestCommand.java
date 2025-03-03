package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/**
 * A command specifically designed to test PathPlanner paths in simulation.
 * This command will only run in simulation mode and provides additional
 * logging and visualization for testing paths.
 */
public class SimulationPathTestCommand extends SequentialCommandGroup {
    
    /**
     * Creates a new command to test a PathPlanner path in simulation
     * 
     * @param pathName The name of the PathPlanner path to test
     */
    public SimulationPathTestCommand(String pathName) {
        addCommands(
            // Print simulation test info
            Commands.runOnce(() -> {
                System.out.println("=== SIMULATION PATH TEST ===");
                System.out.println("Testing path: " + pathName);
                System.out.println("Is simulation: " + RobotBase.isSimulation());
                System.out.println("===========================");
                
                // Reset robot pose to starting position for consistent testing
                if (RobotBase.isSimulation()) {
                    RobotContainer.swerveSubsystem.resetPose(
                        RobotContainer.swerveSubsystem.getPose());
                }
            }),
            
            // Run the actual path
            new PathFollowerCommand(pathName),
            
            // Print completion message
            Commands.runOnce(() -> {
                System.out.println("=== PATH TEST COMPLETE ===");
                System.out.println("Path: " + pathName);
                System.out.println("Final pose: " + RobotContainer.swerveSubsystem.getPose());
                System.out.println("=========================");
            })
        );
    }
} 