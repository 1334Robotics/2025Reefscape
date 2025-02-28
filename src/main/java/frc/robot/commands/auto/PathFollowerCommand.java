package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * A command wrapper around PathPlannerAuto to add custom setup and
 * logging for path following.
 */
public class PathFollowerCommand extends WrapperCommand {
    private final String pathName;

    /**
     * Creates a new command to follow a PathPlanner path
     * 
     * @param pathName The name of the PathPlanner path to follow
     */
    public PathFollowerCommand(String pathName) {
        super(createPathCommand(pathName));
        this.pathName = pathName;
    }

    /**
     * Creates the path following command with proper initialization and teardown
     * 
     * @param pathName Name of the path to follow
     * @return Command to follow the path
     */
    private static Command createPathCommand(String pathName) {
        return new PathPlannerAuto(pathName)
            .beforeStarting(() -> {
                // Make sure we're in field-relative mode for path following
                RobotContainer.swerveSubsystem.setFieldRelative(true);
                System.out.println("[PathFollower] Starting path: " + pathName);
            })
            .finallyDo((interrupted) -> {
                // Log completion status
                if (interrupted) {
                    System.out.println("[PathFollower] Path interrupted: " + pathName);
                } else {
                    System.out.println("[PathFollower] Path completed: " + pathName);
                }
            });
    }

    @Override
    public void initialize() {
        // Print alliance information for debugging
        if (DriverStation.getAlliance().isPresent()) {
            System.out.println("[PathFollower] Alliance: " + DriverStation.getAlliance().get().toString());
        } else {
            System.out.println("[PathFollower] Alliance: Unknown");
        }
        
        // For simulation, log that we're in simulation mode
        if (RobotBase.isSimulation()) {
            System.out.println("[PathFollower] Running in simulation mode");
        }
        
        super.initialize();
    }

    @Override
    public String getName() {
        return "PathFollower(" + pathName + ")";
    }
} 