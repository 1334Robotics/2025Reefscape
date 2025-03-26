package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import frc.robot.constants.AutoConstants;

/**
 * A utility class for creating pathfinding commands using PathPlanner's AutoBuilder.
 */
public class Pathfind {
    
    /**
     * Creates a pathfinding command to navigate to a specified pose.
     * 
     * @param targetPose The target pose to navigate to
     * @param constraints The path constraints to use
     * @param goalEndVelocity The goal end velocity in meters/sec
     * @return A command that will navigate to the specified pose
     */
    public static Command pathfindToPose(
            Pose2d targetPose,
            PathConstraints constraints,
            double goalEndVelocity) {
        
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                goalEndVelocity
        );
    }
    
    /**
     * Creates a pathfinding command to navigate to a specified pose.
     * 
     * @param targetPose The target pose to navigate to
     * @param constraints The path constraints to use
     * @param goalEndVelocity The goal end velocity in meters/sec
     * @param rotationDelayDistance How far the robot should travel before attempting to rotate
     * @return A command that will navigate to the specified pose
     */
    public static Command pathfindToPose(
            Pose2d targetPose,
            PathConstraints constraints,
            double goalEndVelocity,
            double rotationDelayDistance) {
        
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                goalEndVelocity
        );
    }
    
    /**
     * Creates a pathfinding command to navigate to a specified pose with default constraints.
     * 
     * @param targetPose The target pose to navigate to
     * @return A command that will navigate to the specified pose
     */
    public static Command pathfindToPose(Pose2d targetPose) {
        // Use default constraints from AutoConstants or create your own defaults
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                edu.wpi.first.math.util.Units.degreesToRadians(540), 
                edu.wpi.first.math.util.Units.degreesToRadians(720));
        
        return pathfindToPose(targetPose, constraints, 0.0);
    }
}
