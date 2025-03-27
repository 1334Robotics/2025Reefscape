package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import java.io.IOException;
import org.json.simple.parser.ParseException;

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
        // More conservative default constraints for better accuracy
        PathConstraints constraints = new PathConstraints(
                SwerveConstants.MAX_SPEED, // Max speed (m/s)
                SwerveConstants.MAX_SPEED * 0.5, // Max acceleration (m/s²)
                edu.wpi.first.math.util.Units.degreesToRadians(90), // angular velocity 
                edu.wpi.first.math.util.Units.degreesToRadians(180)  // angular acceleration
        );
        
        return pathfindToPose(targetPose, constraints, 0.0);
    }
}
