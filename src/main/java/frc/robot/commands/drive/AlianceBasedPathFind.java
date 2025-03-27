package frc.robot.commands.drive;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import frc.robot.constants.SwerveConstants;
import com.pathplanner.lib.util.FileVersionException;

public class AlianceBasedPathFind {
    /**
     * Creates a command that pathfinds to the start of a path and then follows it.
     * Automatically selects the appropriate path based on current alliance.
     * 
     * @param pathName The name of the path file to load (without folder prefix)
     * @return A command that will pathfind to and follow the specified path
     */
    public static Command pathfindThenFollowPath(String pathName) {
        try {
            // Determine current alliance with improved debugging
            var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
            boolean isBlueAlliance = alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
            
            System.out.println("Alliance present: " + alliance.isPresent() + 
                              ", Alliance: " + (alliance.isPresent() ? alliance.get().toString() : "UNKNOWN") +
                              ", isBlueAlliance: " + isBlueAlliance);

            // Load the path directly from the path name without folder prefix
            System.out.println("Loading path: " + pathName);
            
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            // Only flip the path if we're on Blue alliance
            if (isBlueAlliance) {
                System.out.println("Not Flipping path for Blue alliance");
            } else {
                System.out.println("Flipping path (Red alliance or unknown)");
                path = path.flipPath();
                // Although we are using red aliance path for some reason its fliped already, so I have to flip it for Red Aliance won't even question why @_@ if it works it works
            }
            
            // Use default constraints
            PathConstraints constraints = new PathConstraints(
                    SwerveConstants.MAX_SPEED,
                    SwerveConstants.MAX_SPEED * 0.5,
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720)
            );
            
            return AutoBuilder.pathfindThenFollowPath(path, constraints);
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Error loading path file for alliance: " + pathName, e);
        }
    }
    /**
     * Creates a command that pathfinds to the start of a path and then follows it.
     * 
     * @param pathName The name of the path file to load
     * @param constraints The path constraints to use while pathfinding
     * @return A command that will pathfind to and follow the specified path
     * @throws org.json.simple.parser.ParseException 
     * @throws FileVersionException 
     */
    public static Command pathfindThenFollowPath(String pathName, PathConstraints constraints) throws FileVersionException, org.json.simple.parser.ParseException {
        try {
            // Load the path from the specified file
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            
            return AutoBuilder.pathfindThenFollowPath(path, constraints);
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Error loading path file: " + pathName, e);
        }
    }
    /**
     * Creates a command that pathfinds to the start of a path and then follows it.
     * 
     * @param path The path to follow
     * @param constraints The path constraints to use while pathfinding
     * @return A command that will pathfind to and follow the specified path
     */
    public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints constraints) {
        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    }}