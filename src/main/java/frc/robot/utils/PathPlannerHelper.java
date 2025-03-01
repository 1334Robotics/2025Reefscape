package frc.robot.utils;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.SwerveSubsystem;

/**
 * Helper class for PathPlanner path following and testing
 */
public class PathPlannerHelper {
    private final SwerveSubsystem swerveSubsystem;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final List<String> pathNames = new ArrayList<>();
    
    /**
     * Constructor for PathPlannerHelper
     * @param swerveSubsystem The swerve drive subsystem
     */
    public PathPlannerHelper(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        
        // Configure the AutoBuilder
        AutoHelper.configureAutoBuilder(swerveSubsystem);
        
        // Find all path files
        findAllPaths();
        
        // Create auto chooser for dashboard
        setupAutoChooser();
        
        // Add dashboard buttons for each path
        addPathButtonsToDashboard();
    }
    
    /**
     * Find all path files in the deploy directory
     */
    private void findAllPaths() {
        try {
            // Scan the paths directory to find all .path files
            File pathsDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
            if (pathsDirectory.exists() && pathsDirectory.isDirectory()) {
                File[] pathFiles = pathsDirectory.listFiles((dir, name) -> name.endsWith(".path"));
                
                if (pathFiles != null) {
                    for (File file : pathFiles) {
                        String pathName = file.getName().replace(".path", "");
                        pathNames.add(pathName);
                        DriverStation.reportWarning("Found path: " + pathName, false);
                    }
                }
            }
        } catch (Exception e) {
            DriverStation.reportError("Error finding path files: " + e.getMessage(), e.getStackTrace());
        }
    }
    
    /**
     * Set up the auto chooser with all available paths
     */
    private void setupAutoChooser() {
        // Add default empty command
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        
        // Add each path as an option
        for (String pathName : pathNames) {
            Command pathCommand = loadPathCommand(pathName); 
            if (pathCommand != null) {
                autoChooser.addOption("Path: " + pathName, pathCommand);
            }
        }
        
        // Put chooser on dashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
    }
    
    /**
     * Load a path command directly
     * @param pathName Name of the path to load
     * @return Command to follow the path, or null if loading fails
     */
    private Command loadPathCommand(String pathName) {
        try {
            // Log starting path
            SmartDashboard.putString("Loading Path", pathName);
            DriverStation.reportWarning("Loading path: " + pathName, false);
            
            // Load path from file
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            
            // Create path following command
            Command followCommand = AutoHelper.followPath(swerveSubsystem, path);
            
            // Wrap the command
            return Commands.sequence(
                Commands.runOnce(() -> SmartDashboard.putString("Current Path", pathName)),
                followCommand,
                Commands.runOnce(() -> SmartDashboard.putString("Current Path", "None"))
            );
        } catch (Exception e) {
            DriverStation.reportError("Error loading path " + pathName + ": " + e.getMessage(), e.getStackTrace());
            return null;
        }
    }
    
    /**
     * Add buttons to the dashboard for each path
     */
    private void addPathButtonsToDashboard() {
        for (String pathName : pathNames) {
            // Create a boolean dashboard entry for each path
            String buttonKey = "Run Path: " + pathName;
            SmartDashboard.putBoolean(buttonKey, false);
        }
    }
    
    /**
     * Periodic method that should be called regularly to check dashboard buttons
     */
    public void periodic() {
        // Check dashboard buttons for each path
        for (String pathName : pathNames) {
            String buttonKey = "Run Path: " + pathName;
            boolean buttonPressed = SmartDashboard.getBoolean(buttonKey, false);
            
            if (buttonPressed) {
                // Reset button state
                SmartDashboard.putBoolean(buttonKey, false);
                DriverStation.reportWarning("Button pressed for path: " + pathName, false);
                
                // Load and run the path command
                Command pathCommand = loadPathCommand(pathName);
                if (pathCommand != null) {
                    pathCommand.schedule();
                }
            }
        }
    }
    
    /**
     * Get the currently selected autonomous command
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        Command selectedCommand = autoChooser.getSelected();
        if (selectedCommand == null) {
            return Commands.none();
        }
        return selectedCommand;
    }
} 