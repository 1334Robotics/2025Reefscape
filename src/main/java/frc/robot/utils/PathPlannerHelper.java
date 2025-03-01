package frc.robot.utils;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.SwerveSubsystem;

/**
 * Memory-efficient helper class for PathPlanner path following and testing
 */
public class PathPlannerHelper {
    private final SwerveSubsystem swerveSubsystem;
    private final SendableChooser<String> pathChooser = new SendableChooser<>();
    private final List<String> pathNames = new ArrayList<>();
    private Command currentPathCommand = null;
    private boolean isPathRunning = false;
    private String selectedAutoPath = null;
    
    // Dashboard keys
    private static final String CANCEL_PATH_KEY = "Cancel Current Path";
    private static final String PATH_STATUS_KEY = "Path Status";
    private static final String CURRENT_PATH_KEY = "Current Path";
    private static final String RUN_PATH_PREFIX = "Run Path: ";
    private static final String MEMORY_KEY = "Free Memory MB";
    
    // Flag for detailed memory tracking (only needed in simulation)
    private final boolean detailedMemoryTracking;
    
    /**
     * Constructor for PathPlannerHelper
     * @param swerveSubsystem The swerve drive subsystem
     */
    public PathPlannerHelper(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        
        // Only do detailed memory tracking in simulation
        this.detailedMemoryTracking = RobotMode.isDebugEnabled();
        
        // Configure the AutoBuilder (only done once)
        AutoHelper.configureAutoBuilder(swerveSubsystem);
        
        // Find all path files (just the names, not loading them)
        findAllPaths();
        
        // Create path chooser for dashboard
        setupPathChooser();
        
        // Add dashboard buttons for each path
        addPathButtonsToDashboard();
        
        // Add cancel button to dashboard
        addCancelButtonToDashboard();
        
        // Initialize path status
        SmartDashboard.putString(PATH_STATUS_KEY, "No Path Running");
        SmartDashboard.putString(CURRENT_PATH_KEY, "None");
        
        // Only update memory status if debug is enabled
        if (detailedMemoryTracking) {
            updateMemoryStatus();
        }
    }
    
    /**
     * Find all path files in the deploy directory (by name only, no loading)
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
                        
                        // Only log in debug mode
                        if (RobotMode.isDebugEnabled()) {
                            DriverStation.reportWarning("Found path: " + pathName, false);
                        }
                    }
                    SmartDashboard.putNumber("Path Count", pathNames.size());
                }
            }
        } catch (Exception e) {
            DriverStation.reportError("Error finding path files: " + e.getMessage(), e.getStackTrace());
        }
    }
    
    /**
     * Set up the path chooser with all available paths (only names, not loading paths)
     */
    private void setupPathChooser() {
        // Add default option
        pathChooser.setDefaultOption("Do Nothing", "none");
        
        // Add each path as an option (just the name, not loading the path)
        for (String pathName : pathNames) {
            pathChooser.addOption("Path: " + pathName, pathName);
        }
        
        // Put chooser on dashboard
        SmartDashboard.putData("Auto Path", pathChooser);
    }
    
    /**
     * Load a path command on demand
     * @param pathName Name of the path to load
     * @return Command to follow the path, or null if loading fails
     */
    private Command loadPathCommand(String pathName) {
        if (pathName == null || pathName.isEmpty() || pathName.equals("none")) {
            return Commands.none();
        }
        
        if (detailedMemoryTracking) {
            updateMemoryStatus();
        }
        
        try {
            // Log starting path (only detailed logging in debug mode)
            if (RobotMode.isDebugEnabled()) {
                SmartDashboard.putString("Loading Path", pathName);
                DriverStation.reportWarning("Loading path: " + pathName, false);
            }
            
            // Load path from file - only done when explicitly requested
            long startTime = 0;
            if (detailedMemoryTracking) {
                startTime = System.currentTimeMillis();
            }
            
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            
            if (detailedMemoryTracking) {
                long loadTime = System.currentTimeMillis() - startTime;
                SmartDashboard.putNumber("Path Load Time ms", loadTime);
            }
            
            // Create path following command
            Command followCommand = AutoHelper.followPath(swerveSubsystem, path);
            
            // Wrap the command with status updates and memory cleanup
            return Commands.sequence(
                // Before path starts
                Commands.runOnce(() -> {
                    SmartDashboard.putString(CURRENT_PATH_KEY, pathName);
                    SmartDashboard.putString(PATH_STATUS_KEY, "Running: " + pathName);
                    isPathRunning = true;
                    
                    if (detailedMemoryTracking) {
                        updateMemoryStatus();
                    }
                }),
                // The actual path following command
                followCommand,
                // After path completes - clean up resources
                Commands.runOnce(() -> {
                    SmartDashboard.putString(CURRENT_PATH_KEY, "None");
                    SmartDashboard.putString(PATH_STATUS_KEY, "Completed: " + pathName);
                    isPathRunning = false;
                    currentPathCommand = null;
                    cleanupMemory(); // Request garbage collection
                })
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
            String buttonKey = RUN_PATH_PREFIX + pathName;
            SmartDashboard.putBoolean(buttonKey, false);
        }
    }
    
    /**
     * Add a cancel button to the dashboard
     */
    private void addCancelButtonToDashboard() {
        // Create a boolean dashboard entry for the cancel button
        SmartDashboard.putBoolean(CANCEL_PATH_KEY, false);
    }
    
    /**
     * Cancel the currently running path command
     */
    private void cancelCurrentPath() {
        if (isPathRunning && currentPathCommand != null) {
            if (RobotMode.isDebugEnabled()) {
                DriverStation.reportWarning("Cancelling current path", false);
            }
            
            currentPathCommand.cancel();
            currentPathCommand = null;
            isPathRunning = false;
            SmartDashboard.putString(PATH_STATUS_KEY, "Cancelled by User");
            SmartDashboard.putString(CURRENT_PATH_KEY, "None");
            cleanupMemory();
        }
    }
    
    /**
     * Request garbage collection to free up memory
     */
    private void cleanupMemory() {
        // Force garbage collection to clean up memory
        System.gc();
        
        // Only delay in simulation mode, not on the real robot
        if (RobotMode.isSimulation()) {
            Timer.delay(0.02); // Give a little time for GC to run
        }
        
        if (detailedMemoryTracking) {
            updateMemoryStatus();
        }
    }
    
    /**
     * Update memory status on dashboard
     */
    private void updateMemoryStatus() {
        // Skip if not in debug mode
        if (!detailedMemoryTracking) {
            return;
        }
        
        double freeMB = Runtime.getRuntime().freeMemory() / (1024.0 * 1024.0);
        double totalMB = Runtime.getRuntime().totalMemory() / (1024.0 * 1024.0);
        double usedMB = totalMB - freeMB;
        
        SmartDashboard.putNumber(MEMORY_KEY, freeMB);
        SmartDashboard.putNumber("Used Memory MB", usedMB);
        SmartDashboard.putNumber("Total Memory MB", totalMB);
        SmartDashboard.putNumber("Memory Usage %", (usedMB / totalMB) * 100.0);
    }
    
    /**
     * Run the specified path command
     * @param pathName Name of the path to run
     */
    private void runPath(String pathName) {
        // Don't start a new path if one is already running
        if (isPathRunning) {
            if (RobotMode.isDebugEnabled()) {
                DriverStation.reportWarning("Cannot start path " + pathName + " - another path is already running", false);
            }
            return;
        }
        
        // Load path on demand when requested
        Command pathCommand = loadPathCommand(pathName);
        if (pathCommand != null) {
            currentPathCommand = pathCommand;
            pathCommand.schedule();
        }
    }
    
    /**
     * Periodic method that should be called regularly to check dashboard buttons
     */
    public void periodic() {
        // Check cancel button
        boolean cancelRequested = SmartDashboard.getBoolean(CANCEL_PATH_KEY, false);
        if (cancelRequested) {
            // Reset button state
            SmartDashboard.putBoolean(CANCEL_PATH_KEY, false);
            cancelCurrentPath();
        }
        
        // Check if user selected a different path in the chooser
        String currentChoice = pathChooser.getSelected();
        if (currentChoice != null && !currentChoice.equals(selectedAutoPath)) {
            selectedAutoPath = currentChoice;
            SmartDashboard.putString("Selected Auto", selectedAutoPath);
        }
        
        // Only check path buttons if no path is currently running
        if (!isPathRunning) {
            for (String pathName : pathNames) {
                String buttonKey = RUN_PATH_PREFIX + pathName;
                boolean buttonPressed = SmartDashboard.getBoolean(buttonKey, false);
                
                if (buttonPressed) {
                    // Reset button state
                    SmartDashboard.putBoolean(buttonKey, false);
                    
                    if (RobotMode.isDebugEnabled()) {
                        DriverStation.reportWarning("Button pressed for path: " + pathName, false);
                    }
                    
                    runPath(pathName);
                    break; // Only run one path at a time
                }
            }
        }
        
        // Periodically update memory status (every ~1 second) - only in simulation
        if (detailedMemoryTracking && Timer.getFPGATimestamp() % 1.0 < 0.02) {
            updateMemoryStatus();
        }
    }
    
    /**
     * Get the autonomous command based on dashboard selection
     * Only loads the path when actually needed for autonomous
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        // Get the selected path name from the chooser
        String selectedPath = pathChooser.getSelected();
        
        // If "none" is selected, return an empty command
        if (selectedPath == null || selectedPath.equals("none")) {
            return Commands.none();
        }
        
        // Only load the path when it's actually needed for autonomous
        return loadPathCommand(selectedPath);
    }
    
    /**
     * Check if a path is currently running
     * @return True if a path is running, false otherwise
     */
    public boolean isPathRunning() {
        return isPathRunning;
    }
} 