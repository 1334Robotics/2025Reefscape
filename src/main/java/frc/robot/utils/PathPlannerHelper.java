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
import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;

/**
 * Memory-efficient helper class for PathPlanner path following and testing
 */
public class PathPlannerHelper {
    // Subsystem and state tracking
    private final SwerveSubsystem swerveSubsystem;
    private final SendableChooser<String> pathChooser = new SendableChooser<>();
    private final List<String> pathNames = new ArrayList<>(20); // Pre-sized to avoid resizing
    private Command currentPathCommand = null;
    private boolean isPathRunning = false;
    private String selectedAutoPath = null;
    
    // Dashboard keys - using constants for consistency
    private static final String CANCEL_PATH_KEY = "Cancel Current Path";
    private static final String PATH_STATUS_KEY = "Path Status";
    private static final String CURRENT_PATH_KEY = "Current Path";
    private static final String RUN_PATH_PREFIX = "Run Path: ";
    private static final String MEMORY_KEY = "Free Memory MB";
    private static final String USED_MEMORY_KEY = "Used Memory MB";
    private static final String TOTAL_MEMORY_KEY = "Total Memory MB";
    private static final String MEMORY_USAGE_PERCENT_KEY = "Memory Usage %";
    private static final String PATH_COUNT_KEY = "Path Count";
    private static final String LOADING_PATH_KEY = "Loading Path";
    private static final String PATH_LOAD_TIME_KEY = "Path Load Time ms";
    private static final String SELECTED_AUTO_KEY = "Selected Auto";
    
    // Flag for detailed memory tracking (only needed in simulation)
    private final boolean isDetailedMemoryTrackingEnabled;
    
    // Used for throttling dashboard updates
    private long lastMemoryUpdateTime = 0;
    private static final long MEMORY_UPDATE_INTERVAL_MS = 500; // Update every 500ms
    
    /**
     * Constructor for PathPlannerHelper
     * @param swerveSubsystem The swerve drive subsystem
     */
    public PathPlannerHelper(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        
        // Only do detailed memory tracking in simulation and not in competition mode
        this.isDetailedMemoryTrackingEnabled = RobotMode.isDebugEnabled();
        
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
        if (isDetailedMemoryTrackingEnabled) {
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
                    SmartDashboard.putNumber(PATH_COUNT_KEY, pathNames.size());
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
        
        if (isDetailedMemoryTrackingEnabled) {
            updateMemoryStatus();
        }
        
        try {
            // Log starting path (only detailed logging in debug mode)
            if (RobotMode.isDebugEnabled()) {
                SmartDashboard.putString(LOADING_PATH_KEY, pathName);
                DriverStation.reportWarning("Loading path: " + pathName, false);
            }
            
            // Load path from file - only done when explicitly requested
            long startTime = 0;
            if (isDetailedMemoryTrackingEnabled) {
                startTime = System.currentTimeMillis();
            }
            
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            
            if (isDetailedMemoryTrackingEnabled) {
                long loadTime = System.currentTimeMillis() - startTime;
                SmartDashboard.putNumber(PATH_LOAD_TIME_KEY, loadTime);
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
                    
                    if (isDetailedMemoryTrackingEnabled) {
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
        
        if (isDetailedMemoryTrackingEnabled) {
            updateMemoryStatus();
        }
    }
    
    /**
     * Update memory status on dashboard
     */
    private void updateMemoryStatus() {
        // Skip if not in debug mode
        if (!isDetailedMemoryTrackingEnabled) {
            return;
        }
        
        // Throttle updates to once every 500ms
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastMemoryUpdateTime < MEMORY_UPDATE_INTERVAL_MS) {
            return;
        }
        lastMemoryUpdateTime = currentTime;
        
        double freeMB = Runtime.getRuntime().freeMemory() / (1024.0 * 1024.0);
        double totalMB = Runtime.getRuntime().totalMemory() / (1024.0 * 1024.0);
        double usedMB = totalMB - freeMB;
        
        SmartDashboard.putNumber(MEMORY_KEY, freeMB);
        SmartDashboard.putNumber(USED_MEMORY_KEY, usedMB);
        SmartDashboard.putNumber(TOTAL_MEMORY_KEY, totalMB);
        SmartDashboard.putNumber(MEMORY_USAGE_PERCENT_KEY, (usedMB / totalMB) * 100.0);
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
            SmartDashboard.putString(SELECTED_AUTO_KEY, selectedAutoPath);
        }
        
        // Only check path buttons if no path is currently running
        if (!isPathRunning) {
            checkPathButtons();
        }
        
        // Update memory status (throttled internally to reduce updates)
        if (isDetailedMemoryTrackingEnabled) {
            updateMemoryStatus();
        }
    }
    
    /**
     * Check all path buttons on the dashboard and run the corresponding path if pressed
     */
    private void checkPathButtons() {
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
    
    /**
     * Create a command sequence that combines path following with elevator and mailbox actions
     * @param pathName Name of the path to follow
     * @return Command sequence that follows path, raises elevator, and shoots
     */
    private Command createShootingPathCommand(String pathName) {
        if (pathName == null || pathName.isEmpty() || pathName.equals("none")) {
            return Commands.none();
        }

        // // Get the actual path name based on the auto name
        // String actualPathName = switch(pathName) {
        //     case "TopAuto" -> "AutoDrive20";
        //     case "MidAuto" -> "AutoDrive22";
        //     case "BottomAuto" -> "Audodrive22";
        //     default -> pathName;
        // };

        // Load the path command
        Command pathCommand = loadPathCommand(pathName);
        if (pathCommand == null) {
            return Commands.none();
        }

        // Create the full sequence
        return Commands.sequence(
            // First follow the path to get into position
            pathCommand//simplifying to just path following for now
            
            // // Then raise elevator to L1
            // Commands.runOnce(() -> {
            //     RobotContainer.elevatorSubsystem.runMotor(-ElevatorConstants.ELEVATOR_UP_SPEED);
            // }),
            // // Wait until elevator reaches L1 height
            // Commands.waitSeconds(1.0), // Adjust time based on testing
            // // Stop elevator
            // Commands.runOnce(() -> {
            //     RobotContainer.elevatorSubsystem.runMotor(0);
            // }),
            
            // // Finally shoot at L1 speed
            // Commands.runOnce(() -> {
            //     RobotContainer.mailboxSubsystem.output(false); // false = L1 speed
            // }),
            // // Wait for shot to complete
            // Commands.waitSeconds(1.0), // Adjust time based on testing
            // // Stop mailbox
            // Commands.runOnce(() -> {
            //     RobotContainer.mailboxSubsystem.stop();
            // })
        );
    }
    
    /**
     * Get the autonomous command based on dashboard selection
     * Only loads the path when actually needed for autonomous
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        // Otherwise use dashboard selection
        String selectedPath = pathChooser.getSelected();
        
        // If "none" is selected, return an empty command
        if (selectedPath == null || selectedPath.equals("none")) {
            return Commands.none();
        }
        
        // Create the full autonomous sequence with elevator and shooting
        return createShootingPathCommand(selectedPath);
    }
    
    /**
     * Check if a path is currently running
     * @return True if a path is running, false otherwise
     */
    public boolean isPathRunning() {
        return isPathRunning;
    }
} 