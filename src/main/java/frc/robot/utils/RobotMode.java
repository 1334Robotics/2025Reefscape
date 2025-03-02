package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.Map;

/**
 * Utility class for determining robot mode and controlling debug features
 */
public class RobotMode {
    // Static configuration that can be set in robot initialization
    private static boolean sForceDisableDebug = false;
    private static boolean sVerboseLogging = false;
    private static boolean sCompetitionMode = false;
    
    // Competition-specific settings
    private static DriverStation.Alliance sOverrideAlliance = null;
    private static int sStartingPosition = 1; // Default to position 1
    private static final Map<String, String> sCompetitionAutos = new HashMap<>(3);
    
    /**
     * Determine if the code is running in simulation
     * @return true if running in simulation, false if on real robot
     */
    public static boolean isSimulation() {
        return RobotBase.isSimulation();
    }
    
    /**
     * Determine if the code is running on the real robot
     * @return true if running on real robot, false if in simulation
     */
    public static boolean isRealRobot() {
        return RobotBase.isReal();
    }
    
    /**
     * Check if debug features should be enabled
     * @return true if debug should be enabled, false otherwise
     */
    public static boolean isDebugEnabled() {
        // Competition mode always disables debug features
        if (sCompetitionMode) {
            return false;
        }
        
        // Debug is enabled in simulation unless force disabled,
        // and disabled on real robot unless verbose logging is enabled
        if (isSimulation()) {
            return !sForceDisableDebug;
        } else {
            return sVerboseLogging;
        }
    }
    
    /**
     * Force disable all debug features (even in simulation)
     * Call this method in robotInit if you want to disable debug in all modes
     */
    public static void forceDisableDebug() {
        sForceDisableDebug = true;
    }
    
    /**
     * Check if debug features are force disabled
     * @return true if debug is force disabled
     */
    public static boolean isDebugForceDisabled() {
        return sForceDisableDebug;
    }
    
    /**
     * Enable verbose logging on the real robot
     * Only use this temporarily for troubleshooting
     */
    public static void enableVerboseLogging() {
        sVerboseLogging = true;
    }
    
    /**
     * Disable verbose logging on the real robot (default state)
     */
    public static void disableVerboseLogging() {
        sVerboseLogging = false;
    }
    
    /**
     * Check if verbose logging is enabled
     * @return true if verbose logging is enabled
     */
    public static boolean isVerboseLoggingEnabled() {
        return sVerboseLogging;
    }
    
    /**
     * Enable competition mode - disables ALL non-essential features
     * Call this method before a competition match to maximize performance
     */
    public static void enableCompetitionMode() {
        sCompetitionMode = true;
        sVerboseLogging = false;
        sForceDisableDebug = true;
    }
    
    /**
     * Disable competition mode - allows debug features based on other settings
     */
    public static void disableCompetitionMode() {
        sCompetitionMode = false;
        sForceDisableDebug = false;
    }
    
    /**
     * Check if competition mode is enabled
     * @return true if in competition mode
     */
    public static boolean isCompetitionMode() {
        return sCompetitionMode;
    }
    
    /**
     * Set the alliance color for competition
     * This overrides the driver station alliance for path loading
     * 
     * @param alliance the alliance color (RED or BLUE)
     */
    public static void setAlliance(DriverStation.Alliance alliance) {
        sOverrideAlliance = alliance;
    }
    
    /**
     * Get the alliance color to use
     * @return the alliance color (from override or driver station)
     */
    public static DriverStation.Alliance getAlliance() {
        if (sOverrideAlliance != null) {
            return sOverrideAlliance;
        }
        
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() : DriverStation.Alliance.Blue;
    }
    
    /**
     * Check if on red alliance (either from override or driver station)
     * @return true if on red alliance
     */
    public static boolean isRedAlliance() {
        return getAlliance() == DriverStation.Alliance.Red;
    }
    
    /**
     * Check if on blue alliance (either from override or driver station)
     * @return true if on blue alliance
     */
    public static boolean isBlueAlliance() {
        return getAlliance() == DriverStation.Alliance.Blue;
    }
    
    /**
     * Set the starting position (1, 2, or 3)
     * @param position the starting position (1-3)
     */
    public static void setStartingPosition(int position) {
        if (position < 1 || position > 3) {
            throw new IllegalArgumentException("Starting position must be 1, 2, or 3");
        }
        sStartingPosition = position;
    }
    
    /**
     * Get the current starting position
     * @return starting position (1, 2, or 3)
     */
    public static int getStartingPosition() {
        return sStartingPosition;
    }
    
    /**
     * Set a competition auto path for a specific position
     * 
     * @param position the starting position (1, 2, or 3)
     * @param autoPathName the name of the autonomous path to use
     */
    public static void setCompetitionAuto(int position, String autoPathName) {
        if (position < 1 || position > 3) {
            throw new IllegalArgumentException("Starting position must be 1, 2, or 3");
        }
        sCompetitionAutos.put(String.valueOf(position), autoPathName);
    }
    
    /**
     * Get the competition auto for the current starting position
     * @return the auto path name for the current position, or null if not set
     */
    public static String getCurrentAuto() {
        return sCompetitionAutos.get(String.valueOf(sStartingPosition));
    }
    
    /**
     * Reset all settings to default values
     * Default: debug enabled in simulation, disabled on real robot,
     * competition mode off, alliance from driver station
     */
    public static void resetToDefaults() {
        sForceDisableDebug = false;
        sVerboseLogging = false;
        sCompetitionMode = false;
        sOverrideAlliance = null;
        sStartingPosition = 1;
        sCompetitionAutos.clear();
    }
} 