package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Utility class for determining robot mode and controlling debug features
 */
public class RobotMode {
    // Static configuration that can be set in robot initialization
    private static boolean forceDisableDebug = false;
    private static boolean verboseLogging = false;
    
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
        // Debug is enabled in simulation unless force disabled,
        // and disabled on real robot unless verbose logging is enabled
        if (isSimulation()) {
            return !forceDisableDebug;
        } else {
            return verboseLogging;
        }
    }
    
    /**
     * Force disable all debug features (even in simulation)
     * Call this method in robotInit if you want to disable debug in all modes
     */
    public static void forceDisableDebug() {
        forceDisableDebug = true;
    }
    
    /**
     * Enable verbose logging on the real robot
     * Only use this temporarily for troubleshooting
     */
    public static void enableVerboseLogging() {
        verboseLogging = true;
    }
    
    /**
     * Disable verbose logging on the real robot (default state)
     */
    public static void disableVerboseLogging() {
        verboseLogging = false;
    }
} 