package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.PathPlannerConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.function.BooleanSupplier;

/**
 * Helper class for autonomous path following using PathPlanner
 */
public class AutoHelper {
    private static boolean isConfigured = false;
    
    // Dashboard message constants
    private static final String CONFIG_SUCCESS_MSG = "AutoBuilder configured successfully";
    private static final String REAL_ROBOT_CONFIG_MSG = "Using PathPlanner GUI settings on robot";
    private static final String SIMULATION_CONFIG_MSG = "Using GUI settings in simulation";
    
    /**
     * Configure the AutoBuilder for path following
     * 
     * @param swerveSubsystem The swerve drive subsystem
     * @return True if configuration was successful, false otherwise
     */
    public static boolean configureAutoBuilder(SwerveSubsystem swerveSubsystem) {
        if (isConfigured) {
            return true;
        }
        
        try {
            // Load robot config from GUI settings
            // Note: We use the GUI settings for both simulation and real robot
            // to ensure compatibility with the PathPlanner library version
            RobotConfig config = RobotConfig.fromGUISettings();
            
            // Log different messages based on robot mode
            if (RobotMode.isRealRobot()) {
                if (RobotMode.isDebugEnabled()) {
                    DriverStation.reportWarning(REAL_ROBOT_CONFIG_MSG, false);
                }
            } else if (RobotMode.isDebugEnabled()) {
                DriverStation.reportWarning(SIMULATION_CONFIG_MSG, false);
            }
            
            // Configure AutoBuilder with original method call style
            AutoBuilder.configure(
                swerveSubsystem::getPose,                // Robot pose supplier
                swerveSubsystem::resetOdometry,          // Method to reset odometry
                swerveSubsystem::getChassisSpeeds,       // Robot relative chassis speeds supplier
                // Method to drive the robot using chassis speeds and feedforwards
                (chassisSpeeds, feedforwards) -> {
                    double xVelocity = chassisSpeeds.vxMetersPerSecond;
                    double yVelocity = chassisSpeeds.vyMetersPerSecond;
                    double rotation = chassisSpeeds.omegaRadiansPerSecond;
                    swerveSubsystem.drive(new Translation2d(xVelocity, yVelocity), rotation);
                    // Feedforwards are ignored as the swerve drive handles that internally
                },
                // Create the controller for path following
                createDriveController(),
                config,                                  // Robot config (from GUI settings)
                // Should path flip based on alliance color - use RobotMode for consistency
                RobotMode::isRedAlliance,
                swerveSubsystem                          // Subsystem requirement
            );
            
            isConfigured = true;
            
            // Only log in debug mode
            if (RobotMode.isDebugEnabled()) {
                DriverStation.reportWarning(CONFIG_SUCCESS_MSG, false);
            }
            
            return true;
        } catch (Exception e) {
            // Always report configuration errors
            DriverStation.reportError("Failed to configure AutoBuilder: " + e.getMessage(), e.getStackTrace());
            return false;
        }
    }
    
    /**
     * Create the holonomic drive controller with PID constants from PathPlannerConstants
     * 
     * @return The configured drive controller
     */
    private static PPHolonomicDriveController createDriveController() {
        return new PPHolonomicDriveController(
            new PIDConstants(
                PathPlannerConstants.TRANSLATION_KP, 
                PathPlannerConstants.TRANSLATION_KI, 
                PathPlannerConstants.TRANSLATION_KD
            ),
            new PIDConstants(
                PathPlannerConstants.ROTATION_KP,
                PathPlannerConstants.ROTATION_KI,
                PathPlannerConstants.ROTATION_KD
            )
        );
    }
    
    /**
     * Create a command to follow a path
     * 
     * @param swerveSubsystem The swerve drive subsystem
     * @param path The path to follow
     * @return Command to follow the path
     */
    public static Command followPath(SwerveSubsystem swerveSubsystem, PathPlannerPath path) {
        // Ensure AutoBuilder is configured
        if (!isConfigured) {
            if (!configureAutoBuilder(swerveSubsystem)) {
                DriverStation.reportError("Cannot follow path - AutoBuilder configuration failed", false);
                return Commands.none();
            }
        }
        
        if (path == null) {
            DriverStation.reportError("Cannot follow null path", false);
            return Commands.none();
        }
        
        try {
            // Create a command to follow the path
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Error creating path following command: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
} 