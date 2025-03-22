package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.constants.AutoConstants;

public class AutoConfigurer {
    /**
     * Configure PathPlanner and auto builder
     */
    public static void configure() {
        try {
            // Load the RobotConfig from the GUI settings
            RobotConfig config = RobotConfig.fromGUISettings();
            
            // Configure AutoBuilder with the latest PathPlanner API
            AutoBuilder.configure(
                RobotContainer.swerveSubsystem::getPose,         // Robot pose supplier
                RobotContainer.swerveSubsystem::resetOdometry,   // Method to reset odometry
                RobotContainer.swerveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier
                RobotContainer.swerveSubsystem::autoDrive,       // Method to drive robot
                new PPHolonomicDriveController(
                    new PIDConstants(AutoConstants.PATH_PLANNER_KP, 
                                   AutoConstants.PATH_PLANNER_KI, 
                                   AutoConstants.PATH_PLANNER_KD),  // Translation PID constants
                    new PIDConstants(AutoConstants.PATH_PLANNER_KP, 
                                   AutoConstants.PATH_PLANNER_KI, 
                                   AutoConstants.PATH_PLANNER_KD)   // Rotation PID constants
                ),
                config,                       // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
                    SmartDashboard.putBoolean("PathPlanner/IsRedAlliance", isRed);
                    SmartDashboard.putBoolean("PathPlanner/MirroringPaths", isRed);
                    return isRed;
                },
                RobotContainer.swerveSubsystem  // Reference to swerve subsystem for requirements
            );
            
            System.out.println("PathPlanner configured successfully");
            
        } catch (Exception e) {
            System.err.println("Error configuring PathPlanner: " + e.getMessage());
            e.printStackTrace();
        }
    }
}