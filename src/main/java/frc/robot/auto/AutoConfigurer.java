package frc.robot.auto;

import java.util.Optional;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Robot;
import frc.robot.constants.AutoConstants;

public class AutoConfigurer {
    private static boolean isConfigured = false;

    /**
     * Configure PathPlanner and auto builder
     */
    public static void configure() {
        if (isConfigured) {
            System.out.println("[AutoConfig] Already configured - skipping");
            return;
        }

        try {
            System.out.println("[AutoConfig] Starting configuration...");
            
            // Load the RobotConfig from the GUI settings
            RobotConfig config = RobotConfig.fromGUISettings();
            System.out.println("[AutoConfig] Loaded robot config from GUI settings");

            // Verify deployment directory structure
            verifyDeploymentStructure();
            
            // Configure AutoBuilder with detailed logging
            System.out.println("[AutoConfig] Configuring AutoBuilder...");
            configureAutoBuilder(config);
            
            // Mark as configured
            isConfigured = true;
            System.out.println("[AutoConfig] Configuration complete");
            
        } catch (Exception e) {
            System.err.println("[AutoConfig] ERROR during configuration: " + e.getMessage());
            e.printStackTrace();
            SmartDashboard.putString("[AUTO] Error", "Config failed: " + e.getMessage());
        }
    }

    /**
     * Verify the deployment directory structure exists
     */
    private static void verifyDeploymentStructure() {
        File deployDir = Filesystem.getDeployDirectory();
        File pathsDir = new File(deployDir, "pathplanner/paths");
        File autosDir = new File(deployDir, "pathplanner/autos");
        
        System.out.println("[AutoConfig] Verifying deployment structure...");
        System.out.println("- Deploy dir: " + deployDir.getAbsolutePath());
        System.out.println("- Paths dir: " + pathsDir.getAbsolutePath());
        System.out.println("- Autos dir: " + autosDir.getAbsolutePath());
        
        if (!pathsDir.exists() || !autosDir.exists()) {
            System.err.println("[AutoConfig] WARNING: Missing required directories!");
            System.err.println("- Paths dir exists: " + pathsDir.exists());
            System.err.println("- Autos dir exists: " + autosDir.exists());
        }
        
        // List available auto files
        if (autosDir.exists()) {
            File[] autoFiles = autosDir.listFiles((dir, name) -> name.endsWith(".auto"));
            if (autoFiles != null) {
                System.out.println("[AutoConfig] Found " + autoFiles.length + " auto files:");
                for (File file : autoFiles) {
                    System.out.println("- " + file.getName());
                }
            }
        }
    }

    /**
     * Configure the AutoBuilder with proper logging
     */
    private static void configureAutoBuilder(RobotConfig config) {
        // Get alliance for logging
        var alliance = DriverStation.getAlliance();
        boolean isBlue = !alliance.isPresent() || alliance.get() == Alliance.Blue;
        System.out.println("[AutoConfig] Alliance: " + (isBlue ? "BLUE" : "RED"));
        
        AutoBuilder.configure(
            // Pose supplier with logging
            () -> {
                var pose = RobotContainer.swerveSubsystem.getPose();
                SmartDashboard.putString("[AUTO] Current Pose", 
                    String.format("X: %.2f, Y: %.2f, Rot: %.2f°", 
                        pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
                return pose;
            },
            // Reset odometry with logging
            (pose) -> {
                if (pose != null) {
                    System.out.println("[AutoConfig] Resetting odometry to: " + pose);
                    RobotContainer.swerveSubsystem.resetOdometry(pose);
                    SmartDashboard.putString("[AUTO] Reset Pose", pose.toString());
                } else {
                    System.err.println("[AutoConfig] WARNING: Attempted to reset odometry with null pose!");
                }
            },
            // Chassis speeds supplier with logging
            () -> {
                var speeds = RobotContainer.swerveSubsystem.getChassisSpeeds();
                SmartDashboard.putNumber("[AUTO] VX", speeds.vxMetersPerSecond);
                SmartDashboard.putNumber("[AUTO] VY", speeds.vyMetersPerSecond);
                SmartDashboard.putNumber("[AUTO] Omega", speeds.omegaRadiansPerSecond);
                return speeds;
            },
            // Drive method with logging
            (speeds) -> {
                SmartDashboard.putNumber("[AUTO] Target VX", speeds.vxMetersPerSecond);
                SmartDashboard.putNumber("[AUTO] Target VY", speeds.vyMetersPerSecond);
                SmartDashboard.putNumber("[AUTO] Target Omega", speeds.omegaRadiansPerSecond);
                RobotContainer.swerveSubsystem.autoDrive(speeds);
            },
            // Configure the controllers
            new PPHolonomicDriveController(
                new PIDConstants(AutoConstants.PATH_PLANNER_KP, 
                               AutoConstants.PATH_PLANNER_KI, 
                               AutoConstants.PATH_PLANNER_KD),
                new PIDConstants(AutoConstants.PATH_PLANNER_KP, 
                               AutoConstants.PATH_PLANNER_KI, 
                               AutoConstants.PATH_PLANNER_KD)
            ),
            config,
            // Alliance color handler with logging
            () -> {
                var currentAlliance = DriverStation.getAlliance();
                boolean isRed = currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red;
                SmartDashboard.putBoolean("[AUTO] Is Red Alliance", isRed);
                SmartDashboard.putBoolean("[AUTO] Mirroring Paths", isRed);
                System.out.println("[AutoConfig] Path mirroring: " + (isRed ? "ENABLED" : "DISABLED"));
                return isRed;
            },
            RobotContainer.swerveSubsystem
        );

        // Log configuration details
        SmartDashboard.putNumber("[AUTO] Translation kP", AutoConstants.PATH_PLANNER_KP);
        SmartDashboard.putNumber("[AUTO] Translation kI", AutoConstants.PATH_PLANNER_KI);
        SmartDashboard.putNumber("[AUTO] Translation kD", AutoConstants.PATH_PLANNER_KD);
    }
}