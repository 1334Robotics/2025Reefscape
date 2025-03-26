package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.constants.AutoConstants;

public class AutoConfigurer {
    private static AutoStartHandler autoStartHandler = new AutoStartHandler();
    public static boolean allowDrive = false;
    
    private static void autoDrive(ChassisSpeeds speeds) {
        if(allowDrive) RobotContainer.swerveSubsystem.autoDrive(speeds);
    }

    public static void configure() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch(Exception e) {
            System.err.println("Exception when trying to get config from GUI settings: " + e);
            return;
        }

        AutoBuilder.configure(
                              RobotContainer.swerveSubsystem::getPose,
                              RobotContainer.swerveSubsystem::resetOdometry,
                              RobotContainer.swerveSubsystem::getChassisSpeeds,
                              (speeds, feedforwards) -> AutoConfigurer.autoDrive(speeds),
                              new PPHolonomicDriveController(
                                                             new PIDConstants(AutoConstants.TRANSLATION_KP,
                                                                              AutoConstants.TRANSLATION_KI,
                                                                              AutoConstants.TRANSLATION_KD),
                                                             new PIDConstants(AutoConstants.ROTATION_KP,
                                                                              AutoConstants.ROTATION_KI,
                                                                              AutoConstants.ROTATION_KD)),
                              config,
                              () -> {
                                Optional<Alliance> alliance = DriverStation.getAlliance();
                                if(alliance.isPresent()) {
                                    return alliance.get() == DriverStation.Alliance.Red;
                                }
                                return false;
                              },
                              RobotContainer.swerveSubsystem);
        System.out.println("[AutoConfigurer] AutoBuilder configured");
    }

    public static void setInitialPose() {
        Pose2d initialPose = AutoConfigurer.autoStartHandler.getStartPose();
        if(initialPose == null) return;

        // Print the initial pose for debug purposes
        System.out.println("Setting initial pose to: " + initialPose);

        RobotContainer.swerveSubsystem.resetOdometry(initialPose);
    }
}