package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.constants.AutoConstants;

public class AutoConfigurer {
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
                              RobotContainer.swerveSubsystem::resetPose,
                              RobotContainer.swerveSubsystem::getChassisSpeeds,
                              (speeds, feedforwards) -> RobotContainer.swerveSubsystem.autoDrive(speeds),
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
    }
}