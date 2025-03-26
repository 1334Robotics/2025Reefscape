package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class AutoStartHandler {
    private final SendableChooser<AutoStartPosition> startPositionChooser;

    public AutoStartHandler() {
        // Get the initial start position from the DriverStation
        AutoStartPosition startPosition = AutoStartPosition.getPositionFromDriverStation();

        // Create the startPositionChooser and set the default value
        this.startPositionChooser = new SendableChooser<>();
        if(startPosition != null)
            this.startPositionChooser.setDefaultOption(startPosition.toString(), startPosition);
        else
            this.startPositionChooser.setDefaultOption("No position", null);
        
        // Populate the startPositionChooser with values
        for(AutoStartPosition position : AutoStartPosition.values()) {
            if(position == startPosition) continue;
            this.startPositionChooser.addOption(position.toString(), position);
        }

        // Add the startPositionChooser to SmartDashboard
        SmartDashboard.putData("[AUTO] Start Position Chooser", startPositionChooser);
    }

    public Pose2d getStartPose() {
        AutoStartPosition startPosition = this.startPositionChooser.getSelected();
        if(startPosition == null) return null;
        return startPosition.getStartPose();
    }

    public void periodic() {
        if(getStartPose() == null) return;

        RobotContainer.swerveSubsystem.resetOdometry(getStartPose());
    }
}
