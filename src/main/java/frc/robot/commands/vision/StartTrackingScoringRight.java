package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.TagTrackingTarget;

public class StartTrackingScoringRight extends Command {
    public StartTrackingScoringRight() {
        addRequirements(RobotContainer.tagTrackingHandler);
    }

    @Override
    public void initialize() {
        RobotContainer.tagTrackingHandler.setTarget(TagTrackingTarget.SCORING_RIGHT);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
