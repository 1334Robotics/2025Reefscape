package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class BotRelativeCommand extends Command {
    public BotRelativeCommand() {
        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.swerveSubsystem.setFieldRelative(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}