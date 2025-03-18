package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class FieldRelativeCommand extends Command {
    public FieldRelativeCommand() {
        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.swerveSubsystem.setFieldRelative(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
