package frc.robot.commands.flopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class FlopperZeroCommand extends Command {
    public FlopperZeroCommand() {
        addRequirements(RobotContainer.flopperSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.flopperSubsystem.zero();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
