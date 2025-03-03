package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ForceFeedCommand extends Command {
    public ForceFeedCommand() {
        addRequirements(RobotContainer.mailboxHandler);
        addRequirements(RobotContainer.mailboxSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.mailboxHandler.forceFeeding = true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
