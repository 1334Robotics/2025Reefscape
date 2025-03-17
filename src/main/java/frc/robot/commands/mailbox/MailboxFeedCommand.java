package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class MailboxFeedCommand extends Command {
    public MailboxFeedCommand() {
        addRequirements(RobotContainer.mailboxHandler);
        addRequirements(RobotContainer.mailboxSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.mailboxHandler.allowFeeding = true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
