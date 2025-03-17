package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class MailboxRewindCommand extends Command {
    public MailboxRewindCommand() {
        addRequirements(RobotContainer.mailboxHandler);
        addRequirements(RobotContainer.mailboxSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.mailboxHandler.startRewinding();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.mailboxHandler.stopRewinding();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
