package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootCommand extends Command {
    public ShootCommand() {
        addRequirements(RobotContainer.mailboxHandler);
        addRequirements(RobotContainer.mailboxSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.mailboxHandler.allowShoot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
