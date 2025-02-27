package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class OutputHighCommand extends Command {
    public OutputHighCommand() {
        addRequirements(RobotContainer.mailboxSubsystem);
    }
    
    @Override
    public void initialize() {
        RobotContainer.mailboxSubsystem.output(true);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
