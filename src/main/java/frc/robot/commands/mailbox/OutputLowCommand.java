package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class OutputLowCommand extends Command {
    public OutputLowCommand() {
        addRequirements(RobotContainer.mailboxSubsystem);
    }
    
    @Override
    public void initialize() {
        RobotContainer.mailboxSubsystem.output(false);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
