package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class OutputCommand extends Command {
    public OutputCommand() {
        addRequirements(RobotContainer.mailboxSubsystem);
    }
    
    @Override
    public void initialize() {
        RobotContainer.mailboxSubsystem.output();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
