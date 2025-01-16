package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class StopCommand extends Command {
    public StopCommand() {
        addRequirements(RobotContainer.mailboxSubsystem);
    }
    
    @Override
    public void initialize() {
        RobotContainer.mailboxSubsystem.stop();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
