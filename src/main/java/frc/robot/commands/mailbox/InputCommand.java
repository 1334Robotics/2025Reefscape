package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class InputCommand extends Command {
    public InputCommand() {
        addRequirements(RobotContainer.mailboxSubsystem);
    }
    
    @Override
    public void initialize() {
        RobotContainer.mailboxSubsystem.input();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
