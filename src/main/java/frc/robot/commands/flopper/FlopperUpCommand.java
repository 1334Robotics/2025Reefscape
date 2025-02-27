package frc.robot.commands.flopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class FlopperUpCommand extends Command {
    public FlopperUpCommand() {
        addRequirements(RobotContainer.flopperSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.flopperSubsystem.goUp();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.flopperSubsystem.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
