package frc.robot.commands.flopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class FlopperDownCommand extends Command {
    public FlopperDownCommand() {
        addRequirements(RobotContainer.flopperSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.flopperSubsystem.goDown();
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
