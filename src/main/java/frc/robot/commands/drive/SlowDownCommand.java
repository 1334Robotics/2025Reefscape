package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SlowDownCommand extends Command {
    public SlowDownCommand() {
        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        DriveCommand.goFast = false;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
