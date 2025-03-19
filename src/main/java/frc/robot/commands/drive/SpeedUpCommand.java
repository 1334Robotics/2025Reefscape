package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SpeedUpCommand extends Command {
    public SpeedUpCommand() {
        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        DriveCommand.goFast = true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
