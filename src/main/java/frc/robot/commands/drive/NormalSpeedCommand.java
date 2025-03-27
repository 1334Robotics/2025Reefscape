package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class NormalSpeedCommand extends Command {
    public NormalSpeedCommand() {
        addRequirements(RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        DriveCommand.driveSpeed = DriveSpeed.NORMAL;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
