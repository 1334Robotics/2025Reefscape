package frc.robot.commands.gyro;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class GyroZeroCommand extends Command {
    public GyroZeroCommand() {
        addRequirements(RobotContainer.gyroSubsystem, RobotContainer.swerveSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.gyroSubsystem.zero();
        RobotContainer.swerveSubsystem.zeroGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
