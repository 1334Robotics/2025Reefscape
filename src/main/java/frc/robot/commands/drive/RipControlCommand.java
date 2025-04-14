package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RipControlCommand extends Command {
    public RipControlCommand() {
        addRequirements(RobotContainer.driveController);
    }

    @Override
    public void initialize() {
        RobotContainer.driveController.takeAllControlAway();
        RobotContainer.directionSnapSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
