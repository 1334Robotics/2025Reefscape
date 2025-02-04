package frc.robot.commands.directionSnaps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class StopSnap extends Command {
    public StopSnap() {
        addRequirements(RobotContainer.directionSnapSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.directionSnapSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
