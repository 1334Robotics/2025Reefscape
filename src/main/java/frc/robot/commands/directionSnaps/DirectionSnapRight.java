package frc.robot.commands.directionSnaps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Direction;

public class DirectionSnapRight extends Command {
    public DirectionSnapRight() {
        addRequirements(RobotContainer.directionSnapSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.directionSnapSubsystem.snap(Direction.RIGHT);
    }

    @Override
    public boolean isFinished() {
        return !RobotContainer.directionSnapSubsystem.isAtTarget();
    }
}