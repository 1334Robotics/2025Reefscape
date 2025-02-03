package frc.robot.commands.directionSnaps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Direction;

public class DirectionSnapLeft extends Command {
    public DirectionSnapLeft() {
        addRequirements(RobotContainer.directionSnapSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.directionSnapSubsystem.snap(Direction.LEFT);
    }

    @Override
    public boolean isFinished() {
        return !RobotContainer.directionSnapSubsystem.isAtTarget();
    }
}