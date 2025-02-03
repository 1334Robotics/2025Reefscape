package frc.robot.commands.directionSnaps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Direction;

public class DirectionSnapBackwards extends Command {
    public DirectionSnapBackwards() {
        addRequirements(RobotContainer.directionSnapSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.directionSnapSubsystem.snap(Direction.BACKWARDS);
    }

    @Override
    public boolean isFinished() {
        return !RobotContainer.directionSnapSubsystem.isAtTarget();
    }
}