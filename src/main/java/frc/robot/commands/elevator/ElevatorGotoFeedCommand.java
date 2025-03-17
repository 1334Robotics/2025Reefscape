package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorLevel;

public class ElevatorGotoFeedCommand extends Command {
    public ElevatorGotoFeedCommand() {
        addRequirements(RobotContainer.elevatorSubsystem);
        addRequirements(RobotContainer.elevatorHandler);
    }

    @Override
    public void initialize() {
        RobotContainer.elevatorHandler.setLevel(ElevatorLevel.FEED);
    }

    public boolean isFinished() {
        return true;
    }
}
