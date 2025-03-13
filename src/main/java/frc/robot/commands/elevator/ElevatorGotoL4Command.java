package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorLevel;

public class ElevatorGotoL4Command extends Command {
    public ElevatorGotoL4Command() {
        addRequirements(RobotContainer.elevatorSubsystem);
        addRequirements(RobotContainer.elevatorHandler);
    }

    @Override
    public void initialize() {
        RobotContainer.elevatorHandler.setLevel(ElevatorLevel.L4);
    }

    public boolean isFinished() {
        return true;
    }
}
