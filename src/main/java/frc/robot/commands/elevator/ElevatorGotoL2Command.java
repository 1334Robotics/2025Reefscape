package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorLevel;

public class ElevatorGotoL2Command extends Command {
    public ElevatorGotoL2Command() {
        addRequirements(RobotContainer.elevatorSubsystem);
        addRequirements(RobotContainer.elevatorHandler);
    }

    @Override
    public void initialize() {
        RobotContainer.elevatorHandler.setLevel(ElevatorLevel.L2);
    }

    public boolean isFinished() {
        return true;
    }
}
