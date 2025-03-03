package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.RobotContainer;

public class RaiseElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final ElevatorHeightCalculation targetLevel;

    public RaiseElevatorCommand(ElevatorHeightCalculation level) {
        this.elevator = RobotContainer.elevatorSubsystem;
        this.targetLevel = level;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetPosition(targetLevel.getTargetPrimaryHeight(), targetLevel.getTargetSecondaryHeight());
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtTarget();
    }
}
