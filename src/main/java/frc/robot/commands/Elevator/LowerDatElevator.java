package frc.robot.commands.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class LowerDatElevator extends Command {
    private final ElevatorSubsystem elevator;

    // Coraltype is the level of the coral that the elevator is being raised to
    // 1 is L1, etc.
    public LowerDatElevator(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetPosition(0, 0);
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtTarget();
    }
}
