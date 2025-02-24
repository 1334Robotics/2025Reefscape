package frc.robot.commands.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class RaiseDatElevator extends Command {
    private final ElevatorSubsystem elevator;
    private int coraltype;
    private double targetPrimaryHeight;
    private double targetSecondaryHeight;

    // Coraltype is the level of the coral that the elevator is being raised to
    // 1 is L1, etc.
    public RaiseDatElevator(ElevatorSubsystem elevator, int type) {
        this.elevator = elevator;
        this.coraltype = type;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        SetHeight(coraltype);
        elevator.setTargetPosition(targetPrimaryHeight, targetSecondaryHeight);
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtTarget();
    }

    public void SetHeight(int coraltype) {
        switch (coraltype) {
            case 1:
                targetPrimaryHeight = 2 * (ElevatorConstants.L1Height_INCHES) / 3;
                targetSecondaryHeight = ElevatorConstants.L1Height_INCHES / 3;
                break;
            case 2:
                targetPrimaryHeight = 2 * (ElevatorConstants.L2Height_INCHES) / 3;
                targetSecondaryHeight = ElevatorConstants.L2Height_INCHES / 3;
                break;
            case 3:
                targetPrimaryHeight = 2 * (ElevatorConstants.L3Height_INCHES) / 3;
                targetSecondaryHeight = ElevatorConstants.L3Height_INCHES / 3;
                break;
            case 4:
                targetPrimaryHeight = 2 * (ElevatorConstants.L4Height_INCHES) / 3;
                targetSecondaryHeight = ElevatorConstants.L4Height_INCHES / 3;
                break;
            default:
                throw new IllegalArgumentException("Invalid Coral Level: " + coraltype);
        }
    }
}
