package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorLevel;
import frc.robot.constants.ElevatorConstants;

public class ElevatorGotoL1Command extends Command {
    private boolean isFinishedMoving = false;

    public ElevatorGotoL1Command() {
        addRequirements(RobotContainer.elevatorSubsystem);
        addRequirements(RobotContainer.elevatorHandler);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorGotoL1Command: Starting movement to L1");
        isFinishedMoving = false;
        RobotContainer.elevatorHandler.setLevel(ElevatorLevel.L1);
        SmartDashboard.putString("Elevator Command Status", "Moving to L1");
    }

    @Override
    public void execute() {
        // Check if we've reached the target position
        double currentPosition = RobotContainer.elevatorSubsystem.getPosition();
        double targetPosition = ElevatorLevel.L1.position;
        double error = Math.abs(currentPosition - targetPosition);
        
        isFinishedMoving = error < ElevatorConstants.MAX_ACCEPTABLE_ERROR;
        
        // Log status
        SmartDashboard.putNumber("[ELEVATOR] Elevator Error", error);
        SmartDashboard.putBoolean("[ELEVATOR] Elevator At Target", isFinishedMoving);
    }

    @Override
    public boolean isFinished() {
        return isFinishedMoving;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("ElevatorGotoL1Command: Movement interrupted!");
            SmartDashboard.putString("Elevator Command Status", "Interrupted");
        } else {
            System.out.println("ElevatorGotoL1Command: Successfully reached L1");
            SmartDashboard.putString("Elevator Command Status", "Completed");
        }
    }
}
