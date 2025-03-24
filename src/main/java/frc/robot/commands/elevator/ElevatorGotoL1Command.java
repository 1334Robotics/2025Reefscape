package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorLevel;
import frc.robot.constants.ElevatorConstants;

public class ElevatorGotoL1Command extends Command {
    private boolean isFinishedMoving = false;
    private int logCounter = 0;

    public ElevatorGotoL1Command() {
        addRequirements(RobotContainer.elevatorSubsystem);
        addRequirements(RobotContainer.elevatorHandler);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorGotoL1Command: Starting movement to L1");
        isFinishedMoving = false;
        logCounter = 0;
        
        // Log initial state
        double currentPosition = RobotContainer.elevatorSubsystem.getPosition();
        double targetPosition = ElevatorLevel.L1.position;
        System.out.println(String.format("[ELEVATOR] Initial State - Current: %.4f, Target: %.4f, Error: %.4f", 
            currentPosition, targetPosition, Math.abs(currentPosition - targetPosition)));
        
        RobotContainer.elevatorHandler.setLevel(ElevatorLevel.L1);
        SmartDashboard.putString("[ELEVATOR] Command Status", "Moving to L1");
    }

    @Override
    public void execute() {
        // Check if we've reached the target position
        double currentPosition = RobotContainer.elevatorSubsystem.getPosition();
        double targetPosition = ElevatorLevel.L1.position;
        double error = Math.abs(currentPosition - targetPosition);
        
        isFinishedMoving = error < ElevatorConstants.MAX_ACCEPTABLE_ERROR;
        
        // Log status every 50 iterations (about once per second)
        if (logCounter++ % 50 == 0) {
            System.out.println(String.format("[ELEVATOR] Status - Current: %.4f, Target: %.4f, Error: %.4f, At Target: %b", 
                currentPosition, targetPosition, error, isFinishedMoving));
        }
        
        // Update dashboard
        SmartDashboard.putNumber("[ELEVATOR] Current Position", currentPosition);
        SmartDashboard.putNumber("[ELEVATOR] Target Position", targetPosition);
        SmartDashboard.putNumber("[ELEVATOR] Error", error);
        SmartDashboard.putBoolean("[ELEVATOR] At Target", isFinishedMoving);
    }

    @Override
    public boolean isFinished() {
        return isFinishedMoving;
    }

    @Override
    public void end(boolean interrupted) {
        // Log final state
        double currentPosition = RobotContainer.elevatorSubsystem.getPosition();
        double targetPosition = ElevatorLevel.L1.position;
        double error = Math.abs(currentPosition - targetPosition);
        
        if (interrupted) {
            System.out.println(String.format("[ELEVATOR] Movement interrupted! Final State - Current: %.4f, Target: %.4f, Error: %.4f", 
                currentPosition, targetPosition, error));
            SmartDashboard.putString("[ELEVATOR] Command Status", "Interrupted");
        } else {
            System.out.println(String.format("[ELEVATOR] Successfully reached L1! Final State - Current: %.4f, Target: %.4f, Error: %.4f", 
                currentPosition, targetPosition, error));
            SmartDashboard.putString("[ELEVATOR] Command Status", "Completed");
        }
    }
}
