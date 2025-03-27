package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.drive.PIDController;

public class ElevatorHandler extends SubsystemBase {
    private ElevatorLevel            targetLevel;
    private ElevatorLevel            previousTargetLevel;
    private boolean                  hitTarget;
    private final PIDController      pidController;
    
    public ElevatorHandler() {
        this.targetLevel = null;
        this.previousTargetLevel = null;
        this.hitTarget = true;
        this.pidController = new PIDController(ElevatorConstants.PID_KP,
                                               ElevatorConstants.PID_KI,
                                               ElevatorConstants.PID_KD,
                                               ElevatorConstants.PID_TAU,
                                               ElevatorConstants.PID_LIM_MIN,
                                               ElevatorConstants.PID_LIM_MAX,
                                               ElevatorConstants.PID_LIM_MIN_INT,
                                               ElevatorConstants.PID_LIM_MAX_INT,
                                               ElevatorConstants.PID_SAMPLE_TIME);
    }

    public void setLevel(ElevatorLevel level) {
        this.targetLevel = level;
        this.hitTarget = false;
    }

    public ElevatorLevel getLevel() {
        return this.targetLevel;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[ELEVATOR] Current Position", RobotContainer.elevatorSubsystem.getPosition());
        // Check dashboard for manual control toggle
        boolean dashboardManualControl = SmartDashboard.getBoolean(MANUAL_CONTROL_KEY, false);
        if (dashboardManualControl != this.forceManualControl) {
            setForceManualControl(dashboardManualControl);
        }

        // Update dashboard with current state
        SmartDashboard.putString("Elevator/Control Mode", 
            (this.forceManualControl || ElevatorConstants.MANUAL_ELEVATOR_CONTROL) ? 
            "MANUAL" : "AUTOMATED");
        
        // 1. Safety check - if in manual mode, let manual control handle everything
        if(this.forceManualControl || ElevatorConstants.MANUAL_ELEVATOR_CONTROL) {
            if (this.targetLevel != null) {
                // System.out.println("Elevator: Manual control active, ignoring target " + this.targetLevel);
            }
            return;
        }

        // 2. Stop conditions - no target, hit target, or error within bounds
        double currentPosition = RobotContainer.elevatorSubsystem.getPosition();
        
        if(this.targetLevel == null || this.hitTarget) {
            RobotContainer.elevatorSubsystem.runMotor(0);
            return;
        }

            // Get the desired speed
            this.pidController.update(this.targetLevel.position, RobotContainer.elevatorSubsystem.getPosition());
            RobotContainer.elevatorSubsystem.runMotor(-this.pidController.getOutput());
        } else {
            RobotContainer.elevatorSubsystem.runMotor(0);
        }
    }
}