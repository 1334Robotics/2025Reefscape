package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.drive.PIDController;

public class ElevatorHandler extends SubsystemBase {
    private ElevatorLevel            targetLevel;
    private ElevatorLevel            previousTargetLevel;
    private boolean                  hitTarget;
    private final PIDController      pidController;
    private boolean                  forceManualControl;  // New flag for forcing manual control
    private static final String      MANUAL_CONTROL_KEY = "Elevator/Force Manual Control";
    
    public ElevatorHandler() {
        this.targetLevel = null;
        this.previousTargetLevel = null;
        this.hitTarget = true;
        this.forceManualControl = false;  // Default to not forcing manual
        this.pidController = new PIDController(ElevatorConstants.PID_KP,
                                               ElevatorConstants.PID_KI,
                                               ElevatorConstants.PID_KD,
                                               ElevatorConstants.PID_TAU,
                                               ElevatorConstants.PID_LIM_MIN,
                                               ElevatorConstants.PID_LIM_MAX,
                                               ElevatorConstants.PID_LIM_MIN_INT,
                                               ElevatorConstants.PID_LIM_MAX_INT,
                                               ElevatorConstants.PID_SAMPLE_TIME);
                                               
        // Initialize dashboard controls
        SmartDashboard.putBoolean(MANUAL_CONTROL_KEY, false);
    }

    public void setLevel(ElevatorLevel level) {
        this.targetLevel = level;
        this.hitTarget = false;
    }

    public ElevatorLevel getLevel() {
        return this.targetLevel;
    }

    // New methods to control manual mode
    public void setForceManualControl(boolean force) {
        this.forceManualControl = force;
        SmartDashboard.putBoolean(MANUAL_CONTROL_KEY, force);
        if (force) {
            // Stop any ongoing automated movement
            RobotContainer.elevatorSubsystem.runMotor(0);
            this.hitTarget = true;
        }
    }

    public boolean isForceManualControl() {
        return this.forceManualControl;
    }

    @Override
    public void periodic() {
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
                System.out.println("Elevator: Manual control active, ignoring target " + this.targetLevel);
            }
            return;
        }

        // 2. Stop conditions - no target, hit target, or error within bounds
        double currentPosition = RobotContainer.elevatorSubsystem.getPosition();
        
        if(this.targetLevel == null || this.hitTarget) {
            RobotContainer.elevatorSubsystem.runMotor(0);
            return;
        }

        // 3. Handle state transitions
        if(this.previousTargetLevel != this.targetLevel) {
            this.previousTargetLevel = this.targetLevel;
            this.pidController.zero();
            System.out.println("Elevator: Target changed to " + this.targetLevel + 
                             " at position " + this.targetLevel.position);
        }

        // 4. Check if at target
        double error = Math.abs(currentPosition - this.targetLevel.position);
        System.out.println("Elevator: Current=" + currentPosition + 
                         " Target=" + this.targetLevel.position + 
                         " Error=" + error);
        
        if(error < ElevatorConstants.MAX_ACCEPTABLE_ERROR) {
            this.hitTarget = true;
            RobotContainer.elevatorSubsystem.runMotor(0);
            System.out.println("Elevator: Reached target " + this.targetLevel);
            return;
        }

        // 5. Move to target
        this.pidController.update(this.targetLevel.position, currentPosition);
        double output = -this.pidController.getOutput();
        RobotContainer.elevatorSubsystem.runMotor(output);
        System.out.println("Elevator: Moving with output " + output);
        
        // Update dashboard with movement info
        SmartDashboard.putNumber("[ELEVATOR] Target Position", 
            this.targetLevel != null ? this.targetLevel.position : 0.0);
        SmartDashboard.putNumber("Elevator/Current Position", currentPosition);
        SmartDashboard.putNumber("Elevator/Error", error);
        SmartDashboard.putNumber("Elevator/Motor Output", output);
    }
}