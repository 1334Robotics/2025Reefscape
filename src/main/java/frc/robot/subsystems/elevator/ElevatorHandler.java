package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.drive.PIDController;
import frc.robot.subsystems.led.LedHandler;
import frc.robot.commands.led.LEDColorCommand;

public class ElevatorHandler extends SubsystemBase {
    private ElevatorLevel targetLevel;
    private ElevatorLevel previousTargetLevel;
    private boolean hitTarget;
    private final PIDController pidController;
    private final LedHandler ledHandler;
    
    public ElevatorHandler(LedHandler ledHandler) {
        this.targetLevel = null;
        this.previousTargetLevel = null;
        this.hitTarget = true;
        this.ledHandler = ledHandler;
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
        if(ElevatorConstants.MANUAL_ELEVATOR_CONTROL) return;
        if(!this.hitTarget) {
            ledHandler.requestControl(LedHandler.Controller.ELEVATOR);
            
            if(this.targetLevel == null) {
                ledHandler.setColor(LedHandler.Controller.ELEVATOR, LEDColorCommand.Color.RED);
                return;
            }

            if(this.previousTargetLevel != this.targetLevel) {
                this.previousTargetLevel = this.targetLevel;
                this.pidController.zero();
            }

            double offPos = Math.abs(RobotContainer.elevatorSubsystem.getPosition() - this.targetLevel.position);
            if(offPos < ElevatorConstants.MAX_ACCEPTABLE_ERROR) {
                this.hitTarget = true;
                ledHandler.setColor(LedHandler.Controller.ELEVATOR, LEDColorCommand.Color.GREEN);
                return;
            } 

            // Moving - show yellow
            ledHandler.setColor(LedHandler.Controller.ELEVATOR, LEDColorCommand.Color.YELLOW);
            
            // Get the desired speed
            this.pidController.update(this.targetLevel.position, RobotContainer.elevatorSubsystem.getPosition());
            RobotContainer.elevatorSubsystem.runMotor(-this.pidController.getOutput());
        } else {
            RobotContainer.elevatorSubsystem.runMotor(0);
        }
    }
}