package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SimulationConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();
    private double targetPositionMeters = 0.0;
    private boolean isHomed = false;
    public boolean lock = false; // Added back for compatibility

    public ElevatorSubsystem() {
        if (Robot.isSimulation()) {
            io = new ElevatorIOSim();
        } else {
            // TODO: Replace with real hardware implementation
            io = new ElevatorIOSim();
        }

        // Initialize dashboard values
        SmartDashboard.putNumber("[ELEVATOR] Position", 0.0);
        SmartDashboard.putBoolean("[ELEVATOR] Homed", false);
        SmartDashboard.putNumber("[ELEVATOR] Target Position", 0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Update dashboard
        SmartDashboard.putNumber("[ELEVATOR] Position", getPosition());
        SmartDashboard.putBoolean("[ELEVATOR] Lower Limit", inputs.lowerLimitSwitch);
        SmartDashboard.putBoolean("[ELEVATOR] Upper Limit", inputs.upperLimitSwitch);
        SmartDashboard.putNumber("[ELEVATOR] Velocity", inputs.velocityMetersPerSec);
        SmartDashboard.putNumber("[ELEVATOR] Current", inputs.currentAmps);
        SmartDashboard.putBoolean("[ELEVATOR] Homed", isHomed);
        SmartDashboard.putNumber("[ELEVATOR] Target Position", targetPositionMeters);

        // Basic position control
        if (isHomed && !lock) {
            double error = targetPositionMeters - inputs.positionMeters;
            double output = error * ElevatorConstants.PID_KP;
            
            // Limit output
            output = Math.max(-ElevatorConstants.PID_LIM_MAX, Math.min(output, ElevatorConstants.PID_LIM_MAX));
            
            // Convert to voltage and invert for correct direction
            double voltage = -output * 12.0; // Negative because positive voltage moves down
            io.setVoltage(voltage);
        }
    }

    // Compatibility methods
    public double getPosition() {
        return inputs.positionMeters;
    }

    public boolean limitSwitchSeen() {
        return inputs.lowerLimitSwitch;
    }

    public void resetElevatorPos() {
        // First move down slowly until we hit the limit switch
        if (!inputs.lowerLimitSwitch) {
            io.setVoltage(-ElevatorConstants.ELEVATOR_DOWN_SPEED * 12.0);
        } else {
            // Once we hit the limit switch, zero the position
            io.setPosition(0.0);
            isHomed = true;
            io.setVoltage(0.0);
        }
    }

    // New methods
    public void setTargetPosition(double positionMeters) {
        if (isHomed && !lock) {
            targetPositionMeters = Math.min(Math.max(positionMeters, 0.0), SimulationConstants.ELEVATOR_MAX_HEIGHT);
        }
    }

    public double getPositionMeters() {
        return inputs.positionMeters;
    }

    public void runMotor(double speed) {
        if (!lock) {
            // Invert speed for correct direction
            io.setVoltage(-speed * 12.0); // Negative because positive voltage moves down
        }
    }

    public void stop() {
        io.stop();
    }

    public boolean isAtLowerLimit() {
        return inputs.lowerLimitSwitch;
    }

    public void zeroEncoder() {
        resetElevatorPos();
    }

    public boolean isHomed() {
        return isHomed;
    }
}
