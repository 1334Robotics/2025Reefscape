package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs implements LoggableInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public boolean lowerLimitSwitch = false;
        public boolean upperLimitSwitch = false;

        @Override
        public void toLog(LogTable table) {
            table.put("positionMeters", positionMeters);
            table.put("velocityMetersPerSec", velocityMetersPerSec);
            table.put("appliedVolts", appliedVolts);
            table.put("currentAmps", currentAmps);
            table.put("lowerLimitSwitch", lowerLimitSwitch);
            table.put("upperLimitSwitch", upperLimitSwitch);
        }

        @Override
        public void fromLog(LogTable table) {
            positionMeters = table.get("positionMeters", positionMeters);
            velocityMetersPerSec = table.get("velocityMetersPerSec", velocityMetersPerSec);
            appliedVolts = table.get("appliedVolts", appliedVolts);
            currentAmps = table.get("currentAmps", currentAmps);
            lowerLimitSwitch = table.get("lowerLimitSwitch", lowerLimitSwitch);
            upperLimitSwitch = table.get("upperLimitSwitch", upperLimitSwitch);
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run the elevator motor at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Zero the elevator encoder position. */
    public default void setPosition(double positionMeters) {}

    /** Stop the elevator motor. */
    public default void stop() {}
} 