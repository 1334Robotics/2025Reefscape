package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    public static class ElevatorIOInputs {
        public double positionRots = 0.0;
        public double velocityRotsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run the elevator motor at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Enable or disable brake mode. */
    public default void setBrakeMode(boolean enable) {}
    
    /** Reset the encoder position to zero. */
    public default void resetPosition() {}
} 