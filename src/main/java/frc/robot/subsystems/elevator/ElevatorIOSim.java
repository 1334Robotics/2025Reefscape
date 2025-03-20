package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
    private static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.0); // Update with actual value
    private static final double MASS_KG = 4.0; // Update with actual elevator mass
    private static final double MIN_HEIGHT_METERS = 0.0;
    private static final double MAX_HEIGHT_METERS = Units.inchesToMeters(24.0); // Update with actual max height
    private static final double GEARING = ElevatorConstants.ENCODER_PULSE_PER_ROTATION;
    
    private final ElevatorSim sim;
    private double appliedVolts = 0.0;
    
    public ElevatorIOSim() {
        sim = new ElevatorSim(
            DCMotor.getNEO(1), // One NEO motor
            GEARING,
            MASS_KG,
            DRUM_RADIUS_METERS,
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            true, // Simulate gravity
            0.0, // Starting position
            0.01 // Standard deviation
        );
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02); // 20ms update rate
        
        inputs.positionRots = sim.getPositionMeters() / (2 * Math.PI * DRUM_RADIUS_METERS);
        inputs.velocityRotsPerSec = sim.getVelocityMetersPerSecond() / (2 * Math.PI * DRUM_RADIUS_METERS);
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
        inputs.tempCelsius = new double[] {0.0}; // Not simulated
    }
    
    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        sim.setInputVoltage(volts);
    }
    
    @Override
    public void setBrakeMode(boolean enable) {
        // Not simulated
    }
    
    @Override
    public void resetPosition() {
        sim.setState(0.0, 0.0);
    }
} 