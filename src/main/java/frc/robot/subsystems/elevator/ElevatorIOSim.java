package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.SimulationConstants;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim sim;
    private double appliedVolts = 0.0;

    public ElevatorIOSim() {
        sim = new ElevatorSim(
            DCMotor.getNEO(1),
            SimulationConstants.ELEVATOR_GEARING,
            SimulationConstants.ELEVATOR_CARRIAGE_MASS,
            SimulationConstants.ELEVATOR_DRUM_RADIUS,
            SimulationConstants.ELEVATOR_MIN_HEIGHT,
            SimulationConstants.ELEVATOR_MAX_HEIGHT,
            true,
            0.0
        );
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02); // 20ms update rate

        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.lowerLimitSwitch = sim.getPositionMeters() < (SimulationConstants.ELEVATOR_MIN_HEIGHT + 0.01);
        inputs.upperLimitSwitch = sim.getPositionMeters() > (SimulationConstants.ELEVATOR_MAX_HEIGHT - 0.01);
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void setPosition(double positionMeters) {
        sim.setState(positionMeters, 0.0);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }
} 