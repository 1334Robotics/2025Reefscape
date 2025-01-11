package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.system.LinearSystem;

public class ElevatorSubsystem extends SubsystemBase {
    // Hardware
    private final TalonFX elevatorMotor;
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    
    // State tracking
    private double targetPositionInches = 0.0;
    private double currentPositionInches = 0.0;
    
    // Simulation
    private ElevatorSim elevatorSim;
    
    public ElevatorSubsystem(int motorId) {
        elevatorMotor = new TalonFX(motorId);
        configureMotor();
        
        // Initialize simulation
        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            ElevatorConstants.GEAR_RATIO,
            10.0, // Mass in kg
            0.05, // Drum radius in meters
            ElevatorConstants.MIN_HEIGHT_INCHES,
            ElevatorConstants.MAX_HEIGHT_INCHES,
            true // Simulate gravity
        );
    }
    
    private void configureMotor() {
        elevatorMotor.setPosition(0.0);
        // Add motor configuration here
    }
    
    @Override
    public void periodic() {
        currentPositionInches = elevatorMotor.getPosition().getValue() * ElevatorConstants.GEAR_RATIO;
    }
    
    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(elevatorMotor.get());
        elevatorSim.update(0.02);
        
        double simPosition = elevatorSim.getPositionMeters();
        // Update simulated position
    }
    
    public Command moveToPosition(double heightInches) {
        return runOnce(() -> {
            targetPositionInches = heightInches;
            elevatorMotor.setControl(motionMagic.withPosition(heightInches / ElevatorConstants.GEAR_RATIO));
        });
    }
    
    public Command holdPosition() {
        return run(() -> {
            elevatorMotor.setControl(motionMagic.withPosition(targetPositionInches / ElevatorConstants.GEAR_RATIO));
        });
    }
    
    public boolean atTargetPosition() {
        return Math.abs(currentPositionInches - targetPositionInches) < ElevatorConstants.POSITION_TOLERANCE;
    }
    
    public double getCurrentHeight() {
        return currentPositionInches;
    }
}