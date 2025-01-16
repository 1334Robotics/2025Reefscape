package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
    // Hardware
    private final TalonFX primaryMotor;
    private final TalonFX secondaryMotor;
    
    // State tracking
    private double primaryPositionInches = 0.0;
    private double secondaryPositionInches = 0.0;
    private double primaryTargetInches = 0.0;
    private double secondaryTargetInches = 0.0;
    private double manualPower = 0.0;
    
    private final MotionMagicVoltage primaryMotionMagic = new MotionMagicVoltage(0);
    private final MotionMagicVoltage secondaryMotionMagic = new MotionMagicVoltage(0);
    
    // Simulation
    private ElevatorSim primaryElevatorSim;
    private ElevatorSim secondaryElevatorSim;
    
    public ElevatorSubsystem(int primaryMotorId, int secondaryMotorId) {
        primaryMotor = new TalonFX(primaryMotorId);
        secondaryMotor = new TalonFX(secondaryMotorId);
        configureMotor();
        
        // Initialize simulation for primary motor
        primaryElevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            ElevatorConstants.GEAR_RATIO,
            10.0, // Mass in kg
            0.05, // Drum radius in meters
            ElevatorConstants.MIN_HEIGHT_INCHES,
            ElevatorConstants.MAX_HEIGHT_INCHES,
            true, // Simulate gravity
            primaryMotorId, new double[] {0.01, 0.01} // Measurement standard deviations
        );

        // Initialize simulation for secondary motor
        secondaryElevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            ElevatorConstants.GEAR_RATIO,
            10.0, // Mass in kg
            0.05, // Drum radius in meters
            ElevatorConstants.MIN_HEIGHT_INCHES,
            ElevatorConstants.MAX_HEIGHT_INCHES,
            true, // Simulate gravity
            secondaryMotorId, new double[] {0.01, 0.01} // Measurement standard deviations
        );
    }
    
    private void configureMotor() {
        primaryMotor.setPosition(0.0);
        secondaryMotor.setPosition(0.0);
        // Add motor configuration here
    }
    
    @Override
    public void periodic() {
        // Get position in rotations and convert to inches
        primaryPositionInches = primaryMotor.getPosition().getValueAsDouble() * ElevatorConstants.GEAR_RATIO;
        secondaryPositionInches = secondaryMotor.getPosition().getValueAsDouble() * ElevatorConstants.GEAR_RATIO;

        // Put values on NetworkTables for simulation visualization
        SmartDashboard.putNumber("[ELEVATOR] Primary Position (inches)", primaryPositionInches);
        SmartDashboard.putNumber("[ELEVATOR] Secondary Position (inches)", secondaryPositionInches);
        SmartDashboard.putNumber("[ELEVATOR] Manual Power", manualPower);
    }
    
    @Override
    public void simulationPeriodic() {
        // Update simulation with manual control values
        primaryElevatorSim.setInput(manualPower * 12.0); // Convert -1 to 1 to voltage
        secondaryElevatorSim.setInput(manualPower * 12.0);

        // Update simulation
        primaryElevatorSim.update(0.02);
        secondaryElevatorSim.update(0.02);

        // Update simulated encoder positions
        double primarySimPositionMeters = primaryElevatorSim.getPositionMeters();
        double secondarySimPositionMeters = secondaryElevatorSim.getPositionMeters();
        double primarySimPosition = primarySimPositionMeters * 39.3701; // Convert meters to inches
        double secondarySimPosition = secondarySimPositionMeters * 39.3701; // Convert meters to inches

        // Convert meters to motor rotations for simulation
        primaryMotor.getSimState().setRawRotorPosition(primarySimPosition / (2 * Math.PI * ElevatorConstants.PRIMARY_DRUM_RADIUS_INCHES));
        secondaryMotor.getSimState().setRawRotorPosition(secondarySimPosition / (2 * Math.PI * ElevatorConstants.SECONDARY_DRUM_RADIUS_INCHES));
    }
    
    public Command moveToPosition(double primaryheightInches , double secondaryheightInches) {
        return runOnce(() -> {
            primaryTargetInches = primaryheightInches;
            secondaryTargetInches = secondaryheightInches;
            primaryMotor.setControl(primaryMotionMagic.withPosition(primaryheightInches / ElevatorConstants.GEAR_RATIO));
            secondaryMotor.setControl(secondaryMotionMagic.withPosition(secondaryheightInches / ElevatorConstants.GEAR_RATIO));
        });
    }
    
    public Command holdPosition() {
        return run(() -> {
            primaryMotor.setControl(primaryMotionMagic.withPosition(primaryTargetInches / ElevatorConstants.GEAR_RATIO));
            secondaryMotor.setControl(secondaryMotionMagic.withPosition(secondaryTargetInches / ElevatorConstants.GEAR_RATIO));
        });
    }
    
    public boolean atTargetPosition() {
        return Math.abs(primaryPositionInches - primaryTargetInches) < ElevatorConstants.POSITION_TOLERANCE &&
               Math.abs(secondaryPositionInches - secondaryTargetInches) < ElevatorConstants.POSITION_TOLERANCE;
    }
    
    public double getCurrentHeightPrimary() {
        return primaryPositionInches;
    }

    public double getCurrentHeightSecondary() {
        return secondaryPositionInches;
    }

    public void setTargetPosition(double primaryTargetInches, double secondaryTargetInches) {
        // Clamp primary stage
        this.primaryTargetInches = MathUtil.clamp(
            primaryTargetInches,
            ElevatorConstants.PRIMARY_MIN_HEIGHT_INCHES,
            ElevatorConstants.PRIMARY_MAX_HEIGHT_INCHES
        );
        
        // Clamp secondary stage
        this.secondaryTargetInches = MathUtil.clamp(
            secondaryTargetInches,
            ElevatorConstants.SECONDARY_MIN_HEIGHT_INCHES,
            ElevatorConstants.SECONDARY_MAX_HEIGHT_INCHES
        );
        
        // Convert to rotations for both stages
        double primaryRotations = this.primaryTargetInches / 
            (2 * Math.PI * ElevatorConstants.PRIMARY_DRUM_RADIUS_INCHES);
        double secondaryRotations = this.secondaryTargetInches / 
            (2 * Math.PI * ElevatorConstants.SECONDARY_DRUM_RADIUS_INCHES);
        
        // Set motion magic for both motors
        primaryMotionMagic.Position = primaryRotations;
        secondaryMotionMagic.Position = secondaryRotations;
        
        primaryMotor.setControl(primaryMotionMagic);
        secondaryMotor.setControl(secondaryMotionMagic);
    }

    public void moveToPreset(ElevatorPosition preset) {
        setTargetPosition(preset.primaryHeightInches, preset.secondaryHeightInches);
    }

    public void stop() {
        primaryMotor.stopMotor();
        secondaryMotor.stopMotor();
    }

    public boolean isAtTarget() {
        boolean primaryAtTarget = Math.abs(primaryPositionInches - primaryTargetInches) < 
            ElevatorConstants.POSITION_TOLERANCE_INCHES;
        boolean secondaryAtTarget = Math.abs(secondaryPositionInches - secondaryTargetInches) < 
            ElevatorConstants.POSITION_TOLERANCE_INCHES;
        return primaryAtTarget && secondaryAtTarget;
    }

    // Add this method to enable manual control
    public void setManualControl(double power) {
        manualPower = power;
        primaryMotor.set(power);
        secondaryMotor.set(power);
    }

    public enum ElevatorPosition {
        GROUND(ElevatorConstants.GROUND_PRIMARY_HEIGHT, ElevatorConstants.GROUND_SECONDARY_HEIGHT),
        MIDDLE(ElevatorConstants.MIDDLE_PRIMARY_HEIGHT, ElevatorConstants.MIDDLE_SECONDARY_HEIGHT),
        HIGH(ElevatorConstants.HIGH_PRIMARY_HEIGHT, ElevatorConstants.HIGH_SECONDARY_HEIGHT);

        public final double primaryHeightInches;
        public final double secondaryHeightInches;
        
        ElevatorPosition(double primaryHeight, double secondaryHeight) {
            this.primaryHeightInches = primaryHeight;
            this.secondaryHeightInches = secondaryHeight;
        }
    }
}