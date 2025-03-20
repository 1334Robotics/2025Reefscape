package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    
    public ElevatorIOSparkMax(int deviceId) {
        motor = new CANSparkMax(deviceId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        
        // Configure motor
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);
        
        // Configure encoder
        encoder.setPositionConversionFactor(1.0); // Update if needed
        encoder.setVelocityConversionFactor(1.0 / 60.0); // Convert RPM to RPS
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRots = encoder.getPosition();
        inputs.velocityRotsPerSec = encoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = new double[] {motor.getOutputCurrent()};
        inputs.tempCelsius = new double[] {motor.getMotorTemperature()};
    }
    
    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
    
    @Override
    public void setBrakeMode(boolean enable) {
        motor.setIdleMode(enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
    
    @Override
    public void resetPosition() {
        encoder.setPosition(0.0);
    }
} 