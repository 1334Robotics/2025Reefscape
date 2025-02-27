package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleElevatorSubsystem extends SubsystemBase {
    private final SparkMax motor;

    public SingleElevatorSubsystem(int motorId) {
        this.motor = new SparkMax(motorId, MotorType.kBrushless);
    }

    public void runMotor(double speed) {
        motor.set(speed);
    }
}
