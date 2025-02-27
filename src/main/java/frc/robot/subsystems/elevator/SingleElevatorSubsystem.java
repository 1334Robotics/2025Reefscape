package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class SingleElevatorSubsystem extends SubsystemBase {
    private final SparkMax motor;

    public SingleElevatorSubsystem(int motorId) {
        this.motor = new SparkMax(motorId, MotorType.kBrushless);

        SmartDashboard.putNumber("[ELEVATOR] Position", -2147483648);
    }

    public void runMotor(double speed) {
        motor.set(speed);
    }

    @Override
    public void periodic() {
        double position = motor.getEncoder().getPosition() * ElevatorConstants.GEAR_RATIO;

        SmartDashboard.putNumber("[ELEVATOR] Position", position);
    }
}
