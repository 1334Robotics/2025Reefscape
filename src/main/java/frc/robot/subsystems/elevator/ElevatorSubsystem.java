package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax     motor;
    private final DigitalInput limitSwitch;

    public ElevatorSubsystem() {
        this.motor       = new SparkMax(ElevatorConstants.MOTOR_ONE_ID, MotorType.kBrushless);
        this.limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID);

        SmartDashboard.putNumber("[ELEVATOR] Position", -2147483648);
        SmartDashboard.putBoolean("[ELEVATOR] Limit Switch Seen", !this.limitSwitch.get());
    }

    public void runMotor(double speed) {
        motor.set(speed);
    }

    @Override
    public void periodic() {
        double position = motor.getEncoder().getPosition() * ElevatorConstants.GEAR_RATIO;

        SmartDashboard.putNumber("[ELEVATOR] Position", position);
        SmartDashboard.putBoolean("[ELEVATOR] Limit Switch Seen", !this.limitSwitch.get());
    }
}
