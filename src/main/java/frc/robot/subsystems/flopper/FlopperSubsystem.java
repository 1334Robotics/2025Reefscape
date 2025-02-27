package frc.robot.subsystems.flopper;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlopperConstants;

public class FlopperSubsystem extends SubsystemBase {
    private final TalonFX motor;

    public FlopperSubsystem() {
        this.motor = new TalonFX(FlopperConstants.MOTOR_ID);
    }

    public FlopperSubsystem(int motorId) {
        this.motor = new TalonFX(motorId);
    }

    public void goUp() {
        this.motor.set(FlopperConstants.MOTOR_SPEED);
    }

    public void goDown() {
        this.motor.set(FlopperConstants.MOTOR_SPEED);
    }

    public void stop() {
        this.motor.set(0);
    }
}
