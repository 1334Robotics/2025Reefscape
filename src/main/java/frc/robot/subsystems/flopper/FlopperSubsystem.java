package frc.robot.subsystems.flopper;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlopperConstants;

public class FlopperSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private boolean zeroing;

    public FlopperSubsystem() {
        this.motor = new SparkMax(FlopperConstants.MOTOR_ID, MotorType.kBrushless);
        this.zeroing = false;
        SmartDashboard.putString("[FLOPPER] State", "Unknown");
    }

    public FlopperSubsystem(int motorId) {
        this.motor = new SparkMax(motorId, MotorType.kBrushless);
        SmartDashboard.putString("[FLOPPER] State", "Unknown");
    }

    public void goUp() {
        if(this.zeroing) return;
        this.motor.set(FlopperConstants.MOTOR_SPEED);
        SmartDashboard.putString("[FLOPPER] State", "Going up");
    }

    public void goDown() {
        this.motor.set(-FlopperConstants.MOTOR_SPEED);
        SmartDashboard.putString("[FLOPPER] State", "Going down");
    }

    public void stop() {
        this.motor.set(0);
        SmartDashboard.putString("[FLOPPER] State", "Stopped");
    }

    public void zero() {
        this.zeroing = true;
    }

    public void resetPose() {
        this.motor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[FLOPPER] Position", this.motor.getEncoder().getPosition());

        if(this.zeroing) {
            if(this.motor.getEncoder().getPosition() > 0)
                this.goDown();
            else { this.stop(); this.zeroing = false; }
        }
    }
}
