package frc.robot.subsystems.mailbox;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MailboxConstants;

public class MailboxSubsystem extends SubsystemBase {
    private final TalonSRX[] motors;

    public MailboxSubsystem() {
        this.motors = new TalonSRX[2];
        this.motors[0] = new TalonSRX(MailboxConstants.MOTOR_ONE_ID);
        this.motors[1] = new TalonSRX(MailboxConstants.MOTOR_TWO_ID);
        SmartDashboard.putString("[MAILBOX] State", "Unknown");
        SmartDashboard.putNumber("[MAILBOX] Motor One Speed", 0);
        SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", 0);
    }

    public MailboxSubsystem(int motorOneId, int motorTwoId) {
        this.motors = new TalonSRX[2];
        this.motors[0] = new TalonSRX(motorOneId);
        this.motors[1] = new TalonSRX(motorTwoId);
        SmartDashboard.putString("[MAILBOX] State", "Unknown");
        SmartDashboard.putNumber("[MAILBOX] Motor One Speed", 0);
        SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", 0);
    }

    public void input() {
        this.motors[0].set(ControlMode.PercentOutput, MailboxConstants.INPUT_MOTOR_ONE_SPEED);
        this.motors[1].set(ControlMode.PercentOutput, MailboxConstants.INPUT_MOTOR_TWO_SPEED);
        SmartDashboard.putString("[MAILBOX] State", "Input");
    }

    public void output() {
        // This will need limit switches or other timings to tell when output is complete
        this.motors[0].set(ControlMode.PercentOutput, MailboxConstants.OUTPUT_MOTOR_SPEED);
        this.motors[1].set(ControlMode.PercentOutput, MailboxConstants.OUTPUT_MOTOR_SPEED);
        SmartDashboard.putString("[MAILBOX] State", "Output");
    }

    public void stop() {
        this.motors[0].set(ControlMode.PercentOutput, 0);
        this.motors[1].set(ControlMode.PercentOutput, 0);
        SmartDashboard.putString("[MAILBOX] State", "Stopped");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[MAILBOX] Motor One Speed", this.motors[0].getMotorOutputPercent());
        SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", this.motors[1].getMotorOutputPercent());
    }
}