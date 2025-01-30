package frc.robot.subsystems.mailbox;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MailboxConstants;

public class MailboxSubsystem extends SubsystemBase {
    private final TalonFXS[] motors;

    public MailboxSubsystem() {
        this.motors = new TalonFXS[2];
        this.motors[0] = new TalonFXS(MailboxConstants.MOTOR_ONE_ID);
        this.motors[1] = new TalonFXS(MailboxConstants.MOTOR_TWO_ID);
        SmartDashboard.putString("[MAILBOX] State", "Unknown");
        SmartDashboard.putNumber("[MAILBOX] Motor One Speed", 0);
        SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", 0);
    }

    public MailboxSubsystem(int motorOneId, int motorTwoId) {
        this.motors = new TalonFXS[2];
        this.motors[0] = new TalonFXS(motorOneId);
        this.motors[1] = new TalonFXS(motorTwoId);
        SmartDashboard.putString("[MAILBOX] State", "Unknown");
        SmartDashboard.putNumber("[MAILBOX] Motor One Speed", 0);
        SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", 0);
    }

    public void input() {
        this.motors[0].set(MailboxConstants.INPUT_MOTOR_ONE_SPEED);
        this.motors[1].set(MailboxConstants.INPUT_MOTOR_TWO_SPEED);
        SmartDashboard.putString("[MAILBOX] State", "Input");
    }

    public void output() {
        // This will need limit switches or other timings to tell when output is complete
        this.motors[0].set(MailboxConstants.OUTPUT_MOTOR_SPEED);
        this.motors[1].set(MailboxConstants.OUTPUT_MOTOR_SPEED);
        SmartDashboard.putString("[MAILBOX] State", "Output");
    }

    public void stop() {
        this.motors[0].set(0);
        this.motors[1].set(0);
        SmartDashboard.putString("[MAILBOX] State", "Stopped");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[MAILBOX] Motor One Speed", this.motors[0].get());
        SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", this.motors[1].get());
    }
}