package frc.robot.subsystems.mailbox;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MailboxConstants;

public class MailboxSubsystem extends SubsystemBase {
    private final SparkMax[] motors;

    public MailboxSubsystem() {
        this.motors = new SparkMax[2];
        this.motors[0] = new SparkMax(MailboxConstants.MOTOR_ONE_ID, MotorType.kBrushless);
        this.motors[1] = new SparkMax(MailboxConstants.MOTOR_TWO_ID, MotorType.kBrushless);
        SmartDashboard.putString("[MAILBOX] State", "Unknown");
        SmartDashboard.putNumber("[MAILBOX] Motor One Speed", 0);
        SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", 0);
    }

    public MailboxSubsystem(int motorOneId, int motorTwoId) {
        this.motors = new SparkMax[2];
        this.motors[0] = new SparkMax(motorOneId, MotorType.kBrushless);
        this.motors[1] = new SparkMax(motorTwoId, MotorType.kBrushless);
        SmartDashboard.putString("[MAILBOX] State", "Unknown");
        SmartDashboard.putNumber("[MAILBOX] Motor One Speed", 0);
        SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", 0);
    }

    public void output(boolean high) {
        // This will need limit switches or other timings to tell when output is complete
        if(high) {
            this.motors[0].set(MailboxConstants.OUTPUT_MOTOR_SPEED);
            this.motors[1].set(-MailboxConstants.OUTPUT_MOTOR_SPEED);
            SmartDashboard.putString("[MAILBOX] State", "Output High");
        } else {
            this.motors[0].set(MailboxConstants.OUTPUT_MOTOR_SPEED);
            this.motors[1].set(-MailboxConstants.OUTPUT_MOTOR_SPEED_LOW);
            SmartDashboard.putString("[MAILBOX] State", "Output Low");
        }
    }

    public void stop() {
        this.motors[0].set(0);
        this.motors[1].set(0);
        SmartDashboard.putString("[MAILBOX] State", "Stopped");
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("[MAILBOX] Motor One Speed", this.motors[0].get());
        //SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", this.motors[1].get());
    }
}