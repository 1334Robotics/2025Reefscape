package frc.robot.subsystems.mailbox;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MailboxConstants;
import frc.robot.subsystems.elevator.ElevatorLevel;

public class MailboxSubsystem extends SubsystemBase {
    private final SparkMax[] motors;

    public MailboxSubsystem() {
        this.motors = new SparkMax[2];
        this.motors[0] = new SparkMax(MailboxConstants.MOTOR_ONE_ID, MotorType.kBrushless);
        this.motors[1] = new SparkMax(MailboxConstants.MOTOR_TWO_ID, MotorType.kBrushless);
        SmartDashboard.putNumber("[MAILBOX] Motor One Speed", 0);
        SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", 0);
    }

    public MailboxSubsystem(int motorOneId, int motorTwoId) {
        this.motors = new SparkMax[2];
        this.motors[0] = new SparkMax(motorOneId, MotorType.kBrushless);
        this.motors[1] = new SparkMax(motorTwoId, MotorType.kBrushless);
        SmartDashboard.putNumber("[MAILBOX] Motor One Speed", 0);
        SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", 0);
    }

    public void rewind() {
        this.motors[0].set(-MailboxConstants.REWIND_MOTOR_SPEED);
        this.motors[1].set(MailboxConstants.REWIND_MOTOR_SPEED);
    }

    public void feed() {
        this.motors[0].set(MailboxConstants.FEED_MOTOR_SPEED);
        this.motors[1].set(-MailboxConstants.FEED_MOTOR_SPEED);
    }

    public void output(ElevatorLevel level) {
        if(level == null) return;

        switch(level) {
            case L1:
                this.motors[0].set(MailboxConstants.OUTPUT_MOTOR_SPEED_L1);
                this.motors[1].set(-MailboxConstants.OUTPUT_MOTOR_SPEED_L1_LOW);
                break;
            case L2:
                this.motors[0].set(MailboxConstants.OUTPUT_MOTOR_SPEED_L2);
                this.motors[1].set(-MailboxConstants.OUTPUT_MOTOR_SPEED_L2);
                break;
            case L3:
                this.motors[0].set(MailboxConstants.OUTPUT_MOTOR_SPEED_L3);
                this.motors[1].set(-MailboxConstants.OUTPUT_MOTOR_SPEED_L3);
                break;
            case L4:
                this.motors[0].set(MailboxConstants.OUTPUT_MOTOR_SPEED_L4);
                this.motors[1].set(-MailboxConstants.OUTPUT_MOTOR_SPEED_L4);
        }
    }

    public void stop() {
        this.motors[0].set(0);
        this.motors[1].set(0);
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("[MAILBOX] Motor One Speed", this.motors[0].get());
        //SmartDashboard.putNumber("[MAILBOX] Motor Two Speed", this.motors[1].get());
    }
}