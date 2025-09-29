package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;

import frc.robot.RobotContainer;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.controller.ControllerVibration;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem extends SubsystemBase {
    // // Two lower Neo motors for forcing the pins down
    // private final SparkMax lowerMotor1;
    // private final SparkMax lowerMotor2;
    // // One Neo 550 motor for locking the climb mechanism
    // private final SparkMax lockMotor;

    // private ClimbState state;

    // public ClimbSubsystem() {
    //     // Adjust CAN IDs as needed based on your wiring
    //     lowerMotor1 = new SparkMax(ClimbConstants.LOWER_MOTOR1_CAN_ID, MotorType.kBrushless);
    //     lowerMotor2 = new SparkMax(ClimbConstants.LOWER_MOTOR2_CAN_ID, MotorType.kBrushless);
    //     lockMotor   = new SparkMax(ClimbConstants.LOCK_MOTOR_CAN_ID, MotorType.kBrushless);
    //     this.state  = ClimbState.STOP;
    // }

    // private boolean overClimbMotorLimits() {
    //     if(this.lowerMotor1.getEncoder().getPosition() > ClimbConstants.CLIMB_MOTOR_1_MAX_POS) return true;
    //     if(this.lowerMotor2.getEncoder().getPosition() > ClimbConstants.CLIMB_MOTOR_2_MAX_POS) return true;
    //     return false;
    // }

    // private boolean overLockMotorLimits() {
    //     if(this.lockMotor.getEncoder().getPosition() > ClimbConstants.LOCK_MOTOR_MAX_POS) return true;
    //     return false;
    // }

    // private boolean underClimbMotorLimits() {
    //     if(this.lowerMotor1.getEncoder().getPosition() < ClimbConstants.CLIMB_MOTOR_1_MIN_POS) return true;
    //     if(this.lowerMotor2.getEncoder().getPosition() < ClimbConstants.CLIMB_MOTOR_2_MIN_POS) return true;
    //     return false;
    // }

    // private boolean underLockMotorLimits() {
    //     if(this.lockMotor.getEncoder().getPosition() < ClimbConstants.LOCK_MOTOR_MIN_POS) return true;
    //     return false;
    // }

    // /**
    //  * Runs both lower motors simultaneously to force the pins down.
    //  *
    //  * @param speed Speed from 0.0 to 1.0 (or negative if reversing is needed)
    //  */
    // public void forcePinsDown() {
    //     this.state = ClimbState.CLIMB_DOWN;
    // }

    // /**
    //  * Runs the lock motor to secure the bot to the climb device.
    //  *
    //  * @param speed Speed from 0.0 to 1.0 (or negative if reversing is needed)
    //  */
    // public void lockClimb() {
    //     this.state = ClimbState.LOCK;
    // }

    // public void unlockClimb() {
    //     this.state = ClimbState.UNLOCK;
    // }

    // public void forcePinsUp() {
    //     this.state = ClimbState.CLIMB_UP;
    // }

    // /** Stops all climb motors. */
    // public void stopClimb() {
    //     this.state = ClimbState.STOP;

    //     lowerMotor1.set(0);
    //     lowerMotor2.set(0);
    //     lockMotor.set(0);
    //     SmartDashboard.putString("[Climb] State", "Stopped");
    // }

    // @Override
    // public void periodic() {
    //     switch(this.state) {
    //         case CLIMB_DOWN:
    //             if(this.overClimbMotorLimits()) {
    //                 RobotContainer.operatorControllerSubsystem.vibrate(ControllerVibration.LIGHT);
    //                 this.state = ClimbState.STOP;
    //                 lowerMotor1.set(0);
    //                 lowerMotor2.set(0);
    //                 lockMotor.set(0);
    //                 SmartDashboard.putString("[Climb] State", "Stopped");
    //                 break;
    //             }
    //             lowerMotor1.set(ClimbConstants.FORCE_PIN_MOTOR_SPEED);
    //             lowerMotor2.set(ClimbConstants.FORCE_PIN_MOTOR_SPEED);
    //             SmartDashboard.putString("[CLIMB] State", "Forcing pins down");
    //             break;
    //         case CLIMB_UP:
    //             if(this.underClimbMotorLimits()) {
    //                 RobotContainer.operatorControllerSubsystem.vibrate(ControllerVibration.LIGHT);
    //                 this.state = ClimbState.STOP;
    //                 lowerMotor1.set(0);
    //                 lowerMotor2.set(0);
    //                 lockMotor.set(0);
    //                 SmartDashboard.putString("[Climb] State", "Stopped");
    //                 break;
    //             }
    //             lowerMotor1.set(-ClimbConstants.FORCE_PIN_MOTOR_SPEED);
    //             lowerMotor2.set(-ClimbConstants.FORCE_PIN_MOTOR_SPEED);
    //             SmartDashboard.putString("[CLIMB] State", "Forcing pins up");
    //             break;
    //         case LOCK:
    //             if(this.overLockMotorLimits()) {
    //                 RobotContainer.operatorControllerSubsystem.vibrate(ControllerVibration.LIGHT);
    //                 this.state = ClimbState.STOP;
    //                 lowerMotor1.set(0);
    //                 lowerMotor2.set(0);
    //                 lockMotor.set(0);
    //                 SmartDashboard.putString("[Climb] State", "Stopped");
    //                 break;
    //             }
    //             lockMotor.set(ClimbConstants.LOCK_CLIMB_MOTOR_SPEED);
    //             SmartDashboard.putString("[CLIMB] State", "Locking climb mechanism");
    //             break;
    //         case UNLOCK:
    //             if(this.underLockMotorLimits()) {
    //                 RobotContainer.operatorControllerSubsystem.vibrate(ControllerVibration.LIGHT);
    //                 this.state = ClimbState.STOP;
    //                 lowerMotor1.set(0);
    //                 lowerMotor2.set(0);
    //                 lockMotor.set(0);
    //                 SmartDashboard.putString("[Climb] State", "Stopped");
    //                 break;
    //             }
    //             lockMotor.set(-ClimbConstants.LOCK_CLIMB_MOTOR_SPEED);
    //             SmartDashboard.putString("[CLIMB] State", "Unlocking climb mechanism");
    //             break;
    //         case STOP:
    //             lowerMotor1.set(0);
    //             lowerMotor2.set(0);
    //             lockMotor.set(0);
    //             SmartDashboard.putString("[Climb] State", "Stopped");
    //             break;
    //     }

    //     SmartDashboard.putNumber("Climb/LowerMotor1 Output", lowerMotor1.get());
    //     SmartDashboard.putNumber("Climb/LowerMotor2 Output", lowerMotor2.get());
    //     SmartDashboard.putNumber("Climb/LockMotor Output", lockMotor.get());

    //     SmartDashboard.putNumber("[CLIMB] Climb Motor 1 Position", this.lowerMotor1.getEncoder().getPosition());
    //     SmartDashboard.putNumber("[CLIMB] Climb Motor 2 Position", this.lowerMotor2.getEncoder().getPosition());
    //     SmartDashboard.putNumber("[CLIMB] Lock Motor Position",    this.lockMotor.getEncoder().getPosition());
    // }
}