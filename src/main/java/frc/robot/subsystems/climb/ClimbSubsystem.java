package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import frc.robot.constants.ClimbConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem extends SubsystemBase {
    // Two lower Neo motors for forcing the pins down
    private final SparkMax lowerMotor1;
    private final SparkMax lowerMotor2;
    // One Neo 550 motor for locking the climb mechanism
    private final SparkMax lockMotor;

    public ClimbSubsystem() {
        // Adjust CAN IDs as needed based on your wiring
        lowerMotor1 = new SparkMax(ClimbConstants.LOWER_MOTOR1_CAN_ID, MotorType.kBrushless);
        lowerMotor2 = new SparkMax(ClimbConstants.LOWER_MOTOR2_CAN_ID, MotorType.kBrushless);
        lockMotor   = new SparkMax(ClimbConstants.LOCK_MOTOR_CAN_ID, MotorType.kBrushless);

        // // Restore defaults and set idle mode to brake for better hold
        // lowerMotor1.restoreFactoryDefaults();
        // lowerMotor2.restoreFactoryDefaults();
        // lockMotor.restoreFactoryDefaults();
        
        // lowerMotor1.setIdleMode(SparkMax.IdleMode.kBrake);
        // lowerMotor2.setIdleMode(SparkMax.IdleMode.kBrake);
        // lockMotor.setIdleMode(SparkMax.IdleMode.kBrake);
    }

    /**
     * Runs both lower motors simultaneously to force the pins down.
     *
     * @param speed Speed from 0.0 to 1.0 (or negative if reversing is needed)
     */
    public void forcePinsDown(double speed) {
        lowerMotor1.set(speed);
        lowerMotor2.set(speed);
        SmartDashboard.putString("Climb State", "Forcing pins down");
    }

    /**
     * Runs the lock motor to secure the bot to the climb device.
     *
     * @param speed Speed from 0.0 to 1.0 (or negative if reversing is needed)
     */
    public void lockClimb(double speed) {
        lockMotor.set(speed);
        SmartDashboard.putString("Climb State", "Locking climb mechanism");
    }

    /** Stops all climb motors. */
    public void stopClimb() {
        lowerMotor1.set(0);
        lowerMotor2.set(0);
        lockMotor.set(0);
        SmartDashboard.putString("Climb State", "Stopped");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/LowerMotor1 Output", lowerMotor1.get());
        SmartDashboard.putNumber("Climb/LowerMotor2 Output", lowerMotor2.get());
        SmartDashboard.putNumber("Climb/LockMotor Output", lockMotor.get());
    }
}