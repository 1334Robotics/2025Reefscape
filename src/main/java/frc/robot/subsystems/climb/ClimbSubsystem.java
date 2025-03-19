package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import frc.robot.constants.ClimbConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

public class ClimbSubsystem extends SubsystemBase {
    // Two lower Neo motors for forcing the pins down
    private final SparkMax lowerMotor1;
    private final SparkMax lowerMotor2;
    // One Neo 550 motor for locking the climb mechanism
    private final SparkMax lockMotor;

    private final Encoder lowerMotor1Encoder = new Encoder(0, 1); // DIO ports for encoder
    private final Encoder lowerMotor2Encoder = new Encoder(2, 3); // DIO ports for encoder
    private final Encoder lockMotorEncoder = new Encoder(4, 5); // DIO ports for encoder

    // Conversion factor from encoder ticks to inches
    private static final double TICKS_TO_INCHES = 0.01; // Adjust this value based on your specific encoder and mechanism

    public ClimbSubsystem() {
        // Adjust CAN IDs as needed based on your wiring
        lowerMotor1 = new SparkMax(ClimbConstants.LOWER_MOTOR1_CAN_ID, MotorType.kBrushless);
        lowerMotor2 = new SparkMax(ClimbConstants.LOWER_MOTOR2_CAN_ID, MotorType.kBrushless);
        lockMotor   = new SparkMax(ClimbConstants.LOCK_MOTOR_CAN_ID, MotorType.kBrushless);

        // Initialize encoders if needed
        lowerMotor1Encoder.setDistancePerPulse(TICKS_TO_INCHES);
        lowerMotor2Encoder.setDistancePerPulse(TICKS_TO_INCHES);
        lockMotorEncoder.setDistancePerPulse(TICKS_TO_INCHES);
    }

    /**
     * Runs both lower motors simultaneously to force the pins down.
     *
     * @param speed Speed from 0.0 to 1.0 (or negative if reversing is needed)
     */
    public void forcePinsDown(){
        lowerMotor1.set(ClimbConstants.FORCE_PIN_MOTOR_SPEED);
        lowerMotor2.set(ClimbConstants.FORCE_PIN_MOTOR_SPEED);
        SmartDashboard.putString("[Climb] State", "Forcing pins down");
    }

    /**
     * Runs the lock motor to secure the bot to the climb device.
     *
     * @param speed Speed from 0.0 to 1.0 (or negative if reversing is needed)
     */
    public void lockClimb() {
        lockMotor.set(ClimbConstants.LOCK_CLIMB_MOTOR_SPEED);
        SmartDashboard.putString("[Climb] State", "Locking climb mechanism");
    }

    public void unlockClimb() {
        lockMotor.set(-ClimbConstants.LOCK_CLIMB_MOTOR_SPEED);
        SmartDashboard.putString("[Climb] State", "Unlocking climb mechanism");
    }

    public void forcePinsUp() {
        lowerMotor1.set(-ClimbConstants.FORCE_PIN_MOTOR_SPEED);
        lowerMotor2.set(-ClimbConstants.FORCE_PIN_MOTOR_SPEED);
        SmartDashboard.putString("[Climb] State", "Forcing pins up");
    }

    /** Stops all climb motors. */
    public void stopClimb() {
        lowerMotor1.set(0);
        lowerMotor2.set(0);
        lockMotor.set(0);
        SmartDashboard.putString("[Climb] State", "Stopped");
    }

    public double getLowerMotor1PositionInches() {
        return lowerMotor1Encoder.getDistance();
    }

    public double getLowerMotor2PositionInches() {
        return lowerMotor2Encoder.getDistance();
    }

    public double getLockMotorPositionInches() {
        return lockMotorEncoder.getDistance();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/LowerMotor1 Output", lowerMotor1.get());
        SmartDashboard.putNumber("Climb/LowerMotor2 Output", lowerMotor2.get());
        SmartDashboard.putNumber("Climb/LockMotor Output", lockMotor.get());
    }
}