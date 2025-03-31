package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX            motor;
    private final DigitalInput       limitSwitch;
    private final ThroughBoreEncoder throughBoreEncoder;
    private double                   bottomPos;
    public  boolean                  lock;

    public ElevatorSubsystem() {
        this.motor              = new TalonFX(ElevatorConstants.MOTOR_ONE_ID);
        this.limitSwitch        = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID);
        this.throughBoreEncoder = new ThroughBoreEncoder();
        this.lock               = false;
        
        // Configure Kraken motor
        this.motor.setNeutralMode(NeutralModeValue.Brake);
        
        SmartDashboard.putBoolean("[ELEVATOR] Limit Switch Seen", !this.limitSwitch.get());
    }

    public void resetElevatorPos() {
        this.bottomPos = this.throughBoreEncoder.getDistance();
    }

    public void runMotor(double speed) {
        motor.set(speed);
    }

    public boolean limitSwitchSeen() {
        return !this.limitSwitch.get();
    }

    public double getPosition() {
        return -(this.throughBoreEncoder.getDistance() - this.bottomPos);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("[ELEVATOR] Limit Switch Seen", !this.limitSwitch.get());
    }
}
