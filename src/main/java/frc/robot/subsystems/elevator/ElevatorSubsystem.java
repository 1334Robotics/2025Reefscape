package frc.robot.subsystems.elevator;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax           motor;
    private final DigitalInput       limitSwitch;
    private final ThroughBoreEncoder throughBoreEncoder;
    private double                   bottomPos;
    public  boolean                  lock;

    public ElevatorSubsystem() {
        this.motor              = new SparkMax(ElevatorConstants.MOTOR_ONE_ID, MotorType.kBrushless);
        this.limitSwitch        = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID);
        this.throughBoreEncoder = new ThroughBoreEncoder();
        this.lock               = false;

        SmartDashboard.putNumber("[ELEVATOR] Position", -2147483648);
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
        double position = this.getPosition();

        SmartDashboard.putNumber("[ELEVATOR] Position", position);
        SmartDashboard.putBoolean("[ELEVATOR] Limit Switch Seen", !this.limitSwitch.get());
    }
}
