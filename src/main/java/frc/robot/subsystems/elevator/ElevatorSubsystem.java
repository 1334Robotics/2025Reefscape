package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();
    private final DigitalInput limitSwitch;
    private double bottomPos;
    public boolean lock;

    public ElevatorSubsystem() {
        if (Robot.isSimulation()) {
            io = new ElevatorIOSim();
        } else {
            io = new ElevatorIOSim(); // Temporarily use sim for testing
        }
        
        this.limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID);
        this.lock = false;
        this.bottomPos = 0.0;

        SmartDashboard.putNumber("[ELEVATOR] Position", -2147483648);
        SmartDashboard.putBoolean("[ELEVATOR] Limit Switch Seen", !this.limitSwitch.get());
    }

    public void resetElevatorPos() {
        io.resetPosition();
        this.bottomPos = inputs.positionRots;
    }

    public void runMotor(double speed) {
        io.setVoltage(speed * 12.0); // Convert from -1 to 1 to voltage
    }

    public boolean limitSwitchSeen() {
        return !this.limitSwitch.get();
    }

    public double getPosition() {
        return -(inputs.positionRots - this.bottomPos);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        
        double position = this.getPosition();
        SmartDashboard.putNumber("[ELEVATOR] Position", position);
        SmartDashboard.putBoolean("[ELEVATOR] Limit Switch Seen", !this.limitSwitch.get());
        
        // Additional debug info
        SmartDashboard.putNumber("[ELEVATOR] Raw Position", inputs.positionRots);
        SmartDashboard.putNumber("[ELEVATOR] Velocity", inputs.velocityRotsPerSec);
        SmartDashboard.putNumber("[ELEVATOR] Applied Voltage", inputs.appliedVolts);
        if (inputs.currentAmps.length > 0) {
            SmartDashboard.putNumber("[ELEVATOR] Current", inputs.currentAmps[0]);
        }
    }
}
