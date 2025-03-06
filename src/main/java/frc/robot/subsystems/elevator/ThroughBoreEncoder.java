package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class ThroughBoreEncoder extends SubsystemBase {
    private final Encoder encoder;
    
    public ThroughBoreEncoder() {
        this.encoder = new Encoder(ElevatorConstants.ENCODER_DIO_A,
                                   ElevatorConstants.ENCODER_DIO_B);
        this.encoder.setDistancePerPulse(1.0/ElevatorConstants.ENCODER_PULSE_PER_ROTATION);
        this.encoder.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[ELEVATOR] Encoder Position", this.encoder.getDistance());
    }

    public double getDistance() {
        return this.encoder.getDistance();
    }

    public void resetEncoder() {
        this.encoder.reset();
    }
}
