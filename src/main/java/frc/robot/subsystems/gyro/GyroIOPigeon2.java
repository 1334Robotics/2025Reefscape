package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.Units;

// real implementation with Pigeon2
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon2;
    

    public GyroIOPigeon2(int id) {
        this.pigeon2 = new Pigeon2(id);
        pigeon2.setYaw(0);
    }

    @Override // specified by GyroIOSim interface
    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble());
    }

    @Override // specified by GroIOSim interface
    public AngularVelocity getGyroAngularVelocity() {
        return Units.DegreesPerSecond.of(pigeon2.getAngularVelocityZWorld().getValueAsDouble());
    }
}