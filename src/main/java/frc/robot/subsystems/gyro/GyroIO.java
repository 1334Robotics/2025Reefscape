package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    Rotation2d getGyroRotation();
    AngularVelocity getGyroAngularVelocity();

    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double yawPosition = 0.0;
        public double yawVelocityRadPerSec = 0.0;
    }
}