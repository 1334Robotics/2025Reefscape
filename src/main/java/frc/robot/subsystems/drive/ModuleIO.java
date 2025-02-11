package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double steerAngle = 0.0;
        public double steerVelocityRadPerSec = 0.0;
    }

    /** Sets the drive voltage. */
    public void setDriveVoltage(Voltage voltage);

    /** Sets the steer voltage. */
    public void setSteerVoltage(Voltage voltage);

    /** Gets the steer rotation. */
    public Rotation2d getSteerRotation();

    public Rotation2d getSteerFacing();

    public Angle getSteerRelativePosition();

    public Angle getDriveWheelPosition();
}