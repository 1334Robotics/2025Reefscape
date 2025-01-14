package frc.robot.subsystems.gyro;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GyroConstants;

public class GyroSubsystem extends SubsystemBase {
    private Pigeon2 pigeon;
  
    public GyroSubsystem() {
        pigeon = new Pigeon2(GyroConstants.GYRO_DEVICEID);
    }

    public GyroSubsystem(int deviceId) {
        pigeon = new Pigeon2(deviceId);
    }

    public GyroSubsystem(int deviceId, String canBus) {
        pigeon = new Pigeon2(deviceId, canBus);
    }

    public class GyroData {
        public final double yaw;
        public final double pitch;
        public final double roll;

        public GyroData(double yaw, double pitch, double roll) {
            this.yaw = yaw;
            this.pitch = pitch;
            this.roll = roll;
        }
    }

    public GyroData getData() {
        Angle yaw   = pigeon.getYaw().getValue();
        Angle pitch = pigeon.getPitch().getValue();
        Angle roll  = pigeon.getRoll().getValue();
        
        return new GyroData(yaw.in(Degrees), pitch.in(Degrees), roll.in(Degrees));
    }

    @Override
    public void periodic() {
        // Print the data
        GyroData data = getData();

        SmartDashboard.putNumber("[GYRO] Yaw", data.yaw);
        SmartDashboard.putNumber("[GYRO] Pitch", data.pitch);
        SmartDashboard.putNumber("[GYRO] Roll", data.roll);
    }

    public void reset() {
        pigeon.reset();
    }
}