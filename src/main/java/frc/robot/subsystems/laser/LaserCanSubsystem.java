package frc.robot.subsystems.laser;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LaserCanSubsystem extends SubsystemBase {
    private final LaserCan laserCan;
    private LaserCan.Measurement lastMeasurement;

    public LaserCanSubsystem() {
        laserCan = new LaserCan(0);
        configureDevice();
    }

    private void configureDevice() {
        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
            System.out.println("LaserCan configured successfully");
        } catch (ConfigurationFailedException e) {
            System.out.println("LaserCan configuration failed! " + e);
        }
    }

    @Override
    public void periodic() {
        lastMeasurement = laserCan.getMeasurement();
        updateDashboard();
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("[LASERCAN] Distance (mm)", getDistance());
        SmartDashboard.putBoolean("[LASERCAN] Valid Reading", true);
    }

    public boolean isValidMeasurement() {
        return lastMeasurement != null &&
                lastMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }

    public double getDistance() {
        return isValidMeasurement() ? lastMeasurement.distance_mm : -1;
    }
}