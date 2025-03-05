package frc.robot.subsystems.laser;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LaserCanConstants;

public class LaserCanSubsystem extends SubsystemBase {
    private final LaserCan laserCan;
    private LaserCan.Measurement lastMeasurement;

    public LaserCanSubsystem() {
        laserCan = new LaserCan(LaserCanConstants.can_id);
        configureDevice();
    }

    private void configureDevice() {
        System.out.println("[LaserCan] Configuring LaserCan...");
        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(LaserCanConstants.x,LaserCanConstants.y,LaserCanConstants.width,LaserCanConstants.height)); // Configure in constants
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
            System.out.println("[LaserCan] LaserCan configured successfully");
        } catch (ConfigurationFailedException e) {
            System.out.println("[LaserCan] LaserCan configuration failed! " + e);
        }
    }

    @Override
    public void periodic() {
        lastMeasurement = laserCan.getMeasurement();
        updateDashboard();
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("[LASERCAN] Distance (mm)", getDistance());
        SmartDashboard.putBoolean("[LASERCAN] Valid Reading", isValidMeasurement());
    }

    public boolean isValidMeasurement() {
        return lastMeasurement != null &&
                lastMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }

    public double getDistance() {
        return isValidMeasurement() ? lastMeasurement.distance_mm : -1;
    }
}