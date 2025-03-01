package frc.robot.subsystems.elevator;

import frc.robot.constants.LaserCanConstants;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserCanSubsystem extends SubsystemBase {
    private final LaserCan laserCan;
    private LaserCan.Measurement lastMeasurement;
    
    // Handling the number of the LaserCanSubsystem
    private final int  number;
    private static int lastNumber = 0;

    public LaserCanSubsystem(int id) {
        this.laserCan = new LaserCan(id);
        this.number   = ++lastNumber;

        configureDevice();
    
    }

    private void configureDevice() {
        System.out.printf("[LaserCan %d] Configuring LaserCan...\n", this.number);
        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(LaserCanConstants.X,LaserCanConstants.Y,LaserCanConstants.WIDTH,LaserCanConstants.HEIGHT));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
            System.out.printf("[LaserCan %d] LaserCan configured successfully\n", this.number);
        } catch (ConfigurationFailedException e) {
            System.out.printf("[LaserCan %d] LaserCan configuration failed!\n" + e, this.number);
        }
    }

    @Override
    public void periodic() {
        lastMeasurement = laserCan.getMeasurement();
        updateDashboard();
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("[LASERCAN " + this.number + "] Distance (mm)", getDistance());
        SmartDashboard.putBoolean("[LASERCAN " + this.number + "] Valid Reading", isValidMeasurement());
    }

    public boolean isValidMeasurement() {
        return lastMeasurement != null &&
                lastMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }

    public double getDistance() {
        return isValidMeasurement() ? lastMeasurement.distance_mm : -1;
    }
}