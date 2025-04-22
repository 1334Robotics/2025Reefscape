package frc.robot.subsystems.mailbox;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserCanSubsystem extends SubsystemBase {
    private final LaserCan laserCan;
    private LaserCan.Measurement lastMeasurement;
    private double validRange;
    
    // Handling the number of the LaserCanSubsystems
    private final int  number;
    private static int lastNumber = 0;

    public LaserCanSubsystem(int id) {
        this.laserCan = new LaserCan(id);
        this.number   = ++lastNumber;

        this.validRange = 0;
    }

    public LaserCanSubsystem(int id, double validRange) {
        this.laserCan = new LaserCan(id);
        this.number   = ++lastNumber;

        this.validRange = validRange;
    }

    public void configureDevice(Rectangle regionOfInterest) {
        System.out.printf("[LaserCan %d] Configuring LaserCan...\n", this.number);
        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(regionOfInterest.x,
                                                                       regionOfInterest.y,
                                                                       regionOfInterest.width,
                                                                       regionOfInterest.height));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
            System.out.printf("[LaserCan %d] LaserCan configured successfully\n", this.number);
        } catch(ConfigurationFailedException e) {
            System.out.printf("[LaserCan %d] LaserCan configuration failed: %s!\n", this.number, e.toString());
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

    public void setValidRange(double validRange) {
        this.validRange = validRange;
    }

    public boolean inRange() {
        if(this.validRange == 0) return false;
        return (this.isValidMeasurement() && this.getDistance() <= this.validRange);
    }
}