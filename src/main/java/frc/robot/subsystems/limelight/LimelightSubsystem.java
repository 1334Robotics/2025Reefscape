package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tx; // Horizontal offset
    private final NetworkTableEntry ty; // Vertical offset
    private final NetworkTableEntry ta; // Target area
    private final NetworkTableEntry tv; // Target visibility

    public LimelightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        tv = limelightTable.getEntry("tv");
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0)==1;
    }

    public double getTargetYaw() {
        // return tx.getDouble(0.0);
        return getBotPose()[5];
    }

    public double getTargetPitch() {
        return ty.getDouble(0.0);
    }

    public double[] getBotPose() {
        return limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
    }

    public double getX() {
        // return limelightTable.getEntry("x").getDouble(0.0);
        return getBotPose()[0]; 
    }

    public double getY() {
        // return limelightTable.getEntry("y").getDouble(0.0);
        return getBotPose()[1];
    }

    // public double getRotation() {
    //     return limelightTable.getEntry("rotation").getDouble(0.0);
    // }

    public double getHorizontalOffset() {
        return tx.getDouble(0.0);
    }

    public double getVerticalOffset() {
        return ty.getDouble(0.0);
    }

    public double getTargetArea() {
        return ta.getDouble(0.0);
    }

    public boolean isTargetVisible() {
        return tv.getDouble(0.0) == 1.0;
    }

    public void setPipeline(int pipelineIndex) {
        NetworkTableEntry pipelineEntry = limelightTable.getEntry("pipeline");
        pipelineEntry.setNumber(pipelineIndex);
    }

    public void setLEDMode(int mode) {
        NetworkTableEntry ledModeEntry = limelightTable.getEntry("ledMode");
        ledModeEntry.setNumber(mode);
    }

    public Pose2d getLimelightFieldPose(){
        double[] botPose = getBotPose();
        if (botPose.length < 6) {
            return new Pose2d();
        }

        return new Pose2d(
            botPose[0], // X position
            botPose[1], // Y position
            Rotation2d.fromDegrees(botPose[5]) 
        );
    }
    
}
