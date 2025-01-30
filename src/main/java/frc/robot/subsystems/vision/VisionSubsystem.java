package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineMetadata;

import java.util.List;

/**
 * A simple vision subsystem using PhotonVision to detect and track targets.
 */
public class VisionSubsystem extends SubsystemBase {
    private static final String CAMERA_NAME = "Arducam_OV9782_USB_Camera"; 

    private final PhotonCamera cameraPhotonCamera;
    private PhotonPipelineMetadata cameraMetadata;

    // Store the latest result each loop
    private PhotonPipelineResult latestResult;

    public VisionSubsystem() {
        cameraPhotonCamera = new PhotonCamera(CAMERA_NAME);
    }

    /**
     * Checks if any valid target is visible.
     * @return true if a target is visible; false otherwise.
     */
    public boolean isTargetVisible() {
        return (latestResult != null) && latestResult.hasTargets();
    }

    /**
     * Gets the yaw angle to the best visible target.
     * @return yaw in degrees; 0.0 if no target detected.
     */
    public double getTargetYaw() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getYaw();
        }
        return 0.0;
    }

    /**
     * Gets the pitch angle to the best visible target.
     * @return pitch in degrees; 0.0 if no target detected.
     */
    public double getTargetPitch() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getPitch();
        }
        return 0.0;
    }

    /**
     * Gets the Pose Ambiguity to the best visible target.
     * @return pose ambiguity; 0.0 if no target detected.
     */
    public double getTargetPoseAmbiguity() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getPoseAmbiguity();
        }
        return 0.0;
    }

    /**
     * Gets the area of the best visible target.
     * @return area; 0.0 if no target detected.
     */
    public double getTargetArea() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getArea();
        }
        return 0.0;
    }

    /**
     * Gets the ID of the best visible AprilTag target.
     * @return target ID; -1 if no target detected.
     */
    public int getTargetId() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getFiducialId();
        }
        return -1;
    }

    /**
     * Gets the skew of the best visible target.
     * @return skew; 0.0 if no target detected.
     */
    public double getTargetSkew() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getSkew();
        }
        return 0.0;
    }

    /**
     * Gets the best visible target.
     * @return best target; null if no target detected.
     */
    public PhotonTrackedTarget getTarget() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget();
        }
        return null;
    }
//NEW CAL
    /**
     * Get ID of best visible target
     * @return target ID or -1 if no target
     */
    public int getBestTargetId() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getFiducialId();
        }
        return -1;
    }
//NEW CAL
    /**
     * Check if specific tag ID is visible
     * @param targetId desired tag ID
     * @return true if tag is visible
     */
    public boolean isSpecificTargetVisible(int targetId) {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getFiducialId() == targetId;
        }
        return false;
    }

    @Override
    public void periodic() {
        // Get the latest pipeline result
        latestResult = cameraPhotonCamera.getLatestResult();
        
        // Debug output
        SmartDashboard.putBoolean("Vision/HasResult", latestResult != null);
        if (latestResult != null) {
            SmartDashboard.putBoolean("Vision/HasTargets", latestResult.hasTargets());
            if (latestResult.hasTargets()) {
                var target = latestResult.getBestTarget();
                SmartDashboard.putNumber("Vision/TargetID", target.getFiducialId());
                SmartDashboard.putNumber("Vision/TimestampSeconds", latestResult.getTimestampSeconds());
            }
        }
    }
}
