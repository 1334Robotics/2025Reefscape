package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.littletonrobotics.junction.Logger;

import java.util.List;

/**
 * A simple vision subsystem using PhotonVision to detect and track targets.
 */
public class VisionSubsystem extends SubsystemBase implements VisionSubsystemBase{
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

    @Override
    public void periodic() {
        // 1) Grab all unread results once per loop
        List<PhotonPipelineResult> results = cameraPhotonCamera.getAllUnreadResults();

        // 2) If we got anything new, take the last (most recent) one
        if (!results.isEmpty()) {
            latestResult = results.get(results.size() - 1);
        }

        // 3) Update the dashboard
        boolean hasTarget = (latestResult != null) && latestResult.hasTargets();
        SmartDashboard.putBoolean("[VISION] Has Target", hasTarget);

        // Show pipeline latency
        // this compiles but always returns -1.0
        //*double latencyMs = -1.0;
        //if (cameraMetadata != null && latestResult != null) {
        //    latencyMs = cameraMetadata.getLatencyMillis();
        //}
        // (1) Robot's current time in seconds
        double currentTimeSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // (2) The camera capture time (in seconds). 
        //     Check for a method like getTimestampSeconds(), getFrameTimestampSeconds(), etc.
        //double captureTimeSeconds = latestResult.getTimestampSeconds(); 
        double captureTimeSeconds = (latestResult != null ? latestResult.getTimestampSeconds() : currentTimeSeconds);
        // (3) Convert to milliseconds
        double imageAgeMs = (currentTimeSeconds - captureTimeSeconds) * 1000;

        // (4) Send to dashboard
        SmartDashboard.putNumber("[VISION] Image Age (ms)", imageAgeMs);

        if (latestResult != null && latestResult.hasTargets()) {
            PhotonTrackedTarget target = latestResult.getBestTarget();
            Logger.recordOutput("Vision/TargetYaw", target.getYaw());
            Logger.recordOutput("Vision/TargetPitch", target.getPitch());
            Logger.recordOutput("Vision/TargetArea", target.getArea());
            Logger.recordOutput("Vision/TargetID", target.getFiducialId());
            Logger.recordOutput("Vision/TargetDistance", target.getBestCameraToTarget().getTranslation().getNorm());
            Logger.recordOutput("Vision/PoseAmbiguity", target.getPoseAmbiguity());
        } else {
            Logger.recordOutput("Vision/TargetYaw", 0.0);
            Logger.recordOutput("Vision/TargetPitch", 0.0);
            Logger.recordOutput("Vision/TargetArea", 0.0);
            Logger.recordOutput("Vision/TargetID", -1);
            Logger.recordOutput("Vision/TargetDistance", 0.0);
            Logger.recordOutput("Vision/PoseAmbiguity", 0.0);
        }
    }
}