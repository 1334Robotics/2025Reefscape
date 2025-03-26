package frc.robot.subsystems.vision;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.vision.Distance;
import frc.robot.constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineMetadata;

import static edu.wpi.first.units.Units.Centimeters;

import java.util.List;

/**
 * A simple vision subsystem using PhotonVision to detect and track targets.
 */
public class VisionSubsystem extends SubsystemBase {
    private double imageAge = 0;

    private final PhotonCamera leftCamera;
    private final PhotonCamera rightCamera;
    private PhotonPipelineMetadata cameraMetadata;

    // Store the latest result each loop
    private PhotonPipelineResult latestResult;


    public VisionSubsystem() {
        this.leftCamera  = new PhotonCamera(VisionConstants.LEFT_CAMERA_NAME);
        this.rightCamera = new PhotonCamera(VisionConstants.RIGHT_CAMERA_NAME);
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

    /**
     * Gets the current age of the image as calculated in VisionSubsystem.periodic()
     * @return The image age
     */
    public double getImageAge() {
        return this.imageAge;
    }

    public Distance getDistanceAway() {
        if(latestResult != null && latestResult.hasTargets()) {
            PhotonTrackedTarget target = latestResult.getBestTarget();
            return new Distance(target.getBestCameraToTarget().getMeasureY().in(Centimeters) - VisionConstants.CAMERA_POSITION_X,
                                target.getBestCameraToTarget().getMeasureX().in(Centimeters) - VisionConstants.CAMERA_POSITION_Y);
        }
        return null;
    }

    public double getTargetAngle() {
        if(latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getBestCameraToTarget().getRotation().getZ() * (180 / Math.PI);
        }
        // If the angle is 1000­°, there is a problem
        return 1000;
    }

    @Override
    public void periodic() {
        // 1) Grab all unread results once per loop
        List<PhotonPipelineResult> results = leftCamera.getAllUnreadResults();

        // 2) If we got anything new, take the last (most recent) one that contains a valid target
        if(!results.isEmpty()) {
            this.latestResult = results.get(results.size() - 1);

            /*this.latestResult = null;
            boolean found = false;
            int i;
            for(i=1;i<results.size()&&!found;i++) {
                this.latestResult = results.get(results.size() - i);
                if(this.latestResult != null) {
                    if(this.latestResult.hasTargets()) found = true;
                }
            }*/
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
        this.imageAge = (currentTimeSeconds - captureTimeSeconds) * 1000;

        // (4) Send to dashboard
        SmartDashboard.putNumber("[VISION] Image Age (ms)", this.imageAge);

        if (hasTarget) {
            PhotonTrackedTarget target = latestResult.getBestTarget();
            SmartDashboard.putNumber("[VISION] Target ID", target.getFiducialId());
            SmartDashboard.putNumber("[VISION] Yaw", target.getYaw());
            SmartDashboard.putNumber("[VISION] Pitch", target.getPitch());
            SmartDashboard.putNumber("[VISION] Area", target.getArea());
            SmartDashboard.putNumber("[VISION] Pose Ambiguity", target.getPoseAmbiguity());
            SmartDashboard.putNumber("[VISION] Skew", target.getSkew());
        } else {
            // Clear values when no target is visible
            SmartDashboard.putNumber("[VISION] Target ID", -1);
            SmartDashboard.putNumber("[VISION] Yaw", 0.0);
            SmartDashboard.putNumber("[VISION] Pitch", 0.0);
            SmartDashboard.putNumber("[VISION] Area", 0.0);
            SmartDashboard.putNumber("[VISION] Pose Ambiguity", 0.0);
            SmartDashboard.putNumber("[VISION] Skew", 0.0);
        }
    }
}