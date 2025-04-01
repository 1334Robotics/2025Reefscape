package frc.robot.subsystems.vision;

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
    private double smoothedAngle = 0;
    private int stableFrames = 0;
    private PhotonTrackedTarget lastStableTarget = null;

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
        if (latestResult == null || !latestResult.hasTargets()) return false;
        
        PhotonTrackedTarget target = latestResult.getBestTarget();
        return target.getArea() >= VisionConstants.MIN_TARGET_AREA &&
               target.getPoseAmbiguity() <= VisionConstants.MAX_AMBIGUITY;
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
        if (latestResult != null && latestResult.hasTargets()) {
            double rawAngle = latestResult.getBestTarget().getBestCameraToTarget().getRotation().getZ() * (180 / Math.PI);
            smoothedAngle = (1 - VisionConstants.SMOOTHING_FACTOR) * smoothedAngle + 
                           VisionConstants.SMOOTHING_FACTOR * rawAngle;
            return smoothedAngle;
        }
        // Return 1000° for error case to maintain compatibility with TrackAprilTagCommand
        smoothedAngle = 1000;
        return 1000;
    }

    private double calculateTargetQuality(PhotonTrackedTarget target) {
        // Combine multiple factors into a single quality score
        double areaScore = target.getArea() / 100.0;  // Normalize area to 0-1
        double ambiguityScore = 1.0 - target.getPoseAmbiguity();  // Invert ambiguity
        double quality = (areaScore * 0.7 + ambiguityScore * 0.3);  // Weighted combination
        
        // Return 0 if quality is below threshold
        return quality >= VisionConstants.MIN_TARGET_QUALITY ? quality : 0;
    }

    @Override
    public void periodic() {
        // Get results from both cameras
        List<PhotonPipelineResult> leftResults = leftCamera.getAllUnreadResults();
        List<PhotonPipelineResult> rightResults = rightCamera.getAllUnreadResults();
        
        // Select the best result between both cameras
        PhotonPipelineResult bestResult = null;
        double bestQuality = 0;
        String selectedCamera = "None";
        
        // Check left camera results
        if (!leftResults.isEmpty()) {
            PhotonPipelineResult result = leftResults.get(leftResults.size() - 1);
            if (result.hasTargets()) {
                double quality = calculateTargetQuality(result.getBestTarget());
                if (quality > bestQuality) {
                    bestQuality = quality;
                    bestResult = result;
                    selectedCamera = "Left";
                }
            }
        }
        
        // Check right camera results
        if (!rightResults.isEmpty()) {
            PhotonPipelineResult result = rightResults.get(rightResults.size() - 1);
            if (result.hasTargets()) {
                double quality = calculateTargetQuality(result.getBestTarget());
                if (quality > bestQuality) {
                    bestQuality = quality;
                    bestResult = result;
                    selectedCamera = "Right";
                }
            }
        }

        // Check target stability before using it
        if (bestResult != null && bestResult.hasTargets()) {
            PhotonTrackedTarget currentTarget = bestResult.getBestTarget();
            
            if (lastStableTarget != null && 
                currentTarget.getFiducialId() == lastStableTarget.getFiducialId() &&
                Math.abs(currentTarget.getYaw() - lastStableTarget.getYaw()) < 5.0) {
                stableFrames++;
            } else {
                stableFrames = 0;
                lastStableTarget = currentTarget;
            }
            
            if (stableFrames < 3) {  // Require 3 stable frames
                bestResult = null;
            }
        }

        // Update latest result and dashboard
        this.latestResult = bestResult;
        boolean hasTarget = (latestResult != null) && latestResult.hasTargets();
        SmartDashboard.putBoolean("[VISION] Has Target", hasTarget);
        SmartDashboard.putString("[VISION] Selected Camera", selectedCamera);
        SmartDashboard.putNumber("[VISION] Target Quality", bestQuality);
        SmartDashboard.putNumber("[VISION] Stable Frames", stableFrames);

        // Show pipeline latency
        double currentTimeSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double captureTimeSeconds = (latestResult != null ? latestResult.getTimestampSeconds() : currentTimeSeconds);
        this.imageAge = (currentTimeSeconds - captureTimeSeconds) * 1000;
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