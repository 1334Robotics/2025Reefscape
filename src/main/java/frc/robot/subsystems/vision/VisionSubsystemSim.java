package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.constants.VisionConstants;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionSubsystemSim extends SubsystemBase implements VisionSubsystemBase{
    private final VisionSystemSim visionSystemSim;
    private final PhotonCameraSim cameraSim;
    private final Supplier<Pose2d> robotPoseSupplier; // Store the supplier
    private final AprilTagFieldLayout fieldLayout;
    private PhotonPipelineResult latestResult;

    public VisionSubsystemSim(Supplier<Pose2d> robotPoseSupplier, AprilTagFieldLayout fieldLayout) {
        this.robotPoseSupplier = robotPoseSupplier; // Initialize the supplier
        this.fieldLayout = fieldLayout;

        // Create a real PhotonCamera object
        PhotonCamera camera = new PhotonCamera("Arducam_OV9782_USB_Camera");

        // Create a simulated camera
        visionSystemSim = new VisionSystemSim("main");
        cameraSim = new PhotonCameraSim(camera);

        // Configure the camera simulation
        cameraSim.enableDrawWireframe(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableRawStream(true);

        // Add the camera to the vision system
        visionSystemSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAMERA);

        // Add AprilTags to the vision system
        visionSystemSim.addAprilTags(fieldLayout);

        // Initialize the latest result
        latestResult = new PhotonPipelineResult();
    }

    @Override
    public void periodic() {
        // Update the vision system with the robot's current pose
        visionSystemSim.update(robotPoseSupplier.get());

        // Simulate the camera output
        List<PhotonPipelineResult> results = cameraSim.getCamera().getAllUnreadResults();
        if (!results.isEmpty()) {
            latestResult = results.get(results.size() - 1);
        } else {
            latestResult = null; // Clear the result if no new data is available
        }

        if (latestResult != null && latestResult.hasTargets()) {
            PhotonTrackedTarget target = latestResult.getBestTarget();
            double simulatedYaw = simulateNoise(target.getYaw());
            double simulatedDistance = simulateNoise(target.getBestCameraToTarget().getTranslation().getNorm());
            Logger.recordOutput("Vision/SimulatedYaw", simulatedYaw);
            Logger.recordOutput("Vision/SimulatedDistance", simulatedDistance);
        }

        // Log visible target poses
        List<Pose2d> visibleTargetPoses = new ArrayList<>();
        if (latestResult != null && latestResult.hasTargets()) {
            for (PhotonTrackedTarget target : latestResult.getTargets()) {
                Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    visibleTargetPoses.add(tagPose.get().toPose2d());
                }
            }
        }

        Logger.recordOutput("Vision/VisibleTargetPoses", visibleTargetPoses.toArray(new Pose2d[0]));

        // Log valid pose estimations
        if (latestResult != null && latestResult.hasTargets()) {
            List<Pose3d> validPoseEstimations = new ArrayList<>();
            for (PhotonTrackedTarget target : latestResult.getTargets()) {
                Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    // Calculate the robot's pose based on the tag's pose and the camera's transform
                    Pose3d robotPose = tagPose.get().transformBy(VisionConstants.ROBOT_TO_CAMERA.inverse());
                    validPoseEstimations.add(robotPose);
                }
            }
            Logger.recordOutput("Vision/ValidPoseEstimations", validPoseEstimations.toArray(new Pose3d[0]));
        }

        Optional<Pose2d> robotPose = getRobotPose();
            if (robotPose.isPresent()) {
                Logger.recordOutput("Vision/RobotPose", robotPose.get());
            } else {
                Logger.recordOutput("Vision/RobotPose", new Pose2d());
            }

        // Log AprilTag detection lines
        if (latestResult != null && latestResult.hasTargets()) {
            List<Translation3d[]> detectionLines = new ArrayList<>();
            Pose3d robotPose3d = new Pose3d(robotPoseSupplier.get());
            for (PhotonTrackedTarget target : latestResult.getTargets()) {
                Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    // Create a line from the robot to the tag
                    Translation3d robotPosition = robotPose3d.getTranslation();
                    Translation3d tagPosition = tagPose.get().getTranslation();
                    detectionLines.add(new Translation3d[]{robotPosition, tagPosition});
                }
            }
            Logger.recordOutput("Vision/DetectionLines", detectionLines.toArray(new Translation3d[0][]));
        }

        // Log camera position
        Logger.recordOutput("Camera/FOV", VisionConstants.CAMERA_FOV_DEGREES);
        Logger.recordOutput("Camera/RobotToCamera", VisionConstants.ROBOT_TO_CAMERA);

        // Update the dashboard
        boolean hasTarget = latestResult.hasTargets();
        SmartDashboard.putBoolean("[VISION] Has Target", hasTarget);

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

    public Optional<Pose2d> getRobotPose() {
        if (latestResult != null && latestResult.hasTargets()) {
            PhotonTrackedTarget target = latestResult.getBestTarget();
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                // Calculate the robot's pose based on the tag's pose and the camera's transform
                Pose3d robotPose = tagPose.get().transformBy(VisionConstants.ROBOT_TO_CAMERA.inverse());
                return Optional.of(robotPose.toPose2d());
            }
        }
        return Optional.empty();
    }

    public boolean isTargetVisible() {
        return latestResult != null && latestResult.hasTargets();
    }

    public double getTargetYaw() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getYaw();
        }
        return 0.0; // Return 0.0 if no target is detected
    }

    public double getTargetPitch() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getPitch();
        }
        return 0.0; // Return 0.0 if no target is detected
    }

    public double getTargetPoseAmbiguity() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getPoseAmbiguity();
        }
        return 0.0; // Return 0.0 if no target is detected
    }

    public double getTargetArea() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getArea();
        }
        return 0.0; // Return 0.0 if no target is detected
    }

    public int getTargetId() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getFiducialId();
        }
        return -1; // Return -1 if no target is detected
    }

    public double getTargetSkew() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget().getSkew();
        }
        return 0.0; // Return 0.0 if no target is detected
    }

    public PhotonTrackedTarget getTarget() {
        if (latestResult != null && latestResult.hasTargets()) {
            return latestResult.getBestTarget();
        }
        return null; // Return null if no target is detected
    }

    private double simulateLatency() {
        return 0.1 + (Math.random() * 0.05); // Simulate 100ms ± 50ms latency
    }
    
    private double simulateNoise(double value) {
        return value + (Math.random() - 0.5) * 0.1; // Add ±5% noise
    }
}