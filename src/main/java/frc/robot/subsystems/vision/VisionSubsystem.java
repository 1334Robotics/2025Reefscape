package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.constants.VisionConstants;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout tagLayout;
    private final PhotonCameraSim cameraSim;
    private final Field2d m_field;
    private Pose2d lastPose = new Pose2d();
    private boolean hasLoggedSimWarning = false;

    // Constants for SmartDashboard paths
    private static final String SD_PATH = "Vision/";
    private static final String TARGET_PATH = SD_PATH + "Target/";
    private static final String POSE_PATH = SD_PATH + "Pose/";
    private static final String METRICS_PATH = SD_PATH + "Metrics/";
    private static final String SIM_PATH = SD_PATH + "Simulation/";

    public VisionSubsystem() {
        System.out.println("\nInitializing VisionSubsystem");
        System.out.println("PhotonVision Camera Name: " + VisionConstants.CAMERA_NAME);

        // Check if camera exists in NetworkTables
        var instance = NetworkTableInstance.getDefault();
        var cameraTable = instance.getTable("photonvision").getSubTable(VisionConstants.CAMERA_NAME);

        if (!cameraTable.getKeys().contains("connected")) {
            System.err.println("ERROR: Camera not found in NetworkTables!");
            System.err.println("Available tables:");
            instance.getTable("photonvision").getSubTables().forEach(table ->
                System.err.println("- " + table));
            camera = null;
            poseEstimator = null;
            tagLayout = null;
            cameraSim = null;
            m_field = null;
            return;
        }

        // Initialize camera
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        System.out.println("Camera created: " + VisionConstants.CAMERA_NAME);

        // Wait for camera to initialize
        int attempts = 0;
        while (attempts < 10) {
            if (camera.isConnected()) {
                System.out.println("Camera connected successfully!");
                break;
            }
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
            attempts++;
        }

        if (!camera.isConnected()) {
            System.err.println("WARNING: Camera failed to connect after " + attempts + " attempts");
        }

        // Print camera configuration
        System.out.println("\nCamera Configuration:");
        System.out.println("Expected:");
        System.out.println("- Resolution: " + VisionConstants.CAMERA_RESOLUTION_WIDTH + "x" +
                          VisionConstants.CAMERA_RESOLUTION_HEIGHT);
        System.out.println("- FPS: " + VisionConstants.CAMERA_FPS);
        System.out.println("- FOV: " + VisionConstants.CAMERA_FOV_DEGREES + " degrees");

        System.out.println("\nActual (from NetworkTables):");
        var resolutionWidth = cameraTable.getEntry("resolution_width").getDouble(0);
        var resolutionHeight = cameraTable.getEntry("resolution_height").getDouble(0);
        var fps = cameraTable.getEntry("fps").getDouble(0);
        System.out.println("- Resolution: " + resolutionWidth + "x" + resolutionHeight);
        System.out.println("- FPS: " + fps);
        System.out.println("- Connected: " + cameraTable.getEntry("connected").getBoolean(false));

        // Initial connection test
        System.out.println("\nInitial camera connection test:");
        var results = camera.getAllUnreadResults();
        System.out.println("- Results empty? " + results.isEmpty());
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            System.out.println("- Has targets? " + result.hasTargets());
            if (result.hasTargets()) {
                System.out.println("- Number of targets: " + result.getTargets().size());
            }
        }

        try {
            // Load field layout
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            System.out.println("AprilTag layout loaded successfully");
            System.out.println("Number of tags in layout: " + tagLayout.getTags().size());

            // Initialize pose estimator
            poseEstimator = new PhotonPoseEstimator(
                tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.ROBOT_TO_CAMERA
            );
            System.out.println("Pose estimator initialized with strategy: " + PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

            // Initialize simulation
            System.out.println("Initializing vision simulation");
            m_field = new Field2d();
            cameraSim = new PhotonCameraSim(camera);

            System.out.println("Vision simulation initialized successfully");

            // Initialize SmartDashboard
            initializeSmartDashboard();
            System.out.println("SmartDashboard entries initialized under: " + SD_PATH);
        } catch (Exception e) {
            System.err.println("Error initializing vision subsystem: " + e.getMessage());
            e.printStackTrace();
            throw new RuntimeException("Failed to initialize vision subsystem", e);
        }
    }

    private void initializeSmartDashboard() {
        // Initialize target tracking entries
        SmartDashboard.putBoolean(TARGET_PATH + "HasTarget", false);
        SmartDashboard.putNumber(TARGET_PATH + "ID", -1);
        SmartDashboard.putNumber(TARGET_PATH + "Yaw", 0.0);
        SmartDashboard.putNumber(TARGET_PATH + "Pitch", 0.0);
        SmartDashboard.putNumber(TARGET_PATH + "Ambiguity", -1);

        // Initialize pose estimation entries
        SmartDashboard.putNumber(POSE_PATH + "X", 0.0);
        SmartDashboard.putNumber(POSE_PATH + "Y", 0.0);
        SmartDashboard.putNumber(POSE_PATH + "Rotation", 0.0);

        // Initialize metrics entries
        SmartDashboard.putNumber(METRICS_PATH + "LatencyMS", 0.0);

        // Initialize simulation entries if needed
        if (RobotBase.isSimulation()) {
            SmartDashboard.putNumber(SIM_PATH + "TargetCount", 0);
        }

        // Add Field2d widget
        SmartDashboard.putData(SD_PATH + "Field", m_field);
    }

    @Override
    public void periodic() {
        try {
            if (camera == null) {
                if (!hasLoggedSimWarning) {
                    System.err.println("ERROR: Camera not initialized in VisionSubsystem");
                    hasLoggedSimWarning = true;
                }
                return;
            }

            if (!camera.isConnected()) {
                System.err.println("WARNING: Camera disconnected");
                return;
            }

            PhotonPipelineResult result = null;
            try {
                // Get latest results safely
                var results = camera.getAllUnreadResults();
                if (!results.isEmpty()) {
                    result = results.get(results.size() - 1);
                }
            } catch (Exception e) {
                System.err.println("Error getting vision results: " + e.getMessage());
                e.printStackTrace();
                return;
            }

            if (result == null) {
                return;
            }

            try {
                // Debug logging
                System.out.println("\nVision Update:");
                System.out.println("- Has targets? " + result.hasTargets());
                if (result.hasTargets()) {
                    var targets = result.getTargets();
                    System.out.println("- Number of targets: " + targets.size());
                    for (var target : targets) {
                        System.out.println("  Target ID: " + target.getFiducialId() +
                                         " (Ambiguity: " + target.getPoseAmbiguity() + ")");
                        System.out.println("    Yaw: " + target.getYaw());
                        System.out.println("    Pitch: " + target.getPitch());
                        System.out.println("    Area: " + target.getArea());
                    }
                }

                // Update SmartDashboard
                boolean hasTarget = result.hasTargets();
                SmartDashboard.putBoolean(TARGET_PATH + "HasTarget", hasTarget);

                if (hasTarget) {
                    try {
                        var bestTarget = result.getBestTarget();
                        SmartDashboard.putNumber(TARGET_PATH + "ID", bestTarget.getFiducialId());
                        SmartDashboard.putNumber(TARGET_PATH + "Yaw", bestTarget.getYaw());
                        SmartDashboard.putNumber(TARGET_PATH + "Pitch", bestTarget.getPitch());
                        SmartDashboard.putNumber(TARGET_PATH + "Ambiguity", bestTarget.getPoseAmbiguity());

                        // Update pose estimation if we have a valid target
                        if (bestTarget.getPoseAmbiguity() < VisionConstants.MAX_AMBIGUITY) {
                            updatePoseEstimation(result);
                        }
                    } catch (Exception e) {
                        System.err.println("Error processing best target: " + e.getMessage());
                    }
                } else {
                    // Clear target data
                    SmartDashboard.putNumber(TARGET_PATH + "ID", -1);
                    SmartDashboard.putNumber(TARGET_PATH + "Yaw", 0.0);
                    SmartDashboard.putNumber(TARGET_PATH + "Pitch", 0.0);
                    SmartDashboard.putNumber(TARGET_PATH + "Ambiguity", -1);
                }
            } catch (Exception e) {
                System.err.println("Error in vision processing: " + e.getMessage());
                e.printStackTrace();
            }
        } catch (Exception e) {
            System.err.println("Error in VisionSubsystem periodic: " + e.toString());
            e.printStackTrace();
        }
    }

    private void updatePoseEstimation(PhotonPipelineResult result) {
        try {
            if (poseEstimator == null) {
                return;
            }

            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
            if (estimatedPose.isPresent()) {
                Pose2d currentPose = estimatedPose.get().estimatedPose.toPose2d();
                m_field.setRobotPose(currentPose);

                // Update pose data
                SmartDashboard.putNumber(POSE_PATH + "X", currentPose.getX());
                SmartDashboard.putNumber(POSE_PATH + "Y", currentPose.getY());
                SmartDashboard.putNumber(POSE_PATH + "Rotation", currentPose.getRotation().getDegrees());

                lastPose = currentPose;
            }
        } catch (Exception e) {
            System.err.println("Error updating pose estimation: " + e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void simulationPeriodic() {
        if (!RobotBase.isSimulation() || cameraSim == null) {
            return;
        }

        try {
            PhotonPipelineResult result = null;
            try {
                var results = camera.getAllUnreadResults();
                if (!results.isEmpty()) {
                    result = results.get(results.size() - 1);
                }
            } catch (Exception e) {
                System.err.println("Error getting vision results in simulation: " + e.getMessage());
                return;
            }

            int targetCount = 0;
            if (result != null && result.hasTargets()) {
                targetCount = result.getTargets().size();
            }

            SmartDashboard.putNumber(SIM_PATH + "TargetCount", targetCount);
        } catch (Exception e) {
            System.err.println("Error in vision simulation: " + e.getMessage());
            e.printStackTrace();
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        try {
            if (poseEstimator == null) {
                return Optional.empty();
            }

            PhotonPipelineResult result = null;
            try {
                var results = camera.getAllUnreadResults();
                if (!results.isEmpty()) {
                    result = results.get(results.size() - 1);
                }
            } catch (Exception e) {
                System.err.println("Error getting vision results for pose estimation: " + e.getMessage());
                return Optional.empty();
            }

            if (result == null || !result.hasTargets()) {
                return Optional.empty();
            }

            poseEstimator.setReferencePose(prevEstimatedRobotPose);
            return poseEstimator.update(result);
        } catch (Exception e) {
            System.err.println("Error getting estimated global pose: " + e.getMessage());
            e.printStackTrace();
            return Optional.empty();
        }
    }

    public void updateCameraPosition(Transform3d newTransform) {
        if (poseEstimator != null) {
            poseEstimator.setRobotToCameraTransform(newTransform);
        }
    }
}