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
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.constants.VisionConstants;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout tagLayout;
    private final PhotonPoseEstimator poseEstimator;
    private final Field2d m_field = new Field2d();
    private final NetworkTable visionTable;
    
    // Mutable state
    private Pose2d lastPose = new Pose2d();
    private boolean hasLoggedSimWarning = false;
    
    // Simulation-specific components
    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;

    public VisionSubsystem() {
        // Initialize NetworkTables
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        
        // Initialize vision table entries
        initializeNetworkTableEntries();
        
        System.out.println("Initializing VisionSubsystem");
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        System.out.println("Camera created: " + VisionConstants.CAMERA_NAME);

        try {
            // Load field layout
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            System.out.println("AprilTag layout loaded successfully");
            
            // Initialize pose estimator
            poseEstimator = new PhotonPoseEstimator(
                tagLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.ROBOT_TO_CAMERA
            );

            // Initialize simulation if needed
            if (RobotBase.isSimulation()) {
                System.out.println("Initializing vision simulation");
                var simComponents = initializeSimulation();
                visionSim = simComponents.visionSim;
                cameraSim = simComponents.cameraSim;
            } else {
                visionSim = null;
                cameraSim = null;
            }
        } catch (Exception e) {
            System.err.println("Error initializing vision subsystem: " + e.getMessage());
            e.printStackTrace();
            throw new RuntimeException("Failed to load AprilTag layout: " + e.getMessage());
        }
    }

    private void initializeNetworkTableEntries() {
        // Create subtables for better organization
        NetworkTable targetTable = visionTable.getSubTable("Target");
        NetworkTable poseTable = visionTable.getSubTable("Pose");
        NetworkTable simTable = visionTable.getSubTable("Simulation");
        
        // Initialize target tracking entries
        targetTable.getEntry("HasTarget").setBoolean(false);
        targetTable.getEntry("ID").setDouble(-1);
        targetTable.getEntry("Yaw").setDouble(0.0);
        targetTable.getEntry("Pitch").setDouble(0.0);
        targetTable.getEntry("Ambiguity").setDouble(-1);
        
        // Initialize pose estimation entries
        poseTable.getEntry("X").setDouble(0.0);
        poseTable.getEntry("Y").setDouble(0.0);
        poseTable.getEntry("Rotation").setDouble(0.0);
        
        // Initialize simulation entries
        simTable.getEntry("TargetCount").setDouble(0);
        
        // Add Field2d widget
        SmartDashboard.putData("Vision/Field", m_field);
    }

    private static class SimComponents {
        final VisionSystemSim visionSim;
        final PhotonCameraSim cameraSim;

        SimComponents(VisionSystemSim visionSim, PhotonCameraSim cameraSim) {
            this.visionSim = visionSim;
            this.cameraSim = cameraSim;
        }
    }

    private SimComponents initializeSimulation() {
        var visionSim = new VisionSystemSim("main");
        
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(
            VisionConstants.CAMERA_RESOLUTION_WIDTH,
            VisionConstants.CAMERA_RESOLUTION_HEIGHT,
            Rotation2d.fromDegrees(VisionConstants.CAMERA_FOV_DEGREES)
        );
        cameraProp.setCalibError(
            VisionConstants.SIM_ERROR_MEAN,
            VisionConstants.SIM_ERROR_STD_DEV
        );
        cameraProp.setFPS(VisionConstants.SIM_CAMERA_FPS);
        cameraProp.setAvgLatencyMs(VisionConstants.SIM_AVERAGE_LATENCY_MS);
        cameraProp.setLatencyStdDevMs(VisionConstants.SIM_LATENCY_STD_DEV_MS);

        var cameraSim = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAMERA);
        visionSim.addAprilTags(tagLayout);
        
        System.out.println("Vision simulation initialized successfully");
        return new SimComponents(visionSim, cameraSim);
    }

    @Override
    public void periodic() {
        try {
            if (camera == null) {
                if (!hasLoggedSimWarning) {
                    System.out.println("Camera not initialized in VisionSubsystem");
                    hasLoggedSimWarning = true;
                }
                return;
            }

            NetworkTable targetTable = visionTable.getSubTable("Target");
            var result = camera.getLatestResult();
            boolean hasTarget = result != null && result.hasTargets();
            targetTable.getEntry("HasTarget").setBoolean(hasTarget);
            
            if (hasTarget) {
                PhotonTrackedTarget target = result.getBestTarget();
                if (target != null) {
                    double ambiguity = target.getPoseAmbiguity();
                    
                    if (ambiguity < VisionConstants.MAX_AMBIGUITY) {
                        targetTable.getEntry("ID").setDouble(target.getFiducialId());
                        targetTable.getEntry("Yaw").setDouble(target.getYaw());
                        targetTable.getEntry("Pitch").setDouble(target.getPitch());
                        targetTable.getEntry("Ambiguity").setDouble(ambiguity);
                        
                        updatePoseEstimation();
                    }
                }
            } else {
                // Clear target data when no target is visible
                targetTable.getEntry("ID").setDouble(-1);
                targetTable.getEntry("Yaw").setDouble(0.0);
                targetTable.getEntry("Pitch").setDouble(0.0);
                targetTable.getEntry("Ambiguity").setDouble(-1);
            }
        } catch (Exception e) {
            System.err.println("Error in VisionSubsystem periodic: " + e.toString());
            e.printStackTrace();
        }
    }

    private void updatePoseEstimation() {
        try {
            if (poseEstimator == null) {
                return;
            }
            Pose2d currentPose = getCurrentPose();
            m_field.setRobotPose(currentPose);
            
            // Update pose data in NetworkTables
            NetworkTable poseTable = visionTable.getSubTable("Pose");
            poseTable.getEntry("X").setDouble(currentPose.getX());
            poseTable.getEntry("Y").setDouble(currentPose.getY());
            poseTable.getEntry("Rotation").setDouble(currentPose.getRotation().getDegrees());
            
            lastPose = currentPose;
        } catch (Exception e) {
            System.err.println("Error updating pose estimation: " + e.toString());
            e.printStackTrace();
        }
    }

    private Pose2d getCurrentPose() {
        try {
            Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(lastPose);
            return result.map(pose -> pose.estimatedPose.toPose2d()).orElse(lastPose);
        } catch (Exception e) {
            System.err.println("Error getting current pose: " + e.toString());
            e.printStackTrace();
            return lastPose;
        }
    }

    @Override
    public void simulationPeriodic() {
        if (!RobotBase.isSimulation() || visionSim == null || cameraSim == null) {
            return;
        }

        try {
            visionSim.update(lastPose);
            
            var result = camera.getLatestResult();
            NetworkTable simTable = visionTable.getSubTable("Simulation");
            simTable.getEntry("TargetCount").setDouble(
                result != null && result.hasTargets() ? result.getTargets().size() : 0
            );
        } catch (Exception e) {
            System.err.println("Error in vision simulation: " + e.toString());
            e.printStackTrace();
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        try {
            if (poseEstimator == null || camera == null) {
                return Optional.empty();
            }
            poseEstimator.setReferencePose(prevEstimatedRobotPose);
            var result = camera.getLatestResult();
            if (result == null) {
                return Optional.empty();
            }
            return poseEstimator.update(result);
        } catch (Exception e) {
            System.err.println("Error getting estimated global pose: " + e.toString());
            e.printStackTrace();
            return Optional.empty();
        }
    }

    public void updateCameraPosition(Transform3d newTransform) {
        if (poseEstimator != null) {
            poseEstimator.setRobotToCameraTransform(newTransform);
        }
        if (RobotBase.isSimulation() && visionSim != null) {
            visionSim.adjustCamera(cameraSim, newTransform);
        }
    }

    public boolean isConnected() {
        return camera.isConnected();
    }

    public double getLatencyMillis() {
        var result = camera.getLatestResult();
        if (result == null) {
            return 0.0;
        }
        var currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        return (currentTime - result.getTimestampSeconds()) * 1000.0;
    }
}