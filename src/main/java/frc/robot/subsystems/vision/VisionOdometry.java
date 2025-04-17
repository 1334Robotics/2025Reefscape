package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.VisionConstants;

public class VisionOdometry extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonCamera photonCamera;
    private double previousResultTimestamp;
    
    public VisionOdometry(PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
        this.field = new Field2d();

        // Get the AprilTag field layout
        AprilTagFieldLayout fieldLayout;
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(VisionConstants.APRILTAG_LAYOUT);
            Optional<Alliance> alliance = DriverStation.getAlliance();
            fieldLayout.setOrigin(alliance.get() == Alliance.Red
                                  ? OriginPosition.kRedAllianceWallRightSide
                                  : OriginPosition.kBlueAllianceWallRightSide);
        } catch(IOException e) {
            System.err.println("Failed to load AprilTagFieldLayout: " + e.toString());
            fieldLayout = null;
        }
        this.fieldLayout = fieldLayout;

        // Create the pose estimator
        this.poseEstimator = new SwerveDrivePoseEstimator(RobotContainer.swerveSubsystem.getSwerveDriveKinematics(),
                                                          RobotContainer.gyroSubsystem.getRotation(),
                                                          RobotContainer.swerveSubsystem.getSwerveModulePositions(),
                                                          new Pose2d(),
                                                          null,
                                                          null);
    }

    @Override
    public void periodic() {
        // Update the pose estimator with the best vision reading
        PhotonPipelineResult pipelineResult = this.photonCamera.getLatestResult();
        double resultTimestamp = pipelineResult.getTimestampSeconds();
        if(resultTimestamp != this.previousResultTimestamp && pipelineResult.hasTargets()) {
            this.previousResultTimestamp = resultTimestamp;

            PhotonTrackedTarget target = pipelineResult.getBestTarget();
            int targetId = target.getFiducialId();

            // Get the tag pose from the field layout
            Optional<Pose3d> tagPose = this.fieldLayout == null ? Optional.empty() : this.fieldLayout.getTagPose(targetId);
            if(target.getPoseAmbiguity() <= VisionConstants.MAX_AMBIGUITY && targetId >= 0 && tagPose.isPresent()) {
                Pose3d targetPose = tagPose.get();
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

                Pose3d visionMeasurement = camPose.transformBy(VisionConstants.CAMERA_POSITION);
                this.poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
            }
        }

        // Update the robot's pose with gyro and swerve readings
        this.poseEstimator.update(RobotContainer.gyroSubsystem.getRotation(),
                                  RobotContainer.swerveSubsystem.getSwerveModulePositions());
        this.field.setRobotPose(this.poseEstimator.getEstimatedPosition());
    }

    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d newPose) {
        this.poseEstimator.resetPosition(RobotContainer.gyroSubsystem.getRotation(),
                                         RobotContainer.swerveSubsystem.getSwerveModulePositions(),
                                         newPose);
    }

    public void resetPose() {
        this.field.setRobotPose(new Pose2d());
    }
}
