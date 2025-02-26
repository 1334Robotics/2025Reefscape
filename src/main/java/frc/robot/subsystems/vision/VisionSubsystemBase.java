package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface VisionSubsystemBase extends Subsystem{
    boolean isTargetVisible();
    double getTargetYaw();
    double getTargetPitch();
    double getTargetPoseAmbiguity();
    double getTargetArea();
    int getTargetId();
    double getTargetSkew();
    PhotonTrackedTarget getTarget();
}