package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TagTrackingHandler extends SubsystemBase {
    private TagTrackingTarget target;

    public TagTrackingHandler() {
        this.target = null;
    }

    public void setTarget(TagTrackingTarget target) {
        this.target = target;
    }

    @Override
    public void periodic() {
    }
}
