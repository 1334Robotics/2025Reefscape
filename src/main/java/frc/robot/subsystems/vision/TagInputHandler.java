package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class TagInputHandler extends SubsystemBase {
    private int chosenTag;

    public TagInputHandler() {
        this.chosenTag = 0;
        SmartDashboard.putNumber("[VISION] Chosen Tag", 0);
    }

    @Override
    public void periodic() {
        int gotTag = (int)SmartDashboard.getNumber("[VISION] Chosen Tag", 0);
        if(gotTag != chosenTag && gotTag != 0) {
            chosenTag = gotTag;
            // RobotContainer.trackCommand.setTargetTag(chosenTag);
        }
    }
}
