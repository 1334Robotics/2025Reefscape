package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ControllerConstants;

public class ControllerSubsystem extends SubsystemBase {
    private final XboxController controller;
    private long vibrationStartTime;

    public ControllerSubsystem(XboxController controller) {
        this.controller = controller;
    }

    private static RumbleType getRumbleType(ControllerVibration vibration) {
        return switch(vibration) {
            case LIGHT -> RumbleType.kLeftRumble;
            case HEAVY -> RumbleType.kRightRumble;
        };
    }

    public void stopVibration() {
        this.controller.setRumble(RumbleType.kLeftRumble, 0);
        this.controller.setRumble(RumbleType.kRightRumble, 0);
    }

    public void vibrate(ControllerVibration vibration) {
        this.controller.setRumble(getRumbleType(vibration), ControllerConstants.VIBRATION_STRENGTH);
        this.vibrationStartTime = (long)edu.wpi.first.wpilibj.Timer.getFPGATimestamp() * 1000;
    }

    @Override
    public void periodic() {
        long currentTime = (long)edu.wpi.first.wpilibj.Timer.getFPGATimestamp() * 1000;
        if(currentTime - this.vibrationStartTime > ControllerConstants.VIBRATION_TIME)
            this.stopVibration();
    }
}