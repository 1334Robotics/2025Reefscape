package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.led.LEDColorCommand;
import edu.wpi.first.wpilibj.Timer;

public class LedHandler extends SubsystemBase {
    private final LedSubsystem ledSubsystem;
    private long controller = 0;
    private boolean isBlinking = false;
    private boolean blinkState = false;
    private double lastBlinkTime = 0;
    private static final double BLINK_INTERVAL = 0.5; // seconds

    public enum Controller {
        MAILBOX(1),
        ELEVATOR(2),  // Higher priority for elevator status
        TARGET_HEIGHT(3);  // Highest priority for target height reached

        public final int priority;
        private Controller(int priority) {
            this.priority = priority;
        }
    }

    public LedHandler(LedSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.requestControl(Controller.MAILBOX); // Default control
    }

    public void setColor(Controller controller, LEDColorCommand.Color color) {
        if (!hasControl(controller)) return;
        ledSubsystem.setColor(color.r, color.g, color.b);
    }

    public void setHeightReachedStatus(boolean atTargetHeight) {
        if (atTargetHeight) {
            requestControl(Controller.TARGET_HEIGHT);
            isBlinking = true;
        } else {
            if (hasControl(Controller.TARGET_HEIGHT)) {
                relinquishControl(Controller.TARGET_HEIGHT);
                isBlinking = false;
            }
        }
    }

    private boolean hasControl(Controller controller) {
        return (1L << controller.priority) == Long.highestOneBit(this.controller);
    }

    public boolean requestControl(Controller controller) {
        this.controller |= (1L << controller.priority);
        long highestPriorityBit = Long.highestOneBit(this.controller);
        return highestPriorityBit == (1L << controller.priority);
    }

    public void relinquishControl(Controller controller) {
        this.controller &= ~(1L << controller.priority);
        if(this.controller != 0) this.controller = Long.highestOneBit(this.controller);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[LED] Controller", controller);
        SmartDashboard.putNumber("[LED] Active Controller", Long.highestOneBit(this.controller));

        if (isBlinking && hasControl(Controller.TARGET_HEIGHT)) {
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
                blinkState = !blinkState;
                lastBlinkTime = currentTime;
                if (blinkState) {
                    ledSubsystem.setColor(LEDColorCommand.Color.GREEN.r, 
                                        LEDColorCommand.Color.GREEN.g, 
                                        LEDColorCommand.Color.GREEN.b);
                } else {
                    ledSubsystem.setColor(0, 0, 0); // Off
                }
            }
        }
    }
}
