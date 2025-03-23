package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.led.LEDColourCommand;
import frc.robot.RobotContainer;

public class LedHandler extends SubsystemBase {
    private final LedSubsystem ledSubsystem;
    private long controller = 0;

    public enum Controller {
        VISION(1),
        VISIONTRACK(2);

        public final int priority;
        private Controller(int priority) {
            this.priority = priority;
        }
    }

    public LedHandler(LedSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.requestControl(Controller.VISION); // Default control
    }

    public void setColour(Controller controller, LEDColourCommand.Colour colour) {
        if (!hasControl(controller)) return;
        ledSubsystem.setColour(colour.r, colour.g, colour.b);
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

        // Check vision system for tag visibility
        if (RobotContainer.visionSubsystem.isTargetVisible()) {
            // If we're aligning, show green
            if (RobotContainer.trackCommand.isAligned()) {
                setColour(Controller.VISIONTRACK, LEDColourCommand.Colour.GREEN);
            } else {
                // If we see tag but not aligned, show blue
                setColour(Controller.VISIONTRACK, LEDColourCommand.Colour.BLUE);
            }
        } else {
            // No tag visible, show red
            setColour(Controller.VISION, LEDColourCommand.Colour.RED);
        }
    }
}
