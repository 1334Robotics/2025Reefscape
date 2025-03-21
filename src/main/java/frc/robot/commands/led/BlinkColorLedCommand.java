package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LedSubsystem;

public class BlinkColorLedCommand extends Command {
    private final LedSubsystem ledSubsystem;
    private final LEDColorCommand.Color color;
    private final double blinkInterval;
    private double lastBlinkTime;
    private boolean isLit;

    /**
     * Creates a command to blink LEDs with specified color
     * @param ledSubsystem The LED subsystem
     * @param color The color to blink
     * @param blinkInterval Time in seconds between blinks
     */
    public BlinkColorLedCommand(LedSubsystem ledSubsystem, LEDColorCommand.Color color, double blinkInterval) {
        this.ledSubsystem = ledSubsystem;
        this.color = color;
        this.blinkInterval = blinkInterval;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        lastBlinkTime = System.currentTimeMillis() / 1000.0;
        isLit = false;
    }

    @Override
    public void execute() {
        double currentTime = System.currentTimeMillis() / 1000.0;
        if (currentTime - lastBlinkTime >= blinkInterval) {
            if (isLit) {
                ledSubsystem.turnOff();
                isLit = false;
            } else {
                ledSubsystem.setColor(color.r, color.g, color.b);
                isLit = true;
            }
            lastBlinkTime = currentTime;
        }
    }

    @Override
    public void end(boolean interrupted) {
        ledSubsystem.turnOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
