package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LedSubsystem;

public class LEDColorCommand extends Command {
    private final LedSubsystem ledSubsystem;
    private final Color color;

    public enum Color {
        RED(255, 0, 0),
        GREEN(0, 255, 0),
        BLUE(0, 0, 255),
        YELLOW(255, 255, 0),
        ORANGE(255, 165, 0),
        PURPLE(128, 0, 128),
        WHITE(255, 255, 255),
        OFF(0, 0, 0);

        public final int r;
        public final int g; 
        public final int b;

        Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public LEDColorCommand(LedSubsystem ledSubsystem, Color color) {
        this.ledSubsystem = ledSubsystem;
        this.color = color;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.setColor(color.r, color.g, color.b);
    }

    @Override
    public void execute() {
        ledSubsystem.setColor(color.r, color.g, color.b);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        ledSubsystem.turnOff();
    }
}