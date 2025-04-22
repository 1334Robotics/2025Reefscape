package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.LedSubsystem;

public class ToggleLedCommand extends InstantCommand {
    private final LedSubsystem ledSubsystem;


    public ToggleLedCommand(LedSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        // This runs once when the command is scheduled.
        ledSubsystem.toggle();
    }
}