package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.LedSubsystem;

public class TurnOffLedCommand extends InstantCommand {
    private final LedSubsystem ledSubsystem;

    public TurnOffLedCommand(LedSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.turnOff();
    }
}