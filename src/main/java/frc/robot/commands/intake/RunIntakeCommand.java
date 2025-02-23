package frc.robot.commands.intake;

import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RunIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public RunIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setRunning(true);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRunning(false);
    }
}