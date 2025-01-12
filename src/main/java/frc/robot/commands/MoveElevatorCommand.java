package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final double m_targetHeight;

    public MoveElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetHeight) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_targetHeight = targetHeight;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        // Start moving the elevator to the target height
        m_elevatorSubsystem.moveToPosition(m_targetHeight);
    }

    @Override
    public void execute() {
        // Optionally monitor progress or update any necessary state
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_elevatorSubsystem.stop();
        }
        // Optionally reset any state or perform cleanup here
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.atTargetPosition();
    }
}
