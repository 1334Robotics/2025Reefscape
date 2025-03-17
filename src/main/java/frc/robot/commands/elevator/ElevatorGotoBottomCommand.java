package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorLevel;

public class ElevatorGotoBottomCommand extends Command {
    public ElevatorGotoBottomCommand() {
        addRequirements(RobotContainer.elevatorSubsystem);
        addRequirements(RobotContainer.elevatorHandler);
    }

    @Override
    public void initialize() {
        RobotContainer.elevatorHandler.setLevel(ElevatorLevel.BOTTOM);
    }

    public boolean isFinished() {
        return true;
    }
}
