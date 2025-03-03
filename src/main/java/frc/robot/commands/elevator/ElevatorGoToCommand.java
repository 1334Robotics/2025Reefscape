package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ElevatorGoToCommand extends Command {
    public ElevatorGoToCommand() {
        addRequirements(RobotContainer.elevatorSubsystem);
    }
}
