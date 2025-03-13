package frc.robot.commands.mailbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorLevel;

public class ShootCommand extends Command {
    public ShootCommand() {
        addRequirements(RobotContainer.mailboxHandler);
        addRequirements(RobotContainer.mailboxSubsystem);
        addRequirements(RobotContainer.elevatorHandler);
    }

    @Override
    public void execute() {
        ElevatorLevel level = RobotContainer.elevatorHandler.getLevel();
        if(level == null || level == ElevatorLevel.BOTTOM || level == ElevatorLevel.FEED) return;

        RobotContainer.mailboxHandler.allowShoot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
