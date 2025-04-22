package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;

public class ElevatorResetCommand extends Command {
    private boolean finished;

    public ElevatorResetCommand() {
        this.finished = false;

        addRequirements(RobotContainer.elevatorSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.elevatorSubsystem.lock = true;
    }

    @Override
    public void execute() {
        RobotContainer.elevatorSubsystem.runMotor(ElevatorConstants.ELEVATOR_RESET_SPEED);
        if(RobotContainer.elevatorSubsystem.limitSwitchSeen()) this.end(true);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.elevatorSubsystem.runMotor(0);
        RobotContainer.elevatorSubsystem.resetElevatorPos();
        this.finished = true;
        RobotContainer.elevatorSubsystem.lock = false;
    }

    @Override
    public boolean isFinished() {
        return this.finished;
    }
}
