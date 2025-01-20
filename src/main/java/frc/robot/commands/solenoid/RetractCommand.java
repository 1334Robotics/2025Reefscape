package frc.robot.commands.solenoid;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;



public class RetractCommand extends Command {
    public RetractCommand() {
        addRequirements(RobotContainer.solenoidSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.solenoidSubsystem.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
