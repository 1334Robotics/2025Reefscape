// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;

public class ElevatorDownCommand extends Command {
  public ElevatorDownCommand() {
    addRequirements(RobotContainer.elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Check if the elevator is locked, and if so, don't execute this
    if(RobotContainer.elevatorSubsystem.lock) return;

    // Check if the limit switch has been hit, and if so, stop
    if(RobotContainer.elevatorSubsystem.limitSwitchSeen()) {
      this.end(true);
      return;
    }

    RobotContainer.elevatorSubsystem.runMotor(((RobotContainer.elevatorSubsystem.getPosition() < ElevatorConstants.ELEVATOR_SLOW_LOW_POS)
                                                ? ElevatorConstants.ELEVATOR_SLOW_SPEED
                                                : ElevatorConstants.ELEVATOR_DOWN_SPEED));
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevatorSubsystem.runMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
