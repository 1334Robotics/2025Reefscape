// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;

public class ElevatorUpCommand extends Command {
  public ElevatorUpCommand() {
    addRequirements(RobotContainer.elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Check if the elevator is locked, and if so, don't execute this
    if(RobotContainer.elevatorSubsystem.lock) return;

    RobotContainer.elevatorSubsystem.runMotor(((RobotContainer.elevatorSubsystem.getPosition() > ElevatorConstants.ELEVATOR_SLOW_HIGH_POS)
                                                ? -(ElevatorConstants.ELEVATOR_SLOW_SPEED+0.05)
                                                : -ElevatorConstants.ELEVATOR_UP_SPEED));
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
