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
    RobotContainer.elevatorSubsystem.runMotor(-ElevatorConstants.ELEVATOR_UP_SPEED);
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
