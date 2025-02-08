// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gearbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class BackwardCommand extends Command {
  public BackwardCommand() {
    addRequirements(RobotContainer.gearboxSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.gearboxSubsystem.backward();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
