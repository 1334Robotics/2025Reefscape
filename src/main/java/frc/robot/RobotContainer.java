// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotContainerConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(RobotContainerConstants.DRIVER_CONTROLLER_PORT);

  // Subsystems
  public static final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(RobotContainerConstants.ELEVATOR_PRIMARY_MOTOR_ID,
                                                                                  RobotContainerConstants.ELEVATOR_SECONDARY_MOTOR_ID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Up arrow - Move elevator up
    driverController.povUp().whileTrue(
        elevatorSubsystem.run(() -> {
            elevatorSubsystem.setManualControl(0.5); // 50% power up
        })
    ).whileFalse(
        elevatorSubsystem.runOnce(() -> {
            elevatorSubsystem.setManualControl(0.0); // Stop
        })
    );

    // Down arrow - Move elevator down  
    driverController.povDown().whileTrue(
        elevatorSubsystem.run(() -> {
            elevatorSubsystem.setManualControl(-0.5); // 50% power down
        })
    ).whileFalse(
        elevatorSubsystem.runOnce(() -> {
            elevatorSubsystem.setManualControl(0.0); // Stop
        })
    );
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
