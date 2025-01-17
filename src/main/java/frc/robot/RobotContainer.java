// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.mailbox.InputCommand;
import frc.robot.commands.mailbox.OutputCommand;
import frc.robot.commands.mailbox.StopCommand;
import frc.robot.constants.RobotContainerConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.mailbox.MailboxSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  private final XboxController driverController   = new XboxController(RobotContainerConstants.DRIVER_CONTROLLER_PORT);
  private final XboxController operatorController = new XboxController(RobotContainerConstants.OPERATOR_CONTROLLER_PORT);

  // Controller buttons
  private final JoystickButton mailboxInputButton  = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_INPUT_BUTTON);
  private final JoystickButton mailboxOutputButton = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_OUTPUT_BUTTON);
  private final JoystickButton mailboxStopButton   = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_STOP_BUTTON);

  // Subsystems
  public static final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(RobotContainerConstants.ELEVATOR_PRIMARY_MOTOR_ID,
                                                                                  RobotContainerConstants.ELEVATOR_SECONDARY_MOTOR_ID);
  public static final MailboxSubsystem mailboxSubsystem = new MailboxSubsystem();
  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    DriveCommand xboxDriveCommand = new DriveCommand(() -> MathUtil.applyDeadband(driverController.getLeftX(), RobotContainerConstants.CONTROLLER_MOVEMENT_DEADBAND),
                                                     () -> MathUtil.applyDeadband(driverController.getLeftY(), RobotContainerConstants.CONTROLLER_MOVEMENT_DEADBAND),
                                                     () -> MathUtil.applyDeadband(-driverController.getRightX(), RobotContainerConstants.CONTROLLER_ROTATION_DEADBAND));

    swerveSubsystem.setDefaultCommand(xboxDriveCommand);
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
    mailboxInputButton.onTrue(new InputCommand());
    mailboxOutputButton.onTrue(new OutputCommand());
    mailboxStopButton.onTrue(new StopCommand());
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
