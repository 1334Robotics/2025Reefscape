package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoCache;
import frc.robot.auto.AutoItem;
import frc.robot.commands.climb.ForcePinsDownCommand;
import frc.robot.commands.climb.ForcePinsUpCommand;
import frc.robot.commands.climb.LockClimbCommand;
import frc.robot.commands.climb.StopClimbCommand;
import frc.robot.commands.climb.UnlockClimbCommand;
import frc.robot.commands.directionSnaps.DirectionSnapBackwards;
import frc.robot.commands.directionSnaps.DirectionSnapForwards;
import frc.robot.commands.directionSnaps.DirectionSnapLeft;
import frc.robot.commands.directionSnaps.DirectionSnapRight;
import frc.robot.commands.directionSnaps.StopSnap;
import frc.robot.commands.drive.BotRelativeCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.FieldRelativeCommand;
import frc.robot.commands.drive.NormalSpeedCommand;
import frc.robot.commands.drive.RipControlCommand;
import frc.robot.commands.drive.SlowDownCommand;
import frc.robot.commands.drive.SpeedUpCommand;
import frc.robot.commands.elevator.ElevatorDownCommand;
import frc.robot.commands.elevator.ElevatorGotoBottomCommand;
import frc.robot.commands.elevator.ElevatorGotoFeedCommand;
import frc.robot.commands.elevator.ElevatorGotoL1Command;
import frc.robot.commands.elevator.ElevatorGotoL2Command;
import frc.robot.commands.elevator.ElevatorGotoL3Command;
import frc.robot.commands.elevator.ElevatorGotoL4Command;
import frc.robot.commands.elevator.ElevatorUpCommand;
import frc.robot.commands.flopper.FlopperDownCommand;
import frc.robot.commands.flopper.FlopperUpCommand;
import frc.robot.commands.gyro.GyroZeroCommand;
import frc.robot.commands.mailbox.ShootCommand;
import frc.robot.commands.mailbox.MailboxFeedCommand;
import frc.robot.commands.mailbox.MailboxRewindCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotContainerConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.mailbox.MailboxHandler;
import frc.robot.subsystems.mailbox.MailboxSubsystem;
import frc.robot.subsystems.vision.AutoTagSelector;
import frc.robot.subsystems.vision.TagInputHandler;
import frc.robot.subsystems.vision.TagTrackingHandler;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.commands.vision.StartTrackingScoringLeft;
import frc.robot.commands.vision.StartTrackingScoringRight;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.controller.ControllerSubsystem;
import frc.robot.commands.vision.TrackAprilTagCommand;
import frc.robot.subsystems.drive.DirectionSnapSubsystem;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorHandler;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.led.LedHandler;
import frc.robot.subsystems.led.LedSubsystem;

import frc.robot.subsystems.flopper.FlopperSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  public static final XboxController driverController   = new XboxController(RobotContainerConstants.DRIVER_CONTROLLER_PORT);
  public static final XboxController operatorController = new XboxController(RobotContainerConstants.OPERATOR_CONTROLLER_PORT);

  // Controller buttons
  private static final JoystickButton gyroZeroButton       = new JoystickButton(driverController,   RobotContainerConstants.GYRO_ZERO_BUTON);
  private static final POVButton      forwardsSnapButton   = new POVButton(driverController,        RobotContainerConstants.SNAP_FORWARDS_DIRECTION);
  private static final POVButton      leftSnapButton       = new POVButton(driverController,        RobotContainerConstants.SNAP_LEFT_DIRECTION);
  private static final POVButton      rightSnapButton      = new POVButton(driverController,        RobotContainerConstants.SNAP_RIGHT_DIRECTION);
  private static final POVButton      backwardsSnapButton  = new POVButton(driverController,        RobotContainerConstants.SNAP_BACKWARDS_DIRECTION);
  private static final JoystickButton stopSnapButton       = new JoystickButton(driverController,   RobotContainerConstants.SNAP_STOP_BUTTON);
  private static final JoystickButton flopperUpButton      = new JoystickButton(operatorController, RobotContainerConstants.FLOPPER_UP_BUTTON);
  private static final JoystickButton flopperDownButton    = new JoystickButton(operatorController, RobotContainerConstants.FLOPPER_DOWN_BUTTON);
  private static final Trigger        mailboxShootButton   = new Trigger(() -> operatorController.getRightTriggerAxis()  > RobotContainerConstants.TRIGGER_ACTIVATE_POINT);
  private static final Trigger        mailboxFeedButton    = new Trigger(() -> operatorController.getLeftTriggerAxis()  > RobotContainerConstants.TRIGGER_ACTIVATE_POINT);
  private static final JoystickButton mailboxRewindButton  = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_REWIND_BUTTON);
  private static final POVButton      elevatorUpButton     = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_UP_BUTTON);
  private static final POVButton      elevatorDownButton   = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_DOWN_BUTTON);
  private static final JoystickButton elevatorBottomButton = new JoystickButton(operatorController, RobotContainerConstants.ELEVATOR_BOTTOM_BUTTON);
  private static final JoystickButton elevatorFeedButton   = new JoystickButton(operatorController, RobotContainerConstants.ELEVATOR_FEED_BUTTON);
  private static final POVButton      elevatorL1Button     = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_L1_BUTTON);
  private static final POVButton      elevatorL2Button     = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_L2_BUTTON);
  private static final POVButton      elevatorL3Button     = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_L3_BUTTON);
  private static final POVButton      elevatorL4Button     = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_L4_BUTTON);
  private static final JoystickButton ripControlButton     = new JoystickButton(driverController, RobotContainerConstants.RIP_CONTROL_BUTTON);
  // private static final Trigger        trackLeftButton      = new Trigger(() -> driverController.getLeftTriggerAxis()  > RobotContainerConstants.TRIGGER_ACTIVATE_POINT);
  // private static final Trigger        trackRightButton     = new Trigger(() -> driverController.getRightTriggerAxis() > RobotContainerConstants.TRIGGER_ACTIVATE_POINT);
  private static final JoystickButton botRelativeButton    = new JoystickButton(driverController, RobotContainerConstants.BOT_RELATIVE_BUTTON);
  private static final JoystickButton slowDownButton       = new JoystickButton(driverController, RobotContainerConstants.SLOW_DOWN_BUTTON);
  private static final JoystickButton speedUpButton        = new JoystickButton(driverController, RobotContainerConstants.SPEED_UP_BUTTON);
  // private static final Trigger        pinsDownButton       = new Trigger(() -> operatorController.getLeftTriggerAxis()  > RobotContainerConstants.TRIGGER_ACTIVATE_POINT);
  // private static final Trigger        pinsLockButton       = new Trigger(() -> operatorController.getRightTriggerAxis() > RobotContainerConstants.TRIGGER_ACTIVATE_POINT);
  private static final JoystickButton pinsUpButton         = new JoystickButton(operatorController, RobotContainerConstants.CLIMB_UP_BUTTON);

  // Subsystems
  public static final LedSubsystem           ledSubsystem                = new LedSubsystem(1); 
  public static final LedHandler             ledHandler                  = new LedHandler(ledSubsystem);
  public static final GyroSubsystem          gyroSubsystem               = new GyroSubsystem("CANivore");
  public static final MailboxSubsystem       mailboxSubsystem            = new MailboxSubsystem();
  public static final MailboxHandler         mailboxHandler              = new MailboxHandler();
  public static final VisionSubsystem        visionSubsystem             = new VisionSubsystem();
  public static final SwerveSubsystem        swerveSubsystem             = new SwerveSubsystem();
  public static final DirectionSnapSubsystem directionSnapSubsystem      = new DirectionSnapSubsystem();
  public static final ElevatorSubsystem      elevatorSubsystem           = new ElevatorSubsystem();
  public static final ElevatorHandler        elevatorHandler             = new ElevatorHandler();
  public static final FlopperSubsystem       flopperSubsystem            = new FlopperSubsystem();
  public static final ClimbSubsystem         climbSubsystem              = new ClimbSubsystem();
  public static final TagInputHandler        tagInputHandler             = new TagInputHandler();
  public static final DriveController        driveController             = new DriveController();
  public static final TagTrackingHandler     tagTrackingHandler          = new TagTrackingHandler();
  public static final ControllerSubsystem    driverControllerSubsystem   = new ControllerSubsystem(driverController);
  public static final ControllerSubsystem    operatorControllerSubsystem = new ControllerSubsystem(operatorController);

  // Auto
  public static final TrackAprilTagCommand trackCommand = new TrackAprilTagCommand();
  public static final AutoTagSelector autoTagSelector = new AutoTagSelector();

  public RobotContainer() {
    RobotContainer.configureBindings();
    RobotContainer.setupDefaultCommands();
    RobotContainer.registerClimbCommands();
  }

  private static void registerClimbCommands() {
    SmartDashboard.putData("[CLIMB] Lock",            new LockClimbCommand());
    SmartDashboard.putData("[CLIMB] Unlock",          new UnlockClimbCommand());
    SmartDashboard.putData("[CLIMB] Stop",            new StopClimbCommand());
    SmartDashboard.putData("[CLIMB] Force Pins Down", new ForcePinsDownCommand());
    SmartDashboard.putData("[CLIMB] Force Pins Up",   new ForcePinsUpCommand());
  }

  private static void setupDefaultCommands() {
    DriveCommand xboxDriveCommand = new DriveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftX(), RobotContainerConstants.CONTROLLER_MOVEMENT_DEADBAND),
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
  private static void configureBindings() {
    DriverStation.silenceJoystickConnectionWarning(true);

    gyroZeroButton.onTrue(new GyroZeroCommand()); 
    forwardsSnapButton.onTrue(new DirectionSnapForwards());
    leftSnapButton.onTrue(new DirectionSnapLeft());
    rightSnapButton.onTrue(new DirectionSnapRight());
    backwardsSnapButton.onTrue(new DirectionSnapBackwards());
    stopSnapButton.onTrue(new StopSnap());
    flopperUpButton.whileTrue(new FlopperUpCommand());
    flopperDownButton.whileTrue(new FlopperDownCommand());
    mailboxShootButton.onTrue(new ShootCommand());
    mailboxFeedButton.onTrue(new MailboxFeedCommand());
    mailboxRewindButton.whileTrue(new MailboxRewindCommand());
    if(!ElevatorConstants.MANUAL_ELEVATOR_CONTROL) {
      elevatorBottomButton.onTrue(new ElevatorGotoBottomCommand());
      elevatorFeedButton.onTrue(new ElevatorGotoFeedCommand());
      elevatorL1Button.onTrue(new ElevatorGotoL1Command());
      elevatorL2Button.onTrue(new ElevatorGotoL2Command());
      elevatorL3Button.onTrue(new ElevatorGotoL3Command());
      elevatorL4Button.onTrue(new ElevatorGotoL4Command());
    } else {
      elevatorUpButton.whileTrue(new ElevatorUpCommand());
      elevatorDownButton.whileTrue(new ElevatorDownCommand());
    }
    ripControlButton.onTrue(new RipControlCommand());
    // trackLeftButton.onTrue(new StartTrackingScoringLeft());
    // trackRightButton.onTrue(new StartTrackingScoringRight());
    botRelativeButton.onTrue(new BotRelativeCommand());
    botRelativeButton.onFalse(new FieldRelativeCommand());
    slowDownButton.onTrue(new SlowDownCommand());
    slowDownButton.onFalse(new NormalSpeedCommand());
    speedUpButton.onTrue(new SpeedUpCommand());
    speedUpButton.onFalse(new NormalSpeedCommand());
    // pinsDownButton.whileTrue(new ForcePinsDownCommand());
    // pinsLockButton.whileTrue(new LockClimbCommand());
    pinsUpButton.whileTrue(new ForcePinsUpCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutoItem selectedAuto = AutoCache.getSelectedAuto();
    if(selectedAuto == null) return null;
    return selectedAuto.getCommand();
  }
}
