package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoConfigurer;
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
import frc.robot.commands.drive.DriveCommand;
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
import frc.robot.commands.mailbox.OutputLowCommand;
import frc.robot.commands.mailbox.ShootCommand;
import frc.robot.commands.mailbox.MailboxFeedCommand;
import frc.robot.commands.mailbox.MailboxRewindCommand;
import frc.robot.commands.mailbox.OutputHighCommand;
import frc.robot.commands.mailbox.StopCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.RobotContainerConstants;
import frc.robot.constants.SimulationConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.mailbox.LaserCanSubsystem;
import frc.robot.subsystems.mailbox.MailboxHandler;
import frc.robot.subsystems.mailbox.MailboxSubsystem;
import frc.robot.subsystems.vision.TagInputHandler;
//import frc.robot.subsystems.simulation.SimulationSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.commands.vision.Distance;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.commands.vision.TrackAprilTagCommand;
import frc.robot.subsystems.drive.DirectionSnapSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorHandler;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.subsystems.flopper.FlopperSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  private final JoystickButton gyroZeroButton          = new JoystickButton(driverController,   RobotContainerConstants.GYRO_ZERO_BUTON);
  private final POVButton      forwardsSnapButton      = new POVButton(driverController,        RobotContainerConstants.SNAP_FORWARDS_DIRECTION);
  private final POVButton      leftSnapButton          = new POVButton(driverController,        RobotContainerConstants.SNAP_LEFT_DIRECTION);
  private final POVButton      rightSnapButton         = new POVButton(driverController,        RobotContainerConstants.SNAP_RIGHT_DIRECTION);
  private final POVButton      backwardsSnapButton     = new POVButton(driverController,        RobotContainerConstants.SNAP_BACKWARDS_DIRECTION);
  private final JoystickButton stopSnapButton          = new JoystickButton(driverController,   RobotContainerConstants.SNAP_STOP_BUTTON);
  private final JoystickButton flopperUpButton         = new JoystickButton(operatorController, RobotContainerConstants.FLOPPER_UP_BUTTON);
  private final JoystickButton flopperDownButton       = new JoystickButton(operatorController, RobotContainerConstants.FLOPPER_DOWN_BUTTON);
  private final JoystickButton mailboxShootButton      = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_SHOOT_BUTTON);
  private final JoystickButton mailboxFeedButton       = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_FEED_BUTTON);
  private final JoystickButton mailboxRewindButton     = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_REWIND_BUTTON);
  private final POVButton      elevatorUpButton        = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_UP_BUTTON);
  private final POVButton      elevatorDownButton      = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_DOWN_BUTTON);
  private final JoystickButton elevatorBottomButton    = new JoystickButton(operatorController, RobotContainerConstants.ELEVATOR_BOTTOM_BUTTON);
  private final JoystickButton elevatorFeedButton      = new JoystickButton(operatorController, RobotContainerConstants.ELEVATOR_FEED_BUTTON);
  private final POVButton      elevatorL1Button        = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_L1_BUTTON);
  private final POVButton      elevatorL2Button        = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_L2_BUTTON);
  private final POVButton      elevatorL3Button        = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_L3_BUTTON);
  private final POVButton      elevatorL4Button        = new POVButton(operatorController, RobotContainerConstants.ELEVATOR_L4_BUTTON);

  // Subsystems
  public static final GyroSubsystem           gyroSubsystem          = new GyroSubsystem("CANivore");
  public static final MailboxSubsystem        mailboxSubsystem       = new MailboxSubsystem();
  public static final MailboxHandler          mailboxHandler         = new MailboxHandler();
  public static final VisionSubsystem         visionSubsystem        = new VisionSubsystem();
  public static final SwerveSubsystem         swerveSubsystem        = new SwerveSubsystem();
  public static final DirectionSnapSubsystem  directionSnapSubsystem = new DirectionSnapSubsystem();
  public static final ElevatorSubsystem       elevatorSubsystem      = new ElevatorSubsystem();
  public static final ElevatorHandler         elevatorHandler        = new ElevatorHandler();
  public static final FlopperSubsystem        flopperSubsystem       = new FlopperSubsystem();
  public static final TagInputHandler         tagInputHandler        = new TagInputHandler();

  // Auto
  public static final TrackAprilTagCommand trackCommand = new TrackAprilTagCommand(22,
                                                                                   new Distance(VisionConstants.TRACK_TAG_X,
                                                                                                VisionConstants.TRACK_TAG_Y));
  public static final ClimbSubsystem climbSubsystem                 = new ClimbSubsystem();

  //Conditionally create SimulationSubsystem
  //public final SimulationSubsystem simulationSubsystem;

  // Auto chooser for PathPlanner
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AutoConfigurer.configure();
    autoChooser = AutoBuilder.buildAutoChooser();

    // Configure the trigger bindings
    configureBindings();

    DriveCommand xboxDriveCommand = new DriveCommand(() -> MathUtil.applyDeadband(driverController.getLeftX(), RobotContainerConstants.CONTROLLER_MOVEMENT_DEADBAND),
                                                     () -> MathUtil.applyDeadband(driverController.getLeftY(), RobotContainerConstants.CONTROLLER_MOVEMENT_DEADBAND),
                                                     () -> MathUtil.applyDeadband(-driverController.getRightX(), RobotContainerConstants.CONTROLLER_ROTATION_DEADBAND));

    swerveSubsystem.setDefaultCommand(xboxDriveCommand);

    //Conditionally initialize the simulation subsystem
    if (Robot.isSimulation()) {
    //   simulationSubsystem = new SimulationSubsystem(swerveSubsystem.getSwerveDriveSimulation(), swerveSubsystem);
    //   simulationSubsystem.setInitialPose(new Pose2d(SimulationConstants.ROBOT_STARTING_POSE_X, SimulationConstants.ROBOT_STARTING_POSE_Y, Rotation2d.fromDegrees(0)));
    // } else {
    //   simulationSubsystem = null;
    // }
    }

    // Add climb commands to SmartDashboard
    SmartDashboard.putData("[CLIMB] Force Pins Down", new ForcePinsDownCommand());
    SmartDashboard.putData("[CLIMB] Force Pins Up", new ForcePinsUpCommand());
    SmartDashboard.putData("[CLIMB] Lock", new LockClimbCommand());
    SmartDashboard.putData("[CLIMB] Unlock", new UnlockClimbCommand());
    SmartDashboard.putData("[CLIMB] Stop", new StopClimbCommand());

    SmartDashboard.putData("[ELEVATOR] Bottom", new ElevatorGotoBottomCommand());
    SmartDashboard.putData("[ELEVATOR] Feed",   new ElevatorGotoFeedCommand());
    SmartDashboard.putData("[ELEVATOR] L1",     new ElevatorGotoL1Command());
    SmartDashboard.putData("[ELEVATOR] L2",     new ElevatorGotoL2Command());
    SmartDashboard.putData("[ELEVATOR] L3",     new ElevatorGotoL3Command());
    SmartDashboard.putData("[ELEVATOR] L4",     new ElevatorGotoL4Command());
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return trackCommand;
  }
}
