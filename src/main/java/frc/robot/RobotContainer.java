package frc.robot;

import frc.robot.commands.directionSnaps.DirectionSnapBackwards;
import frc.robot.commands.directionSnaps.DirectionSnapForwards;
import frc.robot.commands.directionSnaps.DirectionSnapLeft;
import frc.robot.commands.directionSnaps.DirectionSnapRight;
import frc.robot.commands.directionSnaps.StopSnap;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.gyro.GyroZeroCommand;
import frc.robot.commands.mailbox.InputCommand;
import frc.robot.commands.mailbox.OutputCommand;
import frc.robot.commands.mailbox.StopCommand;
import frc.robot.commands.solenoid.ExtendCommand;
import frc.robot.commands.solenoid.RetractCommand;
import frc.robot.constants.RobotContainerConstants;
import frc.robot.constants.SimulationConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.mailbox.MailboxSubsystem;
import frc.robot.subsystems.simulation.SimulationSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.commands.vision.TrackAprilTagCommand;
import frc.robot.subsystems.drive.DirectionSnapSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.solenoid.SolenoidSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.laser.LaserCanSubsystem;
import frc.robot.commands.laser.MonitorLaserCanCommand;

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
  private final JoystickButton extendButton        = new JoystickButton(operatorController, RobotContainerConstants.SOLENOID_EXTEND_BUTTON);
  private final JoystickButton retractButton       = new JoystickButton(operatorController, RobotContainerConstants.SOLENOID_RETRACT_BUTTON);
  private final JoystickButton gyroZeroButton      = new JoystickButton(driverController, RobotContainerConstants.GYRO_ZERO_BUTON);
  private final POVButton      forwardsSnapButton  = new POVButton(driverController, RobotContainerConstants.SNAP_FORWARDS_DIRECTION);
  private final POVButton      leftSnapButton      = new POVButton(driverController, RobotContainerConstants.SNAP_LEFT_DIRECTION);
  private final POVButton      rightSnapButton     = new POVButton(driverController, RobotContainerConstants.SNAP_RIGHT_DIRECTION);
  private final POVButton      backwardsSnapButton = new POVButton(driverController, RobotContainerConstants.SNAP_BACKWARDS_DIRECTION);
  private final JoystickButton stopSnapButton      = new JoystickButton(driverController, RobotContainerConstants.SNAP_STOP_BUTTON);

  // Subsystems
  public static final GyroSubsystem gyroSubsystem                   = new GyroSubsystem("CANivore");
  public static final ElevatorSubsystem elevatorSubsystem           = new ElevatorSubsystem(RobotContainerConstants.ELEVATOR_PRIMARY_MOTOR_ID,
                                                                                            RobotContainerConstants.ELEVATOR_SECONDARY_MOTOR_ID);
  public static final MailboxSubsystem mailboxSubsystem             = new MailboxSubsystem();
  public static final VisionSubsystem visionSubsystem               = new VisionSubsystem();
  public static final SwerveSubsystem swerveSubsystem               = new SwerveSubsystem(visionSubsystem);
  public static final SolenoidSubsystem solenoidSubsystem           = new SolenoidSubsystem();
  public static final DirectionSnapSubsystem directionSnapSubsystem = new DirectionSnapSubsystem();
  public static final LaserCanSubsystem laserCanSubsystem           = new LaserCanSubsystem();

     //Conditionally create SimulationSubsystem
     public final SimulationSubsystem simulationSubsystem;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    DriveCommand xboxDriveCommand = new DriveCommand(() -> MathUtil.applyDeadband(driverController.getLeftX(), RobotContainerConstants.CONTROLLER_MOVEMENT_DEADBAND),
                                                     () -> MathUtil.applyDeadband(driverController.getLeftY(), RobotContainerConstants.CONTROLLER_MOVEMENT_DEADBAND),
                                                     () -> MathUtil.applyDeadband(-driverController.getRightX(), RobotContainerConstants.CONTROLLER_ROTATION_DEADBAND));

    swerveSubsystem.setDefaultCommand(xboxDriveCommand);

    //Conditionally initialize the simulation subsystem
    if (Robot.isSimulation()) {
      simulationSubsystem = new SimulationSubsystem(swerveSubsystem.getSwerveDriveSimulation(), swerveSubsystem);
      simulationSubsystem.setInitialPose(new Pose2d(SimulationConstants.ROBOT_STARTING_POSE_X, SimulationConstants.ROBOT_STARTING_POSE_Y, Rotation2d.fromDegrees(0)));
    } else {
      simulationSubsystem = null;
    }

    // Configure default command if you want continuous monitoring
    laserCanSubsystem.setDefaultCommand(new MonitorLaserCanCommand());
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
    extendButton.onTrue(new ExtendCommand());
    retractButton.onTrue(new RetractCommand());
    gyroZeroButton.onTrue(new GyroZeroCommand()); 
    forwardsSnapButton.onTrue(new DirectionSnapForwards());
    leftSnapButton.onTrue(new DirectionSnapLeft());
    rightSnapButton.onTrue(new DirectionSnapRight());
    backwardsSnapButton.onTrue(new DirectionSnapBackwards());
    stopSnapButton.onTrue(new StopSnap());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new TrackAprilTagCommand();
  }
}
