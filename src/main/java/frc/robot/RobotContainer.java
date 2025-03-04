package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoConfigurer;
import frc.robot.commands.climb.ForcePinsDownCommand;
import frc.robot.commands.climb.LockClimbCommand;
import frc.robot.commands.climb.StopClimbCommand;
import frc.robot.auto.AutoConfigurer;
import frc.robot.commands.directionSnaps.DirectionSnapBackwards;
import frc.robot.commands.directionSnaps.DirectionSnapForwards;
import frc.robot.commands.directionSnaps.DirectionSnapLeft;
import frc.robot.commands.directionSnaps.DirectionSnapRight;
import frc.robot.commands.directionSnaps.StopSnap;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.elevator.LowerElevatorCommand;
import frc.robot.commands.elevator.RaiseElevatorCommand;
import frc.robot.commands.drive.TrackAprilTagCommand;
import frc.robot.commands.gyro.GyroZeroCommand;
import frc.robot.commands.mailbox.InputCommand;
import frc.robot.commands.mailbox.OutputCommand;
import frc.robot.commands.mailbox.StopCommand;
import frc.robot.commands.solenoid.ExtendCommand;
import frc.robot.commands.solenoid.RetractCommand;
import frc.robot.constants.RobotContainerConstants;
import frc.robot.constants.SimulationConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.mailbox.MailboxSubsystem;
import frc.robot.subsystems.simulation.SimulationSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemSim;
import frc.robot.commands.vision.PrintTargetInfo;
import frc.robot.subsystems.drive.DirectionSnapSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.solenoid.SolenoidSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.elevator.ElevatorHeightCalculation;

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
  private final JoystickButton mailboxInputButton       = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_INPUT_BUTTON);
  private final JoystickButton mailboxOutputButton      = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_OUTPUT_BUTTON);
  private final JoystickButton mailboxStopButton        = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_STOP_BUTTON);
  private final JoystickButton extendButton             = new JoystickButton(operatorController, RobotContainerConstants.SOLENOID_EXTEND_BUTTON);
  private final JoystickButton retractButton            = new JoystickButton(operatorController, RobotContainerConstants.SOLENOID_RETRACT_BUTTON);
  private final JoystickButton gyroZeroButton           = new JoystickButton(driverController, RobotContainerConstants.GYRO_ZERO_BUTON);
  private final POVButton      forwardsSnapButton       = new POVButton(driverController, RobotContainerConstants.SNAP_FORWARDS_DIRECTION);
  private final POVButton      leftSnapButton           = new POVButton(driverController, RobotContainerConstants.SNAP_LEFT_DIRECTION);
  private final POVButton      rightSnapButton          = new POVButton(driverController, RobotContainerConstants.SNAP_RIGHT_DIRECTION);
  private final POVButton      backwardsSnapButton      = new POVButton(driverController, RobotContainerConstants.SNAP_BACKWARDS_DIRECTION);
  private final JoystickButton stopSnapButton           = new JoystickButton(driverController, RobotContainerConstants.SNAP_STOP_BUTTON);
  private final JoystickButton elevatorL1Button         = new JoystickButton(operatorController, RobotContainerConstants.ELEVATOR_L1_BUTTON);
  private final Trigger elevatorL2Trigger = new Trigger(() -> 
    operatorController.getRawButton(RobotContainerConstants.ELEVATOR_L2_BUTTONS[0]) &&
    operatorController.getRawButton(RobotContainerConstants.ELEVATOR_L2_BUTTONS[1])
  );
  private final Trigger elevatorL3Trigger = new Trigger(() -> 
      operatorController.getRawButton(RobotContainerConstants.ELEVATOR_L3_BUTTONS[0]) &&
      operatorController.getRawButton(RobotContainerConstants.ELEVATOR_L3_BUTTONS[1])
  );

  private final Trigger elevatorL4Trigger = new Trigger(() -> 
      operatorController.getRawButton(RobotContainerConstants.ELEVATOR_L4_BUTTONS[0]) &&
      operatorController.getRawButton(RobotContainerConstants.ELEVATOR_L4_BUTTONS[1])
  );
  private final Trigger elevatorLowerTrigger = new Trigger(() -> 
    operatorController.getRawButton(RobotContainerConstants.ELEVATOR_LOWER_BUTTON[0]) &&
    operatorController.getRawButton(RobotContainerConstants.ELEVATOR_LOWER_BUTTON[1])
  );
  private final JoystickButton climbLockButton         = new JoystickButton(operatorController, RobotContainerConstants.CLIMB_LOCK_BUTTON);
  private final JoystickButton climbForceDownButton     = new JoystickButton(operatorController, RobotContainerConstants.CLIMB_FORCE_DOWN_BUTTON);
  private final JoystickButton climbStopButton          = new JoystickButton(operatorController, RobotContainerConstants.CLIMB_STOP_BUTTON);

  // Subsystems
  public static final GyroSubsystem gyroSubsystem                   = new GyroSubsystem("CANivore");
  public static final ElevatorSubsystem elevatorSubsystem           = new ElevatorSubsystem(RobotContainerConstants.ELEVATOR_PRIMARY_MOTOR_ID,
                                                                                            RobotContainerConstants.ELEVATOR_SECONDARY_MOTOR_ID);
  public static final MailboxSubsystem mailboxSubsystem             = new MailboxSubsystem();
  public static final SwerveSubsystem swerveSubsystem               = new SwerveSubsystem();
  public static final SolenoidSubsystem solenoidSubsystem           = new SolenoidSubsystem();
  public static final DirectionSnapSubsystem directionSnapSubsystem = new DirectionSnapSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(new IntakeIOSim(swerveSubsystem.getSwerveDriveSimulation()));

  public final VisionSubsystem visionSubsystem;
  public final VisionSubsystemSim visionSubsystemSim;
  public static final ClimbSubsystem climbSubsystem                 = new ClimbSubsystem();

  //Conditionally create SimulationSubsystem
  public final SimulationSubsystem simulationSubsystem;

  // Auto chooser for PathPlanner
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure path planner
    AutoConfigurer.configure();

    // Create an auto chooser and add it to SmartDashboard
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("[PATHPLANNER] Auto Chooser", autoChooser);

    // Conditionally initialize the vision subsystem
  if (Robot.isSimulation()) {
      visionSubsystemSim = new VisionSubsystemSim(swerveSubsystem::getPose, VisionConstants.fieldLayout);
      visionSubsystem = null; // No real vision subsystem in simulation
  } else {
      visionSubsystem = new VisionSubsystem();
      visionSubsystemSim = null; // No simulated vision subsystem on the real robot
  }

  // Create the TrackAprilTagCommand with the correct vision subsystem
  if (Robot.isSimulation()) {
      new TrackAprilTagCommand(visionSubsystemSim, swerveSubsystem);
  } else {
      new TrackAprilTagCommand(visionSubsystem, swerveSubsystem);
  }

    // Configure path planner
    AutoConfigurer.configure();

    // Create an auto chooser and add it to SmartDashboard
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("[PATHPLANNER] Auto Chooser", autoChooser);

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
      System.out.println("DEBUG: runIntake() called");
      simulationSubsystem.runIntake();
    } else {
      simulationSubsystem = null;
    }

    SmartDashboard.putData("Elevator L1", new RaiseElevatorCommand(ElevatorHeightCalculation.L1));
    SmartDashboard.putData("Elevator L2", new RaiseElevatorCommand(ElevatorHeightCalculation.L2));
    SmartDashboard.putData("Elevator L3", new RaiseElevatorCommand(ElevatorHeightCalculation.L3));
    SmartDashboard.putData("Elevator L4", new RaiseElevatorCommand(ElevatorHeightCalculation.L4));
    SmartDashboard.putData("Elevator Lower", new LowerElevatorCommand());
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
    elevatorL1Button.onTrue(new RaiseElevatorCommand(ElevatorHeightCalculation.L1));
    elevatorL2Trigger.onTrue(new RaiseElevatorCommand(ElevatorHeightCalculation.L2));
    elevatorL3Trigger.onTrue(new RaiseElevatorCommand(ElevatorHeightCalculation.L3));
    elevatorL4Trigger.onTrue(new RaiseElevatorCommand(ElevatorHeightCalculation.L4));
    elevatorLowerTrigger.onTrue(new LowerElevatorCommand());
    climbLockButton.onTrue(new LockClimbCommand());
    climbForceDownButton.onTrue(new ForcePinsDownCommand());
    climbStopButton.onTrue(new StopClimbCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Using ProointTargetInfo causes a command scheduler loop overrun when fieldRelative is enabled
    //return null; // new PrintTargetInfo(visionSubsystem);
    // return autoChooser.getSelected(visionSubsystem, swerveSubsystem);
    return autoChooser.getSelected();
  
  }
}
