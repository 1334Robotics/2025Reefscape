package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoConfigurer;
import frc.robot.commands.directionSnaps.DirectionSnapBackwards;
import frc.robot.commands.directionSnaps.DirectionSnapForwards;
import frc.robot.commands.directionSnaps.DirectionSnapLeft;
import frc.robot.commands.directionSnaps.DirectionSnapRight;
import frc.robot.commands.directionSnaps.StopSnap;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.elevator.LowerElevatorCommand;
import frc.robot.commands.elevator.RaiseElevatorCommand;
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.commands.auto.PathFollowerCommand;
import frc.robot.commands.auto.SimulationPathTestCommand;
import frc.robot.commands.auto.SimulationTestRunner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.solenoid.SolenoidSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.laser.LaserCanSubsystem;
import frc.robot.commands.laser.MonitorLaserCanCommand;
import frc.robot.commands.elevator.ElevatorHeightCalculation;
import edu.wpi.first.wpilibj.DriverStation;

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

  // Auto chooser for PathPlanner
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure PathPlanner
    AutoConfigurer.configure();
    
    // Create auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Add individual path buttons to dashboard
    addPathPlannerButtonsToDashboard();

    // Configure the trigger bindings
    configureBindings();

    DriveCommand xboxDriveCommand = new DriveCommand(() -> MathUtil.applyDeadband(driverController.getLeftX(), RobotContainerConstants.CONTROLLER_MOVEMENT_DEADBAND),
                                                     () -> MathUtil.applyDeadband(driverController.getLeftY(), RobotContainerConstants.CONTROLLER_MOVEMENT_DEADBAND),
                                                     () -> MathUtil.applyDeadband(-driverController.getRightX(), RobotContainerConstants.CONTROLLER_ROTATION_DEADBAND));

    swerveSubsystem.setDefaultCommand(xboxDriveCommand);

    //Conditionally initialize the simulation subsystem
    if (Robot.isSimulation()) {
      // Set up simulation-specific configurations
      setupSimulation();
      
      // Create simplified simulation subsystem
      simulationSubsystem = new SimulationSubsystem(swerveSubsystem);
      simulationSubsystem.setInitialPose(new Pose2d(SimulationConstants.ROBOT_STARTING_POSE_X, SimulationConstants.ROBOT_STARTING_POSE_Y, Rotation2d.fromDegrees(0)));
    } else {
      simulationSubsystem = null;
    }

    SmartDashboard.putData("Elevator L1", new RaiseElevatorCommand(ElevatorHeightCalculation.L1));
    SmartDashboard.putData("Elevator L2", new RaiseElevatorCommand(ElevatorHeightCalculation.L2));
    SmartDashboard.putData("Elevator L3", new RaiseElevatorCommand(ElevatorHeightCalculation.L3));
    SmartDashboard.putData("Elevator L4", new RaiseElevatorCommand(ElevatorHeightCalculation.L4));
    SmartDashboard.putData("Elevator Lower", new LowerElevatorCommand());

    // Configure default command if you want continuous monitoring
    laserCanSubsystem.setDefaultCommand(new MonitorLaserCanCommand());
  }
  
  /**
   * Configure simulation-specific settings
   */
  private void setupSimulation() {
    System.out.println("Setting up simulation environment");
    
    // Suppress joystick warnings in simulation
    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Add simulation-specific controls
    SmartDashboard.putData("SIM: Zero Gyro", Commands.runOnce(() -> {
      RobotContainer.gyroSubsystem.zero();
      System.out.println("Gyro zeroed in simulation");
    }));
    
    // Add a quick test button that combines zeroing the gyro and resetting pose
    SmartDashboard.putData("SIM: Reset All", Commands.runOnce(() -> {
      RobotContainer.gyroSubsystem.zero();
      
      if (simulationSubsystem != null) {
        simulationSubsystem.setInitialPose(new Pose2d(
          SimulationConstants.ROBOT_STARTING_POSE_X, 
          SimulationConstants.ROBOT_STARTING_POSE_Y, 
          Rotation2d.fromDegrees(0)));
      }
      
      System.out.println("Robot reset in simulation");
    }));
  }
  
  /**
   * Adds buttons to the SmartDashboard to run individual PathPlanner paths
   */
  private void addPathPlannerButtonsToDashboard() {
    // Add buttons for individual paths
    SmartDashboard.putData("Path: Feed12Tag17Left", createPathCommand("Feed12Tag17Left"));
    SmartDashboard.putData("Path: Feed12Tag17Right", createPathCommand("Feed12Tag17Right"));
    SmartDashboard.putData("Path: Feed12Tag18Left", createPathCommand("Feed12Tag18Left"));
    SmartDashboard.putData("Path: Feed12Tag18Right", createPathCommand("Feed12Tag18Right"));
    SmartDashboard.putData("Path: Feed12Tag19Left", createPathCommand("Feed12Tag19Left"));
    SmartDashboard.putData("Path: Feed12Tag19Right", createPathCommand("Feed12Tag19Right"));
    SmartDashboard.putData("Path: Feed12Tag20Left", createPathCommand("Feed12Tag20Left"));
    SmartDashboard.putData("Path: Feed12Tag20Right", createPathCommand("Feed12Tag20Right"));
    SmartDashboard.putData("Path: Feed12Tag21Left", createPathCommand("Feed12Tag21Left"));
    SmartDashboard.putData("Path: Feed12Tag21Right", createPathCommand("Feed12Tag21Right"));
    SmartDashboard.putData("Path: Feed12Tag22Left", createPathCommand("Feed12Tag22Left"));
    SmartDashboard.putData("Path: Feed12Tag22Right", createPathCommand("Feed12Tag22Right"));
    SmartDashboard.putData("Path: AutoDrive20", createPathCommand("AutoDrive20"));
    SmartDashboard.putData("Path: AutoDrive22", createPathCommand("AutoDrive22"));
    
    // Add simulation-specific test buttons
    if (Robot.isSimulation()) {
      addSimulationTestButtons();
    }
  }

  /**
   * Creates a command to follow a path with proper field-relative setup
   * 
   * @param pathName the name of the path to follow
   * @return a command that will follow the specified path
   */
  private Command createPathCommand(String pathName) {
    return new PathFollowerCommand(pathName);
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
    
    // Add controller bindings for common PathPlanner paths if needed
    // Example: Add a quick access path to a commonly used driver controller button
    // new JoystickButton(driverController, XboxMappings.BUTTON_Y).onTrue(createPathCommand("AutoDrive20"));
    // new JoystickButton(driverController, XboxMappings.BUTTON_X).onTrue(createPathCommand("AutoDrive22"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Log the selected auto routine
    Command selectedAuto = autoChooser.getSelected();
    if (selectedAuto != null) {
      System.out.println("Selected autonomous command: " + selectedAuto.getName());
    } else {
      System.out.println("No autonomous command selected!");
    }
    
    return selectedAuto;
  }
  
  /**
   * Gets the simulation subsystem for use in simulation mode
   * 
   * @return the simulation subsystem, or null if not in simulation mode
   */
  public SimulationSubsystem getSimulationSubsystem() {
    return simulationSubsystem;
  }

  /**
   * Adds simulation-specific test buttons to the dashboard
   */
  private void addSimulationTestButtons() {
    // Add simulation test buttons for each path
    SmartDashboard.putData("SIM Test: Feed12Tag17Left", 
        new SimulationPathTestCommand("Feed12Tag17Left"));
    SmartDashboard.putData("SIM Test: Feed12Tag17Right", 
        new SimulationPathTestCommand("Feed12Tag17Right"));
    SmartDashboard.putData("SIM Test: Feed12Tag18Left", 
        new SimulationPathTestCommand("Feed12Tag18Left"));
    SmartDashboard.putData("SIM Test: Feed12Tag18Right", 
        new SimulationPathTestCommand("Feed12Tag18Right"));
    SmartDashboard.putData("SIM Test: AutoDrive20", 
        new SimulationPathTestCommand("AutoDrive20"));
    SmartDashboard.putData("SIM Test: AutoDrive22", 
        new SimulationPathTestCommand("AutoDrive22"));
    
    // Add a button to run the full automated test sequence
    SmartDashboard.putData("SIM: Run Test Sequence", new SimulationTestRunner());
    
    // Add a button to reset the robot pose in simulation
    SmartDashboard.putData("SIM: Reset Robot Pose", Commands.runOnce(() -> {
      if (simulationSubsystem != null) {
        simulationSubsystem.setInitialPose(new Pose2d(
          SimulationConstants.ROBOT_STARTING_POSE_X, 
          SimulationConstants.ROBOT_STARTING_POSE_Y, 
          Rotation2d.fromDegrees(0)));
        System.out.println("Robot pose reset in simulation");
      }
    }));
  }
}
