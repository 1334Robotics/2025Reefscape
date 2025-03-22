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
import frc.robot.commands.drive.BotRelativeCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.FieldRelativeCommand;
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
import frc.robot.subsystems.vision.AutoTagSelector;
import frc.robot.subsystems.vision.TagInputHandler;
import frc.robot.subsystems.vision.TagTrackingHandler;
//import frc.robot.subsystems.simulation.SimulationSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.commands.vision.Distance;
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.flopper.FlopperSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;
import java.io.IOException;
import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.Optional;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.elevator.ElevatorLevel;
import swervelib.imu.SwerveIMU;

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
  private static final JoystickButton mailboxShootButton   = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_SHOOT_BUTTON);
  private static final JoystickButton mailboxFeedButton    = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_FEED_BUTTON);
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
  private static final Trigger        trackLeftButton      = new Trigger(() -> driverController.getLeftTriggerAxis()  > RobotContainerConstants.TRIGGER_ACTIVATE_POINT);
  private static final Trigger        trackRightButton     = new Trigger(() -> driverController.getRightTriggerAxis() > RobotContainerConstants.TRIGGER_ACTIVATE_POINT);
  private static final JoystickButton botRelativeButton    = new JoystickButton(driverController, RobotContainerConstants.BOT_RELATIVE_BUTTON);
  private static final JoystickButton slowDownButton       = new JoystickButton(driverController, RobotContainerConstants.SLOW_DOWN_BUTTON);
  private static final Trigger        pinsDownButton       = new Trigger(() -> operatorController.getLeftTriggerAxis()  > RobotContainerConstants.TRIGGER_ACTIVATE_POINT);
  private static final Trigger        pinsLockButton       = new Trigger(() -> operatorController.getRightTriggerAxis() > RobotContainerConstants.TRIGGER_ACTIVATE_POINT);
  private static final JoystickButton pinsUpButton         = new JoystickButton(operatorController, RobotContainerConstants.CLIMB_UP_BUTTON);

  // Subsystems - change to private final
  private static GyroSubsystem gyroSubsystem;
  private static MailboxSubsystem mailboxSubsystem;
  private static MailboxHandler mailboxHandler;
  private static VisionSubsystem visionSubsystem;
  private static SwerveSubsystem swerveSubsystem;
  private static DirectionSnapSubsystem directionSnapSubsystem;
  private static ElevatorSubsystem elevatorSubsystem;
  private static ElevatorHandler elevatorHandler;
  private static FlopperSubsystem flopperSubsystem;
  private static ClimbSubsystem climbSubsystem;
  private static TagInputHandler tagInputHandler;
  private static DriveController driveController;
  private static TagTrackingHandler tagTrackingHandler;
  private static ControllerSubsystem driverControllerSubsystem;

  // Auto
  public static final TrackAprilTagCommand trackCommand = new TrackAprilTagCommand(22,
                                                                                   new Distance(VisionConstants.TRACK_TAG_X,
                                                                                                VisionConstants.TRACK_TAG_Y));
  public static final AutoTagSelector autoTagSelector = new AutoTagSelector();

  //Conditionally create SimulationSubsystem
  //public final SimulationSubsystem simulationSubsystem;

  // Auto chooser for PathPlanner
  private SendableChooser<Command> autoChooser;

  // Map to store auto names for dashboard reporting
  private final Map<Command, String> autoCommandNames = new HashMap<>();

  // Path cache to improve performance and memory management
  private final Map<String, PathPlannerPath> pathCache = new HashMap<>();

  /**
   * List available path files on the dashboard
   */
  private void listAvailablePaths() {
    try {
      // Use Filesystem to get the deploy directory in a platform-independent way
      File pathsDir = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
      if (pathsDir.exists() && pathsDir.isDirectory()) {
        File[] pathFiles = pathsDir.listFiles((dir, name) -> name.endsWith(".path"));
        if (pathFiles != null) {
          // Create a text listing of available paths for reference
          StringBuilder pathsList = new StringBuilder();
          for (File file : pathFiles) {
            pathsList.append("- ").append(file.getName().replace(".path", "")).append("\n");
          }
          SmartDashboard.putString("Path Files (Reference Only)", pathsList.toString());
        }
      }
      
      // Also list available auto files
      File autosDir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
      if (autosDir.exists() && autosDir.isDirectory()) {
        File[] autoFiles = autosDir.listFiles((dir, name) -> name.endsWith(".auto"));
        if (autoFiles != null) {
          StringBuilder autosList = new StringBuilder();
          for (File file : autoFiles) {
            autosList.append("- ").append(file.getName().replace(".auto", "")).append("\n");
          }
          SmartDashboard.putString("[AUTO] Auto Files (Reference Only)", autosList.toString());
        }
      }
    } catch (Exception e) {
      System.err.println("Error listing path files: " + e.getMessage());
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    
    System.out.println("Initializing RobotContainer...");
    
    // 1. Register named commands FIRST
    System.out.println("1. Registering commands for PathPlanner...");
    registerNamedCommands();
    
    // 2. Create all subsystems
    System.out.println("2. Creating subsystems...");
    createSubsystems();
    
    // 3. Configure auto chooser
    System.out.println("3. Configuring Auto Chooser...");
    configureAutoChooser();
    
    // 4. Configure bindings
    System.out.println("4. Configuring button bindings...");
    configureBindings();
    
    // 5. Set up dashboard
    System.out.println("5. Setting up dashboard information...");
    setupDashboard();
    
    // 6. Preload autonomous routines
    System.out.println("6. Preloading autonomous routines...");
    preloadAutonomousRoutines();
    
    // 7. Configure default commands - MUST be after subsystem creation
    System.out.println("7. Setting up default commands...");
    setupDefaultCommands();
    
    System.out.println("RobotContainer initialization complete!");
  }

  /**
   * Creates all subsystems in the correct order to handle dependencies.
   * This method ensures proper initialization order and error handling.
   */
  private void createSubsystems() {
    try {
      // Core systems first
      System.out.println("  a. Initializing core systems...");
      gyroSubsystem = new GyroSubsystem("CANivore");
      
      // Drive systems next (depends on gyro)
      System.out.println("  b. Initializing drive systems...");
      swerveSubsystem = new SwerveSubsystem();
      directionSnapSubsystem = new DirectionSnapSubsystem();
      driveController = new DriveController();
      
      // Mechanism systems
      System.out.println("  c. Initializing mechanism systems...");
      elevatorSubsystem = new ElevatorSubsystem();
      elevatorHandler = new ElevatorHandler();
      flopperSubsystem = new FlopperSubsystem();
      mailboxSubsystem = new MailboxSubsystem();
      mailboxHandler = new MailboxHandler();
      climbSubsystem = new ClimbSubsystem();
      
      // Vision and tracking systems last
      System.out.println("  d. Initializing vision systems...");
      visionSubsystem = new VisionSubsystem();
      tagInputHandler = new TagInputHandler();
      tagTrackingHandler = new TagTrackingHandler();
      driverControllerSubsystem = new ControllerSubsystem(driverController);
      
      System.out.println("  ✓ All subsystems created successfully");
      
    } catch (Exception e) {
      System.err.println("ERROR: Failed to initialize subsystems: " + e.getMessage());
      e.printStackTrace();
      throw new RuntimeException("Critical failure in subsystem initialization", e);
    }
  }

  private void setupDashboard() {
    // Clear any existing auto data from SmartDashboard
    SmartDashboard.clearPersistent("[AUTO] Auto Routines");
    SmartDashboard.clearPersistent("[AUTO] PathPlanner/currentPath");
    SmartDashboard.clearPersistent("[AUTO] PathPlanner/activePath");
    
    // Get fresh auto chooser from PathPlanner
    autoChooser = AutoBuilder.buildAutoChooser();
    
    // Verify auto chooser exists before putting to dashboard
    if (autoChooser != null) {
        SmartDashboard.putData("[AUTO] Auto Routines", autoChooser);
        System.out.println("Auto chooser added to dashboard");
    } else {
        System.err.println("ERROR: Cannot add null auto chooser to dashboard!");
    }
    
    SmartDashboard.putString("[AUTO] Current Auto Status", "Ready (Press START button to run)");
    SmartDashboard.putString("[AUTO] Auto Button Help", "Press START button on driver controller to run selected auto");
    
    // Create Elevator Control tab
    ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
    
    // Add manual control toggle with a switch widget
    elevatorTab.addBoolean("Force Manual Control", () -> elevatorHandler.isForceManualControl())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .withPosition(0, 0)
        .withSize(2, 1);
        
    // Add control mode indicator
    elevatorTab.addString("Control Mode", 
        () -> elevatorHandler.isForceManualControl() ? "MANUAL" : "AUTOMATED")
        .withPosition(2, 0)
        .withSize(2, 1);
        
    // Add position info
    elevatorTab.addNumber("Current Position", 
        (DoubleSupplier)(() -> elevatorSubsystem.getPosition()))
        .withPosition(0, 1)
        .withSize(2, 1);
        
    elevatorTab.addNumber("Target Position",
        (DoubleSupplier)(() -> {
            ElevatorLevel level = elevatorHandler.getLevel();
            return level != null ? level.position : 0.0;
        }))
        .withPosition(2, 1)
        .withSize(2, 1);
        
    // List available paths for manual selection
    listAvailablePaths();
  }

  private void setupDefaultCommands() {
    DriveCommand xboxDriveCommand = new DriveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftX(), RobotContainerConstants.CONTROLLER_MOVEMENT_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftY(), RobotContainerConstants.CONTROLLER_MOVEMENT_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getRightX(), RobotContainerConstants.CONTROLLER_ROTATION_DEADBAND));

    swerveSubsystem.setDefaultCommand(xboxDriveCommand);
  }

  private void configureAutoChooser() {
    try {
      System.out.println("Starting auto chooser configuration...");
      
      // Build an auto chooser using PathPlanner's built-in method
      autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
      
      if (autoChooser == null) {
        System.err.println("ERROR: AutoBuilder.buildAutoChooser returned null!");
        return;
      }
      
      // Verify the chooser has options
      var options = autoChooser.getSelected();
      if (options == null) {
        System.out.println("Warning: Auto chooser has no selected option - this is expected if 'Do Nothing' is the only option");
      }
      
      // Put auto chooser on dashboard
      SmartDashboard.putData("[AUTO] Auto Routines", autoChooser);
      System.out.println("Auto chooser configured and added to dashboard");
      
      // List available autos in the terminal/log for debugging
      listAvailableAutos();
      
    } catch (Exception e) {
      System.err.println("Error configuring auto chooser: " + e.getMessage());
      e.printStackTrace();
    }
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
    ripControlButton.onTrue(new RipControlCommand());
    trackLeftButton.onTrue(new StartTrackingScoringLeft());
    trackRightButton.onTrue(new StartTrackingScoringRight());
    botRelativeButton.onTrue(new BotRelativeCommand());
    botRelativeButton.onFalse(new FieldRelativeCommand());
    slowDownButton.onTrue(new SlowDownCommand());
    slowDownButton.onFalse(new SpeedUpCommand());
    pinsDownButton.whileTrue(new ForcePinsDownCommand());
    pinsLockButton.whileTrue(new LockClimbCommand());
    pinsUpButton.whileTrue(new ForcePinsUpCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Get the selected command from the auto chooser
    Command selectedCommand = autoChooser.getSelected();
    
    // If no command is selected or it's the "Do Nothing" command, use tracking command
    if (selectedCommand == null || 
        (selectedCommand instanceof InstantCommand && 
         !(selectedCommand instanceof SequentialCommandGroup))) {
        System.out.println("No auto selected or 'Do Nothing' selected - Using tracking command");
        SmartDashboard.putString("[AUTO] Current Auto Status", "Using default tracking command");
        return trackCommand;
    }
    
    // Verify auto is properly loaded
    if (selectedCommand instanceof PathPlannerAuto) {
        PathPlannerAuto auto = (PathPlannerAuto)selectedCommand;
        if (!verifyAutoLoaded(auto)) {
            System.err.println("Auto verification failed for '" + auto.getName() + "', attempting reload...");
            selectedCommand = reloadAuto(auto.getName());
            if (selectedCommand == null) {
                System.err.println("Auto reload failed, falling back to tracking command");
                SmartDashboard.putString("[AUTO] Current Auto Status", "Auto load failed - Using tracking command");
                return trackCommand;
            }
        }
        
        // Prepare robot for auto execution
        prepareForAuto(selectedCommand);
    }
    
    System.out.println("Running selected auto routine: " + selectedCommand.getName());
    SmartDashboard.putString("[AUTO] Current Auto Status", "Running: " + selectedCommand.getName());
    return selectedCommand;
  }

  /**
   * Verifies that an auto routine is properly loaded and ready to run
   * @param auto The auto routine to verify
   * @return true if the auto is ready to run, false otherwise
   */
  private boolean verifyAutoLoaded(PathPlannerAuto auto) {
    try {
        // Check if auto file exists
        File autoFile = new File(Filesystem.getDeployDirectory(), 
                               "pathplanner/autos/" + auto.getName() + ".auto");
        if (!autoFile.exists()) {
            System.err.println("Auto file does not exist: " + autoFile.getAbsolutePath());
            return false;
        }

        // Verify all required subsystems are ready
        return verifySubsystemsReady();
        
    } catch (Exception e) {
        System.err.println("Error verifying auto: " + e.getMessage());
        return false;
    }
  }

  /**
   * Verifies that all required subsystems are ready for auto
   * @return true if all subsystems are ready, false otherwise
   */
  private boolean verifySubsystemsReady() {
    try {
        // Check critical subsystems
        if (swerveSubsystem == null || 
            swerveSubsystem.getSwerveDrive() == null || 
            swerveSubsystem.getSwerveDrive().getGyro() == null) {
            System.err.println("Swerve/Gyro not ready");
            return false;
        }
        
        // Check other required subsystems based on registered commands
        if (elevatorSubsystem == null) {
            System.err.println("Elevator subsystem not ready");
            return false;
        }
        
        if (mailboxSubsystem == null) {
            System.err.println("Mailbox subsystem not ready");
            return false;
        }
        
        return true;
    } catch (Exception e) {
        System.err.println("Error checking subsystems: " + e.getMessage());
        return false;
    }
  }

  /**
   * Prepares the robot for auto execution
   * @param command The command about to be run
   */
  private void prepareForAuto(Command command) {
    try {
        // Reset odometry if this is a PathPlannerAuto
        if (command instanceof PathPlannerAuto) {
            System.out.println("Preparing for auto execution...");
            
            // Get alliance
            var alliance = DriverStation.getAlliance();
            boolean isBlue = !alliance.isPresent() || alliance.get() == DriverStation.Alliance.Blue;
            
            // Set default starting pose based on alliance
            Pose2d startingPose = isBlue ? 
                new Pose2d(new Translation2d(Units.Meter.of(5.89), Units.Meter.of(5.5)), Rotation2d.fromDegrees(180)) :
                new Pose2d(new Translation2d(Units.Meter.of(8.05), Units.Meter.of(5.5)), Rotation2d.fromDegrees(0));
            
            // Reset odometry to starting pose
            swerveSubsystem.resetOdometry(startingPose);
            System.out.println("Reset odometry to starting pose: " + startingPose);
            
            // Clear any existing path following errors
            SmartDashboard.putNumber("[AUTO] Path Following Error", 0.0);
        }
    } catch (Exception e) {
        System.err.println("Error preparing for auto: " + e.getMessage());
    }
  }

  /**
   * Attempts to reload an auto routine
   * @param autoName The name of the auto to reload
   * @return The reloaded auto command, or null if reload failed
   */
  private Command reloadAuto(String autoName) {
    try {
        System.out.println("Attempting to reload auto: " + autoName);
        
        // Force reload the auto file
        PathPlannerAuto auto = new PathPlannerAuto(autoName);
        
        // Update auto chooser
        autoChooser.addOption(autoName, auto);
        autoChooser.setDefaultOption(autoName, auto);
        
        // Update dashboard
        SmartDashboard.putData("[AUTO] Auto Routines", autoChooser);
        
        System.out.println("Successfully reloaded auto: " + autoName);
        return auto;
        
    } catch (Exception e) {
        System.err.println("Failed to reload auto '" + autoName + "': " + e.getMessage());
        return null;
    }
  }

  /**
   * Updates dashboard indicators for current alliance and FMS position
   * Called periodically from Robot to ensure indicators stay current
   */
  public void updateAllianceIndicators() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      boolean isRed = alliance.get() == DriverStation.Alliance.Red;
      SmartDashboard.putBoolean("[AUTO] Is Red Alliance", isRed);
      SmartDashboard.putString("[AUTO] Current Alliance", isRed ? "RED" : "BLUE");
      
      // Get and display FMS position if available
      int location = DriverStation.getLocation().orElse(2); // Default to center (2)
      SmartDashboard.putNumber("[AUTO] FMS Position", location);
    } else {
      SmartDashboard.putBoolean("[AUTO] Is Red Alliance", false);
      SmartDashboard.putString("[AUTO] Current Alliance", "UNKNOWN");
      SmartDashboard.putNumber("[AUTO] FMS Position", 2); // Default to center
    }
  }

  /**
   * Register named commands for use in PathPlanner auto routines
   */
  private void registerNamedCommands() {
    System.out.println("Registering PathPlanner commands...");
    
    try {
        // Register auto sequence commands
        NamedCommands.registerCommand("Shoot", new ShootCommand());
        NamedCommands.registerCommand("MailboxShoot", new ShootCommand());
        System.out.println("✓ Registered Shoot command");
        
        // Register marker event commands
        NamedCommands.registerCommand("ElevatorL1", new ElevatorGotoL1Command());
        System.out.println("✓ Registered ElevatorL1 command (marker event)");
        
        // Log registration status to dashboard
        SmartDashboard.putString("[AUTO] PathPlanner/Commands", "MailboxShoot, ElevatorL1");
        System.out.println("Command registration complete!");
        
    } catch (Exception e) {
        System.err.println("Error registering commands: " + e.getMessage());
        SmartDashboard.putString("[AUTO] PathPlanner/Error", e.getMessage());
    }
  }
  
  /**
   * Preload all autonomous routines to prevent delays during competitions
   */
  private void preloadAutonomousRoutines() {
    try {
        // Get all auto files
        File autosDir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
        if (autosDir.exists() && autosDir.isDirectory()) {
            File[] autoFiles = autosDir.listFiles((dir, name) -> name.endsWith(".auto"));
            if (autoFiles != null) {
                System.out.println("Preloading " + autoFiles.length + " auto routines...");
                for (File file : autoFiles) {
                    String autoName = file.getName().replace(".auto", "");
                    try {
                        // Create the command to ensure it's loaded in memory
                        new PathPlannerAuto(autoName);
                        System.out.println("  - Preloaded auto: " + autoName);
                    } catch (Exception e) {
                        System.err.println("  - Error preloading auto '" + autoName + "': " + e.getMessage());
                    }
                }
                System.out.println("Auto preloading complete!");
            }
        }
    } catch (Exception e) {
        System.err.println("Error during auto preloading: " + e.getMessage());
    }
  }

  /**
   * List available auto files on the dashboard
   */
  private void listAvailableAutos() {
    System.out.println("Checking for available auto files...");
    try {
      // Get the deploy directory path
      File deployDir = Filesystem.getDeployDirectory();
      File autosDir = new File(deployDir, "pathplanner/autos");
      
      System.out.println("Looking for autos in: " + autosDir.getAbsolutePath());
      
      if (!autosDir.exists()) {
        System.err.println("ERROR: Autos directory does not exist!");
        System.err.println("Please ensure you have created the directory: " + autosDir.getAbsolutePath());
        return;
      }
      
      if (!autosDir.isDirectory()) {
        System.err.println("ERROR: Autos path exists but is not a directory!");
        return;
      }
      
      File[] autoFiles = autosDir.listFiles((dir, name) -> name.endsWith(".auto"));
      
      if (autoFiles == null) {
        System.err.println("ERROR: Unable to list files in autos directory");
        return;
      }
      
      if (autoFiles.length == 0) {
        System.out.println("No .auto files found in " + autosDir.getAbsolutePath());
        System.out.println("Please create at least one auto routine in PathPlanner");
        return;
      }
      
      System.out.println("Found " + autoFiles.length + " auto files:");
      StringBuilder autosList = new StringBuilder();
      for (File file : autoFiles) {
        String autoName = file.getName().replace(".auto", "");
        autosList.append("- ").append(autoName).append("\n");
        System.out.println("  - " + autoName);
      }
      
      SmartDashboard.putString("[AUTO] Available Autos", autosList.toString());
      
    } catch (Exception e) {
      System.err.println("Error listing auto files: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /**
   * Creates a command to run the currently selected autonomous routine from the auto chooser
   * This allows running autonomously selected routines without switching robot modes
   * 
   * @return A command that runs the selected autonomous routine
   */
  public Command runSelectedAuto() {
    return Commands.defer(() -> {
      Command selectedAuto = autoChooser.getSelected();
      
      if (selectedAuto == null || 
          (selectedAuto instanceof InstantCommand && 
           !(selectedAuto instanceof SequentialCommandGroup))) {
        System.out.println("No auto selected or 'Do Nothing' selected");
        SmartDashboard.putString("[AUTO] Current Auto Status", "No auto routine selected");
        return Commands.none();
      }
      
      System.out.println("Running selected auto routine via button press");
      
      return Commands.sequence(
        Commands.runOnce(() -> {
          CommandScheduler.getInstance().cancelAll();
          SmartDashboard.putString("[AUTO] Current Auto Status", "Running Auto");
        }),
        selectedAuto,
        Commands.runOnce(() -> {
          System.out.println("Auto routine completed");
          SmartDashboard.putString("[AUTO] Current Auto Status", "Completed");
        })
      );
    }, Set.of()); // Pass an empty set of requirements
  }
}
