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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.flopper.FlopperSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private final JoystickButton runAutoButton           = new JoystickButton(driverController,   RobotContainerConstants.RUN_AUTO_BUTTON);
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
          SmartDashboard.putString("Auto Files (Reference Only)", autosList.toString());
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
    
    // Register named commands BEFORE configuring PathPlanner
    System.out.println("1. Registering commands for PathPlanner...");
    registerNamedCommands();
    
    // Configure PathPlanner after commands are registered
    System.out.println("2. Configuring PathPlanner...");
    swerveSubsystem.setupPathPlanner();
    
    // Initialize and configure auto chooser - MUST come after PathPlanner configuration
    System.out.println("3. Configuring Auto Chooser...");
    configureAutoChooser();
    
    // Verify auto chooser was created
    if (autoChooser == null) {
        System.err.println("ERROR: Auto chooser is null after configuration!");
    } else {
        System.out.println("Auto chooser configured successfully");
    }
    
    // Preload all autonomous routines
    System.out.println("4. Preloading autonomous routines...");
    preloadAutonomousRoutines();

    // Configure the trigger bindings
    System.out.println("5. Configuring button bindings...");
    configureBindings();

    // Set up dashboard information
    System.out.println("6. Setting up dashboard information...");
    setupDashboard();
    
    // Configure default commands
    System.out.println("7. Setting up default commands...");
    setupDefaultCommands();
    
    System.out.println("RobotContainer initialization complete!");
  }

  private void setupDashboard() {
    // Verify auto chooser exists before putting to dashboard
    if (autoChooser != null) {
        SmartDashboard.putData("Auto Routines", autoChooser);
        System.out.println("Auto chooser added to dashboard");
    } else {
        System.err.println("ERROR: Cannot add null auto chooser to dashboard!");
    }
    
    SmartDashboard.putString("Current Auto Status", "Ready (Press START button to run)");
    SmartDashboard.putString("Auto Button Help", "Press START button on driver controller to run selected auto");
    
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
      SmartDashboard.putData("Auto Routines", autoChooser);
      System.out.println("Auto chooser configured and added to dashboard");
      
      // List available autos in the terminal/log for debugging
      listAvailableAutos();
      
    } catch (Exception e) {
      System.err.println("Error configuring auto chooser: " + e.getMessage());
      e.printStackTrace();
    }
  }
  
  /**
   * Get a command to follow a specific path
   * 
   * @param pathName The name of the path to follow
   * @return A command that follows the specified path
   */
  private Command getPathCommand(String pathName) {
    try {
      // Load path from file or use cached version
      final String cleanPathName = pathName.endsWith(".path") 
          ? pathName.substring(0, pathName.length() - 5) 
          : pathName;
      
      // Check path cache first
      PathPlannerPath path;
      if (pathCache.containsKey(cleanPathName)) {
        path = pathCache.get(cleanPathName);
        System.out.println("Using cached path: " + cleanPathName);
      } else {
        System.out.println("Loading path: " + cleanPathName);
        path = PathPlannerPath.fromPathFile(cleanPathName);
        
        // Cache the path if valid
        if (path != null) {
          pathCache.put(cleanPathName, path);
          
          // Limit cache size to prevent memory issues
          if (pathCache.size() > 10) {
            String oldestPath = pathCache.keySet().iterator().next();
            pathCache.remove(oldestPath);
            System.out.println("Removed from cache: " + oldestPath);
          }
        }
      }
      
      if (path == null) {
        System.err.println("ERROR: Path was null for: " + cleanPathName);
        SmartDashboard.putString("Path Error", "Failed to load: " + cleanPathName);
        return Commands.none().withName("NullPathHandler");
      }
      
      // Create a sequence with better error handling
      return Commands.sequence(
        // Log start
        Commands.runOnce(() -> {
          System.out.println("Starting path: " + cleanPathName);
          SmartDashboard.putString("Current Path", cleanPathName);
        }),
        // Follow path with timeout
        AutoBuilder.followPath(path)
          .withTimeout(15) // Safety timeout
          .handleInterrupt(() -> {
            System.out.println("Path interrupted: " + cleanPathName);
            SmartDashboard.putString("Path Status", "Interrupted");
          }),
        // Log completion
        Commands.runOnce(() -> {
          System.out.println("Path completed: " + cleanPathName);
          SmartDashboard.putString("Current Path", "None");
          SmartDashboard.putString("Path Status", "Completed");
        })
      ).withName("Path_" + cleanPathName);
    } catch (Exception e) {
      System.err.println("Error loading path '" + pathName + "': " + e.getMessage());
      e.printStackTrace(); // Print the stack trace for more detailed debugging
      SmartDashboard.putString("Path Error", e.getMessage());
      return Commands.none().withName("ErrorPathHandler");
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

    // Configure elevator controls based on mode
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
    
    // Add binding for running selected auto routine
    runAutoButton.onTrue(runSelectedAuto());
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
      return trackCommand;
    }
    
    System.out.println("Running selected auto routine: " + selectedCommand.getName());
    return selectedCommand;
  }

  /**
   * Updates dashboard indicators for current alliance and FMS position
   * Called periodically from Robot to ensure indicators stay current
   */
  public void updateAllianceIndicators() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      boolean isRed = alliance.get() == DriverStation.Alliance.Red;
      SmartDashboard.putBoolean("Is Red Alliance", isRed);
      SmartDashboard.putString("Current Alliance", isRed ? "RED" : "BLUE");
      
      // Get and display FMS position if available
      int location = DriverStation.getLocation().orElse(2); // Default to center (2)
      SmartDashboard.putNumber("FMS Position", location);
    } else {
      SmartDashboard.putBoolean("Is Red Alliance", false);
      SmartDashboard.putString("Current Alliance", "UNKNOWN");
      SmartDashboard.putNumber("FMS Position", 2); // Default to center
    }
  }

  /**
   * Register named commands for use in PathPlanner auto routines
   */
  private void registerNamedCommands() {
    System.out.println("Starting named command registration...");
    
    try {
      // Create a map to store commands for verification
      Map<String, Command> commandMap = new HashMap<>();
      
      // Elevator Commands
      commandMap.put("ElevatorBottom", new ElevatorGotoBottomCommand());
      commandMap.put("ElevatorFeed", new ElevatorGotoFeedCommand());
      commandMap.put("ElevatorL1", new ElevatorGotoL1Command());
      commandMap.put("ElevatorL2", new ElevatorGotoL2Command());
      commandMap.put("ElevatorL3", new ElevatorGotoL3Command());
      commandMap.put("ElevatorL4", new ElevatorGotoL4Command());
      commandMap.put("ElevatorUp", new ElevatorUpCommand());
      commandMap.put("ElevatorDown", new ElevatorDownCommand());
      
      // Mailbox Commands
      commandMap.put("Shoot", new ShootCommand());
      commandMap.put("Feed", new MailboxFeedCommand());
      commandMap.put("Stop", new StopCommand());
      commandMap.put("MailboxRewind", new MailboxRewindCommand());
      commandMap.put("OutputHigh", new OutputHighCommand());
      commandMap.put("OutputLow", new OutputLowCommand());
      
      // Flopper Commands
      commandMap.put("FlopperUp", new FlopperUpCommand());
      commandMap.put("FlopperDown", new FlopperDownCommand());
      
      // Other Commands
      commandMap.put("ZeroGyro", new GyroZeroCommand());
      
      // Register all commands and verify
      System.out.println("\nRegistering commands with PathPlanner:");
      for (Map.Entry<String, Command> entry : commandMap.entrySet()) {
        String name = entry.getKey();
        Command command = entry.getValue();
        
        try {
          // Register command without asProxy to maintain direct subsystem requirements
          NamedCommands.registerCommand(name, command);
          System.out.println("✓ Registered: " + name);
          
          // Verify command requirements
          Set<edu.wpi.first.wpilibj2.command.Subsystem> requirements = command.getRequirements();
          if (requirements.isEmpty()) {
            System.out.println("  Warning: " + name + " has no subsystem requirements!");
          } else {
            System.out.println("  Requirements: " + requirements);
          }
        } catch (Exception e) {
          System.err.println("✗ Failed to register: " + name);
          System.err.println("  Error: " + e.getMessage());
        }
      }
      
      System.out.println("\nCommand registration complete!");
      System.out.println("Total commands registered: " + commandMap.size());
      
    } catch (Exception e) {
      System.err.println("ERROR during command registration: " + e.getMessage());
      e.printStackTrace();
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
      
      SmartDashboard.putString("Available Autos", autosList.toString());
      
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
        SmartDashboard.putString("Current Auto Status", "No auto routine selected");
        return Commands.none();
      }
      
      System.out.println("Running selected auto routine via button press");
      
      return Commands.sequence(
        Commands.runOnce(() -> {
          CommandScheduler.getInstance().cancelAll();
          SmartDashboard.putString("Current Auto Status", "Running Auto");
        }),
        selectedAuto,
        Commands.runOnce(() -> {
          System.out.println("Auto routine completed");
          SmartDashboard.putString("Current Auto Status", "Completed");
        })
      );
    }, Set.of()); // Pass an empty set of requirements
  }
}
