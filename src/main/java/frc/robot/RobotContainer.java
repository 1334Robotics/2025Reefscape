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
import frc.robot.commands.mailbox.ForceFeedCommand;
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
import java.util.List;
import java.io.IOException;
import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

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
  private final POVButton      elevatorUpButton        = new POVButton(operatorController,      RobotContainerConstants.ELEVATOR_UP_BUTTON);
  private final POVButton      elevatorDownButton      = new POVButton(operatorController,      RobotContainerConstants.ELEVATOR_DOWN_BUTTON);
  private final JoystickButton flopperUpButton         = new JoystickButton(operatorController, RobotContainerConstants.FLOPPER_UP_BUTTON);
  private final JoystickButton flopperDownButton       = new JoystickButton(operatorController, RobotContainerConstants.FLOPPER_DOWN_BUTTON);
  private final JoystickButton mailboxShootButton      = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_SHOOT_BUTTON);
  private final JoystickButton mailboxFeedButton      = new JoystickButton(operatorController, RobotContainerConstants.MAILBOX_FEED_BUTTON);

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
  public static final TrackAprilTagCommand trackCommand = new TrackAprilTagCommand(0,
                                                                                   new Distance(VisionConstants.TRACK_TAG_X,
                                                                                                VisionConstants.TRACK_TAG_Y));
  public static final ClimbSubsystem climbSubsystem                 = new ClimbSubsystem();

  //Conditionally create SimulationSubsystem
  //public final SimulationSubsystem simulationSubsystem;

  // Auto chooser for PathPlanner
  private SendableChooser<Command> autoChooser;
  private SendableChooser<Command> pathChooser; // New path chooser for individual paths
  private SendableChooser<String> masterChooser;

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
    AutoConfigurer.configure();
    DriverStation.silenceJoystickConnectionWarning(true);//silence the warning about the joystick connection
    
    // Initialize all choosers
    autoChooser = new SendableChooser<>();
    pathChooser = new SendableChooser<>();
    masterChooser = new SendableChooser<>();
    
    // Set up autonomous options and path chooser
    configureAutoChooser();

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

    SmartDashboard.putData("[ELEVATOR] L1", new ElevatorGotoL1Command());
    SmartDashboard.putData("[ELEVATOR] L2", new ElevatorGotoL2Command());
    SmartDashboard.putData("[ELEVATOR] L3", new ElevatorGotoL3Command());
    SmartDashboard.putData("[ELEVATOR] L4", new ElevatorGotoL4Command());

    SmartDashboard.putString("Current Path", "None");

    // List available paths for manual selection
    listAvailablePaths();
  }

  private void configureAutoChooser() {
    // Set up master chooser to select between auto routines and individual paths
    masterChooser = new SendableChooser<>();
    masterChooser.setDefaultOption("Use Auto Routines", "AUTO");
    masterChooser.addOption("Use Individual Paths", "PATH");
    SmartDashboard.putData("Chooser Selection", masterChooser);
    
    // Build an auto chooser using PathPlanner's built-in method
    // This will automatically find all autos in deploy/pathplanner/autos
    autoChooser = AutoBuilder.buildAutoChooser();
    
    // Override the default option to be "Do Nothing"
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    
    // Put on dashboard with descriptive name
    SmartDashboard.putData("Full Auto Routines", autoChooser);
    
    // Create a chooser for individual paths
    pathChooser = new SendableChooser<>();
    pathChooser.setDefaultOption("Do Nothing", new InstantCommand());
    
    // Add all paths from the paths directory
    try {
      File pathsDir = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
      if (pathsDir.exists() && pathsDir.isDirectory()) {
        File[] pathFiles = pathsDir.listFiles((dir, name) -> name.endsWith(".path"));
        if (pathFiles != null) {
          for (File file : pathFiles) {
            String pathName = file.getName().replace(".path", "");
            // Create a command that follows just this path
            Command pathCommand = getPathCommand(pathName);
            pathChooser.addOption(pathName, pathCommand);
          }
        }
      }
    } catch (Exception e) {
      System.err.println("Error configuring path chooser: " + e.getMessage());
    }
    
    // Put the path chooser on the dashboard
    SmartDashboard.putData("Individual Path Selector", pathChooser);
    
    // List available paths for reference
    listAvailablePaths();
  }
  
  /**
   * Get a command to follow a specific path
   * 
   * @param pathName The name of the path to follow
   * @return A command that follows the specified path
   */
  private Command getPathCommand(String pathName) {
    try {
      // Load path from file
      System.out.println("Attempting to load path: " + pathName);
      final String cleanPathName;
      if (pathName.endsWith(".path")) {
        cleanPathName = pathName.substring(0, pathName.length() - 5);
      } else {
        cleanPathName = pathName;
      }
      
      PathPlannerPath path = PathPlannerPath.fromPathFile(cleanPathName);
      
      if (path == null) {
        System.err.println("ERROR: Path was null for: " + cleanPathName);
        return new InstantCommand(() -> System.out.println("Path was null"));
      }
      
      // Create command to follow the path
      return AutoBuilder.followPath(path).finallyDo(() -> {
        // Log when path completes
        System.out.println("Path " + cleanPathName + " completed");
      });
    } catch (Exception e) {
      System.err.println("Error loading path '" + pathName + "': " + e.getMessage());
      e.printStackTrace(); // Print the stack trace for more detailed debugging
      return new InstantCommand(() -> System.out.println("Path loading failed: " + e.getMessage()));
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
    elevatorUpButton.whileTrue(new ElevatorUpCommand());
    elevatorDownButton.whileTrue(new ElevatorDownCommand());
    flopperUpButton.whileTrue(new FlopperUpCommand());
    flopperDownButton.whileTrue(new FlopperDownCommand());
    mailboxShootButton.onTrue(new ShootCommand());
    mailboxFeedButton.onTrue(new ForceFeedCommand());
    
    // Add a button to test the currently selected path
    // Modify this to use any available button on your controller - this example uses Y button
    //new JoystickButton(driverController, XboxController.Button.kY.value) //uncomment this to test paths with Y button on controller
    //    .onTrue(runSelectedPath());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {//auto path selection via chooser
    String selection = masterChooser.getSelected();
    
    if (selection != null && selection.equals("PATH")) {
      // Use the path chooser
      System.out.println("Using PATH chooser for autonomous");
      return pathChooser.getSelected();
    } else {
      // Use the auto chooser (default)
      System.out.println("Using AUTO chooser for autonomous");
      return autoChooser.getSelected();
    }
  }

  public Command getAutonomousPathCommand() {//manual path selection via path name 
    // Return the command selected from the path chooser
    return pathChooser.getSelected();
  }

  /**
   * Get an autonomous command for a specific named auto routine
   * 
   * @param autoName The name of the auto routine to run
   * @return The command to run the specified auto
   */
  public Command getAutonomousCommandByName(String autoName) {
    try {
      return new PathPlannerAuto(autoName);
    } catch (Exception e) {
      System.err.println("Error loading auto '" + autoName + "': " + e.getMessage());
      return Commands.none();
    }
  }

  /**
   * Get a command to execute the currently selected path from the path chooser
   * This is useful for testing paths during teleop
   * 
   * @return Command to execute the selected path
   */
  public Command runSelectedPath() {
    return pathChooser.getSelected();
  }
}
