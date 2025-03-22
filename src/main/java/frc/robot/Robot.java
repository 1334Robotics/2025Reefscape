// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.elevator.ElevatorResetCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ElevatorConstants;
import com.pathplanner.lib.commands.PathPlannerAuto;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Virtual Machine is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the LoggedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the Main.java file in the project.
 *
 * This class extends LoggedRobot to enable advanced logging and simulation
 * capabilities. LoggedRobot is essential for the simulation to work properly, as it integrates with
 * the AdvantageKit logging framework to provide detailed data recording and playback functionality.
 * This is particularly useful for debugging, testing, and analyzing robot behavior in both
 * simulation and real-world scenarios.
 *
 * By using LoggedRobot, we gain access to features such as:
 * - Real-time logging of robot state, sensor data, and command execution.
 * - Playback of logged data for post-match analysis.
 * - Enhanced simulation support, including logging of simulated game pieces and field elements.
 */

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final Field2d m_field = new Field2d();

  private boolean elevatorReset = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Put the check to disable the elevator reset on TeleOp enable
    SmartDashboard.putBoolean("[ELEVATOR] Reset On TeleOp Enable", true);
  }
  @Override
  public void robotInit() {
      Logger.addDataReceiver(new NT4Publisher());
      Logger.start();
      SmartDashboard.putData("Field", m_field);
      
      // Check and log library versions
      logLibraryVersions();
  }

  /**
   * Logs the versions of important libraries to help with debugging
   */
  private void logLibraryVersions() {
    try {
      // Log PathPlanner version
      String pathPlannerVersion = "Unknown";
      try {
        // Try to get version using reflection to avoid direct dependency issues
        Class<?> ppLib = Class.forName("com.pathplanner.lib.PathPlannerLib");
        java.lang.reflect.Method getVersion = ppLib.getMethod("getVersion");
        pathPlannerVersion = (String) getVersion.invoke(null);
      } catch (Exception e) {
        pathPlannerVersion = "Error getting version";
      }
      
      System.out.println("=== Library Versions ===");
      System.out.println("PathPlanner: " + pathPlannerVersion);
      System.out.println("=====================");
      
      SmartDashboard.putString("Library/PathPlanner", pathPlannerVersion);
      
    } catch (Exception e) {
      System.err.println("Error logging library versions: " + e.getMessage());
    }

      // Setup tracking
      RobotContainer.trackCommand.disable();
      RobotContainer.trackCommand.schedule();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    RobotContainer.trackCommand.execute(); // THIS IS BAD. IT SHOULDN'T NEED TO DO THIS
    m_field.setRobotPose(RobotContainer.swerveSubsystem.getPose());
    
    // Update alliance indicators on dashboard
    m_robotContainer.updateAllianceIndicators();
  }
   /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    SimulatedArena.getInstance();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    
    // Debug alliance and position in simulation
    if (count++ % 50 == 0) { // Only log every ~1 second
      var alliance = DriverStation.getAlliance();
      String allianceStr = alliance.isPresent() ? alliance.get().toString() : "Unknown";
      Pose2d robotPose = RobotContainer.swerveSubsystem.getPose();
      SmartDashboard.putString("Simulation/Alliance", allianceStr);
      SmartDashboard.putString("Simulation/RobotPose", 
          String.format("X: %.2f, Y: %.2f, Rot: %.2f°", 
              robotPose.getX(), robotPose.getY(), 
              robotPose.getRotation().getDegrees()));
    }
    
    // Log game piece positions
    Logger.recordOutput("FieldSimulation/Algae", 
    SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    Logger.recordOutput("FieldSimulation/Coral", 
    SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
  }

  // Counter for periodic debug logs
  private int count = 0;

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Keep field-relative mode enabled for path following
    RobotContainer.swerveSubsystem.setFieldRelative(true);
    
    // Force automatic control for autonomous
    RobotContainer.elevatorHandler.setForceManualControl(false);
    
    // First, reset the elevator if it hasn't been reset yet
    if (!elevatorReset) {
      System.out.println("Autonomous Init: Zeroing elevator...");
      Command resetCommand = new ElevatorResetCommand();
      resetCommand.schedule();
      // Wait for the reset to complete (up to 2 seconds)
      int attempts = 0;
      while (!resetCommand.isFinished() && attempts < 100) {
        try {
          Thread.sleep(20);  // Wait 20ms between checks
          attempts++;
        } catch (InterruptedException e) {
          break;
        }
      }
      elevatorReset = true;
      System.out.println("Elevator zeroing completed");
    }
    
    // Set initial robot pose based on FMS data
    setInitialPose();
    
    // Small delay to ensure odometry reset is complete
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    
    // Enable tracking if needed
    RobotContainer.trackCommand.enable();
    
    // Get the autonomous command from RobotContainer
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // Schedule the autonomous command
    if (m_autonomousCommand != null) {
      System.out.println("Starting autonomous command: " + m_autonomousCommand.getName());
      m_autonomousCommand.schedule();
    } else {
      System.out.println("No autonomous command selected!");
    }
  }

  /**
   * Sets the initial robot pose based on FMS data
   * This method is called at the start of autonomous to ensure the robot has the
   * correct pose based on the latest alliance and FMS position settings.
   */
  private void setInitialPose() {
    // Get the selected autonomous command
    Command selectedCommand = m_robotContainer.getAutonomousCommand();
    
    // If we're using a PathPlanner auto, let PathPlanner handle the initial pose
    if (selectedCommand instanceof PathPlannerAuto) {
      System.out.println("Using PathPlanner auto - skipping manual pose initialization");
      return;
    }
    
    // Only set initial pose for non-PathPlanner autos
    var alliance = DriverStation.getAlliance();
    int location = DriverStation.getLocation().orElse(2); // Default to center (2) if not available
    
    // Default starting pose (Center, Blue alliance)
    Pose2d startingPose = new Pose2d(5.89, 5.5, Rotation2d.fromDegrees(180));
    
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        // Blue alliance positions
        switch (location) {
          case 1:   // Left
            startingPose = AutoConstants.BLUE_LEFT_STARTING_POSE;
            break;
          case 2:   // Center
            startingPose = AutoConstants.BLUE_CENTER_STARTING_POSE;
            break;
          case 3: // Right
            startingPose = AutoConstants.BLUE_RIGHT_STARTING_POSE;
            break;
        }
      } else {
        // Red alliance positions (mirrored across field center)
        switch (location) {
          case 1:   // Left
            startingPose = AutoConstants.RED_LEFT_STARTING_POSE;
            break;
          case 2:   // Center
            startingPose = AutoConstants.RED_CENTER_STARTING_POSE;
            break;
          case 3: // Right
            startingPose = AutoConstants.RED_RIGHT_STARTING_POSE;
            break;
        }
      }
    }
    
    // Log the pose we're setting
    System.out.println(String.format("Setting initial pose to: %s (Alliance: %s, Position: %d)",
        startingPose.toString(),
        alliance.isPresent() ? alliance.get().toString() : "Unknown",
        location));
    
    // Set the pose
    RobotContainer.swerveSubsystem.resetOdometry(startingPose);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.swerveSubsystem.setFieldRelative(true);
    RobotContainer.trackCommand.disable();
    
    // Restore manual control if that's what we're using
    if (ElevatorConstants.MANUAL_ELEVATOR_CONTROL) {
      RobotContainer.elevatorHandler.setForceManualControl(true);
    }
    
    // Reset elevator if needed
    if(SmartDashboard.getBoolean("[ELEVATOR] Reset On TeleOp Enable", false) && !this.elevatorReset) {
      (new ElevatorResetCommand()).schedule();
      this.elevatorReset = true;
    }
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

 
}