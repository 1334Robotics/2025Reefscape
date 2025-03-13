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

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
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
    
    // Set initial robot pose based on alliance and position
    setInitialPose();
    
    // Get the selected autonomous command
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * Sets the initial robot pose based on alliance and selected starting position
   * This method is called at the start of autonomous to ensure the robot has the
   * correct pose based on the latest alliance and position settings, overriding
   * any initial pose set during robot construction.
   * 
   * This year the bots face the team so they need to start with a 180 degree rotation
   * so red is facing the red alliance and blue is facing the blue alliance
   * WE NEED TO ADD THE ROBOT OFFSET FROM CENTER TO BUMPER TO THE X VALUE BEFORE COMP!!! - TO DO
   * THESE NEED TO BE BUDDY CHECKED!!!! - TO DO 
   */
  private void setInitialPose() {
    var alliance = DriverStation.getAlliance();
    String position = m_robotContainer.getSelectedPosition();
    
    // Default starting pose (Center, Blue alliance)
    Pose2d startingPose = new Pose2d(5.89, 5.5, Rotation2d.fromDegrees(180));
    
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        // Blue alliance positions
        if (position.equals("Left")) {
          startingPose = new Pose2d(5.89, 7.0, Rotation2d.fromDegrees(180));
        } else if (position.equals("Right")) {
          startingPose = new Pose2d(5.89, 1.0, Rotation2d.fromDegrees(180));
        }
        // Center position uses default pose
      } else {
        // Red alliance positions (mirrored across field center)
        if (position.equals("Left")) {
          startingPose = new Pose2d(8.05, 1.0, Rotation2d.fromDegrees(0));
        } else if (position.equals("Right")) {
          startingPose = new Pose2d(8.05, 7.0, Rotation2d.fromDegrees(0));
        } else {
          // Center position
          startingPose = new Pose2d(8.05, 5.5, Rotation2d.fromDegrees(0));
        }
      }
    }
    
    // Log the starting position
    System.out.println("Setting initial pose to: " + startingPose + 
                      " (Alliance: " + (alliance.isPresent() ? alliance.get() : "Unknown") + 
                      ", Position: " + position + ")");
                      
    // Reset odometry to the starting pose
    RobotContainer.swerveSubsystem.resetOdometry(startingPose);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    RobotContainer.swerveSubsystem.setFieldRelative(true);
    // Have some flag to do this only once
    (new ElevatorResetCommand()).schedule();
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