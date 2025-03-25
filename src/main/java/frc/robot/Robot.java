// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.simulation.AIRobotInSimulation;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

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
      // Get the default instance of the simulation world
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
  }
   /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    SimulatedArena.getInstance();
    AIRobotInSimulation.startOpponentRobotSimulations();
    // Use m_robotContainer instead of robotContainer
    if (m_robotContainer.simulationSubsystem != null) {
      System.out.println("DEBUG: runIntake() called in simulationInit()");
      m_robotContainer.simulationSubsystem.runIntake();
    }
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());
    Logger.recordOutput("Power/TotalCurrent", RobotController.getInputCurrent());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.swerveSubsystem.setFieldRelative(false);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    RobotContainer.swerveSubsystem.setFieldRelative(true);
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