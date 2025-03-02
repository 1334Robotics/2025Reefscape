// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import frc.robot.utils.RobotMode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;

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
    // Dashboard keys
    private static final String ENABLE_VERBOSE_KEY = "Enable Verbose Logging";
    private static final String SIMULATION_MODE_KEY = "Simulation Mode";
    private static final String COMPETITION_MODE_KEY = "Competition Mode";
    
    // Status messages
    private static final String ROBOT_MODE_MSG = "Running on REAL ROBOT - Debug features disabled by default";
    private static final String SIM_MODE_MSG = "Running in SIMULATION - Debug features enabled";
    private static final String VERBOSE_ENABLED_MSG = "Verbose logging ENABLED - this may impact performance";
    private static final String VERBOSE_DISABLED_MSG = "Verbose logging DISABLED";
    private static final String COMPETITION_ENABLED_MSG = "COMPETITION MODE ENABLED - All debug features disabled";
    
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private final Field2d m_field = new Field2d();
    private long lastFieldUpdateTime = 0;
    private static final long FIELD_UPDATE_INTERVAL_MS = 100; // 10 times per second in simulation

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
        
        configureRobotMode();
        
        // Configure competition auto paths with descriptive names
        RobotMode.setCompetitionAuto(1, "TopAuto");    // Top position - follows AutoDrive20 then shoots
        RobotMode.setCompetitionAuto(2, "MidAuto");    // Middle position - follows AutoDrive22 then shoots
        RobotMode.setCompetitionAuto(3, "BottomAuto"); // Bottom position - follows Audodrive22 then shoots
    }
    
    /**
     * Configure robot mode based on whether we're in simulation or on the real robot
     */
    private void configureRobotMode() {
        if (RobotMode.isRealRobot()) {
            // On the real robot, disable debug features to save memory and processing power
            DriverStation.reportWarning(ROBOT_MODE_MSG, false);
            
            // Add toggles for verbose logging and competition mode
            SmartDashboard.putBoolean(ENABLE_VERBOSE_KEY, false);
            SmartDashboard.putBoolean(COMPETITION_MODE_KEY, false);
        } else {
            // In simulation, enable all debug features
            DriverStation.reportWarning(SIM_MODE_MSG, false);
            
            // Debug features are already enabled by default in simulation
            // Just add dashboard indicators
            SmartDashboard.putBoolean(SIMULATION_MODE_KEY, true);
            SmartDashboard.putBoolean(COMPETITION_MODE_KEY, false);
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
        
        // Throttle field updates in simulation mode to reduce network traffic
        if (RobotMode.isRealRobot() || !RobotMode.isCompetitionMode()) {
            long currentTime = System.currentTimeMillis();
            if (RobotMode.isRealRobot() || currentTime - lastFieldUpdateTime >= FIELD_UPDATE_INTERVAL_MS) {
                m_field.setRobotPose(RobotContainer.swerveSubsystem.getPose());
                if (RobotMode.isSimulation()) {
                    lastFieldUpdateTime = currentTime;
                }
            }
        }
        
        // Call periodic method in RobotContainer to handle dashboard buttons
        m_robotContainer.periodic();
        
        // Check dashboard toggles
        checkVerboseLoggingToggle();
        checkCompetitionModeToggle();
    }
    
    /**
     * Check if verbose logging was toggled on the dashboard (only for real robot)
     */
    private void checkVerboseLoggingToggle() {
        if (RobotMode.isRealRobot()) {
            boolean enableVerboseLogging = SmartDashboard.getBoolean(ENABLE_VERBOSE_KEY, false);
            
            if (enableVerboseLogging && !RobotMode.isVerboseLoggingEnabled()) {
                RobotMode.enableVerboseLogging();
                DriverStation.reportWarning(VERBOSE_ENABLED_MSG, false);
            } else if (!enableVerboseLogging && RobotMode.isVerboseLoggingEnabled()) {
                RobotMode.disableVerboseLogging();
                DriverStation.reportWarning(VERBOSE_DISABLED_MSG, false);
            }
        }
    }
    
    /**
     * Check if competition mode was toggled on the dashboard
     */
    private void checkCompetitionModeToggle() {
        boolean enableCompetitionMode = SmartDashboard.getBoolean(COMPETITION_MODE_KEY, false);
        
        if (enableCompetitionMode && !RobotMode.isCompetitionMode()) {
            RobotMode.enableCompetitionMode();
            DriverStation.reportWarning(COMPETITION_ENABLED_MSG, false);
        } else if (!enableCompetitionMode && RobotMode.isCompetitionMode()) {
            RobotMode.disableCompetitionMode();
        }
    }
    
    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        SimulatedArena.getInstance();
        
        // Set initial robot pose for simulation
        edu.wpi.first.math.geometry.Pose2d initialPose = new edu.wpi.first.math.geometry.Pose2d(
            7.036, // X position in meters
            7.036, // Y position in meters
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-161.026) // Heading in degrees
        );
        RobotContainer.swerveSubsystem.resetOdometry(initialPose);
        m_field.setRobotPose(initialPose);
        
        // Set blue alliance for simulation
        SmartDashboard.putString("Alliance", "Blue");
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
        // Log game piece positions
        Logger.recordOutput("FieldSimulation/Algae", 
        SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral", 
        SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
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