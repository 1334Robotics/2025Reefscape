package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PID;  // Assuming the PID class is located in the same package or imported

// Need photon vision data

public class PIDSubsystem extends SubsystemBase {
  
  private PID pidTheta,pidY;  // PID controller for theta and Y, could add X later
  
  private double setPointTheta = 0.0;
  private double setPointY = 0.0;
  
  
  public AprilTagTrackingSubsystem() {
    pidY = new PID(0.1, 0.01, 0.0, 0.0, -1.0, 1.0, -0.5, 0.5, 0.02);  // Example PID parameters
    pidTheta = new PID(0.1, 0.01, 0.0, 0.0, -1.0, 1.0, -0.5, 0.5, 0.02);
  }
  
  @Override
  public void periodic() {
    // Get data from PhotonVision
    pidY.update(setPointY, y);
    pidTheta.update(setPointTheta, theta);

    double outputY = pidY.getSteer();
    double outputTheta = pidTheta.getSteer();
    
      // Use the outputs to control the robot's movement with driveSubsystem
    }
  }


  @Override
  public void simulationPeriodic() {
    // Called during simulation
  }
}
