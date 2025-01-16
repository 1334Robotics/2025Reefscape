
package frc.robot.subsystems.solenoid;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SolenoidConstants;


public class SolenoidSubsystem extends SubsystemBase {
  private final SparkMax sparkMax;

  public SolenoidSubsystem() {
    this.sparkMax = new SparkMax(SolenoidConstants.DEVICE_ID, MotorType.kBrushed);
    SmartDashboard.putString("[SOLENOID] State", "Unknown");
  }

  public SolenoidSubsystem(int deviceId) {
    this.sparkMax = new SparkMax(deviceId, MotorType.kBrushed);
    SmartDashboard.putString("[SOLENOID] State", "Unknown");
  }

  public void extend() {
    this.sparkMax.set(1);
    SmartDashboard.putString("[SOLENOID] State", "Extended");
  }

  public void retract() {
    this.sparkMax.set(0);
    SmartDashboard.putString("[SOLENOID] State", "Retracted");
  }
}