package frc.robot.subsystems.gearbox;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GearboxConstants;


public class GearboxSubsystem extends SubsystemBase {
  private final SparkMax sparkMax_1;
  private final SparkMax sparkMax_2;
  private int state; // 0 = Unknown, 1 = Forward, 2 = Backward

  public GearboxSubsystem() {
    this.sparkMax_1 = new SparkMax(GearboxConstants.DEVICE_ID_1, MotorType.kBrushless);
    this.sparkMax_2 = new SparkMax(GearboxConstants.DEVICE_ID_2, MotorType.kBrushless);
    this.state = 0;
  }

  public GearboxSubsystem(int deviceId_1, int deviceId_2) {
    this.sparkMax_1 = new SparkMax(deviceId_1, MotorType.kBrushless);
    this.sparkMax_2 = new SparkMax(deviceId_2, MotorType.kBrushless);
    this.state = 0;
  }

  public void forward() {
    this.state = 1;
  }

  public void backward() {
    this.state = 2;
  }

  @Override
  public void periodic() {
    switch(state) {
      case 0:
        SmartDashboard.putString("[GEARBOX] State", "Unknown");
        this.sparkMax_1.set(0);
        this.sparkMax_2.set(0);
        break;
      case 1:
        this.sparkMax_1.set(-1 * GearboxConstants.forwardSpeed); // Times negative one as motors turn in opposite directions at this angle
        this.sparkMax_2.set(GearboxConstants.forwardSpeed);
        SmartDashboard.putString("[GEARBOX] State", "Forward");
        break;
      case 2:
        this.sparkMax_1.set(-1 * GearboxConstants.backwardSpeed); // Times negative one as motors turn in opposite directions at this angle
        this.sparkMax_2.set(GearboxConstants.backwardSpeed);
        SmartDashboard.putString("[GEARBOX] State", "Backward");
        break;
    }
  }
}