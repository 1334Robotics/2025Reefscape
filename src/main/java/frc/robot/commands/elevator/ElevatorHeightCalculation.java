package frc.robot.commands.elevator;
import frc.robot.constants.ElevatorConstants;

//The Maxium Height of the Primary Reach is 48 inches
//The Maximum Height of the Secondary Reach is 24 inches
//L4 Height is 72 inches, so utilizing the /3 techniques will allow the primary reach and secondary reach to reach L4
public enum ElevatorHeightCalculation {
    L1(ElevatorConstants.L1_HEIGHT_INCHES * (2.0 / 3), ElevatorConstants.L1_HEIGHT_INCHES / 3),
    L2(ElevatorConstants.L2_HEIGHT_INCHES * (2.0 / 3), ElevatorConstants.L2_HEIGHT_INCHES / 3),
    L3(ElevatorConstants.L3_HEIGHT_INCHES * (2.0 / 3), ElevatorConstants.L3_HEIGHT_INCHES / 3),
    L4(ElevatorConstants.L4_HEIGHT_INCHES * (2.0 / 3), ElevatorConstants.L4_HEIGHT_INCHES / 3);

    private final double targetPrimaryHeight;
    private final double targetSecondaryHeight;

    ElevatorHeightCalculation(double targetPrimaryHeight, double targetSecondaryHeight) {
        this.targetPrimaryHeight = targetPrimaryHeight;
        this.targetSecondaryHeight = targetSecondaryHeight;
    }

    public double getTargetPrimaryHeight() {
        return this.targetPrimaryHeight;
    }

    public double getTargetSecondaryHeight() {
        return this.targetSecondaryHeight;
    }
}