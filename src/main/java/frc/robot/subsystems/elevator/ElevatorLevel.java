package frc.robot.subsystems.elevator;

import frc.robot.constants.ElevatorConstants;

public enum ElevatorLevel {
    L1(ElevatorConstants.L1_POSITION),
    L2(ElevatorConstants.L2_POSITION),
    L3(ElevatorConstants.L3_POSITION),
    L4(ElevatorConstants.L4_POSITION);

    public final double position;
    private ElevatorLevel(double position) {
        this.position = position;
    }
}
