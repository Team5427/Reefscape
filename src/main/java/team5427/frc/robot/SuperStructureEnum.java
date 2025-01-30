package team5427.frc.robot;

public enum SuperStructureEnum {
  ENABLED,
  DISABLED;

  public enum EndEffectorStates {
    ALGAE_INTAKE,
    ALGAE_OUTAKE,
    CORAL_INTAKE,
    CORAL_OUTAKE,
    DISABLED,
    IDLE;
  }

  public enum ClimbingStates {
    INACTIVE,
    SHALLOW,
    DEEP;
  }

  public enum DrivingStates {
    INACTIVE,
    NORMAL,
    DEFENSE,
    AUTONOMOUS,
    /** Gyro Locked for alignment */
    LOCKED,
    /** Driver Assist adjust by itself to align automatically */
    ASSIST;
  }
}
