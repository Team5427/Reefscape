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

    public enum EndEffectorLockedStates {
      ALGAE_ROLLER,
      PIVOT,
      CORAL_ROLLER,
      WRIST;
    }
  }

  public enum CascadeStates {
    STATION_INTAKE,
    REEF_INTAKE,
    GROUND_INTAKE,
    CLIMBING,
    SCORING,
    SHOOTING,
    PROCESSING,
    IDLE;

    public enum CasacdeLockedStates {
      CASCADE,
      PIVOT;
    }
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
