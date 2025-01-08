package team5427.frc.robot;

public enum SuperStructureEnum {
    ENABLED,
    DISABLED;
    public enum IntakeStates {
        INACTIVE,
        INTAKING;
        public enum CoralIntake{
            STATION,
            GROUND;
        }
        public enum AlgaeIntake {
            REEF,
            GROUND;
        }
    }
    public enum OutakingStates {
        INACTIVE,
        OUTAKING;
        public enum CoralOutake{
            L1,
            L2,
            L3,
            L4,
            EJECT;
        }
        public enum AlgaeOutake {
            PROCESSOR,
            SHOOTING,
            EJECT;
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
