package team5427.frc.robot.io;

import team5427.frc.robot.Constants.BlinkinConstants;
import team5427.frc.robot.SuperStructureEnum.CascadeStates;
import team5427.frc.robot.subsystems.LightsSubsystem;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;

public class LightTriggers {
    CascadeSubsystem cascadeSubsystem = CascadeSubsystem.getInstance();
    LightsSubsystem lights = LightsSubsystem.getInstance();

    @SuppressWarnings("static-access")
    public LightTriggers() {
        //!Cascade
        CascadeStates state = cascadeSubsystem.state;
        switch (state) {
            case CLIMBING:
                lights.setPattern(BlinkinConstants.kCp1Strobe);
                break;
            case SCORING:
                lights.setPattern(BlinkinConstants.kLawnGreen);
                break;
            case SHOOTING:
                lights.setPattern(BlinkinConstants.kCp2LarsonScanner);
                break;
            default:
                lights.setPattern(BlinkinConstants.kBreath);
                break;
        }
    }
}
