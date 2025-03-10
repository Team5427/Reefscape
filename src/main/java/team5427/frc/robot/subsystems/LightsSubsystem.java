package team5427.frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5427.frc.robot.Constants.BlinkinConstants;

public class LightsSubsystem extends SubsystemBase{
    private Spark blinkin;
    private double pattern;

    private static LightsSubsystem m_instance;

    public LightsSubsystem() {
        blinkin = new Spark(BlinkinConstants.kBlinkinChannel);
    }

    @Override
    public void periodic() {
        blinkin.set(pattern);
        Logger.recordOutput("Blinkin PWM Input", blinkin.getPwmHandle());
    }

    public void setPattern(double pattern) {
        this.pattern = pattern;
    }

    public static LightsSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new LightsSubsystem();
        }
        return m_instance;
    }
}