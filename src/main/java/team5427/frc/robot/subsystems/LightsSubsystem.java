package team5427.frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants.BlinkinConstants;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

public class LightsSubsystem extends SubsystemBase {

  private Spark blinkin;
  private double pattern;

  private ProngSubsystem prongSubsystem;

  private static LightsSubsystem m_instance;

  public static LightsSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new LightsSubsystem();
    }
    return m_instance;
  }

  private LightsSubsystem() {
    blinkin = new Spark(BlinkinConstants.kBlinkinChannel);

    prongSubsystem = ProngSubsystem.getInstance();
  }

  @Override
  public void periodic() {

    // if (ProngSubsystem.task == EETask.INTAKING) {
    //   if (ProngSubsystem.gamePieceMode == GamePieceMode.ALGAE) {
    //     pattern = BlinkinConstants.kAqua;
    //   } else {
    //     pattern = BlinkinConstants.kWhite;
    //   }
    // }

    // if (ProngSubsystem.task == EETask.EJECTING) {
    //   if (CascadeSubsystem.getInstance().cascadeAtGoal()) {
    //     pattern = BlinkinConstants.kCp1BreathFast;
    //   }
    // }

    // if (DriverStation.isDisabled()) {
    //   pattern = BlinkinConstants.kLightChaseRed;
    // }

    // blinkin.set(pattern);
    // Logger.recordOutput("Blinkin PWM Input", blinkin.getPwmHandle());
  }

  public void setPattern(double pattern) {
    this.pattern = pattern;
  }

  public double getPattern() {
    return pattern;
  }
}
