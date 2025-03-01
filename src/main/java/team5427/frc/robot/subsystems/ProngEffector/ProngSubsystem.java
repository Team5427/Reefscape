package team5427.frc.robot.subsystems.ProngEffector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5427.frc.robot.subsystems.ProngEffector.io.ProngIO;
import team5427.frc.robot.subsystems.ProngEffector.io.ProngIOInputsAutoLogged;

public class ProngSubsystem extends SubsystemBase {

  private ProngIO io;
  private ProngIOInputsAutoLogged inputsAutoLogged;

  private Rotation2d wristSetpoint;
  private LinearVelocity rollerVelocity;

  private static ProngSubsystem m_instance;

  public static ProngSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ProngSubsystem();
    }
    return m_instance;
  }
}
