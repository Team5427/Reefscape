package team5427.frc.robot.subsystems.ProngEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants.ProngEffectorConstants;
import team5427.frc.robot.SuperStructureEnum.ProngEffectorStates;
import team5427.frc.robot.subsystems.ProngEffector.io.ProngIO;
import team5427.frc.robot.subsystems.ProngEffector.io.ProngIOInputsAutoLogged;

public class ProngSubsystem extends SubsystemBase {

  private ProngIO io;
  private ProngIOInputsAutoLogged inputsAutoLogged = new ProngIOInputsAutoLogged();

  private Rotation2d wristSetpoint;
  private LinearVelocity rollerVelocity;

  private static ProngSubsystem m_instance;

  public static ProngEffectorStates state;

  public static ProngSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ProngSubsystem();
    }
    return m_instance;
  }

  public void setWristSetpoint(Rotation2d setpoint) {
    this.wristSetpoint = setpoint;
  }

  public void setRollerSetpoint(LinearVelocity rollerVelocity) {
    this.rollerVelocity = rollerVelocity;
  }

  public Rotation2d getWristSetpoint() {
    return wristSetpoint;
  }

  public LinearVelocity getRollerSetpoint() {
    return rollerVelocity;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputsAutoLogged);

    double rollerCurrentFiltered =
        ProngEffectorConstants.kIntakeFilter.calculate(inputsAutoLogged.rollerCurrent.in(Amps));

    if (state == ProngEffectorStates.CORAL_INTAKE) {
      if (rollerCurrentFiltered > ProngEffectorConstants.kIntakeMaxCurrent.in(Amps)
          && inputsAutoLogged.rollerAcceleration.in(MetersPerSecondPerSecond) <= 1.0) {

        state = ProngEffectorStates.CORAL_STOWED;
        Info.coralIntaked.set(true);
      } else {
        Info.coralIntaked.set(false);
      }
    } else if (state == ProngEffectorStates.ALGAE_INTAKE) {
      if (rollerCurrentFiltered > ProngEffectorConstants.kIntakeMaxCurrent.in(Amps)
          && inputsAutoLogged.rollerAcceleration.in(MetersPerSecondPerSecond) <= 1.0) {
        state = ProngEffectorStates.ALGAE_STOWED;
        Info.algaeIntaked.set(true);
      } else {
        Info.algaeIntaked.set(false);
      }
    }

    if (Math.abs(rollerVelocity.in(MetersPerSecond))
        > ProngEffectorConstants.kRollerConfiguration.maxVelocity) {
      Errors.rollerConstraint.set(true);
    } else {
      io.setRollerSpeeds(rollerVelocity);
    }

    if (wristSetpoint.getDegrees() > ProngEffectorConstants.kMaxPivotAngle.getDegrees()
        || wristSetpoint.getDegrees() < ProngEffectorConstants.kMinPivotAngle.getDegrees()) {
      Errors.pivotConstraint.set(true);
    } else {
      io.setWristSetpoint(wristSetpoint);
    }

    Logger.processInputs("ProngEffector/Inputs", inputsAutoLogged);
  }

  private static class Errors {
    public static Alert rollerConstraint =
        new Alert(
            "Constraint Violations",
            "Prong Effector Roller given a setpoint outside its bounds. ",
            AlertType.kError);
    public static Alert pivotConstraint =
        new Alert(
            "Constraint Violations",
            "Prong Effector Pivot given a setpoint outside its bounds. ",
            AlertType.kError);
  }

  private static class Info {
    public static Alert coralIntaked = new Alert("Intake", "Coral Intaked", AlertType.kInfo);
    public static Alert algaeIntaked = new Alert("Intake", "Algae Intaked", AlertType.kInfo);
  }
}
