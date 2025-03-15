package team5427.frc.robot.subsystems.ProngEffector;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.SuperStructureEnum.ProngEffectorStates;
import team5427.frc.robot.subsystems.ProngEffector.io.ProngIO;
import team5427.frc.robot.subsystems.ProngEffector.io.ProngIOInputsAutoLogged;
import team5427.frc.robot.subsystems.ProngEffector.io.ProngIOTalon;

public class ProngSubsystem extends SubsystemBase {

  public static enum GamePieceMode {
    CORAL,
    ALGAE
  }

  public static enum Level {
    FLOOR,
    LOW,
    HIGH
  }

  public static enum EETask {
    INTAKING,
    EJECTING
  }

  public static GamePieceMode gamePieceMode = GamePieceMode.CORAL;
  public static Level level = Level.LOW;
  public static EETask task = EETask.INTAKING;

  private ProngIO io;
  private ProngIOInputsAutoLogged inputsAutoLogged = new ProngIOInputsAutoLogged();

  @Getter @Setter private Rotation2d wristSetpoint;
  @Getter @Setter private LinearVelocity rollerVelocity;

  private static ProngSubsystem m_instance;

  public static ProngEffectorStates state;

  public static ProngSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ProngSubsystem();
    }
    return m_instance;
  }

  private ProngSubsystem() {
    switch (Constants.currentMode) {
      case REAL:
        io = new ProngIOTalon();
        break;

      default:
        break;
    }
    rollerVelocity = MetersPerSecond.of(-0.5);
    wristSetpoint = Rotation2d.kZero;
    state = ProngEffectorStates.IDLE;
  }

  public boolean hasObject() {
    return io.hasResistance();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputsAutoLogged);

    io.setRollerSpeeds(rollerVelocity);
    io.setWristSetpoint(wristSetpoint);

    // double rollerCurrentFiltered =
    //     ProngEffectorConstants.kIntakeFilter.calculate(inputsAutoLogged.rollerCurrent.in(Amps));

    // if (state == ProngEffectorStates.CORAL_INTAKE) {
    //   if (rollerCurrentFiltered > ProngEffectorConstants.kIntakeMaxCurrent.in(Amps)
    //       && inputsAutoLogged.rollerAcceleration.in(MetersPerSecondPerSecond) <= 1.0) {

    //     state = ProngEffectorStates.CORAL_STOWED;
    //     Info.coralIntaked.set(true);
    //   } else {
    //     Info.coralIntaked.set(false);
    //   }
    // } else if (state == ProngEffectorStates.ALGAE_INTAKE) {
    //   if (rollerCurrentFiltered > ProngEffectorConstants.kIntakeMaxCurrent.in(Amps)
    //       && inputsAutoLogged.rollerAcceleration.in(MetersPerSecondPerSecond) <= 1.0) {
    //     state = ProngEffectorStates.ALGAE_STOWED;
    //     Info.algaeIntaked.set(true);
    //   } else {
    //     Info.algaeIntaked.set(false);
    //   }
    // }

    // if (Math.abs(rollerVelocity.in(MetersPerSecond))
    //     > ProngEffectorConstants.kRollerConfiguration.maxVelocity) {
    //   Errors.rollerConstraint.set(true);
    // } else {
    //   io.setRollerSpeeds(rollerVelocity);
    // }

    // if (wristSetpoint.getDegrees() > ProngEffectorConstants.kMaxPivotAngle.getDegrees()
    //     || wristSetpoint.getDegrees() < ProngEffectorConstants.kMinPivotAngle.getDegrees()) {
    //   Errors.pivotConstraint.set(true);
    // } else {
    //   io.setWristSetpoint(wristSetpoint);
    // }

    Logger.processInputs("ProngEffector/Inputs", inputsAutoLogged);
    Logger.recordOutput("Has Object", hasObject());
    Logger.recordOutput("Algae Intake Level", level.toString());

    SmartDashboard.putString("Algae Intake Level", level.toString());
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
