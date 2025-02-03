package team5427.frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.EndEffectorConstants;
import team5427.frc.robot.SuperStructureEnum.CascadeStates.CasacdeLockedStates;
import team5427.frc.robot.SuperStructureEnum.EndEffectorStates.EndEffectorLockedStates;
import team5427.frc.robot.SuperStructureEnum.EndEffectorStates;
import team5427.frc.robot.subsystems.EndEffector.io.EndEffectorIO;
import team5427.frc.robot.subsystems.EndEffector.io.EndEffectorIOInputsAutoLogged;
import team5427.frc.robot.subsystems.EndEffector.io.EndEffectorIOSim;
import team5427.frc.robot.subsystems.EndEffector.io.EndEffectorIOTalon;

public class EndEffectorSubsystem extends SubsystemBase {
  private EndEffectorIO io;
  private EndEffectorIOInputsAutoLogged inputsAutoLogged = new EndEffectorIOInputsAutoLogged();
  public boolean isCoralIntaked = false;

  public boolean isAlgaeIntaked = false;

  private static final AngularAcceleration kFlywheelAccelerationMaxBuffer = RotationsPerSecondPerSecond.of(0.5);

  @AutoLogOutput(key = "EndEffector/State")
  public static EndEffectorStates state;

  private static EndEffectorSubsystem m_instance;

  @AutoLogOutput(key = "EndEffector/LockedStates")
  public static List<EndEffectorLockedStates> lockedStates;

  private Rotation2d wristSetpoint;
  private Rotation2d pivotSetpoint;
  private LinearVelocity coralRollerSetpoint = MetersPerSecond.of(0.0);
  private LinearVelocity algaeRollerSetpoint = MetersPerSecond.of(0.0);

  private EndEffectorSubsystem() {
    switch (Constants.currentMode) {
      case REAL:
        io = new EndEffectorIOTalon();
        break;
      case REPLAY:
        io = new EndEffectorIOSim();
        break;
      case SIM:
        io = new EndEffectorIOSim();
        break;
      default:
        break;
    }
    wristSetpoint = Rotation2d.fromDegrees(0.0);
    pivotSetpoint = Rotation2d.fromDegrees(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputsAutoLogged);

    if (state == EndEffectorStates.CORAL_INTAKE || state == EndEffectorStates.CORAL_OUTAKE) {
      isCoralIntaked = isCoralIntaked(inputsAutoLogged.coralRollerMotorCurrent);
    }
    if (state == EndEffectorStates.ALGAE_INTAKE || state == EndEffectorStates.ALGAE_OUTAKE) {
      isAlgaeIntaked = isAlgaeIntaked(inputsAutoLogged.algaeRollerMotorCurrent);
    }

    io.setCoralRollerSetpoint(coralRollerSetpoint);
    io.setAlgaeRollerSetpoint(algaeRollerSetpoint);

    if (Math.abs(pivotSetpoint.getDegrees()) <= Math.abs(EndEffectorConstants.kPivotMaximumAngle.getDegrees())
        && Math.abs(pivotSetpoint.getDegrees()) >= Math.abs(EndEffectorConstants.kPivotMinimumAngle.getDegrees())) {
      io.setPivotSetpoint(pivotSetpoint);
      Errors.pivotConstraint.set(false);
    } else {
      Errors.pivotConstraint.set(true);
    }

    if (Math.abs(wristSetpoint.getDegrees()) <= Math.abs(EndEffectorConstants.kWristMaximumAngle.getDegrees())
        && Math.abs(wristSetpoint.getDegrees()) >= Math.abs(EndEffectorConstants.kWristMinimumAngle.getDegrees())) {

      io.setCoralWristSetpoint(wristSetpoint);
      Errors.wristConstraint.set(false);
    } else {
      Errors.wristConstraint.set(true);
    }

    Logger.processInputs("End Effector", inputsAutoLogged);
    Logger.recordOutput("isCoralIntaked", isCoralIntaked);
    Logger.recordOutput("isAlgaeIntaked", isAlgaeIntaked);
    // Logger.recordOutput("End Effector State", state);
    Logger.recordOutput("Wrist Setpoint", wristSetpoint);
    Logger.recordOutput("Pivot Setpoint", pivotSetpoint);
    Logger.recordOutput("Coral Roller Setpoint", coralRollerSetpoint);
    Logger.recordOutput("Algae Roller Setpoint", algaeRollerSetpoint);

    super.periodic();
  }

  public void setWristSetpoint(Rotation2d setpoint) {
    wristSetpoint = setpoint;
  }

  public void setPivotSetpoint(Rotation2d setpoint) {
    pivotSetpoint = setpoint;
  }

  public boolean isWristAtSetpoint() {
    if (Math.abs(inputsAutoLogged.wristAngle.minus(wristSetpoint).getDegrees()) < 0.05) {
      return true;
    }
    return false;
  }

  public boolean isPivotAtSetpoint() {
    if (Math.abs(inputsAutoLogged.pivotAngle.minus(pivotSetpoint).getDegrees()) < 0.05) {
      return true;
    }
    return false;
  }

  public boolean isCoralRollerAtSetpoint() {
    if (Math.abs(inputsAutoLogged.coralRollerMotorLinearVelocity
        .minus(coralRollerSetpoint)
        .in(MetersPerSecond)) < 0.5) {
      return true;
    }
    return false;
  }

  public boolean isAlgaeRollerAtSetpoint() {
    if (Math.abs(inputsAutoLogged.algaeRollerMotorLinearVelocity
        .minus(algaeRollerSetpoint)
        .in(MetersPerSecond)) < 0.5) {
      return true;
    }
    return false;
  }

  public void setCoralRollerSetpoint(LinearVelocity velocity) {
    coralRollerSetpoint = velocity;
  }

  public void setAlgaeRollerSetpoint(LinearVelocity velocity) {
    algaeRollerSetpoint = velocity;
  }

  private boolean isAlgaeIntaked(Current current) {
    return current.in(Amps) > 20.0
        && inputsAutoLogged.algaeRollerMotorAngularAcceleration.compareTo(
            kFlywheelAccelerationMaxBuffer) < 0.5;
  }

  private boolean isCoralIntaked(Current current) {
    return current.in(Amps) > 20.0;
  }

  public static void lock(ArrayList<EndEffectorLockedStates> lock) {
    lockedStates = lock;
  }

  public static void lock(EndEffectorLockedStates[] lock) {
    lockedStates = Arrays.asList(lock);
  }

  public static void lock(EndEffectorLockedStates lock) {
    if (!lockedStates.contains(lock)) {
      lockedStates.add(lock);
    }
  }

  public static void unLock(ArrayList<EndEffectorLockedStates> lock) {
    lockedStates.removeAll(lock);
  }

  public static void unLock(EndEffectorLockedStates[] lock) {
    lockedStates.removeAll(Arrays.asList(lock));
  }

  public static void unLock(EndEffectorLockedStates lock) {
    lockedStates.remove(lock);
  }

  public static EndEffectorSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new EndEffectorSubsystem();
    }
    return m_instance;
  }

  private static class Errors {
    public static Alert wristConstraint = new Alert(
        "Constraint Violations",
        "End Effector Wrist given a setpoint outside its bounds. ",
        AlertType.kError);
    public static Alert pivotConstraint = new Alert(
        "Constraint Violations",
        "End Effector Pivot given a setpoint outside its bounds. ",
        AlertType.kError);
  }

  private static class Info {
    public static Alert pivotLocked = new Alert("Locked Systems", "End Effector Pivot Locked", AlertType.kInfo);
    public static Alert algaeRollerLocked = new Alert("Locked Systems", "End Effector Algae Roller Locked",
        AlertType.kInfo);
  }
}
