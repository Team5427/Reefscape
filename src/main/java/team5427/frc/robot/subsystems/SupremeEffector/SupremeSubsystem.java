package team5427.frc.robot.subsystems.SupremeEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.SupremeEffectorConstants;
import team5427.frc.robot.subsystems.SupremeEffector.io.SupremeIO;
import team5427.frc.robot.subsystems.SupremeEffector.io.SupremeIOInputsAutoLogged;
import team5427.frc.robot.subsystems.SupremeEffector.io.SupremeIOTalon;

public class SupremeSubsystem extends SubsystemBase {

  private static SupremeSubsystem m_instance;

  private SupremeIO io;
  private SupremeIOInputsAutoLogged inputsAutoLogged;

  @Getter @Setter private Rotation2d pivotSetpoint = new Rotation2d(0);
  @Getter @Setter private LinearVelocity coralRollerSetpoint = MetersPerSecond.of(0.0);
  @Getter @Setter private LinearVelocity algaeRollerSetpoint = MetersPerSecond.of(0.0);

  public static SupremeSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new SupremeSubsystem();
    }
    return m_instance;
  }

  private SupremeSubsystem() {
    switch (Constants.currentMode) {
      case REAL:
        io = new SupremeIOTalon();
        break;
      case REPLAY:
        break;
      case SIM:
        break;
      default:
        io = new SupremeIOTalon();
        break;
    }
    inputsAutoLogged = new SupremeIOInputsAutoLogged();

    // pivotSetpoint = SupremeEffectorConstants.kAlgaeStowPosition;
    pivotSetpoint = SupremeEffectorConstants.kCoralFloorIntakePosition;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputsAutoLogged);
    Logger.processInputs("Supreme Effector", inputsAutoLogged);
    io.setAlgaeMotorSetpoint(algaeRollerSetpoint);
    io.setCoralMotorSetpoint(coralRollerSetpoint);
    io.setPivotMotorSetpoint(pivotSetpoint);
  }

  public boolean isAlgaeIntaked() {
    return SupremeEffectorConstants.kAlgaeIntakeDebouncer.calculate(
        inputsAutoLogged.algaeMotorStatorCurrent.in(Amps)
                >= SupremeEffectorConstants.kIntakeMaxCurrent.in(Amps)
            && inputsAutoLogged.algaeMotorLinearVelocity.in(MetersPerSecond)
                < SupremeEffectorConstants.kIntakeThresholdVelocity.in(MetersPerSecond));
  }

  public boolean isCoralIntaked() {
    // return SupremeEffectorConstants.kCoralIntakeDebouncer.calculate(
    //     inputsAutoLogged.coralMotorStatorCurrent.in(Amps) >= 10.0
    //         && inputsAutoLogged.coralMotorLinearVelocity.in(MetersPerSecond) < 0.2);
    return inputsAutoLogged.canRangeDetectedObject;
  }
}
