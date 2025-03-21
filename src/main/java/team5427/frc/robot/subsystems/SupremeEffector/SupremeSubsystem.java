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
  private SupremeIO io;
  private SupremeIOInputsAutoLogged inputsAutoLogged;

  @Getter @Setter private Rotation2d pivotSetpoint;
  @Getter @Setter private LinearVelocity coralRollerSetpoint;
  @Getter @Setter private LinearVelocity algaeRollerSetpoint;

  public SupremeSubsystem() {
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
        inputsAutoLogged.algaeMotorStatorCurrent.in(Amps) >= 10.0
            && inputsAutoLogged.algaeMotorLinearVelocity.in(MetersPerSecond) < 0.1);
  }

  public boolean isCoralIntaked() {
    // return SupremeEffectorConstants.kCoralIntakeDebouncer.calculate(
    //     inputsAutoLogged.coralMotorStatorCurrent.in(Amps) >= 10.0
    //         && inputsAutoLogged.coralMotorLinearVelocity.in(MetersPerSecond) < 0.2);
    return inputsAutoLogged.canRangeDetectedObject;
  }
}
