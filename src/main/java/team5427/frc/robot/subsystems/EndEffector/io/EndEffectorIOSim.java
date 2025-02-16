package team5427.frc.robot.subsystems.EndEffector.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOSim implements EndEffectorIO {

  private DCMotorSim pivotMotor;
  private DCMotorSim wristMotor;
  private DCMotorSim coralRollerMotor;
  private DCMotorSim algaeRollerMotor;

  private double pivotMotorVoltage = 0.0;
  private double wristMotorVoltage = 0.0;

  private static final double pivotMotorInertia = 0.01;
  private static final double wristMotorInertia = 0.01;
  private static final double coralRollerMotorInertia = 0.004;
  private static final double algaeRollerMotorInertia = 0.004;

  public EndEffectorIOSim() {
    pivotMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                EndEffectorConstants.kPivotMotorConfiguration.withFOC
                    ? DCMotor.getKrakenX60Foc(1)
                    : DCMotor.getKrakenX60(1),
                pivotMotorInertia,
                EndEffectorConstants.kPivotMotorConfiguration.gearRatio.getMathematicalGearRatio()),
            EndEffectorConstants.kPivotMotorConfiguration.withFOC
                ? DCMotor.getKrakenX60Foc(1)
                : DCMotor.getKrakenX60(1));

    wristMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                EndEffectorConstants.kWristMotorConfiguration.withFOC
                    ? DCMotor.getKrakenX60Foc(1)
                    : DCMotor.getKrakenX60(1),
                wristMotorInertia,
                EndEffectorConstants.kWristMotorConfiguration.gearRatio.getMathematicalGearRatio()),
            EndEffectorConstants.kWristMotorConfiguration.withFOC
                ? DCMotor.getKrakenX60Foc(1)
                : DCMotor.getKrakenX60(1));
    coralRollerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                EndEffectorConstants.kCoralRollerMotorConfiguration.withFOC
                    ? DCMotor.getKrakenX60Foc(1)
                    : DCMotor.getKrakenX60(1),
                coralRollerMotorInertia,
                EndEffectorConstants.kCoralRollerMotorConfiguration.gearRatio
                    .getMathematicalGearRatio()),
            EndEffectorConstants.kCoralRollerMotorConfiguration.withFOC
                ? DCMotor.getKrakenX60Foc(1)
                : DCMotor.getKrakenX60(1));
    algaeRollerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                EndEffectorConstants.kAlgaeRollerMotorConfiguration.withFOC
                    ? DCMotor.getKrakenX60Foc(1)
                    : DCMotor.getKrakenX60(1),
                algaeRollerMotorInertia,
                EndEffectorConstants.kAlgaeRollerMotorConfiguration.gearRatio
                    .getMathematicalGearRatio()),
            EndEffectorConstants.kAlgaeRollerMotorConfiguration.withFOC
                ? DCMotor.getKrakenX60Foc(1)
                : DCMotor.getKrakenX60(1));
  }

  @Override
  public void setCoralRollerSetpoint(LinearVelocity velocity) {
    coralRollerMotor.setAngularVelocity(
        velocity.in(MetersPerSecond)
            / (0.5 * EndEffectorConstants.kCoralRollerMotorConfiguration.finalDiameterMeters));
  }

  @Override
  public void setCoralWristSetpoint(Rotation2d setpoint) {
    EndEffectorConstants.kSIMWristController.setGoal(setpoint.getRotations());
  }

  @Override
  public void setAlgaeRollerSetpoint(LinearVelocity velocity) {
    algaeRollerMotor.setState(
        algaeRollerMotor.getAngularPositionRad(),
        velocity.in(MetersPerSecond)
            / (0.5 * EndEffectorConstants.kAlgaeRollerMotorConfiguration.finalDiameterMeters));
  }

  @Override
  public void setPivotSetpoint(Rotation2d setpoint) {
    EndEffectorConstants.kSIMPivotController.setGoal(setpoint.getRotations());
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    pivotMotorVoltage =
        EndEffectorConstants.kSIMPivotController.calculate(
            pivotMotor.getAngularPositionRotations());
    wristMotorVoltage =
        EndEffectorConstants.kSIMWristController.calculate(
            wristMotor.getAngularPositionRotations());

    pivotMotor.setInputVoltage(MathUtil.clamp(pivotMotorVoltage, -12.0, 12.0));
    wristMotor.setInputVoltage(MathUtil.clamp(wristMotorVoltage, -12.0, 12.0));
    // coralRollerMotor.setInputVoltage(coralRollerMotorVoltage);
    // algaeRollerMotor.setInputVoltage(algaeRollerMotorVoltage);

    pivotMotor.update(Constants.kLoopSpeed);
    wristMotor.update(Constants.kLoopSpeed);
    coralRollerMotor.update(Constants.kLoopSpeed);
    algaeRollerMotor.update(Constants.kLoopSpeed);

    inputs.algaeRollerMotorAngularVelocity = algaeRollerMotor.getAngularVelocity();
    inputs.algaeRollerMotorConnected = true;
    inputs.algaeRollerMotorCurrent = Amps.of(algaeRollerMotor.getCurrentDrawAmps());
    inputs.algaeRollerMotorLinearVelocity =
        MetersPerSecond.of(
            (algaeRollerMotor.getAngularVelocityRPM()
                    * EndEffectorConstants.kAlgaeRollerMotorConfiguration.finalDiameterMeters
                    * Math.PI)
                / 60.0);
    inputs.algaeRollerMotorVoltage = Volts.of(algaeRollerMotor.getInputVoltage());
    inputs.algaeRollerMotorAngularAcceleration = algaeRollerMotor.getAngularAcceleration();

    inputs.coralRollerMotorAngularVelocity = coralRollerMotor.getAngularVelocity();
    inputs.coralRollerMotorConnected = true;
    inputs.coralRollerMotorCurrent = Amps.of(coralRollerMotor.getCurrentDrawAmps());
    inputs.coralRollerMotorLinearVelocity =
        MetersPerSecond.of(
            (coralRollerMotor.getAngularVelocityRPM()
                    * EndEffectorConstants.kCoralRollerMotorConfiguration.finalDiameterMeters
                    * Math.PI)
                / 60.0);
    inputs.coralRollerMotorVoltage = Volts.of(coralRollerMotor.getInputVoltage());
    inputs.coralRollerMotorAngularAcceleration = coralRollerMotor.getAngularAcceleration();

    inputs.pivotAngle = Rotation2d.fromRotations(pivotMotor.getAngularPositionRotations());
    inputs.pivotMotorAngularAcceleration = pivotMotor.getAngularAcceleration();
    inputs.pivotMotorAngularVelocity = pivotMotor.getAngularVelocity();
    inputs.pivotMotorConnected = true;
    inputs.pivotMotorCurrent = Amps.of(pivotMotor.getCurrentDrawAmps());
    inputs.pivotMotorVoltage = Volts.of(pivotMotor.getInputVoltage());

    inputs.wristAngle = Rotation2d.fromRotations(wristMotor.getAngularPosition().in(Rotation));
    inputs.wristMotorAngularAcceleration = wristMotor.getAngularAcceleration();
    inputs.wristMotorAngularVelocity = wristMotor.getAngularVelocity();
    inputs.wristMotorConnected = true;
    inputs.wristMotorCurrent = Amps.of(wristMotor.getCurrentDrawAmps());
    inputs.wristMotorVoltage = Volts.of(wristMotor.getInputVoltage());
  }
}
