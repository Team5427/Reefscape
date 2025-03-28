package team5427.frc.robot.subsystems.ProngEffector.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import team5427.frc.robot.Constants.ProngEffectorConstants;

public class ProngIOSim implements ProngIO {

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

  public ProngIOSim() {
    pivotMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ProngEffectorConstants.kWristConfiguration.withFOC
                    ? DCMotor.getKrakenX60Foc(1)
                    : DCMotor.getKrakenX60(1),
                pivotMotorInertia,
                ProngEffectorConstants.kWristConfiguration.gearRatio.getMathematicalGearRatio()),
            ProngEffectorConstants.kWristConfiguration.withFOC
                ? DCMotor.getKrakenX60Foc(1)
                : DCMotor.getKrakenX60(1));

    coralRollerMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ProngEffectorConstants.kRollerConfiguration.withFOC
                    ? DCMotor.getKrakenX60Foc(1)
                    : DCMotor.getKrakenX60(1),
                coralRollerMotorInertia,
                ProngEffectorConstants.kRollerConfiguration.gearRatio.getMathematicalGearRatio()),
            ProngEffectorConstants.kRollerConfiguration.withFOC
                ? DCMotor.getKrakenX60Foc(1)
                : DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(ProngIOInputs inputs) {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public void setWristSetpoint(Rotation2d setpoint) {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'setWristSetpoint'");
  }

  @Override
  public void setRollerSpeeds(LinearVelocity velocity) {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'setRollerSpeeds'");
  }

  @Override
  public void stopRollers() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'stopRollers'");
  }

  @Override
  public boolean hasResistance() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'hasResistance'");
    return false;
  }
}
