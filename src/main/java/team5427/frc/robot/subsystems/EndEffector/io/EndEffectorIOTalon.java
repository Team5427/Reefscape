package team5427.frc.robot.subsystems.EndEffector.io;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import team5427.frc.robot.Constants.EndEffectorConstants;
import team5427.lib.motors.real.MagicSteelTalonFX;
import team5427.lib.motors.real.SteelTalonFX;

public class EndEffectorIOTalon implements EndEffectorIO {
    private MagicSteelTalonFX pivotMotor;
    private MagicSteelTalonFX wristMotor;
    private SteelTalonFX coralRollerMotor;
    private SteelTalonFX algaeRollerMotor;

    public EndEffectorIOTalon() {
        pivotMotor = new MagicSteelTalonFX(EndEffectorConstants.kPivotMotorCanID);
        wristMotor = new MagicSteelTalonFX(EndEffectorConstants.kWristMotorCanID);
        coralRollerMotor = new SteelTalonFX(EndEffectorConstants.kCoralRollerMotorCanID);
        algaeRollerMotor = new SteelTalonFX(EndEffectorConstants.kAlgaeRollerMotorCanID);

        pivotMotor.apply(EndEffectorConstants.kPivotMotorConfiguration);
        wristMotor.apply(EndEffectorConstants.kWristMotorConfiguration);
        coralRollerMotor.apply(EndEffectorConstants.kCoralRollerMotorConfiguration);
        algaeRollerMotor.apply(EndEffectorConstants.kAlgaeRollerMotorConfiguration);
    }

    @Override
    public void setCoralRollerSetpoint(LinearVelocity velocity) {
        coralRollerMotor.setSetpoint(velocity.in(MetersPerSecond));
    }

    @Override
    public void setCoralWristSetpoint(Rotation2d setpoint) {
        wristMotor.setSetpoint(setpoint);
    }

    @Override
    public void setAlgaeRollerSetpoint(LinearVelocity velocity) {
        algaeRollerMotor.setSetpoint(velocity.in(MetersPerSecond));
    }

    @Override
    public void setPivotSetpoint(Rotation2d setpoint) {
        pivotMotor.setSetpoint(setpoint);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.algaeRollerMotorAngularVelocity = algaeRollerMotor.getTalonFX().getVelocity().getValue();
        inputs.algaeRollerMotorConnected = algaeRollerMotor.getTalonFX().isConnected();
        inputs.algaeRollerMotorCurrent = algaeRollerMotor.getTalonFX().getSupplyCurrent().getValue();
        inputs.algaeRollerMotorLinearVelocity = MetersPerSecond.of(algaeRollerMotor.getEncoderVelocity());
        inputs.algaeRollerMotorVoltage = algaeRollerMotor.getTalonFX().getMotorVoltage().getValue();

        inputs.coralRollerMotorAngularVelocity = coralRollerMotor.getTalonFX().getVelocity().getValue();
        inputs.coralRollerMotorConnected = coralRollerMotor.getTalonFX().isConnected();
        inputs.coralRollerMotorCurrent = coralRollerMotor.getTalonFX().getSupplyCurrent().getValue();
        inputs.coralRollerMotorLinearVelocity = MetersPerSecond.of(coralRollerMotor.getEncoderVelocity());
        inputs.coralRollerMotorVoltage = coralRollerMotor.getTalonFX().getMotorVoltage().getValue();

        inputs.pivotAngle = Rotation2d.fromRotations(pivotMotor.getEncoderPosition());
        inputs.pivotMotorAngularAcceleration = pivotMotor.getTalonFX().getAcceleration().getValue();
        inputs.pivotMotorAngularVelocity = RotationsPerSecond.of(pivotMotor.getEncoderVelocity() / 60.0);
        inputs.pivotMotorConnected = pivotMotor.getTalonFX().isConnected();
        inputs.pivotMotorCurrent = pivotMotor.getTalonFX().getStatorCurrent().getValue();
        inputs.pivotMotorVoltage = pivotMotor.getTalonFX().getMotorVoltage().getValue();
        
        inputs.wristAngle = Rotation2d.fromRotations(wristMotor.getEncoderPosition());
        inputs.wristMotorAngularAcceleration = wristMotor.getTalonFX().getAcceleration().getValue();
        inputs.wristMotorAngularVelocity = RotationsPerSecond.of(wristMotor.getEncoderVelocity() / 60.0);
        inputs.wristMotorConnected = wristMotor.getTalonFX().isConnected();
        inputs.wristMotorCurrent = wristMotor.getTalonFX().getStatorCurrent().getValue();
        inputs.wristMotorVoltage = wristMotor.getTalonFX().getMotorVoltage().getValue();
    }

}
