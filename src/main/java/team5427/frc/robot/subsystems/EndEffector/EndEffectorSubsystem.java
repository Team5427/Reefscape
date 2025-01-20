package team5427.frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.EndEffectorConstants;
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

    public static EndEffectorStates state;

    private static EndEffectorSubsystem m_instance;

    private Rotation2d wristSetpoint;
    private Rotation2d pivotSetpoint;
    private LinearVelocity coralRollerSetpoint;
    private LinearVelocity algaeRollerSetpoint;

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
    }

    @Override
    public void periodic() {
        io.updateInputs(inputsAutoLogged);
        if (state == EndEffectorStates.CORAL_INTAKE || state == EndEffectorStates.CORAL_OUTAKE) {
            isCoralIntaked = isCoralIntaked(inputsAutoLogged.coralRollerMotorCurrent);
        } else {
            isCoralIntaked = false;
        }
        if (state == EndEffectorStates.ALGAE_INTAKE || state == EndEffectorStates.ALGAE_OUTAKE) {
            isAlgaeIntaked = isAlgaeIntaked(inputsAutoLogged.algaeRollerMotorCurrent);
        } else {
            isAlgaeIntaked = false;
        }
        Logger.processInputs("End Effector", inputsAutoLogged);
        Logger.recordOutput("isCoralIntaked", isCoralIntaked);
        Logger.recordOutput("isAlgaeIntaked", isAlgaeIntaked);
        Logger.recordOutput("End Effector State", state);
        super.periodic();
    }

    public void setWristSetpoint(Rotation2d setpoint) {
        if (setpoint.getDegrees() < EndEffectorConstants.kWristMaximumAngle.getDegrees()
                && setpoint.getDegrees() > EndEffectorConstants.kWristMinimumAngle.getDegrees()) {
                    wristSetpoint = setpoint;
            io.setCoralWristSetpoint(setpoint);
            Errors.wristConstraint.set(false);
        } else {
            Errors.wristConstraint.setText(Errors.wristConstraint.getText() + "(" + setpoint.getDegrees() + ")");
            Errors.wristConstraint.set(true);

        }
    }

    public void setPivotSetpoint(Rotation2d setpoint) {
        if (setpoint.getDegrees() < EndEffectorConstants.kPivotMaximumAngle.getDegrees()
                && setpoint.getDegrees() > EndEffectorConstants.kPivotMinimumAngle.getDegrees()) {
                    pivotSetpoint  = setpoint;
            io.setPivotSetpoint(setpoint);
            Errors.pivotConstraint.set(false);
        } else {
            Errors.pivotConstraint.setText(Errors.pivotConstraint.getText() + "(" + setpoint.getDegrees() + ")");
            Errors.pivotConstraint.set(true);

        }
    }

    public boolean isWristAtSetpoint(){
        if(inputsAutoLogged.wristAngle.minus(wristSetpoint).getDegrees() < EndEffectorConstants.kWristMotorConfiguration.tolerance){
            return true;
        }
        return false;
    }

    public boolean isPivotAtSetpoint(){
        if(inputsAutoLogged.pivotAngle.minus(pivotSetpoint).getDegrees() < EndEffectorConstants.kPivotMotorConfiguration.tolerance){
            return true;
        }
        return false;
    }

    public boolean isCoralRollerAtSetpoint(){
        if(inputsAutoLogged.coralRollerMotorLinearVelocity.minus(coralRollerSetpoint).baseUnitMagnitude() < EndEffectorConstants.kCoralRollerMotorConfiguration.tolerance){
            return true;
        }
        return false;
    }

    public boolean isAlgaeRollerAtSetpoint(){
        if(inputsAutoLogged.algaeRollerMotorLinearVelocity.minus(algaeRollerSetpoint).baseUnitMagnitude() < EndEffectorConstants.kAlgaeRollerMotorConfiguration.tolerance){
            return true;
        }
        return false;
    }

    public void setCoralRollerSetpoint(LinearVelocity velocity) {
        coralRollerSetpoint = velocity;
        io.setCoralRollerSetpoint(velocity);
    }

    public void setAlgaeRollerSetpoint(LinearVelocity velocity) {
        algaeRollerSetpoint = velocity;
        io.setAlgaeRollerSetpoint(velocity);
    }

    private boolean isAlgaeIntaked(Current current) {
        return current.in(Amps) > 20.0
                && inputsAutoLogged.algaeRollerMotorAngularAcceleration.compareTo(kFlywheelAccelerationMaxBuffer) < 0.5;
    }

    private boolean isCoralIntaked(Current current) {
        return current.in(Amps) > 20.0;
    }

    public static EndEffectorSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new EndEffectorSubsystem();
        }
        return m_instance;
    }

    private static class Errors {
        public static Alert wristConstraint = new Alert("Constraint Violations",
                "WARNING: End Effector Wrist given a setpoint outside its bounds. ", AlertType.kError);
        public static Alert pivotConstraint = new Alert("Constraint Violations",
                "WARNING: End Effector Pivot given a setpoint outside its bounds. ", AlertType.kError);
    }
}
