package team5427.frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
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
        pivotSetpoint  = Rotation2d.fromDegrees(0.0);
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

        if (pivotSetpoint.getDegrees() <= EndEffectorConstants.kPivotMaximumAngle.getDegrees()
                && pivotSetpoint.getDegrees() >= EndEffectorConstants.kPivotMinimumAngle.getDegrees()) {
            io.setPivotSetpoint(pivotSetpoint);
            Errors.pivotConstraint.set(false);
        } else {
            Errors.pivotConstraint.set(true);

        }

        if (wristSetpoint.getDegrees() <= EndEffectorConstants.kWristMaximumAngle.getDegrees()
                && wristSetpoint.getDegrees() >= EndEffectorConstants.kWristMinimumAngle.getDegrees()) {

            io.setCoralWristSetpoint(wristSetpoint);
            Errors.wristConstraint.set(false);
        } else {
            Errors.wristConstraint.set(true);

        }

        Logger.processInputs("End Effector", inputsAutoLogged);
        Logger.recordOutput("isCoralIntaked", isCoralIntaked);
        Logger.recordOutput("isAlgaeIntaked", isAlgaeIntaked);
        Logger.recordOutput("End Effector State", state);
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
        if (inputsAutoLogged.wristAngle.minus(wristSetpoint)
                .getDegrees() < EndEffectorConstants.kWristMotorConfiguration.tolerance) {
            return true;
        }
        return false;
    }

    public boolean isPivotAtSetpoint() {
        if (inputsAutoLogged.pivotAngle.minus(pivotSetpoint)
                .getDegrees() < EndEffectorConstants.kPivotMotorConfiguration.tolerance) {
            return true;
        }
        return false;
    }

    public boolean isCoralRollerAtSetpoint() {
        if (inputsAutoLogged.coralRollerMotorLinearVelocity.minus(coralRollerSetpoint)
                .baseUnitMagnitude() < EndEffectorConstants.kCoralRollerMotorConfiguration.tolerance) {
            return true;
        }
        return false;
    }

    public boolean isAlgaeRollerAtSetpoint() {
        if (inputsAutoLogged.algaeRollerMotorLinearVelocity.minus(algaeRollerSetpoint)
                .baseUnitMagnitude() < EndEffectorConstants.kAlgaeRollerMotorConfiguration.tolerance) {
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
                "End Effector Wrist given a setpoint outside its bounds. ", AlertType.kError);
        public static Alert pivotConstraint = new Alert("Constraint Violations",
                "End Effector Pivot given a setpoint outside its bounds. ", AlertType.kError);
    }
}
