package team5427.frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.EndEffectorConstants;
import team5427.frc.robot.subsystems.EndEffector.io.EndEffectorIO;
import team5427.frc.robot.subsystems.EndEffector.io.EndEffectorIOInputsAutoLogged;
import team5427.frc.robot.subsystems.EndEffector.io.EndEffectorIOSim;
import team5427.frc.robot.subsystems.EndEffector.io.EndEffectorIOTalon;

public class EndEffectorSubsystem extends SubsystemBase {
    private EndEffectorIO io;
    private EndEffectorIOInputsAutoLogged inputsAutoLogged = new EndEffectorIOInputsAutoLogged();
    public boolean isCoralIntaked = false;
    public boolean isAlgaeIntaked = false;

    private static EndEffectorSubsystem m_instance;

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
        isCoralIntaked = isCoralIntaked(inputsAutoLogged.coralRollerMotorCurrent);
        isAlgaeIntaked = isAlgaeIntaked(inputsAutoLogged.algaeRollerMotorCurrent);
        Logger.processInputs("End Effector", inputsAutoLogged);
        super.periodic();
    }

    public void setCoralRollerSpeed(LinearVelocity velocity) {
        io.setCoralRollerSetpoint(velocity);
    }

    public void setAlgaeRollerSpeed(LinearVelocity velocity) {
        io.setAlgaeRollerSetpoint(velocity);
    }

    public void setWristSetpoint(Rotation2d setpoint) {
        if (setpoint.getDegrees() < EndEffectorConstants.kWristMaximumAngle.getDegrees()
                && setpoint.getDegrees() > EndEffectorConstants.kWristMinimumAngle.getDegrees()) {
            io.setCoralWristSetpoint(setpoint);
            Errors.wristConstraint.set(false);
        } else {
            Errors.wristConstraint.setText(Errors.wristConstraint.getText() + "(" + setpoint.getDegrees() + ")");
            Errors.wristConstraint.set(true);

        }
    }

    public void setPivotSetpoint(Rotation2d setpoint) {
        io.setPivotSetpoint(setpoint);
    }

    private boolean isAlgaeIntaked(Current current) {
        return current.in(Amps) > 20.0;
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

    public static class Errors {
        public static Alert wristConstraint = new Alert("Constraint Violations",
                "WARNING: End Effector Wrist given a setpoint outside its bounds. ", AlertType.kError);
    }
}
