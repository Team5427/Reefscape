package team5427.frc.robot.subsystems.Cascade;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.SuperStructureEnum.CascadeStates;
import team5427.frc.robot.SuperStructureEnum.CascadeStates.CasacdeLockedStates;
import team5427.frc.robot.subsystems.Cascade.io.CascadeIO;
import team5427.frc.robot.subsystems.Cascade.io.CascadeIOInputsAutoLogged;
import team5427.frc.robot.subsystems.Cascade.io.CascadeIOMagicTalon;
import team5427.frc.robot.subsystems.Cascade.io.CascadeIOSim;

public class CascadeSubsystem extends SubsystemBase {

    private CascadeIO io;
    private CascadeIOInputsAutoLogged inputsAutoLogged = new CascadeIOInputsAutoLogged();

    private Distance cascadeSetpoint;
    private Rotation2d pivotSetpoint;

    private static CascadeSubsystem m_instance;

    @AutoLogOutput(key = "CascadeOutputs/State")
    public static CascadeStates state;

    @AutoLogOutput(key = "CascadeOutputs/LockedStates")
    public static List<CasacdeLockedStates> lockedStates;

    public static CascadeSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new CascadeSubsystem();
        }
        return m_instance;
    }

    private CascadeSubsystem() {
        switch (Constants.currentMode) {
            case REAL:
                io = new CascadeIOMagicTalon();
                break;
            case REPLAY:
            case SIM:
                io = new CascadeIOSim();
                break;
            default:
                break;
        }

        cascadeSetpoint = Meters.zero();
        pivotSetpoint = Rotation2d.kZero;
        state = CascadeStates.IDLE;
        lockedStates = new ArrayList<>();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputsAutoLogged);
        if (lockedStates.contains(CasacdeLockedStates.CASCADE)) {
            io.stopCascadeMotors(true);
            Info.cascadeLocked.set(true);
        } else {
            io.stopCascadeMotors(false);
            Info.cascadeLocked.set(false);
        }

        if (lockedStates.contains(CasacdeLockedStates.PIVOT)) {
            io.stopPivotMotors(true);
            Info.pivotLocked.set(true);
        } else {
            io.stopPivotMotors(false);
            Info.pivotLocked.set(false);
        }

        if (Math.abs(cascadeSetpoint.in(Meters)) < CascadeConstants.kCascadeMinimumHeight.in(Meters)
                || Math.abs(cascadeSetpoint.in(Meters)) > CascadeConstants.kCascadeMaximumHeight.in(Meters)) {
            Errors.cascadeConstraint.set(true);
        } else {
            Errors.cascadeConstraint.set(false);
            io.setCascadeSetpoint(cascadeSetpoint);
        }

        if (Math.abs(pivotSetpoint.getDegrees()) < CascadeConstants.kCascadePivotMinimumAngle.getDegrees()
                || Math.abs(pivotSetpoint.getDegrees()) > CascadeConstants.kCascadePivotMaximumAngle.getDegrees()) {
            Errors.pivotConstraint.set(true);
        } else {
            Errors.pivotConstraint.set(false);
            io.setPivotSetpoint(pivotSetpoint);
        }
        Logger.processInputs("Cascade", inputsAutoLogged);

        super.periodic();
    }

    public void setCascadeSetpoint(Distance setpoint) {
        this.cascadeSetpoint = setpoint;
    }

    public void setCascadeEncoderPosition(Distance setpoint) {
        io.setCascadeEncoderPosition(setpoint);
    }

    public boolean cascadeAtGoal() {
        return (Math.abs(inputsAutoLogged.cascadeHeightMeters.minus(cascadeSetpoint)
                .in(Meters)) < CascadeConstants.kCascadeTolerance.in(Meters));
    }

    public void setPivotSetpoint(Rotation2d setpoint) {
        this.pivotSetpoint = setpoint;
    }

    public void resetCANCoder() {
        io.setCANCoderPosition(Rotation2d.kZero);
    }

    public boolean pivotAtGoal() {
        return (Math.abs(inputsAutoLogged.pivotRotation.minus(pivotSetpoint)
                .getDegrees()) < CascadeConstants.kPivotTolerance.getDegrees());
    }

    public static void lock(ArrayList<CasacdeLockedStates> lock) {
        lockedStates = lock;
    }

    public static void lock(CasacdeLockedStates[] lock) {
        lockedStates = Arrays.asList(lock);
    }

    public static void lock(CasacdeLockedStates lock) {
        if (!lockedStates.contains(lock)) {
            lockedStates.add(lock);
        }
    }

    public static void unLock(ArrayList<CasacdeLockedStates> lock) {
        lockedStates.removeAll(lock);
    }

    public static void unLock(CasacdeLockedStates[] lock) {
        lockedStates.removeAll(Arrays.asList(lock));
    }

    public static void unLock(CasacdeLockedStates lock) {
        lockedStates.remove(lock);
    }

    private static class Errors {
        public static Alert pivotConstraint = new Alert(
                "Constraint Violations",
                "Cascade Pivot given a setpoint outside its bounds. ",
                AlertType.kError);
        public static Alert cascadeConstraint = new Alert(
                "Constraint Violations",
                "Cascade given a setpoint outside its bounds. ",
                AlertType.kError);
    }

    private static class Info {
        public static Alert pivotLocked = new Alert("Locked Systems", "Cascade Pivot locked.", AlertType.kInfo);
        public static Alert cascadeLocked = new Alert("Locked Systems", "Cascade locked.", AlertType.kInfo);
    }
}
