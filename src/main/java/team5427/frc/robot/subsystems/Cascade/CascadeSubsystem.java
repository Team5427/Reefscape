package team5427.frc.robot.subsystems.Cascade;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.subsystems.Cascade.io.CascadeIO;
import team5427.frc.robot.subsystems.Cascade.io.CascadeIOInputsAutoLogged;
import team5427.frc.robot.subsystems.Cascade.io.CascadeIOMagicTalon;
import team5427.frc.robot.subsystems.Cascade.io.CascadeIOSim;

public class CascadeSubsystem extends SubsystemBase {

    private CascadeIO io;
    private CascadeIOInputsAutoLogged inputsAutoLogged;

    private Distance cascadeSetpoint;
    private Rotation2d pivotSetpoint;

    private static CascadeSubsystem m_instance;

    public static CascadeSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new CascadeSubsystem();
        }
        return m_instance;
    }

    private CascadeSubsystem () {
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
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    public void setCascadeSetpoint(Distance setpoint) {
        io.setCascadeSetpoint(setpoint);
    }

    public void setCascadeEncoderPosition(Distance setpoint) {
        io.setCascadeEncoderPosition(setpoint);
    }

    public boolean cascadeAtGoal() {
        return (inputsAutoLogged.cascadeHeightMeters.minus(cascadeSetpoint).in(Meters) < CascadeConstants.kCascadeTolerance.in(Meters));
    }

    public void setPivotSetpoint(Rotation2d setpoint) {
        io.setPivotSetpoint(setpoint);
    }

    public void resetCANCoder() {
        io.setCANCoderPosition(Rotation2d.kZero);
    }

    public boolean pivotAtGoal() {
        return (Math.abs(inputsAutoLogged.pivotRotation.minus(pivotSetpoint).getDegrees()) < CascadeConstants.kPivotTolerance.getDegrees());
    }

    
    
}
