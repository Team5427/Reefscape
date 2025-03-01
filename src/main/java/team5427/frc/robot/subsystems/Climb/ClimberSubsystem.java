package team5427.frc.robot.subsystems.Climb;

import java.time.Period;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.ClimbConstants;
import team5427.frc.robot.subsystems.Climb.io.ClimbIO;
import team5427.frc.robot.subsystems.Climb.io.ClimbIOInputsAutoLogged;
import team5427.frc.robot.subsystems.Climb.io.ClimbIOTalon;

public class ClimberSubsystem extends SubsystemBase {

    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputsAutoLogged;

    private Rotation2d climbHooksSetpoint;

    private static ClimberSubsystem m_instance;

    public static ClimberSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new ClimberSubsystem();
        }
        return m_instance;
    }

    private ClimberSubsystem () {
        switch(Constants.currentMode) {
            case REAL:
                io = new ClimbIOTalon();
                break;
            default:
                break;
        }

        climbHooksSetpoint = ClimbConstants.kStowPosition;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputsAutoLogged);
        io.setHookSetpoint(climbHooksSetpoint);

        super.periodic();
    }

    public void setSetpoint(Rotation2d setpoint) {
        climbHooksSetpoint = setpoint;
    }

    
}
