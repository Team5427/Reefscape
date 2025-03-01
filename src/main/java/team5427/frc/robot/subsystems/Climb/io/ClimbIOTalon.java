package team5427.frc.robot.subsystems.Climb.io;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import team5427.lib.motors.real.SteelTalonFX;

public class ClimbIOTalon implements ClimbIO {

    private SteelTalonFX hookServo;

    private StatusSignal<Angle> hookPosition;
    private StatusSignal<AngularVelocity> hookVelocity;
    private StatusSignal<AngularAcceleration> hookAcceleration;

    private Rotation2d hookSetpoint;

    @Override
    public void setHookSetpoint(Rotation2d setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setHookSetpoint'");
    }
    
}
