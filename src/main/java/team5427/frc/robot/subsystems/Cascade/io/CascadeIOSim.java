package team5427.frc.robot.subsystems.Cascade.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import team5427.frc.robot.Constants.CascadeConstants;

public class CascadeIOSim implements CascadeIO {

    private DCMotorSim cascadeMotors;
    private Distance cascadeSetpointMeters;

    private DCMotorSim pivotMotors;
    private Rotation2d pivotSetpoint;

    private static final double cascadeMotorInertia = 0.05;
    private static final double pivotMotorInertia = 0.03;

    public CascadeIOSim () {
        cascadeMotors = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                CascadeConstants.kCascadeDriverConfiguration.withFOC ? DCMotor.getKrakenX60Foc(2): DCMotor.getKrakenX60(2), 
                cascadeMotorInertia,
                CascadeConstants.kCascadeDriverConfiguration.gearRatio.getMathematicalGearRatio()
            ),
            CascadeConstants.kCascadeDriverConfiguration.withFOC ? DCMotor.getKrakenX60Foc(2): DCMotor.getKrakenX60(2)
        );

        pivotMotors = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                CascadeConstants.kPivotConfiguration.withFOC ? DCMotor.getKrakenX60Foc(2): DCMotor.getKrakenX60(2), 
                pivotMotorInertia,
                CascadeConstants.kPivotConfiguration.gearRatio.getMathematicalGearRatio()
            ), 
            CascadeConstants.kPivotConfiguration.withFOC ? DCMotor.getKrakenX60Foc(2): DCMotor.getKrakenX60(2)
        );
    }

    @Override
    public void updateInputs(CascadeIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void setCascadeSetpoint(Distance setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCascadeSetpoint'");
    }

    @Override
    public void setCascadeEncoderPosition(Distance setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCascadeEncoderPosition'");
    }

    @Override
    public void setPivotSetpoint(Rotation2d setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPivotSetpoint'");
    }

    @Override
    public void setCANCoderPosition(Rotation2d angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCANCoderPosition'");
    }
    
}
