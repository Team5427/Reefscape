package team5427.frc.robot.subsystems.Swerve.io;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.Constants.SwerveConstants.SimulationConstants;

public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveMotor;
    private final DCMotorSim steerMotor;

    private static final DCMotor driveMotorGearbox = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor steerMotorGearbox = DCMotor.getKrakenX60Foc(1);

    private final PIDController driveController;
    private final PIDController steerController;

    private SwerveModuleState targetModuleState;

    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double steerAppliedVolts = 0.0;
    public final int index;

    public ModuleIOSim(int index) {
        this.index  = index;
        driveMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(driveMotorGearbox, 0.005,
                        SwerveConstants.kDriveMotorConfiguration.gearRatio.getMathematicalGearRatio()),
                driveMotorGearbox);
        steerMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(steerMotorGearbox, 0.0004,
                        SwerveConstants.kSteerMotorConfiguration.gearRatio.getMathematicalGearRatio()),
                steerMotorGearbox);
        steerController = new PIDController(SimulationConstants.steerkP, SimulationConstants.steerkI,
                SimulationConstants.steerkD);
        driveController = new PIDController(SimulationConstants.drivekP, SimulationConstants.drivekI,
                SimulationConstants.drivekD);
        steerController.enableContinuousInput(-0.5, 0.5);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        driveAppliedVolts = driveFFVolts + driveController.calculate(driveMotor.getAngularVelocityRPM() / 60.0);
        steerAppliedVolts = steerController.calculate(steerMotor.getAngularPositionRotations());
        driveMotor.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        steerMotor.setInputVoltage(MathUtil.clamp(steerAppliedVolts, -12.0, 12.0));
        driveMotor.update(0.02);
        steerMotor.update(0.02);

        inputs.currentModuleState = new SwerveModuleState(
                driveMotor.getAngularVelocityRPM() * Math.PI * SwerveConstants.kWheelDiameterMeters / 60.0,
                Rotation2d.fromRotations(steerMotor.getAngularPositionRotations()));
        inputs.driveMotorPosition = Rotation2d.fromRotations(driveMotor.getAngularPositionRotations());
        inputs.steerMotorVelocityRotationsPerSecond = steerMotor.getAngularVelocityRPM() / 60.0;
        inputs.targetModuleState = targetModuleState;

        inputs.steerPosition = Rotation2d.fromRotations(steerMotor.getAngularPositionRotations());

        inputs.currentModulePosition = new SwerveModulePosition(
                driveMotor.getAngularPositionRotations() * Math.PI * SwerveConstants.kWheelDiameterMeters,
                Rotation2d.fromRotations(steerMotor.getAngularPositionRotations()));
        inputs.driveMotorVoltage = driveMotor.getInputVoltage();
        inputs.steerMotorVoltage = steerMotor.getInputVoltage();

        inputs.driveMotorConnected = true;
        inputs.steerMotorConnected = true;

        inputs.driveMotorCurrent = driveMotor.getCurrentDrawAmps();
        inputs.steerMotorCurrent = steerMotor.getCurrentDrawAmps();

        inputs.odometryTimestamps = new double[] { Timer.getTimestamp() };
        inputs.odometryDrivePositionsMeters = new double[] {
                inputs.driveMotorPosition.getRotations() * SwerveConstants.kWheelDiameterMeters * Math.PI };
        inputs.odometryTurnPositions = new Rotation2d[] { inputs.steerPosition };
    }

    /** @param speed rotations per second */
    public void setDriveSpeedSetpoint(Double speed) {
        driveFFVolts = SimulationConstants.drivekS + SimulationConstants.drivekV * speed;
        driveController.setSetpoint(speed);
    }

    /*
     * Needs a voltage
     */
    public void setDriveSpeedSetpoint(double volts) {
        driveAppliedVolts = volts;
    }

    public void setSteerPositionSetpoint(Rotation2d position) {
        steerController.setSetpoint(position.getRotations());
    }

    /*
     * Needs a voltage
     */
    public void setSteerPositionSetpoint(double volts) {
        steerAppliedVolts = volts;
    }

    public void setModuleState(SwerveModuleState state) {
        targetModuleState = state;
        setDriveSpeedSetpoint(Double.valueOf(state.speedMetersPerSecond));
        setSteerPositionSetpoint(state.angle);
    }

    public void resetMotorSetpoint() {
    }
}
