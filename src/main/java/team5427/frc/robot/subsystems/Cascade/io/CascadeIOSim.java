package team5427.frc.robot.subsystems.Cascade.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.CascadeConstants;

public class CascadeIOSim implements CascadeIO {
        private ElevatorSim cascadeSim;
        private SingleJointedArmSim pivotSim;

        private DCMotorSim cascadeMotors;
        private Distance cascadeSetpointMeters;

        private DCMotorSim pivotMotors;
        private Rotation2d pivotSetpoint;

        private static final double cascadeMotorInertia = 0.003;
        private static final double pivotMotorInertia = 0.03;

        private ProfiledPIDController cascadeController;
        private ElevatorFeedforward cascadeFeedforward;

        private ProfiledPIDController pivotController;
        private ArmFeedforward pivotFeedforward;

        public boolean cascadeMotorsStopped;
        public boolean pivotMotorsStopped;

        public CascadeIOSim() {
                cascadeMotors = new DCMotorSim(
                                LinearSystemId.createDCMotorSystem(
                                                CascadeConstants.kCascadeDriverConfiguration.withFOC
                                                                ? DCMotor.getKrakenX60Foc(2)
                                                                : DCMotor.getKrakenX60(2),
                                                cascadeMotorInertia,
                                                CascadeConstants.kCascadeDriverConfiguration.gearRatio
                                                                .getMathematicalGearRatio()),
                                CascadeConstants.kCascadeDriverConfiguration.withFOC ? DCMotor.getKrakenX60Foc(2)
                                                : DCMotor.getKrakenX60(2));

                pivotMotors = new DCMotorSim(
                                LinearSystemId.createDCMotorSystem(
                                                CascadeConstants.kPivotConfiguration.withFOC
                                                                ? DCMotor.getKrakenX60Foc(2)
                                                                : DCMotor.getKrakenX60(2),
                                                pivotMotorInertia,
                                                CascadeConstants.kPivotConfiguration.gearRatio
                                                                .getMathematicalGearRatio()),
                                CascadeConstants.kPivotConfiguration.withFOC ? DCMotor.getKrakenX60Foc(2)
                                                : DCMotor.getKrakenX60(2));
                // cascadeSim = new ElevatorSim(CascadeConstants.kCascadeDriverConfiguration.kV,
                // CascadeConstants.kCascadeDriverConfiguration.kA, cascadeMotors.getGearbox(),
                // 0.0,
                // Units.feetToMeters(6.0), true, 0.0, new double[] { 0.0, 0.0, 0.0 });

                cascadeSim = new ElevatorSim(cascadeMotors.getGearbox(),
                                CascadeConstants.kCascadeDriverConfiguration.gearRatio.getMathematicalGearRatio(), 15.0,
                                Units.inchesToMeters(0.5), CascadeConstants.kCascadeMinimumHeight.in(Meters),
                                CascadeConstants.kCascadeMaximumHeight.in(Meters), true, 0.0);

                pivotSim = new SingleJointedArmSim(pivotMotors.getGearbox(),
                                CascadeConstants.kPivotConfiguration.gearRatio.getMathematicalGearRatio(),
                                cascadeMotorInertia, Units.feetToMeters(3.5), 0, Math.PI, true, Math.PI / 2.0);

                cascadeController = new ProfiledPIDController(CascadeConstants.kCascadeDriverConfiguration.kP,
                                CascadeConstants.kCascadeDriverConfiguration.kI,
                                CascadeConstants.kCascadeDriverConfiguration.kD,
                                new Constraints(CascadeConstants.kCascadeDriverConfiguration.maxVelocity,
                                                CascadeConstants.kCascadeDriverConfiguration.maxAcceleration));
                cascadeFeedforward = new ElevatorFeedforward(CascadeConstants.kCascadeDriverConfiguration.kS,
                                CascadeConstants.kCascadeDriverConfiguration.kG,
                                CascadeConstants.kCascadeDriverConfiguration.kV / (2*Math.PI), CascadeConstants.kCascadeDriverConfiguration.kA / (2*Math.PI));

                pivotController = new ProfiledPIDController(CascadeConstants.kPivotConfiguration.kP,
                                CascadeConstants.kPivotConfiguration.kI, CascadeConstants.kPivotConfiguration.kD,
                                new Constraints(CascadeConstants.kPivotConfiguration.maxVelocity,
                                                CascadeConstants.kPivotConfiguration.maxAcceleration));

                pivotFeedforward = new ArmFeedforward(CascadeConstants.kPivotConfiguration.kS,
                                CascadeConstants.kPivotConfiguration.kG, CascadeConstants.kPivotConfiguration.kV / (2*Math.PI),
                                CascadeConstants.kPivotConfiguration.kA / (2*Math.PI));
        }

        @Override
        public void updateInputs(CascadeIOInputs inputs) {

                cascadeMotors.update(Constants.kLoopSpeed);
                pivotMotors.update(Constants.kLoopSpeed);

                cascadeSim.setInputVoltage(cascadeMotors.getInputVoltage());
                pivotSim.setInputVoltage(pivotMotors.getInputVoltage());

                cascadeSim.update(Constants.kLoopSpeed);
                pivotSim.update(Constants.kLoopSpeed);

                cascadeMotors.setState(cascadeSim.getOutput());
                pivotMotors.setState(pivotSim.getOutput());

                inputs.acceleration = LinearAcceleration.ofBaseUnits(
                                cascadeMotors.getAngularAcceleration().in(RotationsPerSecondPerSecond)
                                                * CascadeConstants.kCascadeDriverConfiguration.finalDiameterMeters
                                                * Math.PI,
                                MetersPerSecondPerSecond);

                inputs.velocity = LinearVelocity.ofBaseUnits(
                                cascadeMotors.getAngularVelocity().in(RotationsPerSecond)
                                                * CascadeConstants.kCascadeDriverConfiguration.finalDiameterMeters
                                                * Math.PI,
                                MetersPerSecond);

                inputs.velocityRotations = cascadeMotors.getAngularVelocity();

                // inputs.cascadeHeightMeters = Distance.ofBaseUnits(
                // cascadeMotors.getAngularPosition().in(Rotations)
                // * CascadeConstants.kCascadeDriverConfiguration.finalDiameterMeters
                // * Math.PI,
                // Meters);
                inputs.cascadeHeightMeters = Distance.ofBaseUnits(cascadeSim.getPositionMeters(), Meters);

                inputs.cascadeMasterMotorCurrent = Amps.of(cascadeMotors.getCurrentDrawAmps());
                inputs.cascadeSlaveMotorCurrent = Amps.of(cascadeMotors.getCurrentDrawAmps());

                inputs.cascadeMasterMotorVoltage = Volts.of(cascadeMotors.getInputVoltage());
                inputs.cascadeSlaveMotorVoltage = Volts.of(cascadeMotors.getInputVoltage());

                inputs.pivotRotation = Rotation2d.fromRotations(pivotMotors.getAngularPosition().in(Rotations));
                inputs.pivotRotationVelocity = pivotMotors.getAngularVelocity();
                inputs.pivotRotationAcceleration = pivotMotors.getAngularAcceleration();

                inputs.pivotMasterMotorCurrent = Amps.of(pivotMotors.getCurrentDrawAmps());
                inputs.pivotSlaveMotorCurrent = Amps.of(pivotMotors.getCurrentDrawAmps());

                inputs.pivotMasterMotorVoltage = Volts.of(pivotMotors.getInputVoltage());
                inputs.pivotSlaveMotorVoltage = Volts.of(pivotMotors.getInputVoltage());

        }

        @Override
        public void setCascadeSetpoint(Distance setpoint) {

                if (cascadeMotorsStopped) {
                        cascadeMotors.setAngularVelocity(0.0);
                        return;
                }
                cascadeMotors.setInputVoltage(cascadeController.calculate(cascadeSim.getPositionMeters(),
                                setpoint.in(Meters))
                                + cascadeFeedforward.calculate(cascadeSim.getVelocityMetersPerSecond()));

        }

        @Override
        public void setCascadeEncoderPosition(Distance setpoint) {
                cascadeMotors.setAngle(0.0);
                cascadeSim.setState(0.0, 0.0);
        }

        @Override
        public void setPivotSetpoint(Rotation2d setpoint) {
                if (pivotMotorsStopped) {
                        pivotMotors.setAngularVelocity(0.0);
                        return;
                }
                pivotMotors.setInputVoltage(pivotController.calculate(pivotMotors.getAngularPositionRotations(),
                                setpoint.getRotations())
                                + pivotFeedforward.calculate(pivotMotors.getAngularPositionRad(),
                                                (pivotMotors.getAngularVelocityRPM() / 60.0)
                                                                - pivotController.getVelocityError()));
        }

        @Override
        public void setCANCoderPosition(Rotation2d angle) {
                pivotMotors.setAngle(angle.getRadians());
        }

        @Override
        public void stopCascadeMotors(boolean stopped) {
                this.cascadeMotorsStopped = stopped;
        }

        @Override
        public void stopPivotMotors(boolean stopped) {
                this.pivotMotorsStopped = stopped;
        }

}