package team5427.frc.robot.subsystems.Swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import team5427.frc.robot.Constants;
import team5427.frc.robot.subsystems.Swerve.PhoenixOdometryThread;

public class GyroIOPigeon implements GyroIO {
  private Pigeon2 gyro;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOPigeon() {
    gyro =
        new Pigeon2(
            Constants.SwerveConstants.kPigeonCANId.getDeviceNumber(),
            Constants.SwerveConstants.kPigeonCANId.getBus());
    gyro.reset();
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.FutureProofConfigs = true;
    gyro.getConfigurator().apply(config);
    gyro.getConfigurator().setYaw(0.0);
    yaw = gyro.getYaw();
    yawVelocity = gyro.getAngularVelocityZWorld();
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.kOdometryFrequency, yawVelocity, yaw);
    gyro.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.connected  = gyro.isConnected();
    inputs.odometryYawTimestamp = yaw.getAllTimestamps().getDeviceTimestamp().getTime();
    inputs.yawPosition =
        Rotation2d.fromDegrees(
            BaseStatusSignal.getLatencyCompensatedValueAsDouble(yaw, yawVelocity));
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void resetGyroYawAngle(Rotation2d angle) {
    gyro.setYaw(angle.getDegrees());
  }
}
