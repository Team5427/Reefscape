package team5427.frc.robot.subsystems.Swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.SwerveConstants;

public class GyroIOSim implements GyroIO {
  private Pigeon2 gyro =
      new Pigeon2(
          SwerveConstants.kPigeonCANId.getDeviceNumber(), SwerveConstants.kPigeonCANId.getBus());
  private Pigeon2SimState gyroSimState = gyro.getSimState();
  private final StatusSignal<Angle> yaw = gyro.getYaw();
  private final StatusSignal<AngularVelocity> yawVelocity = gyro.getAngularVelocityZWorld();

  public GyroIOSim() {
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Constants.kLoopSpeed);
    yawVelocity.setUpdateFrequency(50.0);
    gyro.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }
}
