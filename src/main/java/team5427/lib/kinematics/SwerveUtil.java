package team5427.lib.kinematics;

import team5427.lib.drivers.CANDeviceId;
import team5427.lib.drivers.ComplexGearRatio;

public class SwerveUtil {

  public static final int kDefaultNumModules = 4;

  public static final int kFrontLeftModuleIdx = 0;
  public static final int kFrontRightModuleIdx = 1;
  public static final int kRearLeftModuleIdx = 2;
  public static final int kRearRightModuleIdx = 3;

  public static final ComplexGearRatio kSDSSteerGearRatioMK4 =
      new ComplexGearRatio((15.0 / 32.0), (10.0 / 60.0));
  public static final ComplexGearRatio kSDSSteerGearRatioMK4i =
      new ComplexGearRatio(1.0 / (150.0 / 7.0));
  public static final ComplexGearRatio kSDSSteerGearRatioMK4n =
      new ComplexGearRatio((16.0 / 50.0), (10.0 / 60.0));
  // public static final ComplexGearRatio kSDSSteerGearRatioMK4n =
  //     new ComplexGearRatio(1.0 / 18.75);
  public static final ComplexGearRatio kSDSL1GearRatio =
      new ComplexGearRatio((14.0 / 50.0), (25.0 / 19.0), (15.0 / 45.0));
  public static final ComplexGearRatio kSDSL2GearRatio =
      new ComplexGearRatio((14.0 / 50.0), (27.0 / 17.0), (15.0 / 45.0));
  public static final team5427.lib.drivers.ComplexGearRatio kSDSL3GearRatio =
      new ComplexGearRatio((14.0 / 50.0), (28.0 / 16.0), (15.0 / 45.0));
  public static final ComplexGearRatio kSDSL4GearRatio =
      new ComplexGearRatio((16.0 / 48.0), (28.0 / 16.0), (15.0 / 45.0));

  public final CANDeviceId[] kSteerMotorIds = new CANDeviceId[4];
  public final CANDeviceId[] kDriveMotorIds = new CANDeviceId[4];
  public final CANDeviceId[] kCancoderIds = new CANDeviceId[4];
  public final double[] kModuleOffsets = new double[4];
  public final boolean[] kSteerInversion = new boolean[4];
  public final boolean[] kDriveInversion = new boolean[4];
}
