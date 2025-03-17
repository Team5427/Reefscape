package team5427.frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import team5427.frc.robot.Constants.VisionConstants;

public class QuestNav {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);
  private static QuestNav questInstance;
  public static QuestNav getInstance(){
    if(questInstance == null){
      questInstance = new QuestNav();
    }
    return questInstance;
  }

  public QuestNav(){
    
  }

  
  public void setResetPose(Pose2d resetPose){
    this.resetPosition = resetPose;
  }
  // Local heading helper variables
  private float yaw_offset = 0.0f;
  private Pose2d resetPosition = new Pose2d();

  // Gets the Quest's measured position.
  public Pose2d getPose() {
    return new Pose2d(getQuestNavPose().minus(resetPosition).getTranslation(), Rotation2d.fromDegrees(getOculusYaw())).transformBy(new Transform2d(VisionConstants.kQuestCameraTransform.getX(), VisionConstants.kQuestCameraTransform.getY(), VisionConstants.kQuestCameraTransform.getRotation().toRotation2d()));
  }

  // Gets the battery percent of the Quest.
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  // Returns if the Quest is connected.
  public boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Gets the Quaternion of the Quest.
  public Quaternion getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
  }

  // Gets the Quests's timestamp in NT Server Time.
  public double timestamp() {
    return questTimestamp.getAtomic().serverTime;
  }

  // Zero the relativerobot heading
  public void zeroHeading() {
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
  }

  // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
  public void zeroPosition() {
    resetPosition = getPose();
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  // Clean up questnav subroutine messages after processing on the headset
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  // Get the yaw Euler angle of the headset
  private float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1] - yaw_offset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return ret;
  }

  private Translation2d getQuestNavTranslation() {
    float[] questnavPosition = questPosition.get();
    return new Translation2d(questnavPosition[2], -questnavPosition[0]);
  }

  private Pose2d getQuestNavPose() {
    var oculousPositionCompensated = getQuestNavTranslation().minus(new Translation2d(0, 0.1651)); // 6.5
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
  }
}