package team5427.frc.robot.subsystems.Vision.io.Quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.RobotState;
import team5427.frc.robot.Constants.VisionConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.lib.drivers.VirtualSubsystem;

public class Quest extends VirtualSubsystem {
  private final QuestIO io;
  private final QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Quest Disconnected!", AlertType.kWarning);
  private final Alert lowBatteryAlert =
      new Alert("Quest Battery is Low! (<25%)", AlertType.kWarning);

  private final QuestCalibration calibration = new QuestCalibration();

  private Pose2d fieldToRobotOrigin = Pose2d.kZero;

  private static Quest questInstance;

  public static Quest getInstance() {
    if (questInstance == null) {
      questInstance = new Quest();
    }
    return questInstance;
  }

  public Quest getInstance(QuestIO io) {
    if (questInstance == null) {
      questInstance = new Quest(io);
    }
    return questInstance;
  }

  private Quest() {
    this.io = new QuestIOReal();
    resetRobotPose(Pose2d.kZero);
  }

  private Quest(QuestIO io) {
    this.io = io;
    resetRobotPose(Pose2d.kZero);
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Oculus", inputs);

    disconnectedAlert.set(!inputs.connected);
    lowBatteryAlert.set(inputs.connected && inputs.batteryLevel < 25);

    Pose2d fieldToRobot = getFieldToRobot();

    // Only enable this when we know we're ready
    if (DriverStation.isEnabled() && Constants.currentMode == Constants.Mode.REAL) {
      //   RobotState.getInstance().addVisionMeasurment(fieldToRobot, inputs.timestamp,
      // VecBuilder.fill(VisionConstants.kQuestStdDevBaseline,
      // VisionConstants.kQuestStdDevBaseline));
    }

    // Do this always for now just to confirm our transforms are correct.
    // Or, you may want to always track rotation. Do science.
    if (inputs.connected) {
      RobotState.getInstance().addQuestMeasurment(fieldToRobot, inputs.timestamp);
    }
  }

  /**
   * The Oculus tracks relative to where it started, so we need to tell the Quest where the robot
   * started on the field.
   *
   * @param robotResetPose
   */
  public void resetRobotPose(Pose2d robotResetPose) {
    this.fieldToRobotOrigin = robotResetPose;
    io.zeroPosition();
    io.zeroHeading();
  }

  /**
   * Compares the current position of the headset to where it was last reset, then transforms that
   * by robot-to-quest and the known reset point on the field.
   *
   * @return Field to Quest
   */
  @AutoLogOutput
  public Pose2d getFieldToQuest() {
    return fieldToRobotOrigin
        .transformBy(QuestConstants.robotToQuestTransform)
        // .transformBy(new Transform2d(VisionConstants.kQuestCameraTransform.inverse().getX(), VisionConstants.kQuestCameraTransform.inverse().getY(), VisionConstants.kQuestCameraTransform.inverse().getRotation().toRotation2d()))
        .transformBy(inputs.uncorrectedResetToQuest);
  }

  /**
   * Using the known position of the Quest on the field, get the pose of the Robot relative to the
   * Quest.
   *
   * @return Robot to Field
   */
  @AutoLogOutput
  public Pose2d getFieldToRobot() {
    return getFieldToQuest().transformBy(QuestConstants.robotToQuestTransform.inverse());
  }

  public Command calibrateCommand(SwerveSubsystem swerve) {
    return calibration.determineOffsetToRobotCenter(
        swerve,
        () ->
            new Pose2d(
                inputs.uncorrectedResetToQuest.getTranslation(),
                inputs.uncorrectedResetToQuest.getRotation()));
  }
}
