package team5427.lib.drivers;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SteelTalonsLogger {

    private static final String kLoggerTab = "5427Logger";
    private static ShuffleboardTab loggerTab = Shuffleboard.getTab(kLoggerTab);
    private static HashMap<String, GenericEntry> widgetList = new HashMap<>();
    private static StructArrayPublisher<SwerveModuleState> swervePublisher;

    static {
        swervePublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/MyStates", SwerveModuleState.struct).publish();
    }

    public static boolean post(String key, Object obj) {
        if (!widgetList.containsKey(key)) {
            widgetList.put(key, loggerTab.add(key, obj).getEntry());
            return widgetList.get(key).setValue(obj);
        } else {
            return widgetList.get(key).setValue(obj);
        }
    }

    public static void postComplex(String key, Sendable sendable) {
        try {
            loggerTab.add(key, sendable);
        } catch (Exception e) {
            return;
        }
    }

    // Logs the pose of an object into network tables for Advantage Scope to Utilize
    public static void post3dPose(Pose3d... poses) {
        StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Pose Array", Pose3d.struct).publish();
        arrayPublisher.set(poses);
    }

    public static void post2dPose(Pose2d... poses) {
        StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Pose Array", Pose2d.struct).publish();
        arrayPublisher.set(poses);
    }

    public static void logPoint(Translation2d... points) {
        StructArrayPublisher<Translation2d> publisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("MyTranslations", Translation2d.struct).publish();
        publisher.set(points);

    }

    public static void logSwerve(SwerveModuleState... states) {

        swervePublisher.set(states);

    }

    public static void logJoystickData() { // Still needs testing to verify its functionality
        // Starts recording to data log
        DataLogManager.start();
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
    }

}