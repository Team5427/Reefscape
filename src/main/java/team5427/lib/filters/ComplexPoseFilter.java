package team5427.lib.filters;
// package lib.filters;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;

// public class ComplexPoseFilter { // Probably not needed and/or not correct

// private LazyPoseFilter poseFilter;

// private ChassisSpeeds speeds;

// public ComplexPoseFilter() {
// poseFilter = new LazyPoseFilter();
// }

// public ComplexPoseFilter(double threshold, int degrees) {
// poseFilter = new LazyPoseFilter(threshold, degrees);
// }

// public Pose2d filter(Pose2d newPose, ChassisSpeeds speeds) {
// this.speeds = speeds;

// Pose2d futurePose = new Pose2d(
// newPose.getX() + speeds.vxMetersPerSecond,
// newPose.getY() + speeds.vyMetersPerSecond,
// newPose.getRotation().plus(new Rotation2d(speeds.omegaRadiansPerSecond)));

// return poseFilter.filter(futurePose);
// }

// public Pose2d filterWithReferences(Pose2d newPose, Pose2d[]
// externalReferences, ChassisSpeeds speeds) {
// this.speeds = speeds;

// Pose2d futurePose = new Pose2d(
// newPose.getX() + speeds.vxMetersPerSecond,
// newPose.getY() + speeds.vyMetersPerSecond,
// newPose.getRotation().plus(new Rotation2d(speeds.omegaRadiansPerSecond)));

// return poseFilter.filterWithReferences(futurePose, externalReferences);
// }

// public ChassisSpeeds getLastChassisSpeed() {
// return speeds;
// }

// }
