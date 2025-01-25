package team5427.frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class ScoringConfiguration {
    /** Relative to the respective color origin */
    private Pose2d drivetrain;
    private Distance cascadeLength;
    private Rotation2d cascadePivotAngle;
    private Rotation2d endEffectorPivotAngle;
    private Rotation2d endEffectorWristAngle;
    private LinearVelocity coralFlywheelSpeed;
    private LinearVelocity algaeFlywheelSpeed;

    /** Relative to the robot center */
    private Pose3d robotRelativeEndEffectorPosition;

    /** Relative to the global blue alliance origin */
    private Pose3d fieldRelativeEndEffectorPosition;

    public ScoringConfiguration(Pose2d drivetrain, Distance cascadeLength, Rotation2d cascadePivotAngle,
            Rotation2d endEffectorPivotAngle, Rotation2d endEffectorWristAngle,
            LinearVelocity coralFlywheelSpeed, LinearVelocity algaeFlywheelSpeed) {
        this.drivetrain = drivetrain;
        this.cascadeLength = cascadeLength;
        this.cascadePivotAngle = cascadePivotAngle;
        this.endEffectorPivotAngle = endEffectorPivotAngle;
        this.endEffectorWristAngle = endEffectorWristAngle;
        this.coralFlywheelSpeed = coralFlywheelSpeed;
        this.algaeFlywheelSpeed = algaeFlywheelSpeed;
    }

    // /**
    //  * Location directly represents the midpoint of the coral rollers on the end effector.
    //  */
    // public Pose3d getRobotRelativeEndEffectorPosition(){

    // }
}
