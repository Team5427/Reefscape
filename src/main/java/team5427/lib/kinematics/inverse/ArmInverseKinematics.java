package team5427.lib.kinematics.inverse;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Inverse kinematics for robotic arms. This is for arms with fixed arm segments and 2 joints. This
 * DOES NOT recalculate if joint angles are calculated to violate constraints. Target Location is
 * given relative to the first joint
 */
public class ArmInverseKinematics {
  /** 2 Fixed length arms with variable angles */
  public class FixedLengthArm {
    private Rotation2d[] finalAngles;
    private double[] segmentLengths;
    private Rotation2d[] minAngles;
    private Rotation2d[] maxAngles;
    private Translation3d targetLocation;

    public FixedLengthArm(
        double[] segmentLengths,
        Rotation2d[] minAngles,
        Rotation2d[] maxAngles,
        Translation3d targetLocation) {

      this.segmentLengths = segmentLengths;
      this.minAngles = minAngles;
      this.maxAngles = maxAngles;
      this.targetLocation = targetLocation;

      finalAngles = new Rotation2d[minAngles.length];

      if (!areAngleLengthsValid()) {
        throw new IllegalArgumentException(
            "Error: The number of min angles, max angles, and segment lengths must be equal");
      }
    }

    public FixedLengthArm(double[] segmentLengths, Translation3d targetLocation) {
      this.segmentLengths = segmentLengths;
      this.targetLocation = targetLocation;

      int length = segmentLengths.length;
      this.minAngles = new Rotation2d[length];
      this.maxAngles = new Rotation2d[length];
      this.finalAngles = new Rotation2d[length];

      for (int i = 0; i < length; i++) {
        this.minAngles[i] = new Rotation2d(0);
        this.maxAngles[i] = Rotation2d.fromDegrees(360);
      }
      finalAngles = new Rotation2d[minAngles.length];
    }

    public Rotation2d[] getAngles() {
      double dist = targetLocation.getDistance(new Translation3d(0, 0, 0));
      // second angle
      double gamma =
          Math.acos(
              (Math.pow(segmentLengths[0], 2) + Math.pow(segmentLengths[1], 2) - Math.pow(dist, 2))
                  / (2.0 * segmentLengths[0] * segmentLengths[1]));
      // first angle
      double theta =
          Math.acos(
              (Math.pow(segmentLengths[0], 2) + Math.pow(dist, 2) - Math.pow(segmentLengths[1], 2))
                  / (2.0 * segmentLengths[0] * dist));

      finalAngles[0] = Rotation2d.fromDegrees(theta);
      finalAngles[1] = Rotation2d.fromDegrees(gamma);
      return finalAngles;
    }

    private boolean areAngleLengthsValid() {
      return minAngles.length == maxAngles.length && minAngles.length == segmentLengths.length;
    }
  }

  /** 1 Fixed length arm and 1 Fixed angle with 1 variable angle and 1 variable length */
  public class VariableLengthArm {
    private Rotation2d finalAngle;
    private double finalLength;
    private Rotation2d fixedAngle;
    private double[] segmentLengths;
    private Rotation2d minAngle;
    private Rotation2d maxAngle;
    private Translation3d targetLocation;

    // public VariableLengthArm(
    //     double[] segmentLengths,
    //     Rotation2d[] minAngles,
    //     Rotation2d[] maxAngles,
    //     Translation3d targetLocation) {

    //   this.segmentLengths = segmentLengths;
    //   this.minAngles = minAngles;
    //   this.maxAngles = maxAngles;
    //   this.targetLocation = targetLocation;

    //   finalAngles = new Rotation2d[minAngles.length];

    //   if (!areAngleLengthsValid()) {
    //     throw new IllegalArgumentException(
    //         "Error: The number of min angles, max angles, and segment lengths must be equal");
    //   }
    // }

    // public VariableLengthArm(double[] segmentLengths, Translation3d targetLocation) {
    //   this.segmentLengths = segmentLengths;
    //   this.targetLocation = targetLocation;

    //   int length = segmentLengths.length;
    //   this.minAngles = new Rotation2d[length];
    //   this.maxAngles = new Rotation2d[length];
    //   this.finalAngles = new Rotation2d[length];

    //   for (int i = 0; i < length; i++) {
    //     this.minAngles[i] = new Rotation2d(0);
    //     this.maxAngles[i] = Rotation2d.fromDegrees(360);
    //   }
    //   finalAngles = new Rotation2d[minAngles.length];
    // }

    // public Rotation2d[] getAngles() {
    //   double dist = targetLocation.getDistance(new Translation3d(0, 0, 0));
    //   // second angle
    //   double gamma =
    //       Math.acos(
    //           (Math.pow(segmentLengths[0], 2) + Math.pow(segmentLengths[1], 2) - Math.pow(dist,
    // 2))
    //               / (2.0 * segmentLengths[0] * segmentLengths[1]));
    //   // first angle
    //   double theta =
    //       Math.acos(
    //           (Math.pow(segmentLengths[0], 2) + Math.pow(dist, 2) - Math.pow(segmentLengths[1],
    // 2))
    //               / (2.0 * segmentLengths[0] * dist));

    //   finalAngles[0] = Rotation2d.fromDegrees(theta);
    //   finalAngles[1] = Rotation2d.fromDegrees(gamma);
    //   return finalAngles;
    // }

    // private boolean areAngleLengthsValid() {
    //   return minAngles.length == maxAngles.length && minAngles.length == segmentLengths.length;
    // }
  }
}
