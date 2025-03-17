package team5427.lib.kinematics.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Getter;
import lombok.Setter;

/**
 * Constrained Chassis Speeds are Chassis Speeds with velocity maximums in the x, y, or omega
 * directions. Constrained Speeds limit the maximum speeds of the x, y, and omega directions,
 * without regarding absolute position.
 */
public class ConstrainedChassisSpeeds {

  @Getter @Setter private double xConstraint;
  @Getter @Setter private double yConstraint;
  @Getter @Setter private double omegaConstraint;

  public ConstrainedChassisSpeeds(double xConstraint, double yConstraint, double omegaConstraint) {
    this.xConstraint = xConstraint;
    this.yConstraint = yConstraint;
    this.omegaConstraint = omegaConstraint;
  }

  public ChassisSpeeds calculate(ChassisSpeeds speeds) {
    if (Math.abs(speeds.vxMetersPerSecond) > Math.abs(xConstraint))
      speeds.vxMetersPerSecond = xConstraint;
    if (Math.abs(speeds.vyMetersPerSecond) > Math.abs(yConstraint))
      speeds.vyMetersPerSecond = yConstraint;
    if (Math.abs(speeds.omegaRadiansPerSecond) > Math.abs(omegaConstraint))
      speeds.omegaRadiansPerSecond = omegaConstraint;
    return speeds;
  }
}
