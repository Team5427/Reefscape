package team5427.frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class RawScoringConfiguration {

    private final Rotation2d cascadeAngle;
    private final Distance cascadeHeight;
    private final Rotation2d wristAngle;

    public RawScoringConfiguration() {
        this.cascadeAngle = Rotation2d.kZero;
        this.cascadeHeight = Meters.of(0.0);
        this.wristAngle = Rotation2d.kZero;
    }

    public RawScoringConfiguration(Rotation2d cascadeAngle, Distance cascadeHeight, Rotation2d wristAngle) {
        this.cascadeAngle = cascadeAngle;
        this.cascadeHeight = cascadeHeight;
        this.wristAngle = wristAngle;
    }

    public Rotation2d getCascadeAngle() {
        return cascadeAngle;
    }

    public Distance getCascadeHeight() {
        return cascadeHeight;
    }

    public Rotation2d getWristAngle() {
        return wristAngle;
    }
    
}
