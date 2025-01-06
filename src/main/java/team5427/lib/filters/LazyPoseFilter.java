package team5427.lib.filters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LazyPoseFilter {

    private double threshold;

    private int degrees;
    private Pose2d[] references;

    public LazyPoseFilter() {

    }

    public LazyPoseFilter(double threshold, int degrees) {
        this.threshold = threshold;
        this.degrees = degrees;

        references = new Pose2d[this.degrees];
    }

    public Pose2d filter(Pose2d newPose) {
        double xCompound = newPose.getX();
        double yCompound = newPose.getY();
        double rotationCompound = newPose.getRotation().getRadians();

        for (Pose2d reference : references) {
            if (reference == null)
                continue;
            xCompound = (reference.getX() + xCompound) / 2.0;
            yCompound = (reference.getY() + yCompound) / 2.0;
            rotationCompound = (reference.getRotation().getRadians() + rotationCompound) / 2.0;
        }

        Pose2d robotPose;
        if (Math.abs(newPose.getX() - xCompound) <= threshold &&
                Math.abs(newPose.getY() - yCompound) <= threshold &&
                Math.abs(newPose.getRotation().getRadians() - rotationCompound) <= (threshold / 2.0)) {
            robotPose = newPose;
        } else {
            robotPose = new Pose2d(xCompound, yCompound, new Rotation2d(rotationCompound));
        }

        System.arraycopy(references, 1, references, 0, references.length - 1);
        references[references.length - 1] = robotPose;

        return robotPose;
    }

    public Pose2d filterWithReferences(Pose2d newPose, Pose2d[] externalReferences) {
        double xCompound = newPose.getX();
        double yCompound = newPose.getY();
        double rotationCompound = newPose.getRotation().getRadians();

        for (Pose2d reference : externalReferences) {
            if (reference == null)
                continue;
            xCompound = (reference.getX() + xCompound) / 2.0;
            yCompound = (reference.getY() + yCompound) / 2.0;
            rotationCompound = (reference.getRotation().getRadians() + rotationCompound) / 2.0;
        }

        if (Math.abs(newPose.getX() - xCompound) <= threshold &&
                Math.abs(newPose.getY() - yCompound) <= threshold &&
                Math.abs(newPose.getRotation().getRadians() - rotationCompound) <= (threshold / 2.0)) {
            return newPose;
        }

        return filter(new Pose2d(xCompound, yCompound, new Rotation2d(rotationCompound)));
    }

}
