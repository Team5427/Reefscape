package team5427.lib.detection;

import edu.wpi.first.math.geometry.Rotation2d;

public final class DetectionMath {
    // Prevent Instantiation
    private DetectionMath() {
    }

    public static double sigmoid(double x) {
        return 1 / (1 + Math.exp(-x));
    }

    public static double distance(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public static Rotation2d angleFromXandY(double x, double y) {
        return new Rotation2d(Math.atan2(x, y));
    }

    public static double[] averagedPoints(double x1, double y1, double x2, double y2) {
        double[] averagedPoint = new double[2];
        averagedPoint[0] = (x1 + x2) / 2;
        averagedPoint[1] = (y1 + y2) / 2;
        return averagedPoint;
    }

    /*
     * Bias means the bias for x1 and y1
     * 0, favors true midpoint;
     * >0, favors point1;
     * <0, favors point2;
     */
    public static double[] averagedPoints(double x1, double y1, double x2, double y2, double bias) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double vx = dx * bias;
        double vy = dy * bias;
        x2 -= vx;
        y2 -= vy;
        double[] averagedPoint = new double[2];
        averagedPoint[0] = (x1 + x2) / 2;
        averagedPoint[1] = (y1 + y2) / 2;
        return averagedPoint;
    }

    /*
     * BiasX means the bias for x1
     * BiasY means bias for y1
     * 0, favors true midpoint;
     * >0, favors point1;
     * <0, favors point2;
     */
    public static double[] averagedPoints(double x1, double y1, double x2, double y2, double biasX, double biasY) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double vx = dx * biasX;
        double vy = dy * biasY;
        x2 -= vx;
        y2 -= vy;
        double[] averagedPoint = new double[2];
        averagedPoint[0] = (x1 + x2) / 2;
        averagedPoint[1] = (y1 + y2) / 2;
        return averagedPoint;
    }
}
