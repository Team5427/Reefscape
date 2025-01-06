package team5427.lib.filters;

public class AprilTagFilter { // Will add Complex April Tag filter as well
    private double ambiguity;
    private double threshold;

    public AprilTagFilter() {
        this.ambiguity = 0.0;
        this.threshold = 0.0;
    }

    public AprilTagFilter(double ambiguity, double threshold) {
        this.ambiguity = ambiguity;
        this.threshold = threshold;
    }

    public double filter(double value) {
        if (value < threshold) {
            return 0.0;
        } else {
            return value;
        }
    }

    public double filterWithAmbiguity(double value) {
        if (value < threshold) {
            return ambiguity;
        } else {
            return value;
        }
    }

    public double getAmbiguity() {
        return ambiguity;
    }

    public double getThreshold() {
        return threshold;
    }

    public void setAmbiguity(double ambiguity) {
        this.ambiguity = ambiguity;
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }
}
