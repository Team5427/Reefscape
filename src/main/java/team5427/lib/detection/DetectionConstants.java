package team5427.lib.detection;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class DetectionConstants {
    public static final double kThreshold = 0.5;

    public static InterpolatingDoubleTreeMap taTreeMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap txTreeMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap tyTreeMap = new InterpolatingDoubleTreeMap();

    public void addTaData() {
        taTreeMap.put(0.0, 0.0);
        // add more based on calibration
    }

    public void addTxData() {
        txTreeMap.put(0.0, 0.0);
        // add more based on calibration
    }

    public void addTyData() {
        tyTreeMap.put(0.0, 0.0);
        // add more based on calibration
    }
}
