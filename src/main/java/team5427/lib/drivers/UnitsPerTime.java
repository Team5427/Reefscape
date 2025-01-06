package team5427.lib.drivers;
// package team5427.lib.drivers;

// import java.util.Objects;

// import org.littletonrobotics.junction.AutoLog;
// import org.littletonrobotics.junction.LogTable;
// import org.littletonrobotics.junction.inputs.LoggableInputs;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.util.Units;

// public class UnitsPerTime implements LoggableInputs {

//     private Rotation2d rotations;
//     private double lastConvertedValue;
//     private double rotatingDiameterMeters;
//     private double timeSeconds;

//     public UnitsPerTime() {
//         rotations = new Rotation2d(0);
//         timeSeconds = 1;
//     }

//     public UnitsPerTime(Rotation2d rotations, double timeSeconds) throws IllegalArgumentException {
//         if (timeSeconds <= 0)
//             throw new IllegalArgumentException(
//                     "Error: UnitsPerTime cannot accept a time value less than or equal to 0");
//         this.timeSeconds = timeSeconds;
//         this.rotations = rotations;
//     }

//     public UnitsPerTime(double rotatingDiameterMeters, Rotation2d rotations, double timeSeconds)
//             throws IllegalArgumentException {
//         if (timeSeconds <= 0)
//             throw new IllegalArgumentException(
//                     "Error: UnitsPerTime cannot accept a time value less than or equal to 0");
//         if (rotatingDiameterMeters <= 0)
//             throw new IllegalArgumentException(
//                     "Error: UnitsPerTime cannot accept a rotating diameter value less than or equal to 0");
//         this.timeSeconds = timeSeconds;
//         this.rotatingDiameterMeters = rotatingDiameterMeters;
//         this.rotations = rotations;
//     }

//     public static UnitsPerTime fromRotation2dPerMinutes(Rotation2d rotations, double timeMinutes) {
//         return new UnitsPerTime(rotations, timeMinutes * 60.0);
//     }

//     public static UnitsPerTime fromRotation2dPerHour(Rotation2d rotations, double timeHours) {
//         return new UnitsPerTime(rotations, timeHours * 60.0 * 60.0);
//     }

//     public static UnitsPerTime fromMetersPerMinute(double rotatingMeters, double meters, double timeMinutes) {
//         return UnitsPerTime.fromMetersPerSecond(rotatingMeters, meters, timeMinutes / 60.0);
//     }

//     public static UnitsPerTime fromMetersPerSecond(double rotatingDiameterMeters, double metersTraveled,
//             double timeSeconds) {

//         Rotation2d rotations = new Rotation2d(metersTraveled / rotatingDiameterMeters * 2.0);
//         return new UnitsPerTime(rotatingDiameterMeters, rotations, timeSeconds);
//     }

//     public UnitsPerTime plus(UnitsPerTime other) {
//         UnitsPerTime newUnits = new UnitsPerTime(this.rotations.plus(other.rotations),
//                 this.timeSeconds + other.timeSeconds);
//         return newUnits;
//     }

//     public UnitsPerTime unaryMinus() {
//         return new UnitsPerTime(this.rotations.unaryMinus(), this.timeSeconds);
//     }

//     public UnitsPerTime times(double scalar) {
//         return new UnitsPerTime(this.rotations.times(scalar), this.timeSeconds);
//     }

//     public UnitsPerTime div(double scalar) {
//         return new UnitsPerTime(this.rotations.div(scalar), this.timeSeconds);
//     }

//     public UnitsPerTime minus(UnitsPerTime other) {
//         return plus(other.unaryMinus());
//     }

//     public UnitsPerTime getRotations() {
//         lastConvertedValue = rotations.getRotations();
//         return this;
//     }

//     public UnitsPerTime getRadians() {
//         lastConvertedValue = rotations.getRadians();
//         return this;
//     }

//     public UnitsPerTime getDegrees() {
//         lastConvertedValue = rotations.getDegrees();
//         return this;
//     }

//     public UnitsPerTime getMeters() {
//         lastConvertedValue = rotations.getRadians() * rotatingDiameterMeters / 2.0;
//         return this;
//     }

//     public UnitsPerTime getCentiMeters() {

//         lastConvertedValue = getMeters().lastConvertedValue * 100.0;
//         return this;
//     }

//     public UnitsPerTime getKiloMeters() {
//         lastConvertedValue = getMeters().lastConvertedValue / 1000.0;
//         return this;
//     }

//     public UnitsPerTime getInches() {
//         lastConvertedValue = Units.metersToInches(getMeters().lastConvertedValue);
//         return this;
//     }

//     public UnitsPerTime getFeet() {
//         lastConvertedValue = Units.metersToFeet(getMeters().lastConvertedValue);
//         return this;
//     }

//     public double getPerMilliSeconds() {
//         return getPerSeconds() / 1000.0;
//     }

//     public double getPerSeconds() {
//         return lastConvertedValue / timeSeconds;
//     }

//     public double getPerMinutes() {
//         return getPerSeconds() * 60.0;
//     }

//     public double getPerHours() {
//         return getPerMinutes() * 60.0;
//     }

//     public void setRotations(Rotation2d newRotation2d) {
//         this.rotations = newRotation2d;
//     }

//     public void setNewTime(double newTime) {
//         this.timeSeconds = newTime;
//     }

//     public void setRotatingDiameterMeters(double diameter) {
//         this.rotatingDiameterMeters = diameter;
//     }

//     @Override
//     public void toLog(LogTable table) {
//         table.put("Rotations", this.rotations);
//         // table.put("LastConvertedValue", this.lastConvertedValue);
//         table.put("RotatingDiameterMeters", this.rotatingDiameterMeters);
//         table.put("TimeSeconds", this.timeSeconds);
//     }

//     @Override
//     public void fromLog(LogTable table) {
//         this.rotations = table.get("Rotations", rotations);
//         // this.lastConvertedValue = table.get("LastConvertedValue",
//         // lastConvertedValue);
//         this.rotatingDiameterMeters = table.get("RotatingDiameterMeters", rotatingDiameterMeters);
//         this.timeSeconds = table.get("TimeSeconds", timeSeconds);
//     }

//     public UnitsPerTime clone() {
//         UnitsPerTime copy = new UnitsPerTime();
//         copy.setRotations(this.rotations);
//         // copy.lastConvertedValue = this.lastConvertedValue;
//         copy.setRotatingDiameterMeters(this.rotatingDiameterMeters);
//         copy.setNewTime(this.timeSeconds);
//         return copy;
//     }

//     @Override
//     public int hashCode() {
//         return Objects.hash(rotations, timeSeconds);
//     }
// }
