package team5427.lib.drivers;

public class ComplexGearRatio {

  private double[] stages;

  public ComplexGearRatio() {
    stages = new double[] {1.0};
  }

  public ComplexGearRatio(double... stages) {
    this.stages = stages;
  }

  public double getMathematicalGearRatio() {
    double gearing = stages[0];

    for (int i = 1; i < stages.length; i++) {
      gearing *= stages[i];
    }

    return gearing;
  }

  public double getSensorToMechanismRatio() {
    double gearing = Math.pow(stages[0], -1);

    for (int i = 1; i < stages.length; i++) {
      gearing *= Math.pow(stages[i], -1);
    }

    return gearing;
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof ComplexGearRatio) {
      return ((ComplexGearRatio) obj).getMathematicalGearRatio() == this.getMathematicalGearRatio()
          && ((ComplexGearRatio) obj).getSensorToMechanismRatio()
              == this.getSensorToMechanismRatio();
    }
    return false;
  }
}
