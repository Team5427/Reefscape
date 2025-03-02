package team5427.lib.detection;

public class Tuple2Plus<R, T> {
  public R r;
  public T t;

  public Tuple2Plus(R r, T t) {
    this.r = r;
    this.t = t;
  }

  public String toString() {
    return r.toString() + t.toString();
  }
}
