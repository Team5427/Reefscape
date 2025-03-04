package team5427.lib.detection.tuples;

public class Tuple3Plus<R, T, A> {
  public R r;
  public T t;
  public A a;

  public Tuple3Plus(R r, T t, A a) {
    this.r = r;
    this.t = t;
    this.a = a;
  }

  public String toString() {
    return r.toString() + t.toString() + a.toString();
  }
}
