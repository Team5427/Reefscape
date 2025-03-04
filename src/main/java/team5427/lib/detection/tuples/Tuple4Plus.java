package team5427.lib.detection.tuples;

public class Tuple4Plus<R, T, A, B> {
  public R r;
  public T t;
  public A a;
  public B b;

  public Tuple4Plus(R r, T t, A a, B b) {
    this.r = r;
    this.t = t;
    this.a = a;
    this.b = b;
  }

  public String toString() {
    return r.toString() + t.toString() + a.toString() + b.toString();
  }
}
