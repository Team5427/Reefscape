package team5427.lib.detection.tuples;

public class Tuple5Plus<R, T, A, B, C> {
  public R r;
  public T t;
  public A a;
  public B b;
  public C c;

  public Tuple5Plus(R r, T t, A a, B b, C c) {
    this.r = r;
    this.t = t;
    this.a = a;
    this.b = b;
    this.c = c;
  }

  public String toString() {
    return r.toString() + t.toString() + a.toString() + b.toString() + c.toString();
  }
}
