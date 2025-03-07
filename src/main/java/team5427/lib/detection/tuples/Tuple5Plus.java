package team5427.lib.detection.tuples;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class Tuple5Plus<R, T, A, B, C> implements TuplePlus{
  public R r;
  public T t;
  public A a;
  public B b;
  public C c;
  Object[] array;
  List<Object> list;

  public Tuple5Plus(R r, T t, A a, B b,C c) {
    this.r = r;
    this.t = t;
    this.a = a;
    this.b = b;

    array = new Object[]{r,t,a, b, c};
    list = new LinkedList<Object>(Arrays.asList(array));
  }

  public String toString() {
    return r.toString() + t.toString() + a.toString() + b.toString() + c.toString();
  }

  @Override
  public Object[] toArray() {
    return array;
  }

  @Override
  public List<Object> toList() {
    return list;
  }
}
