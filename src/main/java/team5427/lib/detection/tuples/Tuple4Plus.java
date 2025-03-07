package team5427.lib.detection.tuples;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class Tuple4Plus<R, T, A, B> implements TuplePlus{
  public R r;
  public T t;
  public A a;
  public B b;
  Object[] array;
  List<Object> list;

  public Tuple4Plus(R r, T t, A a, B b) {
    this.r = r;
    this.t = t;
    this.a = a;
    this.b = b;

    array = new Object[]{r,t,a, b};
    list = new LinkedList<Object>(Arrays.asList(array));
  }

  public String toString() {
    return r.toString() + t.toString() + a.toString() + b.toString();
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
