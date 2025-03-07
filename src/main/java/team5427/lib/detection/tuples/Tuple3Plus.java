package team5427.lib.detection.tuples;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class Tuple3Plus<R, T, A> implements TuplePlus {
  public R r;
  public T t;
  public A a;
  Object[] array;
  List<Object> list;

  public Tuple3Plus(R r, T t, A a) {
    this.r = r;
    this.t = t;
    this.a = a;

    array = new Object[] {r, t, a};
    list = new LinkedList<Object>(Arrays.asList(array));
  }

  public String toString() {
    return r.toString() + t.toString() + a.toString();
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
