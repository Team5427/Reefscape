package team5427.lib.detection.tuples;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class Tuple2Plus<R, T> implements TuplePlus {
  public R r;
  public T t;
  Object[] array;
  List<Object> list;

  public Tuple2Plus(R r, T t) {
    this.r = r;
    this.t = t;

    array = new Object[] {r, t};
    list = new LinkedList<Object>(Arrays.asList(array));
  }

  public String toString() {
    return r.toString() + t.toString();
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
