package Common;

public class Utilities {
  
//Value Mapping
  public static double map(double Value, double FromMin, double FromMax, double ToMin, double ToMax) {
    return (((Value - FromMin) / (FromMax - FromMin)) * (ToMax - ToMin)) + ToMin;
  }
  
}
