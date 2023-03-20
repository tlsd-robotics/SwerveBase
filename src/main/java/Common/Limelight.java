package Common;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Creates a limelight object with simple data collection methods
 */
public class Limelight {

  public enum LedModes {
    LED_ON(3),
    LED_OFF(1);

    public final int id;

    LedModes(int id) {
      this.id = id;
    }
  }


// Instance Variables
  private NetworkTable table;
  private NetworkTableEntry ledMode;

  public double distanceFromGroundMeters;

  // =================== Limelight Constructor ======================
  private Limelight(String NetworkTableName, LimelightPipeline DefaultPipeline, double distanceFromGroundMeters) {

      this.table = NetworkTableInstance.getDefault().getTable(NetworkTableName);
      this.ledMode = table.getEntry("ledMode");
      this.distanceFromGroundMeters = distanceFromGroundMeters;

      setPipeline(DefaultPipeline);
      setLedOn(false);

  }

  // ========================================================================
  // ======================== Setters and Getters ===========================

  public void setPipeline(LimelightPipeline pipeline){
      this.table.getEntry("pipeline").setNumber(pipeline.id);
      ledMode.setNumber(pipeline.ledState ? LedModes.LED_ON.id : LedModes.LED_OFF.id);
  }

  public void setLedOn(boolean isOn) {
      ledMode.setNumber(isOn ? LedModes.LED_ON.id : LedModes.LED_OFF.id);
  }

    public double getTagID() {
      NetworkTableEntry tid = table.getEntry("tid");
      return tid.getDouble(0.0);
  }

  public double getTagYaw() {
      NetworkTableEntry camtran = table.getEntry("campose");
      double[] temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      double[] values = camtran.getDoubleArray(temp);
      return values[4];
  }
  /**
   * tv   Whether the limelight has any valid targets (0 or 1)
   * @return True if target found and vise-versa
   */
  public boolean getIsTargetFound() {
      NetworkTableEntry tv = table.getEntry("tv");
      double v = tv.getDouble(0);
      return (v == 0.0f ? false : true);
  }
  /**
   * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
   * @return Current Horizontal Error
   */
  public double getHorizontalError() {
      NetworkTableEntry tx = table.getEntry("tx");
      double x = tx.getDouble(0.0);
      return x;
  }
  /**
   * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
   * @return Current Vertical Error
   */
  public double getVerticalError() {
      NetworkTableEntry ty = table.getEntry("ty");
      double y = ty.getDouble(0.0);
      return y;
  }
  /**
   * ta Target Area (0% of image to 100% of image)
   * @return Area taken up by target
   */
  public double getTargetArea() {
      NetworkTableEntry ta = table.getEntry("ta");
      double a = ta.getDouble(0.0);
      return a;
  }

  public Integer getPipelineInt(){
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    Integer pipe = (int) pipeline.getDouble(0.0);
    return pipe;
}

public double[] getCorners() {
  return table.getEntry("tcornxy").getDoubleArray(new double[] {});
}

public double getDistanceToTarget(double targetHeightMeters) {
  return (targetHeightMeters - distanceFromGroundMeters) / Math.tan(Math.toRadians(getVerticalError()));
}

public NetworkTable getNetworkTable(Limelight limelight) {
  return table;
}
}
