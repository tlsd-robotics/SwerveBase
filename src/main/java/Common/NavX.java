package Common;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

public class NavX {

  public static AHRS gyroscope = new AHRS(Port.kMXP, (byte) 200);


  public static AHRS getNavX() {
      return gyroscope;
  }

  public static void zeroYaw() {
      gyroscope.zeroYaw();
  }

}
