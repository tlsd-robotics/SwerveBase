package frc.robot;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Superstructure {
    private final PneumaticHub pneumatics = new PneumaticHub(Constants.PNEUMATICS_HUB_ID);
    private final  PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

    public Superstructure() {
        pneumatics.enableCompressorDigital();
        pdp.clearStickyFaults();
        
    }
}
