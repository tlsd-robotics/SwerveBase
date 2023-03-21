package frc.robot.subsystems;

import Common.Limelight;
import Common.LimelightPipeline;
import Common.Target;
import Common.Target.DetectionType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

public static class Pipelines {
  static LimelightPipeline defaultPipeline = new LimelightPipeline(0, false);
}

public static class Limelights {
  static Limelight defaultLimelight = new Limelight("limelight", Pipelines.defaultPipeline, 15, 15);
}

public static class Targets {
  static Target exampleTarget = new Target(36, DetectionType.Reflective, 0.5);
}

  public VisionSubsystem() {}
  @Override

  public void periodic() {}

 
}
