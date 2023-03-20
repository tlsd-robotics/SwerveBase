package frc.robot;

import Common.ThrustMaster;
import Common.Utilities;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {

//==============================================================================
//======================== Create Subsystem Instance ===========================
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  private final ThrustMaster driveJoy = new ThrustMaster(Constants.DRIVE_CONTROLLER_ID);

  public RobotContainer() {
      drivetrain.register();

      drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, this::getForwardInput, this::getStrafeInput, this::getRotationInput, this::getThrottleInput));

      configureBindings();
  }


//==============================================================================
//=============================== Getter Methods ===============================
  public DrivetrainSubsystem getDrivetrain() {
    return drivetrain;
  }

  public ThrustMaster getDriveJoystick() {
    return driveJoy;
  }


//==============================================================================
//=========================== Configure Bindings ===============================
  private void configureBindings() {
    
    driveJoy.getMiddle().onTrue(new RunCommand(drivetrain::zeroGyroscope));
  }


//==============================================================================
//=========================== Modify Joystick Input ============================
  private static double deadband(double value, double tolerance) {
    if (Math.abs(value) < tolerance) {
      return 0.0;
    }
      return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
  }

  private static double square(double value) {
    return Math.copySign(value * value, value);
  }


//============================================================================
//============================ Get Joystick Input ============================
  private double getForwardInput() {
    return -square(deadband(driveJoy.getY(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
  }

  private double getStrafeInput() {
    return -square(deadband(driveJoy.getX(), 0.1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
  }
    
  private double getRotationInput() {
    return -square(deadband(driveJoy.getZ(), 0.1)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  }
  
  private double getThrottleInput() {
    return Utilities.map(driveJoy.getThrottle(), 1, -1, 0, 1);
  }
}
