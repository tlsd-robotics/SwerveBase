package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier throttleSupplier;

    public DefaultDriveCommand(
            DrivetrainSubsystem drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            DoubleSupplier throttleSupplier
    ) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.throttleSupplier = throttleSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double translationXPercent = translationXSupplier.getAsDouble();
        double translationYPercent = translationYSupplier.getAsDouble();
        double rotationPercent = rotationSupplier.getAsDouble();
        double throttlePercent = throttleSupplier.getAsDouble();

        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * throttlePercent,
                        translationYPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * throttlePercent,
                        rotationPercent * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * throttlePercent,
                        drivetrain.getRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
