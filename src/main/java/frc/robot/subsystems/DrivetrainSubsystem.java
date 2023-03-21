package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)) * 0.5;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final SwerveModule[] SwerveModules;

    private final AHRS gyroscope = Common.NavX.getNavX();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
    private final SwerveDriveOdometry odometry;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L3)
                .withDriveMotor(MotorType.NEO, Constants.FrontLeft.DRIVE_MOTOR)
                .withSteerMotor(MotorType.NEO, Constants.FrontLeft.STEER_MOTOR)
                .withSteerEncoderPort(Constants.FrontLeft.STEER_ENCODER)
                .withSteerOffset(Constants.FrontLeft.STEER_MOTOR)
                .build();

        frontRightModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L3)
                .withDriveMotor(MotorType.NEO, Constants.FrontRight.DRIVE_MOTOR)
                .withSteerMotor(MotorType.NEO, Constants.FrontRight.STEER_MOTOR)
                .withSteerEncoderPort(Constants.FrontRight.STEER_ENCODER)
                .withSteerOffset(Constants.FrontRight.STEER_OFFSET)
                .build();

        backLeftModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L3)
                .withDriveMotor(MotorType.NEO, Constants.BackLeft.DRIVE_MOTOR)
                .withSteerMotor(MotorType.NEO, Constants.BackLeft.STEER_MOTOR)
                .withSteerEncoderPort(Constants.BackLeft.STEER_ENCODER)
                .withSteerOffset(Constants.BackLeft.STEER_OFFSET)
                .build();

        backRightModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L3)
                .withDriveMotor(MotorType.NEO, Constants.BackRight.DRIVE_MOTOR)
                .withSteerMotor(MotorType.NEO, Constants.BackRight.STEER_MOTOR)
                .withSteerEncoderPort(Constants.BackRight.STEER_ENCODER)
                .withSteerOffset(Constants.BackRight.STEER_OFFSET)
                .build();

        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() }
        );

        SwerveModules = new SwerveModule[] {frontLeftModule, frontLeftModule, backLeftModule, backRightModule};

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
    }

    public void zeroGyroscope() {
        odometry.resetPosition(
                Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() },
                new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
        );
    }

    public void resetToAbsolute(){
        for (SwerveModule mod : SwerveModules) {
             mod.resetToAbsolute();   
        }
    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        odometry.update(
                Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() }
        );

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

//=============================================================================================
//=================================== Path Planner ============================================

        public void setDesiredState(SwerveModuleState[] states) {
                frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
                frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
                backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
                backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
        }

        public Pose2d getPose() {
                return odometry.getPoseMeters();
        }

        public void resetOdometry(Pose2d pose) {
                odometry.resetPosition(getAngle(), getPositions(), pose);
        }

        private SwerveModulePosition[] getPositions() {
                SwerveModulePosition[] swerveStates = { frontLeftModule.getPosition(), frontRightModule.getPosition(),
                        backLeftModule.getPosition(), backRightModule.getPosition() };
                return swerveStates;
        }

    public Rotation2d getAngle() {
        var degrees = gyroscope.getRotation2d().getDegrees();
        degrees = -degrees;
        return Rotation2d.fromDegrees(degrees);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

}
