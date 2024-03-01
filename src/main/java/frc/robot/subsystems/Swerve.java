package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.inputs.PoseEstimator;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;

public class Swerve extends SubsystemBase {

  private static final Measure<Angle> ACCEPTABLE_ANGLE_ERROR = Degrees.of(0.1); // TODO: tune/fill
  @Getter private final SwerveDrive swerveDrive;
  private final ShooterDataTable table;
  private final PoseEstimator poseEstimator;
  private final Measure<Velocity<Velocity<Distance>>> MAXIMUM_ACCELERATION =
      Units.MetersPerSecondPerSecond.of(4); // TODO: Fill
  private final Measure<Velocity<Velocity<Angle>>> MAXIMUM_ANGULAR_ACCELERATION =
      Units.DegreesPerSecond.per(Units.Seconds).of(720); // TODO: Fill

  public Swerve(
      SwerveDrive swerveDrive, ShooterDataTable shooterDataTable, PoseEstimator poseEstimator) {
    this.swerveDrive = swerveDrive;

    this.table = shooterDataTable;
    this.poseEstimator = poseEstimator;

    AutoBuilder.configureHolonomic(
        poseEstimator::getPose,
        swerveDrive::resetOdometry,
        swerveDrive::getRobotVelocity,
        swerveDrive::drive,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // TODO: Max module speed, in m/s
            0.4, // TODO: Drive base radius in meters. Distance from robot center to furthest
            // module.
            new ReplanningConfig() // TODO: Default path replanning config. See the API for the
            // options here
            ),
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
        },
        this);
  }

  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angSped) {
    swerveDrive.setHeadingCorrection(false);
    return run(
        () -> {
          double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth control
          double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth control
          swerveDrive.drive(
              new Translation2d(xInput * 20, yInput * 20),
              angSped.getAsDouble() * 20 / 1.07,
              true,
              false);
        });
  }

  public Command sysIdDriveCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive, 12),
        3.0,
        5.0,
        3.0);
  }

  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive),
        3.0,
        5.0,
        3.0);
  }

  public Command resetHeading() {
    return runOnce(swerveDrive::zeroGyro);
  }

  public Command faceSpeaker() {
    return AutoBuilder.pathfindToPose(
        new Pose2d(
            poseEstimator.getPosition(),
            poseEstimator
                .translationToSpeaker()
                .getAngle()
                .plus(
                    new Rotation2d(
                        table
                            .get(poseEstimator.translationToSpeaker())
                            .map(ShooterSpec::offset)
                            .orElse(Degrees.of(0))))),
        new PathConstraints(
            swerveDrive.getMaximumVelocity(),
            MAXIMUM_ACCELERATION.in(Units.MetersPerSecondPerSecond),
            swerveDrive.getMaximumAngularVelocity(),
            MAXIMUM_ANGULAR_ACCELERATION.in(Units.RadiansPerSecond.per(Units.Seconds))));
  }

  public boolean ready() {
    return Math.abs(
            poseEstimator
                .translationToSpeaker()
                .getAngle()
                .plus(
                    new Rotation2d(
                        table
                            .get(poseEstimator.translationToSpeaker())
                            .map(ShooterSpec::offset)
                            .orElse(Degrees.of(0))))
                .minus(poseEstimator.getPose().getRotation())
                .getRadians())
        < ACCEPTABLE_ANGLE_ERROR.in(Radians);
  }
}
