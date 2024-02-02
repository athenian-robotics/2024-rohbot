package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.inputs.PoseEstimator;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Minute;

public class Swerve extends SubsystemBase {
    private final double maximumSpeed = Units.feetToMeters(20);
    private final CANSparkMax backLeft;
    private final CANSparkMax backRight;
    private final CANSparkMax frontLeft;
    private final CANSparkMax frontRight;
    private final CANSparkMax backLeftAngle;
    private final CANSparkMax backRightAngle;
    private final CANSparkMax frontLeftAngle;
    private final CANSparkMax frontRightAngle;
    private final RelativeEncoder frontLeftEncoder;
    private final SysIdRoutine linearRoutine;
    private final SysIdRoutine angularRoutine;
    private final Pigeon2 gyro;
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(Rotations.per(Minute).of(0));
  // speed in m/s
  // private SwerveDrive swerveDrive;

  public Swerve(File swerveJsonDirectory) {
    // Angle conversion factor = 360 / (GEAR RATIO * ENCODER RESOLUTION)
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 4096);
    // with real values
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
    // ENCODER RESOLUTION).
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(0), 6.75, 1);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    frontLeft = new CANSparkMax(7, CANSparkLowLevel.MotorType.kBrushless);
    frontLeftAngle = new CANSparkMax(8, CANSparkLowLevel.MotorType.kBrushless);
    frontRight = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);
    frontRightAngle = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless);
    backLeft = new CANSparkMax(5, CANSparkLowLevel.MotorType.kBrushless);
    backLeftAngle = new CANSparkMax(6, CANSparkLowLevel.MotorType.kBrushless);
    backRight = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);
    backRightAngle = new CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless);
    frontLeftAngle.setIdleMode(CANSparkBase.IdleMode.kBrake);
    frontRightAngle.setIdleMode(CANSparkBase.IdleMode.kBrake);
    backLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
    backRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
    frontRight.follow(frontLeft);
    backLeft.follow(frontLeft);
    backRight.follow(frontLeft);
    frontLeftEncoder = frontLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    gyro = new Pigeon2(13, "*");
    linearRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Seconds).of(1), Volts.of(12), Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> voltage) -> frontLeft.set(voltage.in(Volts) / 12),
                    this::logLinear,
                    this,
                    "linear drive"
            )
    );
      angularRoutine = new SysIdRoutine(
              new SysIdRoutine.Config(Volts.per(Seconds).of(1), Volts.of(12), Seconds.of(10)),
              new SysIdRoutine.Mechanism(
                      (Measure<Voltage> voltage) -> frontLeft.set(voltage.in(Volts) / 12),
                      this::logAngular,
                      this,
                      "angular drive"
              )
      );
//    try {
//      swerveDrive = new SwerveParser(swerveJsonDirectory)
//          .createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
//    } catch (Exception e) {
//      throw new RuntimeException(e);
//    }

//    AutoBuilder.configureHolonomic(() -> swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition(),
//        pose -> swerveDrive.resetOdometry(pose),
//        () -> swerveDrive.getRobotVelocity(), swerveDrive::drive,
//        new HolonomicPathFollowerConfig(
//            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
//            4.5, // TODO: Max module speed, in m/s
//            0.4, // TODO: Drive base radius in meters. Distance from robot center to furthest module.
//            new ReplanningConfig() // TODO: Default path replanning config. See the API for the options here
//        ), () -> {
//          var alliance = DriverStation.getAlliance();
//          if (alliance.isPresent()) {
//            return alliance.get() == DriverStation.Alliance.Red;
//          }
//          return false;
//        },
//        this
//    );
  }

  private void logLinear(SysIdRoutineLog log) {
      log.motor("front-left-motor")
              .voltage(appliedVoltage.mut_replace(frontLeft.getBusVoltage() * frontLeft.get(), Volts))
              .angularVelocity(velocity.mut_replace(frontLeftEncoder.getVelocity(), Rotations.per(Minute)));
  }

    private void logAngular(SysIdRoutineLog log) {
        log.motor("front-left-motor")
                .voltage(appliedVoltage.mut_replace(frontLeft.getBusVoltage() * frontLeft.get(), Volts))
                .angularVelocity(velocity.mut_replace(gyro.getRate(), DegreesPerSecond));
    }

    public Command SysIdQuasiLinear(SysIdRoutine.Direction direction) {
      return linearRoutine.quasistatic(direction);
    }

    public Command SysIdDynamicLinear(SysIdRoutine.Direction direction) {
      return linearRoutine.dynamic(direction);
    }

    public Command SysIdQuasiAngular(SysIdRoutine.Direction direction) {
      return angularRoutine.quasistatic(direction);
    }

    public Command SysIdDynamicAngular(SysIdRoutine.Direction direction) {
      return angularRoutine.dynamic(direction);
    }

//  public Command driveCommand(
//      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angSped) {
//    swerveDrive.setHeadingCorrection(false);
//    return run(
//        () -> {
//          double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth control
//          double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth control
//          swerveDrive.drive(
//              new Translation2d(xInput * 20, yInput * 20),
//              angSped.getAsDouble() * 20 / 1.07,
//              true,
//              false);
//        });
//  }
  // TODO: Implement a command to face speaker when shooting
}
