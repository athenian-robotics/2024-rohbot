package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.inputs.PoseEstimator;

import frc.robot.lib.GyroIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Swerve extends SubsystemBase {

  private static final Measure<Angle> ACCEPTABLE_ANGLE_ERROR = Degrees.of(0.1); // TODO: Tune
  private final PoseEstimator poseEstimator;
  private final Measure<Velocity<Velocity<Distance>>> MAXIMUM_ACCELERATION =
      Units.MetersPerSecondPerSecond.of(4); // TODO: Fill
  private final Measure<Velocity<Velocity<Angle>>> MAXIMUM_ANGULAR_ACCELERATION =
      Units.DegreesPerSecond.per(Units.Seconds).of(720); // TODO: Fill
    private final ShooterDataTable table;

    private static final Measure<Velocity<Distance>> MAX_LINEAR_SPEED = Units.MetersPerSecond.of(20);
    private static final Measure<Distance> TRACK_WIDTH_X = Units.Inches.of(26.0);
    private static final Measure<Distance> TRACK_WIDTH_Y = Units.Inches.of(26.0);
    private static final Measure<Distance> DRIVE_BASE_RADIUS =
            Inches.of(Math.hypot(TRACK_WIDTH_X.divide(2.0).in(Inches), TRACK_WIDTH_Y.divide(2.0).in(Inches)));
    private static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(MAX_LINEAR_SPEED.in(InchesPerSecond) / DRIVE_BASE_RADIUS.in(Inches));
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };

    // TODO: fix pose estimator and periodic and switch to high frequency odometry
    public Swerve(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
       ShooterDataTable shooterDataTable, PoseEstimator poseEstimator) {

    this.table = shooterDataTable;
    this.poseEstimator = poseEstimator;
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, Module.ModulePosition.FRONT_LEFT);
        modules[1] = new Module(frModuleIO, Module.ModulePosition.FRONT_RIGHT);
        modules[2] = new Module(blModuleIO, Module.ModulePosition.BACK_LEFT);
        modules[3] = new Module(brModuleIO, Module.ModulePosition.BACK_RIGHT);



        AutoBuilder.configureHolonomic(
                poseEstimator::getPose,
                poseEstimator::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity,
                new HolonomicPathFollowerConfig(
                        MAX_LINEAR_SPEED.in(MetersPerSecond), DRIVE_BASE_RADIUS.in(Meters), new ReplanningConfig()),
                () ->
                        DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                this);

        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    for (int i = 0; i < 4; i++) {
                                        modules[i].runCharacterization(voltage.in(Volts));
                                    }
                                },
                                null,
                                this));
  }


  // TODO: consider implementing open loop control if I dont like this
    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the current odometry pose. */
    public Pose2d getPose() {
        return poseEstimator.getPose();
    }




    /**
     * Returns the maximum linear speed in meters per sec.
     */
    public Measure<Velocity<Distance>> getMaxLinearSpeedMetersPerSec() {
        return MAX_LINEAR_SPEED;
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     */
    public Measure<Velocity<Angle>> getMaxAngularSpeedRadPerSec() {
        return MAX_ANGULAR_SPEED;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[]{
                new Translation2d(TRACK_WIDTH_X.divide(2), TRACK_WIDTH_Y.divide(2)),
                new Translation2d(TRACK_WIDTH_X.divide( 2.0), TRACK_WIDTH_Y.negate().divide(2.0)),
                new Translation2d(TRACK_WIDTH_X.negate().divide( 2.0), TRACK_WIDTH_Y.divide(2.0)),
                new Translation2d(TRACK_WIDTH_X.negate().divide( 2.0), TRACK_WIDTH_Y.negate().divide( 2.0))
        };
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
            MAX_LINEAR_SPEED.in(Units.MetersPerSecond),
                MAXIMUM_ACCELERATION.in(Units.MetersPerSecondPerSecond),
            MAX_ANGULAR_SPEED.in(Units.RadiansPerSecond),
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
