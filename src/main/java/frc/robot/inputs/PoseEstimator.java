package frc.robot.inputs;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.io.IOException;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import monologue.Annotations;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PoseEstimator implements Subsystem {
  private static final Translation2d BLUE_SPEAKER_POSITION = new Translation2d(-1.5, 218.42);
  private static final Translation2d RED_SPEAKER_POSITION = new Translation2d(652.73, 218.42);
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final Transform3d robotToCam;
  private final PhotonPoseEstimator photonPoseEstimator;
  private final Pigeon2 gyro;
  private final BiConsumer<Pose2d, Double> addVisionMeasurement;
  private final Supplier<Pose2d> getEstimatedPosition;
  private final Supplier<SwerveModulePosition[]> getModulePositions;
  private final BiConsumer<Rotation2d, SwerveModulePosition[]> update;
  private Translation2d lastKnownPosition = null;

  public PoseEstimator(
      PhotonCamera photonCamera,
      BiConsumer<Pose2d, Double> addVisionMeasurement,
      Supplier<Pose2d> getEstimatedPosition,
      Supplier<SwerveModulePosition[]> getModulePositions,
      BiConsumer<Rotation2d, SwerveModulePosition[]> update)
      throws IOException {
    this.addVisionMeasurement = addVisionMeasurement;
    this.getEstimatedPosition = getEstimatedPosition;
    this.getModulePositions = getModulePositions;
    this.update = update;
    gyro = new Pigeon2(13, "*");
    aprilTagFieldLayout =
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

    robotToCam =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(14.5), Units.inchesToMeters(4.5), Units.inchesToMeters(18)),
            new Rotation3d(0, 0, 0)); // TODO: Replace with real cam rotation
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, robotToCam);
  }

  @Annotations.Log.NT
  public Translation2d getPosition() {
    Pose2d swervePose2d = getEstimatedPosition.get();
    return new Translation2d(swervePose2d.getX(), swervePose2d.getY());
  }

  @Annotations.Log.NT
  public Pose2d getPose() {
    Pose2d swervePose2d = getEstimatedPosition.get();
    return new Pose2d(swervePose2d.getX(), swervePose2d.getY(), gyro.getRotation2d());
  }

  public Translation2d translationToSpeaker() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return RED_SPEAKER_POSITION.minus(getPosition());
    }
    return BLUE_SPEAKER_POSITION.minus(getPosition());
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
    if (estimatedRobotPose.isPresent()) {
      Pose3d photonPose = estimatedRobotPose.get().estimatedPose;
      Translation2d currentRobotPosition = new Translation2d(photonPose.getX(), photonPose.getY());

      if (!lastKnownPosition.equals(currentRobotPosition)) {
        addVisionMeasurement.accept(
            new Pose2d(currentRobotPosition, gyro.getRotation2d()), Timer.getFPGATimestamp());
        lastKnownPosition = currentRobotPosition;
      }
      update.accept(gyro.getRotation2d(), getModulePositions.get());
    }
  }
}
