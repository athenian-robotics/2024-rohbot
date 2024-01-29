package frc.robot.inputs;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import swervelib.SwerveDrive;

public class PoseEstimator implements Subsystem {
  private static final Translation2d SPEAKER_POSE = new Translation2d(); // TODO: Fill
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final Transform3d robotToCam;
  private final PhotonPoseEstimator photonPoseEstimator;
  private final Pigeon2 gyro;
  private Translation2d lastKnownPosition = null;

  private final SwerveDrive swerveDrive;

  public PoseEstimator(PhotonCamera photonCamera, SwerveDrive swerveDrive) throws IOException {
    this.swerveDrive = swerveDrive;
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

  public Translation2d getPose() {
    Pose2d swervePose2d = swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition();
    return new Translation2d(swervePose2d.getX(), swervePose2d.getY());
  }

  public Translation2d translationToSpeaker() {
    return getPose().minus(SPEAKER_POSE);
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
    if (estimatedRobotPose.isPresent()) {
      Pose3d photonPose = estimatedRobotPose.get().estimatedPose;
      Translation2d currentRobotPosition = new Translation2d(photonPose.getX(), photonPose.getY());
      if (lastKnownPosition != currentRobotPosition) {
        swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
            new Pose2d(currentRobotPosition, gyro.getRotation2d()), Timer.getFPGATimestamp());
        lastKnownPosition = currentRobotPosition;
      }
      swerveDrive.swerveDrivePoseEstimator.update(
          gyro.getRotation2d(), swerveDrive.getModulePositions());
    }
  }
}
