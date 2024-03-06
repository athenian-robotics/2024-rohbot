package frc.robot.inputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import java.util.List;
import java.util.Optional;

import frc.robot.inputs.poseEstimator.PoseEstimator;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NoteDetector {
  private static final Measure<Distance> CAMERA_HEIGHT = Units.Inches.of(14.5);
  private static final Rotation2d CAMERA_PITCH = new Rotation2d(0.0);
  private final PhotonCamera camera;
  private final PoseEstimator poseEstimator;

  public NoteDetector(PhotonCamera camera, PoseEstimator poseEstimator) {
    this.camera = camera;
    this.poseEstimator = poseEstimator;
  }

  public Optional<Translation2d> getClosestNoteTranslation() {
    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      Translation2d closestTranslation = null;
      double closestDistance = Double.MAX_VALUE;

      for (PhotonTrackedTarget target : targets) {
        Measure<Distance> distance =
            Units.Meters.of(
                PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT.in(Units.Meters),
                    0,
                    CAMERA_PITCH.getRadians(),
                    Units.Degrees.of(target.getPitch()).in(Units.Radians)));

        Translation2d translation =
            PhotonUtils.estimateCameraToTargetTranslation(
                distance.in(Units.Meters), Rotation2d.fromDegrees(-target.getYaw()));
        Translation2d estimatedPose = poseEstimator.getPosition();

        if (estimatedPose != null) {
          double currentDistance = estimatedPose.getDistance(translation);
          if (currentDistance < closestDistance) {
            closestDistance = currentDistance;
            closestTranslation = translation;
          }
        }
      }

      if (closestTranslation != null) {
        return Optional.of(closestTranslation);
      }
    }

    return Optional.empty();
  }
}
