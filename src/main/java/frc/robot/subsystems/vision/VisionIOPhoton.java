package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhoton implements VisionIO {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private double lastEstTimestamp = 0;

  public VisionIOPhoton() {
    camera = new PhotonCamera("cam");
    photonEstimator =
        new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0))); // TODO: chat is this rizz?
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
    photonEstimator.setReferencePose(currentEstimate);

    var result = getLatestResult();
    inputs.estimate = currentEstimate;
    if (result.hasTargets()) {
      photonEstimator.update().ifPresent(est -> inputs.estimate = est.estimatedPose.toPose2d());
      inputs.tagCount = result.getTargets().size();
      lastEstTimestamp = inputs.timestamp;
    } else {
      inputs.tagCount = 0;
    }
    inputs.timestamp = lastEstTimestamp;
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VecBuilder.fill(4, 4, 8); // TODO: chat is this rizz?
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VecBuilder.fill(0.5, 0.5, 1); // TODO: chat is this rizz?
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }
}
