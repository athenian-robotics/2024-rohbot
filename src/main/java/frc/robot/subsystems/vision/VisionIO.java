package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  /** Updates the set of loggable inputs. */
  default void updateInputs(VisionIOInputs inputs, Pose2d estimate) {}

  default PhotonPipelineResult getLatestResult() {
    return null;
  }

  default Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    return null;
  }

  default Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return null;
  }

  @AutoLog
  class VisionIOInputs {
    public Pose2d estimate = new Pose2d();
    public int tagCount = 0;
    public double timestamp = 0;
  }
}
