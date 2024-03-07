package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs;

  public Vision(VisionIO io) {
    this.io = io;
    inputs = new VisionIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs, inputs.estimate);
    Logger.processInputs("vision", inputs);
  }

  public PhotonPipelineResult getLatestResult() {
    return io.getLatestResult();
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    return io.getEstimationStdDevs(inputs.estimate);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return io.getEstimatedGlobalPose();
  }
}
