package frc.robot.inputs;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import swervelib.SwerveDrive;

public class PoseEstimator {
    private final PhotonCamera photonCamera;
    private final SwerveDrive swerveDrive;

    public PoseEstimator(PhotonCamera photonCamera, SwerveDrive swerveDrive) {
        this.photonCamera = photonCamera;
        this.swerveDrive = swerveDrive;
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }
}
