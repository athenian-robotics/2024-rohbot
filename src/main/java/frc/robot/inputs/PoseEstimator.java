package frc.robot.inputs;

import java.io.IOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.SwerveDrive;

import java.util.Optional;

public class PoseEstimator {
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final Transform3d robotToCam;
    private final PhotonPoseEstimator photonPoseEstimator;

    public PoseEstimator(PhotonCamera photonCamera, SwerveDrive swerveDrive) throws IOException {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); // TODO: Replace with
                                                                                                 // real cam pos
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                photonCamera, robotToCam);
    }

    public Translation2d getEstimatedPose() {
        Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
        if (estimatedRobotPose.isPresent()) {
            Pose3d pose3d = estimatedRobotPose.get().estimatedPose;
            return new Translation2d(pose3d.getX(), pose3d.getY());
        } else {
            return null; 
        }
    }
}
