package frc.robot.inputs;

import java.io.IOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SwerveModule;
import swervelib.SwerveDrive;

import java.util.Optional;

public class PoseEstimator implements Subsystem {
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final Transform3d robotToCam;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final Pigeon2 gyro;
    private final SwerveDriveKinematics m_kinematics;

    private final SwerveModule frontLeftModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backRightModule;

    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public PoseEstimator(PhotonCamera photonCamera, SwerveDrive swerveDrive) throws IOException {
        frontLeftModule = new SwerveModule(0, 0, 0, 0); // TODO: Replace with actual ports
        backLeftModule = new SwerveModule(0, 0, 0, 0);
        frontRightModule = new SwerveModule(0, 0, 0, 0);
        backRightModule = new SwerveModule(0, 0, 0, 0);
        m_kinematics = new SwerveDriveKinematics(
                new Translation2d(0.3048, 0.3048),
                new Translation2d(0.3048, -0.3048),
                new Translation2d(-0.3048, 0.3048),
                new Translation2d(-0.3048, -0.3048));
        gyro = new Pigeon2(4, "*"); // TODO: replace with device id

        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(14.5), Units.inchesToMeters(4.5), Units.inchesToMeters(18)),
                new Rotation3d(0, 0, 0)); // TODO: Replace with real cam pos
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                photonCamera, robotToCam);

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(m_kinematics, gyro.getRotation2d(), getCurrentSwerveModulePositions(),
                new Pose2d(0, 0, new Rotation2d()), VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    }

    public Translation2d getPose() {
        Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
        if (estimatedRobotPose.isPresent()) {
            Pose3d pose3d = estimatedRobotPose.get().estimatedPose;
            Translation2d translation = new Translation2d(pose3d.getX(), pose3d.getY());
            swerveDrivePoseEstimator.addVisionMeasurement(new Pose2d(translation, gyro.getRotation2d()), Timer.getFPGATimestamp());
            Pose2d swervePose2d = swerveDrivePoseEstimator.getEstimatedPosition();
            return new Translation2d(swervePose2d.getX(), swervePose2d.getY());
        } else {
            return null;
        }
    }

    public SwerveModulePosition[] getCurrentSwerveModulePositions()
    {
        return new SwerveModulePosition[]{
            frontLeftModule.getPosition(),
            backLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backRightModule.getPosition()
        };
    }

    @Override
    public void periodic() {
        swerveDrivePoseEstimator.update(gyro.getRotation2d(), getCurrentSwerveModulePositions());
    }
}
