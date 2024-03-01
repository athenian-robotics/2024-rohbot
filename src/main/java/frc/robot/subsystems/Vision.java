package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class Vision extends SubsystemBase implements Logged {
  private double CAMERA_HEIGHT_METERS = 0.5; // TODO: Fix these values of all the constants
  private double TARGET_HEIGHT_METERS = 0.3;
  private double CAMERA_PITCH_RADIANS = 0.3;
  private String LEFT_CAM_ID = "Leftcam"; // TODO: Make sure actual names
  private String RIGHT_CAM_ID = "Rightcam";
  private PhotonCamera leftCamera;
  private PhotonCamera
      rightCamera; // only going to use this if dual camera manual triangulation is better

  private double camDist = 0.5; // camera spacing in meters
  private double camleftFL = 55; // left camera focal in mm
  private double camrightFL = 55; // right camera focal in mm
  private double canvasWidth = 640; // width of view for cameras
  private double canvasHeight = 480; // height of view for cameras

  public ArrayList<Translation2d> objectPoses = new ArrayList<>() {};

  public Vision() {
    leftCamera = new PhotonCamera(LEFT_CAM_ID);
    objectPoses.add(new Translation2d(1, 2));
    // rightCamera = new PhotonCamera(rightCamID); Currently only using one camera
  }

  @Log.NT
  public Translation2d[] getPoses() {
    return (Translation2d[]) objectPoses.toArray();
  }

  @Log.NT
  public Translation2d testTranslation2d() {
    return objectPoses.get(0);
  }

  @Override
  public void periodic() {
    // objectPoses.clear();
    // populatePose2d(leftCamera.getLatestResult());
    System.out.println(objectPoses);
  }

  public void populatePose2d(PhotonPipelineResult latestResult) {
    List<PhotonTrackedTarget> targets = latestResult.getTargets();
    for (PhotonTrackedTarget t : targets) {
      double distance =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT_METERS,
              TARGET_HEIGHT_METERS,
              CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(t.getPitch()));
      Translation2d targetTrans =
          PhotonUtils.estimateCameraToTargetTranslation(
              distance, Rotation2d.fromDegrees(-t.getYaw()));
      objectPoses.add(targetTrans);
    }
  }

  public double[] triangulate(PhotonTrackedTarget object, PhotonTrackedTarget object2) {
    double[] center = getCenter(object);
    double[] center2 = getCenter(object2);
    double xOffsetLeft = center[0] - canvasWidth / 2;
    double xOffsetRight = center2[0] - canvasWidth / 2;
    double yOffsetLeft = center[1] - canvasHeight / 2;
    double yOffsetRight = center2[1] - canvasHeight / 2;

    // Triangulation methods using similar triangles of camera rays (assuming they
    // are parallel to
    // each other)
    double zCoord =
        camDist * camrightFL * camleftFL / (camrightFL * xOffsetRight - camleftFL * xOffsetLeft);
    double xCoord =
        ((zCoord * xOffsetRight / camrightFL + camDist)
                + (zCoord * xOffsetLeft / camleftFL + camDist))
            / 2; // average of 2 different methods to minimize error
    double yCoord =
        (yOffsetLeft * zCoord / camleftFL + yOffsetRight * zCoord / camrightFL)
            / 2; // average same as above

    return new double[] {xCoord, yCoord, zCoord};
  }

  public double[] getCenter(PhotonTrackedTarget object) {
    List<TargetCorner> corners = object.getDetectedCorners();
    return new double[] {
      (corners.get(1).x - corners.get(0).x) / 2.0, (corners.get(0).y - corners.get(2).y) / 2.0
    };
  }
}
