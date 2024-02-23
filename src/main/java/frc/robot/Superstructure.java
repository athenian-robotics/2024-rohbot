package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.inputs.NoteDetector;
import frc.robot.inputs.PoseEstimator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Superstructure extends SubsystemBase {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Swerve swerve;

  private final PathPlannerPath leftTopToNoteToAmpTraj =
      PathPlannerPath.fromChoreoTrajectory("leftTopToAmp");
  private final PathPlannerPath topLeftToMiddle1 =
      PathPlannerPath.fromChoreoTrajectory("topLeftToMiddle1");

  private final PathPlannerPath middle1ToMiddle2 =
      PathPlannerPath.fromChoreoTrajectory("middle1ToMiddle2");
  private final PathPlannerPath middle1ToSpeaker =
      PathPlannerPath.fromChoreoTrajectory("middle1ToSpeaker");
  private final PathPlannerPath middle2ToMiddle3 =
      PathPlannerPath.fromChoreoTrajectory("middle2ToMiddle3");
  private final PathPlannerPath middle2ToSpeaker =
      PathPlannerPath.fromChoreoTrajectory("middle2ToSpeaker");
  private final PathPlannerPath middle3ToMiddle4 =
      PathPlannerPath.fromChoreoTrajectory("middle3ToMiddle4");
  private final PathPlannerPath middle3ToSpeaker =
      PathPlannerPath.fromChoreoTrajectory("middle3ToSpeaker");
  private final PathPlannerPath middle4ToMiddle5 =
      PathPlannerPath.fromChoreoTrajectory("middle4ToMiddle5");
  private final PathPlannerPath middle4ToSpeaker =
      PathPlannerPath.fromChoreoTrajectory("middle4ToSpeaker");
  private final PathPlannerPath middle5ToSpeaker =
      PathPlannerPath.fromChoreoTrajectory("middle5ToMiddle6");

  private final PathPlannerPath ampToMiddle1 = PathPlannerPath.fromChoreoTrajectory("ampToMiddle1");
  private final PathPlannerPath bottomLeftToMiddle5 =
      PathPlannerPath.fromChoreoTrajectory("bottomLeftToMiddle5");

  private final PathPlannerPath middle5ToMiddle4 =
      PathPlannerPath.fromChoreoTrajectory("middle5ToMiddle4");
  private final PathPlannerPath middle4ToMiddle3 =
      PathPlannerPath.fromChoreoTrajectory("middle4ToMiddle3");
  private final PathPlannerPath middle3ToMiddle2 =
      PathPlannerPath.fromChoreoTrajectory("middle3ToMiddle2");
  private final PathPlannerPath middle2ToMiddle1 =
      PathPlannerPath.fromChoreoTrajectory("middle2ToMiddle1");
  private final PathPlannerPath speakerToMiddle2 =
      PathPlannerPath.fromChoreoTrajectory("speakerToMiddle2");
  private final PathPlannerPath speakerToMiddle3 =
      PathPlannerPath.fromChoreoTrajectory("speakerToMiddle3");
  private final PathPlannerPath speakerToMiddle4 =
      PathPlannerPath.fromChoreoTrajectory("speakerToMiddle4");
  private final PathPlannerPath speakerToMiddle5 =
      PathPlannerPath.fromChoreoTrajectory("speakerToMiddle5");

  private final PathPlannerPath fromStartingMiddleToMiddle3 =
      PathPlannerPath.fromChoreoTrajectory("fromStartingMiddleToMiddle3");

  public Superstructure(
      Intake intake,
      Indexer indexer,
      Shooter shooter,
      Swerve swerve,
      NoteDetector noteDetector,
      PoseEstimator poseEstimator) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.swerve = swerve;
  }

  @Override
  public void periodic() {
    ParallelCommandGroup toDo = new ParallelCommandGroup();
    if (indexer.isInactive() && intake.isNotePassed()) { // fired note
      toDo.addCommands(intake.startIntake());
    }

    toDo.schedule();
  }

  public Command fireShot() {
    return shooter
        .requestShot()
        .alongWith(swerve.faceSpeaker())
        .alongWith(indexer.fire())
        .andThen(
            shooter.waitUntilReady().alongWith(indexer.waitUntilReady()).andThen(shooter.fire()));
  }

  public Command fromTopWithAmp() {
    return new SequentialCommandGroup(
        intake.startIntake(),
        fromLeftTopToNoteToAmp(),
        fireShot(),
        intake.startIntake(),
        fromAmpToMiddle1(),
        fromMiddle1ToMiddle5());
  }

  public Command fromTopWithoutAmp() {
    return intake.startIntake().andThen(fromTopToMiddle1()).andThen(fromMiddle1ToMiddle5());
  }

  public Command fromBottomWithoutAmp() {
    return intake.startIntake().andThen(fromBottomToMiddle5()).andThen(fromMiddle5ToMiddle1());
  }

  public Command fromStartingMiddleWithoutAmp() {
    return new SequentialCommandGroup(
        intake.startIntake(),
        fromStartingMiddleToMiddle3(),
        checkAndHandleNote(fromMiddle3ToSpeaker(), fromMiddle3ToMiddle2(), fromSpeakerToMiddle2()),
        checkAndHandleNote(
            fromMiddle2ToSpeaker(), fromMiddle2ToMiddle3(), null) // NOTE: we could do more...
        );
  }

  public Command fromMiddle1ToMiddle5() {
    return new SequentialCommandGroup(
        checkAndHandleNote(fromMiddle1ToSpeaker(), fromMiddle1ToMiddle2(), fromSpeakerToMiddle2()),
        checkAndHandleNote(fromMiddle2ToSpeaker(), fromMiddle2ToMiddle3(), fromSpeakerToMiddle3()),
        checkAndHandleNote(fromMiddle3ToSpeaker(), fromMiddle3ToMiddle4(), fromSpeakerToMiddle4()),
        checkAndHandleNote(fromMiddle4ToSpeaker(), fromMiddle4ToMiddle5(), fromSpeakerToMiddle5()),
        checkAndHandleNote(fromMiddle5ToSpeaker(), null, null));
  }

  public Command fromMiddle5ToMiddle1() {
    return new SequentialCommandGroup(
        checkAndHandleNote(fromMiddle5ToMiddle4(), fromMiddle5ToSpeaker(), fromSpeakerToMiddle4()),
        checkAndHandleNote(fromMiddle4ToMiddle3(), fromMiddle4ToSpeaker(), fromSpeakerToMiddle3()),
        checkAndHandleNote(fromMiddle3ToMiddle2(), fromMiddle3ToSpeaker(), fromSpeakerToMiddle2()),
        checkAndHandleNote(fromMiddle2ToMiddle1(), fromMiddle2ToSpeaker(), fromSpeakerToMiddle2()),
        checkAndHandleNote(fromMiddle1ToMiddle2(), fromMiddle1ToSpeaker(), fromSpeakerToMiddle2()));
  }

  private Command checkAndHandleNote(
      Command toSpeaker, Command toNextNoteDirectly, Command fromSpeakerToNextNote) {
    return defer(
        () -> {
          if (intake.isNoteFound()) {
            return new SequentialCommandGroup(
                // onTheFlyRobotToNote(),
                toSpeaker, fireShot(), intake.startIntake(), fromSpeakerToNextNote);
          } else if (toNextNoteDirectly != null) {
            return new SequentialCommandGroup(intake.startIntake(), toNextNoteDirectly);
          } else {
            return null; // No next action if at the last note and no note is present
          }
        });
  }

  // private Command onTheFlyRobotToNote() {
  // Translation2d robotPosition = poseEstimator.getPose().getTranslation();
  // Translation2d notePosition = noteDetector.getClosestNoteTranslation().get();

  // ArrayList<Translation2d> bezierPoints = new ArrayList<Translation2d>();
  // bezierPoints.add(robotPosition);
  // bezierPoints.add(notePosition);

  // double dotProduct = robotPosition.getX() * notePosition.getX() +
  // robotPosition.getY() * notePosition.getY();
  // Rotation2d rotationNeeded = new Rotation2d(dotProduct /
  // (robotPosition.getNorm() * notePosition.getNorm()));

  // PathPlannerPath pathToNote =
  // new PathPlannerPath(
  // bezierPoints,
  // new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // TODO: Tune
  // new GoalEndState(
  // 0.5,
  // rotationNeeded)); // NOTE: rotationNeeded might not work here

  // return AutoBuilder.followPath(pathToNote);
  // }

  private Command fromAmpToMiddle1() {
    return AutoBuilder.followPath(ampToMiddle1);
  }

  private Command fromLeftTopToNoteToAmp() {
    return AutoBuilder.followPath(leftTopToNoteToAmpTraj);
  }

  private Command fromTopToMiddle1() {
    return AutoBuilder.followPath(topLeftToMiddle1);
  }

  private Command fromMiddle1ToMiddle2() {
    return AutoBuilder.followPath(middle1ToMiddle2);
  }

  private Command fromMiddle1ToSpeaker() {
    return AutoBuilder.followPath(middle1ToSpeaker);
  }

  private Command fromMiddle2ToMiddle3() {
    return AutoBuilder.followPath(middle2ToMiddle3);
  }

  private Command fromMiddle2ToSpeaker() {
    return AutoBuilder.followPath(middle2ToSpeaker);
  }

  private Command fromMiddle3ToMiddle4() {
    return AutoBuilder.followPath(middle3ToMiddle4);
  }

  private Command fromMiddle3ToSpeaker() {
    return AutoBuilder.followPath(middle3ToSpeaker);
  }

  private Command fromMiddle4ToMiddle5() {
    return AutoBuilder.followPath(middle4ToMiddle5);
  }

  private Command fromMiddle4ToSpeaker() {
    return AutoBuilder.followPath(middle4ToSpeaker);
  }

  private Command fromMiddle5ToSpeaker() {
    return AutoBuilder.followPath(middle5ToSpeaker);
  }

  private Command fromMiddle5ToMiddle4() {
    return AutoBuilder.followPath(middle5ToMiddle4);
  }

  private Command fromMiddle4ToMiddle3() {
    return AutoBuilder.followPath(middle4ToMiddle3);
  }

  private Command fromMiddle3ToMiddle2() {
    return AutoBuilder.followPath(middle3ToMiddle2);
  }

  private Command fromMiddle2ToMiddle1() {
    return AutoBuilder.followPath(middle2ToMiddle1);
  }

  private Command fromBottomToMiddle5() {
    return AutoBuilder.followPath(bottomLeftToMiddle5);
  }

  private Command fromSpeakerToMiddle2() {
    return AutoBuilder.followPath(speakerToMiddle2);
  }

  private Command fromSpeakerToMiddle3() {
    return AutoBuilder.followPath(speakerToMiddle3);
  }

  private Command fromSpeakerToMiddle4() {
    return AutoBuilder.followPath(speakerToMiddle4);
  }

  private Command fromSpeakerToMiddle5() {
    return AutoBuilder.followPath(speakerToMiddle5);
  }

  private Command fromStartingMiddleToMiddle3() {
    return AutoBuilder.followPath(fromStartingMiddleToMiddle3);
  }
}
