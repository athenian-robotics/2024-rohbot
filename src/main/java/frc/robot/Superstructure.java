package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.inputs.NoteDetector;
import frc.robot.inputs.PoseEstimator;
import frc.robot.subsystems.indexer.IndexerIOPhysical;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.drive.Swerve;
import lombok.Getter;
import monologue.Logged;

public class Superstructure extends SubsystemBase implements Logged {
  @Getter private final IntakeIOSparkMax intake;
  @Getter private final IndexerIOPhysical indexer;
  @Getter private final Shooter shooter;
  @Getter private final Swerve swerve;
  private final PoseEstimator poseEstimator;
  private final ShooterDataTable dataTable;

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
  private State state = State.NO_NOTE;
  private State.RangeStatus rangeStatus = State.RangeStatus.OUTSIDE_RANGE;

  public Superstructure(
      IntakeIOSparkMax intake,
      IndexerIOPhysical indexer,
      Shooter shooter,
      Swerve swerve,
      NoteDetector noteDetector,
      PoseEstimator poseEstimator,
      ShooterDataTable dataTable) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.swerve = swerve;
    this.poseEstimator = poseEstimator;
    this.dataTable = dataTable;
  }

  // TODO: test whether we can actually move notes from the intake to the hood while the hood is
  // angled
  @Override
  public void periodic() {
    switch (state) {
      case NO_NOTE -> {
        if (intake.hasNote()) {
          state = State.HAS_NOTE;
          break;
        }

        intake.on();
        switch (rangeStatus) { // if we know what the hood angle should be, adjust
          case IN_RANGE -> indexer.adjusting();
          case OUTSIDE_RANGE -> indexer.idle();
        }
      }

      case HAS_NOTE -> {
        intake.off(); // hold note in intake
        switch (rangeStatus) {
          case IN_RANGE -> indexer.adjusting();
          case OUTSIDE_RANGE -> indexer.idle();
        }
      }

      case SHO0TING -> {
        switch (rangeStatus) {
          case IN_RANGE -> {
            swerve.faceSpeaker().schedule();
            if (shooter.ready() && indexer.ready() && swerve.ready())
              shooter
                  .activate()
                  .schedule(); // run intake so note goes into hood/shooter/indexer/shootdexer
          }

          case OUTSIDE_RANGE -> {
            // log?, should never happen
          }
        }
        if (intake.empty() && shooter.empty()) {
          state = State.NO_NOTE; // when we are done shooting reset state
          shooter.deactivate().schedule();
        }
      }
    }

    rangeStatus =
        dataTable.get(poseEstimator.translationToSpeaker()).isPresent()
            ? State.RangeStatus.IN_RANGE
            : State.RangeStatus.OUTSIDE_RANGE;
  }

  public Command shoot() {
    return runOnce(() -> state = State.SHO0TING)
        .onlyIf(intake::hasNote)
        .onlyIf(() -> rangeStatus == State.RangeStatus.IN_RANGE);
  }

  public Command cancelShot() {
    return runOnce(
        intake.empty() && shooter.empty()
            ? () -> state = State.NO_NOTE
            : () -> state = State.HAS_NOTE);
  }

  public Command fromTopWithAmp() {
    return sequence(fromLeftTopToNoteToAmp(), shoot(), fromAmpToMiddle1(), fromMiddle1ToMiddle5());
  }

  public Command fromTopWithoutAmp() {
    return fromTopToMiddle1().andThen(fromMiddle1ToMiddle5());
  }

  public Command fromBottomWithoutAmp() {
    return fromBottomToMiddle5().andThen(fromMiddle5ToMiddle1());
  }

  public Command fromStartingMiddleWithoutAmp() {
    return sequence(
        fromStartingMiddleToMiddle3(),
        checkAndHandleNote(fromMiddle3ToSpeaker(), fromMiddle3ToMiddle2(), fromSpeakerToMiddle2()),
        checkAndHandleNote(
            fromMiddle2ToSpeaker(), fromMiddle2ToMiddle3(), null) // NOTE: we could do more...
        );
  }

  public Command fromMiddle1ToMiddle5() {
    return sequence(
        checkAndHandleNote(fromMiddle1ToSpeaker(), fromMiddle1ToMiddle2(), fromSpeakerToMiddle2()),
        checkAndHandleNote(fromMiddle2ToSpeaker(), fromMiddle2ToMiddle3(), fromSpeakerToMiddle3()),
        checkAndHandleNote(fromMiddle3ToSpeaker(), fromMiddle3ToMiddle4(), fromSpeakerToMiddle4()),
        checkAndHandleNote(fromMiddle4ToSpeaker(), fromMiddle4ToMiddle5(), fromSpeakerToMiddle5()));
  }

  public Command fromMiddle5ToMiddle1() {
    return sequence(
        checkAndHandleNote(fromMiddle4ToMiddle3(), fromMiddle4ToSpeaker(), fromSpeakerToMiddle3()),
        checkAndHandleNote(fromMiddle3ToMiddle2(), fromMiddle3ToSpeaker(), fromSpeakerToMiddle2()),
        checkAndHandleNote(fromMiddle2ToMiddle1(), fromMiddle2ToSpeaker(), fromSpeakerToMiddle2()),
        checkAndHandleNote(fromMiddle1ToMiddle2(), fromMiddle1ToSpeaker(), fromSpeakerToMiddle2()));
  }

  private Command checkAndHandleNote(
      Command toSpeaker, Command toNextNoteDirectly, Command fromSpeakerToNextNote) {
    return defer(
        () -> {
          // No next action if at the last note and no note is present
          if (intake.hasNote()) {
            return sequence(
                // onTheFlyRobotToNote(),
                toSpeaker, shoot(), fromSpeakerToNextNote);
          } else return toNextNoteDirectly;
        });
  }

  private Command fromAmpToMiddle1() {
    return AutoBuilder.followPath(ampToMiddle1);
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

  private enum State {
    NO_NOTE,
    HAS_NOTE,
    SHO0TING;

    private enum RangeStatus {
      IN_RANGE,
      OUTSIDE_RANGE
    }
  }
}
