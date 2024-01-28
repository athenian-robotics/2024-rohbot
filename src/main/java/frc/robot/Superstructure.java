package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Superstructure extends SubsystemBase {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;

  public Superstructure(Intake intake, Indexer indexer, Shooter shooter, Swerve swerve) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    ParallelCommandGroup toDo = new ParallelCommandGroup();
    if (intake.getState() == Intake.State.NOTE_FOUND
        && indexer.getState() != Indexer.State.LOADING) {
      toDo.addCommands(indexer.startLoading());
    }
    if (indexer.getState() == Indexer.State.EMPTY && intake.getState() != Intake.State.NO_NOTE) {
      toDo.addCommands(intake.shotFired());
    }
    toDo.schedule();
  }

  public Command fireShot() {
    return shooter.waitUntilReady().andThen(indexer.waitUntilReady()).andThen(indexer.fire());
  }
}
