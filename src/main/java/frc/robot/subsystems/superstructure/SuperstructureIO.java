package frc.robot.subsystems.superstructure;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.*;
import java.nio.ByteBuffer;
import org.littletonrobotics.junction.AutoLog;

public interface SuperstructureIO {
  void iterateStateMachine();

  Command shoot();

  Command test();

  Command cancelShot();

  void updateInputs(SuperstructureInputs inputs);

  Command waitUntilEmpty();

  @AutoLog
  class SuperstructureInputs {
    double sensorRange;
    boolean shooterEmpty;
    State state; // its loggable, but at what cost? also does not show up in elastic...
    String stateString; // hack but like chill on me
  }

  record State(SubsystemState subsystemState, RangeState rangeState)
      implements StructSerializable, Serializable {
    public static final StateStruct struct = new StateStruct();

    enum SubsystemState {
      NO_NOTE,
      HAS_NOTE,
      SHOOTING,
      TESTING,
      SYSID,
      AMP
    }

    enum RangeState {
      IN_RANGE,
      OUTSIDE_RANGE
    }

    State changeSubsystemState(SubsystemState newSubsystemState) {
      return new State(newSubsystemState, rangeState);
    }

    State changeRangeState(RangeState newRangeState) {
      return new State(subsystemState, newRangeState);
    }

    private static class StateStruct implements Struct<State> {
      @Override
      public Class<State> getTypeClass() {
        return State.class;
      }

      @Override
      public String getTypeString() {
        return "struct:SuperstructureState";
      }

      @Override
      public int getSize() {
        return 2 * kSizeInt32;
      }

      @Override
      public String getSchema() {
        return "SubsystemState subsystemState;RangeState rangeState";
      }

      // ordinals are not supposed to be used like this... but idc
      @Override
      public State unpack(ByteBuffer byteBuffer) {
        SubsystemState state = SubsystemState.values()[byteBuffer.getInt()];
        RangeState rangeState = RangeState.values()[byteBuffer.getInt()];
        return new State(state, rangeState);
      }

      @Override
      public void pack(ByteBuffer byteBuffer, State state) {
        byteBuffer.putInt(state.subsystemState.ordinal());
        byteBuffer.putInt(state.rangeState.ordinal());
      }
    }
  }

  // testing
  static void main(String[] args) {
    ByteArrayOutputStream bOut = new ByteArrayOutputStream();
    ObjectOutputStream oOut;
    try {
      oOut = new ObjectOutputStream(bOut);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    try {
      oOut.writeObject(new State(State.SubsystemState.NO_NOTE, State.RangeState.IN_RANGE));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    try {
      oOut.close();
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    System.out.println(bOut.toByteArray().length);

    ByteArrayInputStream bIn = new ByteArrayInputStream(bOut.toByteArray());
    ObjectInputStream oIn;
    try {
      oIn = new ObjectInputStream(bIn);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    try {
      State state = (State) oIn.readObject();
      System.out.println(state);
    } catch (IOException | ClassNotFoundException e) {
      throw new RuntimeException(e);
    }
  }
}
