package frc.lib.utilities;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class TrapezoidProfileRunner extends SubsystemBase {
  private final TrapezoidProfile m_profile;
  private final Consumer<Double> m_output;
  private double m_goal;
  private TrapezoidProfile.State m_currentState;
  private TrapezoidProfile.State m_profileStartState = new TrapezoidProfile.State();
  private final Supplier<Double> m_motorPosition;
  private final double m_setpointTolerance;
  private final ExposedTimer m_timer = new ExposedTimer();

  public TrapezoidProfileRunner(
      TrapezoidProfile profile,
      Consumer<Double> output,
      Supplier<Double> motorPosition,
      double setpointTolerance) {
    m_profile = profile;
    m_output = output;
    m_motorPosition = motorPosition;
    m_setpointTolerance = setpointTolerance;

    m_currentState = new TrapezoidProfile.State(m_motorPosition.get(), 0);
  }

  public void startProfile(double position) {
    if (m_timer.getIsRunning()) {
      m_profileStartState = m_currentState;
    } else {
      m_profileStartState = new TrapezoidProfile.State(m_motorPosition.get(), 0);
    }

    m_timer.restart();
    m_goal = position;
  }

  @Override
  public void periodic() {
    if (m_timer.getIsRunning()) {
      m_currentState =
          m_profile.calculate(
              m_timer.get(), m_profileStartState, new TrapezoidProfile.State(m_goal, 0));

      m_output.accept(m_currentState.position);
      if (Math.abs(m_motorPosition.get() - m_goal) < m_setpointTolerance
          && m_currentState.position == m_goal) {
        m_timer.stop();
      }
    }
  }
}
