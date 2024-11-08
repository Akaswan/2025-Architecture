// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that runs a {@link TrapezoidProfile}. Useful for smoothly controlling mechanism motion.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class TrapezoidProfileToPosition extends Command {
  private final TrapezoidProfile m_profile;
  private final Consumer<State> m_output;
  private final double m_goal;
  private final Supplier<State> m_currentState;
  private State m_profileStartState;
  private final Supplier<Boolean> m_isAtSetpoint;
  private final Timer m_timer = new Timer();

  /**
   * Creates a new TrapezoidProfileCommand that will execute the given {@link TrapezoidProfile}.
   * Output will be piped to the provided consumer function.
   *
   * @param profile The motion profile to execute.
   * @param output The consumer for the profile output.
   * @param goal The supplier for the desired state
   * @param currentState The current state
   * @param requirements The subsystems required by this command.
   */
  public TrapezoidProfileToPosition(
      TrapezoidProfile profile,
      Consumer<State> output,
      double goal,
      Supplier<State> currentState,
      Supplier<Boolean> isAtSetpoint,
      Subsystem... requirements) {
    m_profile = requireNonNullParam(profile, "profile", "TrapezoidProfileCommand");
    m_output = requireNonNullParam(output, "output", "TrapezoidProfileCommand");
    m_goal = goal;
    m_currentState = currentState;
    m_isAtSetpoint = isAtSetpoint;

    m_profileStartState = m_currentState.get();
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_profileStartState = m_currentState.get();
  }

  @Override
  public void execute() {
    m_output.accept(m_profile.calculate(m_timer.get(), m_profileStartState, new State(m_goal, 0)));
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_profile.totalTime()) && m_isAtSetpoint.get();
  }
}
