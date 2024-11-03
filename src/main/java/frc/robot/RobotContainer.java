// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class RobotContainer {

  private static Shoulder m_shoulder = new Shoulder();
  private static Wrist m_wrist = new Wrist();

  public static SubsystemCommandComposer subsystemStateManager = new SubsystemCommandComposer(m_shoulder, m_wrist);

  private CommandXboxController m_controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_shoulder.setDefaultCommand(Commands.run(() -> m_shoulder.manualControl(m_controller::getLeftY), m_shoulder));
    m_wrist.setDefaultCommand(Commands.run(() -> m_wrist.manualControl(() -> m_controller.getHID().getLeftBumper() ? 1.0 : m_controller.getHID().getRightBumper() ? -1.0 : 0.0), m_wrist));

    m_controller.a().onTrue(subsystemStateManager.stow());
    m_controller.y().onTrue(subsystemStateManager.up());
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
