// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.ControlInterfaces.PositionSubsystem;
import frc.robot.Constants.MotorIds;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.commands.TrapezoidProfileToPosition;

public class Shoulder extends SubsystemBase implements PositionSubsystem {
  /** Creates a new Shoulder. */

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;

  private double m_desiredPosition;

  private TrapezoidProfile.State m_idealState = new TrapezoidProfile.State(0, 0);

  public Shoulder() {

    m_motor = new CANSparkMax(MotorIds.kShoulderId, MotorType.kBrushless);

    m_encoder = m_motor.getEncoder();
    m_controller = m_motor.getPIDController();

    m_motor.setIdleMode(ShoulderConstants.kIdleMode);
    m_motor.setSmartCurrentLimit(ShoulderConstants.kCurrentLimit);
    m_motor.setInverted(ShoulderConstants.kInverted);

    m_controller.setP(ShoulderConstants.kKp);
    m_controller.setI(ShoulderConstants.kKi);
    m_controller.setD(ShoulderConstants.kKd);

    m_encoder.setPosition(ShoulderConstants.kHomePosition);
    m_encoder.setPositionConversionFactor(ShoulderConstants.kPositionConversionFactor);
    m_encoder.setVelocityConversionFactor(ShoulderConstants.kPositionConversionFactor / 60);

    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65534);

    m_idealState.position = ShoulderConstants.kHomePosition;
    m_desiredPosition = ShoulderConstants.kHomePosition;

    runToPosition(ShoulderConstants.kHomePosition);
  }

  @Override
  public double getPosition() {
    return RobotBase.isReal() ? m_encoder.getPosition() : m_idealState.position;
  }

  @Override
  public void runToPosition(double position) {
    System.out.println(position);
    m_controller.setReference(position, ControlType.kPosition);
  }

  @Override
  public Command runProfileToPosition(double position) {
    return new TrapezoidProfileToPosition(
      new TrapezoidProfile(new TrapezoidProfile.Constraints(50, 100)),
        (state) -> {
          runToPosition(state.position);
          m_idealState = state;
        },
        position,
        () -> m_idealState,
        this::getIsAtSetpoint,
        this).beforeStarting(() -> m_desiredPosition = position);
  }

  @Override
  public boolean getIsAtSetpoint() {
    return Math.abs(getPosition() - m_desiredPosition) < ShoulderConstants.kSetpointTolerance;
  }

  public Command stow() {
    return runProfileToPosition(0);
  }

  public Command up() {
    return runProfileToPosition(100);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Desired Position", m_desiredPosition);
    SmartDashboard.putNumber("Ideal Position", m_idealState.position);
    SmartDashboard.putNumber("Actual Position", getPosition());
  }
}
