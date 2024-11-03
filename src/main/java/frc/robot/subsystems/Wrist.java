// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.ControlInterfaces.PositionSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.MotorIds;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TrapezoidProfileToPosition;

public class Wrist extends SubsystemBase implements PositionSubsystem {
  /** Creates a new Shoulder. */

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;

  private double m_desiredPosition;

  private TrapezoidProfile.State m_idealState = new TrapezoidProfile.State(0, 0);

  public Wrist() {

    m_motor = new CANSparkMax(MotorIds.kWristId, MotorType.kBrushless);

    m_encoder = m_motor.getEncoder();
    m_controller = m_motor.getPIDController();

    m_motor.setIdleMode(WristConstants.kIdleMode);
    m_motor.setSmartCurrentLimit(WristConstants.kCurrentLimit);
    m_motor.setInverted(WristConstants.kInverted);

    m_controller.setP(WristConstants.kKp);
    m_controller.setI(WristConstants.kKi);
    m_controller.setD(WristConstants.kKd);

    m_encoder.setPosition(WristConstants.kHomePosition);
    m_encoder.setPositionConversionFactor(WristConstants.kPositionConversionFactor);
    m_encoder.setVelocityConversionFactor(WristConstants.kPositionConversionFactor / 60);

    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65534);

    m_idealState.position = WristConstants.kHomePosition;
    m_desiredPosition = WristConstants.kHomePosition;

    runToPosition(WristConstants.kHomePosition);
  }

  @Override
  public double getPosition() {
    return RobotBase.isReal() ? m_encoder.getPosition() : m_idealState.position;
  }

  @Override
  public void runToPosition(double position) {
    m_controller.setReference(position, ControlType.kPosition);
  }

  @Override
  public Command runProfileToPosition(double position) {
    return new TrapezoidProfileToPosition(
      new TrapezoidProfile(new TrapezoidProfile.Constraints(WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration)),
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
    return Math.abs(getPosition() - m_desiredPosition) < WristConstants.kSetpointTolerance;
  }

  @Override
  public void manualControl(Supplier<Double> throttle) {
    double adjustedThrottle = MathUtil.applyDeadband(-throttle.get(), 0.1);
    double newDesiredPosition = MathUtil.clamp(m_desiredPosition + adjustedThrottle, WristConstants.kMinPosition, WristConstants.kMaxPosition);
    double velocity = (newDesiredPosition - m_desiredPosition) / Constants.kdt;
    
    m_desiredPosition = newDesiredPosition;
    m_idealState.position = m_desiredPosition;
    m_idealState.velocity = velocity;

    runToPosition(m_desiredPosition);
  }

  public Command stow() {
    return runProfileToPosition(0);
  }

  public Command up() {
    return runProfileToPosition(50);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Desired Position", m_desiredPosition);
    SmartDashboard.putNumber("Wrist Ideal Position", m_idealState.position);
    SmartDashboard.putNumber("Wrist Actual Position", getPosition());
  }
}
