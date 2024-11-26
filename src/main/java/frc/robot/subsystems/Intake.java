// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.architecture.ControlInterfaces.VoltageSubsystem;
import frc.robot.Constants.MotorIds;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase implements VoltageSubsystem {

  private CANSparkMax m_motor;

  private double m_idealVoltage;

  /** Creates a new Intake. */
  public Intake() {
    m_motor = new CANSparkMax(MotorIds.kIntakeId, MotorType.kBrushless);

    m_motor.setIdleMode(IntakeConstants.kIdleMode);
    m_motor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
    m_motor.setInverted(IntakeConstants.kInverted);

    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65534);
  }

  @Override
  public double getVoltage() {
    return m_motor.getAppliedOutput();
  }

  @Override
  public void setVoltage(double voltage) {
    m_idealVoltage = voltage;
    m_motor.setVoltage(voltage);
  }

  public Command off() {
    return new InstantCommand(() -> setVoltage(0), this);
  }

  public Command intake() {
    return new InstantCommand(() -> setVoltage(-12), this);
  }

  public Command outtake() {
    return new InstantCommand(() -> setVoltage(12), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Voltage", getVoltage());
    SmartDashboard.putNumber("Ideal Intake Voltage", m_idealVoltage);
  }
}
