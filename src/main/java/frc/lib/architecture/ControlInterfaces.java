package frc.lib.architecture;

import edu.wpi.first.wpilibj2.command.Command;

public class ControlInterfaces {
    public interface PositionSubsystem {

        /**
         * 
         * Returns the position of the subsystems motor
         * 
         * <p>Within this method you should you shouldreturn the position that comes from your subsystems motor encoder.</p>
         * 
         * <p>Example implementation for different motor controllers:</p>
         * <pre>
         * 
         * // EXAMPLE \\
         * 
         * // Example implementation for TalonFX motor controller
         * return m_motor.getPosition().getValue();
         *
         * // Example implementation for SparkMax/SparkFlex motor controller
         * return m_encoder.getPosition()
         *
         * </pre>
         * @return The current position of the subsystem's motor, represented in the appropriate unit of measurement
         * for the motor's application (generally meters or degrees).
         */
        double getPosition();

        /**
         * Executes the motion of the motor to a specified position using its integrated PID control loop.
         * 
         * <p>Within this method you should set your motor to run to the specified position using its integrated PID loop.</p>
         * 
         * <p>Example implementation for different motor controllers:</p>
         * <pre>
         * 
         * // EXAMPLE \\
         * 
         * // Example implementation for TalonFX motor controller
         * m_motor.setControl(new PositionVoltage(position));
         *
         * // Example implementation for SparkMax/SparkFlex motor controller
         * m_controller.setReference(position, ControlType.kPosition);
         *
         * </pre>
         * @param position The target position to which the motor should move. The position value is specified in 
         *                 the unit of measurement appropriate for the motor's application (generally meters or degrees).
         */
        void runToPosition(double position);

        /**
         * Returns a command that executes the motion of the motor to a specified position using a motion profile.
         * 
         * <p>Within this method you should create a command that sets your motor to run to the specified position using a motion profiling 
         * approach.</p>
         * 
         * <p>Example implementation for different motor controllers:</p>
         * <pre>
         * 
         * // EXAMPLE \\ 
         * 
         * // Example implementation for TalonFX motor controller
         * return new FunctionalCommand(
         *  () -> {
         *      m_desiredPosition = position; 
         *      m_motor.setControl(new MotionMagicVoltage(position));
         *  },
         *  null,
         *  null,
         *  this::getIsAtSetpoint,
         *  this
         * );
         *
         * // Example implementation for SparkMax/SparkFlex motor controller
         * return new TrapezoidProfileToPosition(
      new TrapezoidProfile(new TrapezoidProfile.Constraints(50, 100)),
        (state) -> {
          runToPosition(state.position);
          m_idealState = state;
        },
        position,
        () -> m_idealState,
        this::getIsAtSetpoint,
        this).beforeStarting(() -> m_desiredPosition = position);
         * 
         * 
         * 
         * m_profileRunner.startProfile(position);
         * 
         * </pre>
         * @param position The target position to which the motor should move. The position value is specified in 
         *                 the unit of measurement appropriate for the motor's application (generally meters or degrees).
         */
        Command runProfileToPosition(double position);

        /**
         * Returns whether a motor is at its desired position
         * 
         * <p>Within this method you should set up logic to determine whether your motor is at its setpoint within a tolerance</p>
         * 
         * <p>Example implementation:</p>
         * <pre>
         * 
         * return Math.abs(getPosition() - m_desiredPosition) < m_setpointTolerance;
         *
         * </pre>
         * 
         * @return <code>true</code> if the motor is within the specified tolerance of the desired position; 
         * <code>false</code> otherwise.
         */
        boolean getIsAtSetpoint();
    }

    // public interface MultiMotorPositionSubsystem {
    //     /**
    //      * 
    //      * Multi Motor Position Subsystem <b>getPosition()</b> Method
    //      * 
    //      * <p>When creating a Position Subsystem you will have one or more motors. This method represents the position of multiple motors.
    //      * When using this method, run a switch statement based on the <b>motorName</b> parameter, and return the correspoding position of that motor encoder.</p>
    //      * 
    //      * @return Position of the subsystems motor
    //      */
    //     default double getPosition(Enum<?> motorName) {
    //         throw new UnsupportedOperationException("Single-motor position subsystems must override getPosition()");
    //     }

    //     default double runToPosition(double... position) {
    //         throw new UnsupportedOperationException("Multi-motor position subsystems must override runToPosition(MotorID)");
    //     }

    // }
}
