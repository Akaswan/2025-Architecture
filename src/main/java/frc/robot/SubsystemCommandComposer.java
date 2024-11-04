package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class SubsystemCommandComposer {
    private Shoulder m_shoulder;
    private Wrist m_wrist;
    private Intake m_intake;


    public SubsystemCommandComposer(Shoulder shoulder, Wrist wrist, Intake intake) {
        m_shoulder = shoulder;
        m_wrist = wrist;
        m_intake = intake;
    }

    public Command stow() {
        return m_shoulder.stow().alongWith(m_wrist.stow(), m_intake.off());
    }

    public Command up() {
        return m_shoulder.up().andThen(m_wrist.up()).andThen(m_intake.intake());
    }
}
