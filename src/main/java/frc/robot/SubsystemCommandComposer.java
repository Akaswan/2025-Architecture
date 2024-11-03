package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class SubsystemCommandComposer {
    private Shoulder m_shoulder;
    private Wrist m_wrist;


    public SubsystemCommandComposer(Shoulder shoulder, Wrist wrist) {
        m_shoulder = shoulder;
        m_wrist = wrist;
    }

    public Command stow() {
        return m_shoulder.stow().alongWith(m_wrist.stow());
    }

    public Command up() {
        return m_shoulder.up().andThen(m_wrist.up());
    }
}
