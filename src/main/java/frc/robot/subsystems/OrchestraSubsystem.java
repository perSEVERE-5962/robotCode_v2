package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OrchestraSubsystem extends SubsystemBase {

    private final Orchestra m_orchestra = new Orchestra();
    private boolean m_isPlaying = false;

    public OrchestraSubsystem(TalonFX... motors) {
        for (TalonFX motor : motors) {
            m_orchestra.addInstrument(motor);
        }

        var status = m_orchestra.loadMusic("FromTheStart.chrp");
        if (!status.isOK()) {
            System.out.println("Orchestra failed to load music: " + status.toString());
        }
    }

    public void togglePlayPause() {
        if (m_isPlaying) {
            m_orchestra.pause();
            m_isPlaying = false;
        } else {
            m_orchestra.play();
            m_isPlaying = true;
        }
    }

    public void stop() {
        m_orchestra.stop();
        m_isPlaying = false;
    }
}