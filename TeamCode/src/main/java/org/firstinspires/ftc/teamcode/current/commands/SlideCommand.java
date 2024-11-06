package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;

public class SlideCommand extends CommandBase {

   private Arm2025 m_armSubsytem;
   private SlideSetting m_slideSetting;

    public enum SlideSetting {
        SLIDE_EXTENDED,
        SLIDE_RETRACTED

    }

    public SlideCommand(Arm2025 armSubsystem, SlideSetting slideSetting) {
       m_armSubsytem = armSubsystem;
       m_slideSetting = slideSetting;

       addRequirements(m_armSubsytem);
    }

    @Override
    public void initialize() {
        switch(m_slideSetting) {
            case SLIDE_EXTENDED:
                m_armSubsytem.setSlidePosition(m_armSubsytem.getSLIDE_EXTENDED());
                break;
            case SLIDE_RETRACTED:
                m_armSubsytem.setSlidePosition(m_armSubsytem.getSLIDE_RETRACTED());
        }


    }

}
