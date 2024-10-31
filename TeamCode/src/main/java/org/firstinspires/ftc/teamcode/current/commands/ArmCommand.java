package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;


public class ArmCommand extends CommandBase {
    public enum ArmPosition {
        ARM_COLLAPSED_INTO_ROBOT,
        ARM_COLLECT,
        ARM_CLEAR_BARRIER,
        ARM_SCORE_SPECIMEN,
        ARM_SCORE_SAMPLE_IN_LOW,
        ARM_ATTACH_HANGING_HOOK,
        ARM_WINCH_ROBOT,

        LEFT_TRIGGER_PRESSED,

        RIGHT_TRIGGER_PRESSED
    }

    private Arm2025 m_armSubsystem;
    private ArmPosition m_armPosition;

    public ArmCommand(Arm2025 armSubsystem, ArmPosition armPosition) {
        m_armPosition = armPosition;
        m_armSubsystem = armSubsystem;

        addRequirements(m_armSubsystem);
        // TODO See if the line above is actually needed

    }

    public void initialize() {
        switch (m_armPosition) {
            case ARM_COLLAPSED_INTO_ROBOT:
                m_armSubsystem.setArmPosition(m_armSubsystem.getARM_COLLAPSED_INTO_ROBOT());
                m_armSubsystem.setWristPosition(m_armSubsystem.getWRIST_FOLDED_IN());
                break;

            case ARM_COLLECT:
                m_armSubsystem.setArmPosition(m_armSubsystem.getARM_COLLECT());
                m_armSubsystem.setWristPosition(m_armSubsystem.getWRIST_FOLDED_OUT());
                break;

            case ARM_CLEAR_BARRIER:
                m_armSubsystem.setArmPosition(m_armSubsystem.getARM_CLEAR_BARRIER());
                break;

            case ARM_SCORE_SPECIMEN:
                m_armSubsystem.setArmPosition(m_armSubsystem.getARM_SCORE_SPECIMEN());
                m_armSubsystem.setWristPosition(m_armSubsystem.getWRIST_FOLDED_IN());
                break;

            case ARM_SCORE_SAMPLE_IN_LOW:
                m_armSubsystem.setArmPosition(m_armSubsystem.getARM_SCORE_SAMPLE_IN_LOW());
                m_armSubsystem.setWristPosition(m_armSubsystem.getWRIST_FOLDED_OUT());
                break;

            case ARM_ATTACH_HANGING_HOOK:
                m_armSubsystem.setArmPosition(m_armSubsystem.getARM_ATTACH_HANGING_HOOK());
                break;

            case ARM_WINCH_ROBOT:
                m_armSubsystem.setArmPosition(m_armSubsystem.getARM_WINCH_ROBOT());
                break;

            case LEFT_TRIGGER_PRESSED:
                m_armSubsystem.setArmPower(-0.1);
                break;

            case RIGHT_TRIGGER_PRESSED:
                m_armSubsystem.setArmPower(0.1);
                break;
        }

    }
}
