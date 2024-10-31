package org.firstinspires.ftc.teamcode.current.commands;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;

public class MoveToPosition extends CommandBase {
    private Mecanum2025 m_mecanumDrive;
    private Pose2d m_targetPose;

    public MoveToPosition(Mecanum2025 mecanumDrive, Pose2d targetPose) {
        m_mecanumDrive = mecanumDrive;
        m_targetPose = targetPose;

        addRequirements(m_mecanumDrive);
    }

    @Override
    public void initialize() {
        m_mecanumDrive.setTargetPose(m_targetPose);
    }

    @Override
    public void execute() {
        m_mecanumDrive.moveFieldRelative(0.5,0.5,0.5);
    }

    public boolean isFinished() {
        return m_mecanumDrive.atTargetPose();
    }

    public void end(boolean interrupted) {
        m_mecanumDrive.stop();
    }
}
