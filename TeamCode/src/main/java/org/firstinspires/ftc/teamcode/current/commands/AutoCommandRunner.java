package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.current.opmodes.DriveToPositionTest;
import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;

public class AutoCommandRunner extends SequentialCommandGroup {

    public Arm2025 m_armSubsystem;
    public Mecanum2025 m_mecanumDrive;

    public AutoCommandRunner(Mecanum2025 mecanum2025, Arm2025 armSubsystem) {

        m_mecanumDrive = mecanum2025;
        m_armSubsystem = armSubsystem;



        addCommands(
                new ParrallelCommandExecute(m_mecanumDrive, m_armSubsystem).withTimeout(3000) // Drives and rotates robot to proper position
        );
    }


}
