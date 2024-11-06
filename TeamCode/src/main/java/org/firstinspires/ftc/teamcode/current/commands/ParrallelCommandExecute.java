package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;

public class ParrallelCommandExecute extends ParallelCommandGroup {

    Mecanum2025 m_mecanumDrive;
    Arm2025 m_armSubsystem;

    public ParrallelCommandExecute(Mecanum2025 mecanumDrive, Arm2025 armSubsystem) {
        m_mecanumDrive = mecanumDrive;
        m_armSubsystem = armSubsystem;

        addCommands(
                new DriveToPosition(m_mecanumDrive, new Pose2d(40,0, Rotation2d.fromDegrees(180))).withTimeout(2000),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLAPSED_INTO_ROBOT).withTimeout(2000).andThen(
                        new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT)).withTimeout(1000).andThen(
                                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN).withTimeout(1000).andThen(
                                        new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1000).andThen(
                                                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_OFF)
                                        )
                                )
                )
        );
    }
}
