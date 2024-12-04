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
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(1),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1),

                // Parallel
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN).withTimeout(1000),
                new DriveToPosition(m_mecanumDrive, new Pose2d(3, 102, Rotation2d.fromDegrees(90))).withTimeout(1750),

                new DriveToPosition(m_mecanumDrive, new Pose2d(3, 135, Rotation2d.fromDegrees(90))).withTimeout(600),
                // new DriveToPosition(m_mecanumDrive, new Pose2d(3, 140, Rotation2d.fromDegrees(90))).withTimeout(300),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(900),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_OFF).withTimeout(1),

                // Parallel
                new DriveToPosition(m_mecanumDrive, new Pose2d(-89 , 87, Rotation2d.fromDegrees(270))).withTimeout(1250),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLAPSED_INTO_ROBOT).withTimeout(1000),

                new DriveToPosition(m_mecanumDrive, new Pose2d(-89, 20, Rotation2d.fromDegrees(270))).withTimeout(900),
                new DriveToPosition(m_mecanumDrive, new Pose2d(-133, 20, Rotation2d.fromDegrees(270))).withTimeout(1000), //135
                new DriveToPosition(m_mecanumDrive, new Pose2d(-133, 140, Rotation2d.fromDegrees(270))).withTimeout(1250),
                new DriveToPosition(m_mecanumDrive, new Pose2d(-133, 100, Rotation2d.fromDegrees(270))).withTimeout(500),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(700),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1),

                // Grabbing Time, Don't Touch
                new DriveToPosition(m_mecanumDrive, new Pose2d(-133, 90, Rotation2d.fromDegrees(270))).withTimeout(3000),
                new DriveToPosition(m_mecanumDrive, new Pose2d(-133, 106, Rotation2d.fromDegrees(270))).withTimeout(1500),

                // Parallel
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN).withTimeout(750),
                new DriveToPosition(m_mecanumDrive, new Pose2d(8, 140, Rotation2d.fromDegrees(90))).withTimeout(1750),

                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1),
                new DriveToPosition(m_mecanumDrive, new Pose2d(8, 100, Rotation2d.fromDegrees(90))).withTimeout(1500),
                new DriveToPosition(m_mecanumDrive, new Pose2d(8, 140, Rotation2d.fromDegrees(90))).withTimeout(1000),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(900),

                // second push

                new DriveToPosition(m_mecanumDrive, new Pose2d(-20, 161, Rotation2d.fromDegrees(270))).withTimeout(3000)
        );
    }


}
