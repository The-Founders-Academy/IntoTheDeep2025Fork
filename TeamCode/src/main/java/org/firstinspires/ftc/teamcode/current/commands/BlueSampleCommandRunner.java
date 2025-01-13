package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;

public class BlueSampleCommandRunner extends SequentialCommandGroup {

    private Mecanum2025 m_mecanumDrive;
    private Arm2025 m_armSubsystem;

    public BlueSampleCommandRunner(Mecanum2025 mecanum2025, Arm2025 armSubsystem) {
        m_mecanumDrive = mecanum2025;
        m_armSubsystem = armSubsystem;

        // Starting Position:
        addCommands(
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_CLEAR_BARRIER).withTimeout(1),

                // Hooks preloaded specimen onto bar
                new ParallelCommandGroup(
                        new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN).withTimeout(1500),
                        new DriveToPosition(m_mecanumDrive, new Pose2d(3, 135, Rotation2d.fromDegrees(90))).withTimeout(2000)
                ),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1),
                new DriveToPosition(m_mecanumDrive, new Pose2d(3, 98, Rotation2d.fromDegrees(90))).withTimeout(2000), // was 1750
                // Deposits preloaded specimen onto bar
                //new DriveToPosition(m_mecanumDrive, new Pose2d(3, 135, Rotation2d.fromDegrees(90))).withTimeout(600),

                new ParallelCommandGroup(
                        new DriveToPosition(m_mecanumDrive, new Pose2d(3, 135, Rotation2d.fromDegrees(90))).withTimeout(600)
                ),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(600),    // was 900

                new DriveToPosition(m_mecanumDrive, new Pose2d(3, 145, Rotation2d.fromDegrees(90))).withTimeout(900),
                new DriveToPosition(m_mecanumDrive, new Pose2d(60, 145, Rotation2d.fromDegrees(90))).withTimeout(900)

//                new DriveToPosition(m_mecanumDrive, new Pose2d(50, 105, Rotation2d.fromDegrees(157))).withTimeout(2000),//y WAS 98
//                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(1500),
//                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1),
//
//                new DriveToPosition(m_mecanumDrive, new Pose2d(70, 87, Rotation2d.fromDegrees(157))).withTimeout(3000),
//                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1500),
//
//                new DriveToPosition(m_mecanumDrive, new Pose2d(120.82 ,135.68, Rotation2d.fromDegrees(221))).withTimeout(2000),
//                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(2000),
//
//                new DriveToPosition(m_mecanumDrive, new Pose2d(88, 61.4, Rotation2d.fromDegrees(180))).withTimeout(2000),
//                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(1500),
//                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1),
//                new DriveToPosition(m_mecanumDrive, new Pose2d(100, 61.4, Rotation2d.fromDegrees(180))).withTimeout(2000),
//
//                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1500),
//                new DriveToPosition(m_mecanumDrive, new Pose2d(120.82 ,135.68, Rotation2d.fromDegrees(221))).withTimeout(2000),
//                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(1)

                );
    }

}
