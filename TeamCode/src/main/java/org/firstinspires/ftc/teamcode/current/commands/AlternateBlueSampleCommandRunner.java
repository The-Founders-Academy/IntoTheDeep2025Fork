package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Lift2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;

public class AlternateBlueSampleCommandRunner extends SequentialCommandGroup {

    private Mecanum2025 m_mecanumDrive;
    private Arm2025 m_armSubsystem;
    private Lift2025 m_liftSubsystem;

    public AlternateBlueSampleCommandRunner(Mecanum2025 mecanum2025, Arm2025 armSubsystem, Lift2025 liftSubsystem) {
        m_mecanumDrive = mecanum2025;
        m_armSubsystem = armSubsystem;
        m_liftSubsystem = liftSubsystem;


        addCommands(
                // SETUP
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(1),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLAPSED_INTO_ROBOT).withTimeout(1),

                // SCORING FIRST SAMPLE
                new ParallelCommandGroup(
                        new DriveToPosition(m_mecanumDrive, new Pose2d(126, 126, Rotation2d.fromDegrees(217))).withTimeout(3000),
                        new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1500)
                ),
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.LEFT_BUMPER_PRESSED).withTimeout(800),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(700),
                new DriveToPosition(m_mecanumDrive, new Pose2d(120, 120, Rotation2d.fromDegrees(217))).withTimeout(500),
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(800),


                // COLLECTING SECOND SAMPLE
                new DriveToPosition(m_mecanumDrive, new Pose2d(55, 105, Rotation2d.fromDegrees(151))).withTimeout(6000), // 142 deg and was 2200  // moves to second sample
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(900), // was 1200 // collects second sample
                new DriveToPosition(m_mecanumDrive, new Pose2d(70.68, 97, Rotation2d.fromDegrees(151))).withTimeout(2000), // was 2500  // moves to second sample

                // SCORING SECOND SAMPLE
                 new ParallelCommandGroup(
                new DriveToPosition(m_mecanumDrive, new Pose2d(126, 126, Rotation2d.fromDegrees(217))).withTimeout(4000),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1500)
                ),

                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.LEFT_BUMPER_PRESSED).withTimeout(800),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(700),
                new DriveToPosition(m_mecanumDrive, new Pose2d(120, 120, Rotation2d.fromDegrees(217))).withTimeout(500),
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(800),


                // COLLECTING THIRD SAMPLE
                new ParallelCommandGroup(
                    new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1), // intakes third sample
                    new DriveToPosition(m_mecanumDrive, new Pose2d(63, 102, Rotation2d.fromDegrees(155))).withTimeout(6000)   // moves near third sample
                ),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(900),  // was 1200 // collects second sample
                new DriveToPosition(m_mecanumDrive, new Pose2d(81, 87, Rotation2d.fromDegrees(155))).withTimeout(2200),

                // SCORING THIRD SAMPLE
                new ParallelCommandGroup(
                    new DriveToPosition(m_mecanumDrive, new Pose2d(126, 126, Rotation2d.fromDegrees(217))).withTimeout(3000),
                    new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1500)
                ),

                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.LEFT_BUMPER_PRESSED).withTimeout(700),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(700),

                new DriveToPosition(m_mecanumDrive, new Pose2d(120, 120, Rotation2d.fromDegrees(217))).withTimeout(200),
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(700)

        );
    }
}
