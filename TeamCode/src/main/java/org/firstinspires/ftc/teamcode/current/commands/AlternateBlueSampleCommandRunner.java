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
                        new DriveToPosition(m_mecanumDrive, new Pose2d(133, 133, Rotation2d.fromDegrees(218))).withTimeout(1800), //was 2000
                        new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1700)
                ),
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.LEFT_BUMPER_PRESSED).withTimeout(800),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(700),
                new DriveToPosition(m_mecanumDrive, new Pose2d(120, 120, Rotation2d.fromDegrees(218))).withTimeout(200), // was 400


                // COLLECTING SECOND SAMPLE

                new ParallelCommandGroup(
                        new DriveToPosition(m_mecanumDrive, new Pose2d(/*55*/ 52.7725542650382, 92 /*105*/, Rotation2d.fromDegrees(154))).withTimeout(2000),// was 2050 // moves to second sample
                        new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(800)
                ),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(900), // was 1200 // collects second sample
                new DriveToPosition(m_mecanumDrive, new Pose2d(/*68*/ 83.22048495703797, 77 /*96*/, Rotation2d.fromDegrees(154))).withTimeout(1300), // was 1500 // moves to second sample

                // SCORING SECOND SAMPLE
                new ParallelCommandGroup(
                        new DriveToPosition(m_mecanumDrive, new Pose2d(129, 135, Rotation2d.fromDegrees(226))).withTimeout(2500),
                        new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1500)
                ),

                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.LEFT_BUMPER_PRESSED).withTimeout(800),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(700),

                new DriveToPosition(m_mecanumDrive, new Pose2d(120, 120, Rotation2d.fromDegrees(215))).withTimeout(200), // was 500


                // COLLECTING Third SAMPLE

                new ParallelCommandGroup(
                        new DriveToPosition(m_mecanumDrive, new Pose2d(/*55*/ 81, 90 /*105*/, Rotation2d.fromDegrees(154))).withTimeout(2000), // moves to third sample, was 2100
                        new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(800)
                        ),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(900), // was 1200 // collects second sample
                new DriveToPosition(m_mecanumDrive, new Pose2d(/*68*/ 113, 79 /*96*/, Rotation2d.fromDegrees(154))).withTimeout(1300), // was 1500 // moves to third sample
                new DriveToPosition(m_mecanumDrive, new Pose2d(0, 75, Rotation2d.fromDegrees(154))).withTimeout(120),
                // SCORING Third SAMPLE
                new ParallelCommandGroup(
                        new DriveToPosition(m_mecanumDrive, new Pose2d(128, 135, Rotation2d.fromDegrees(226))).withTimeout(2400), // was 2500
                        new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1500)
                ),

                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.LEFT_BUMPER_PRESSED).withTimeout(800),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(700),
                new DriveToPosition(m_mecanumDrive, new Pose2d(120, 120, Rotation2d.fromDegrees(215))).withTimeout(200),

                // TOUCHING BAR
//                new DriveToPosition(m_mecanumDrive, new Pose2d(100, 27, Rotation2d.fromDegrees(0))).withTimeout(2400),
//                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_OFF).withTimeout(1),
//
//                new ParallelCommandGroup(
//                    new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN).withTimeout(2000),
//                    new DriveToPosition(m_mecanumDrive, new Pose2d(60, 27, Rotation2d.fromDegrees(0))).withTimeout(3000)
//                )d



                // COLLECTING FOURTH SAMPLE
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1), // intakes third sample

                new ParallelCommandGroup(
                    new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(800),
                    new DriveToPosition(m_mecanumDrive, new Pose2d(100, 55, Rotation2d.fromDegrees(180))).withTimeout(2200)  // was 2500 // moves near third sample
                ),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(900),  // collects second sample
                new DriveToPosition(m_mecanumDrive, new Pose2d(125, 50, Rotation2d.fromDegrees(180))).withTimeout(1000), // was 1300
                new DriveToPosition(m_mecanumDrive, new Pose2d(0, 50, Rotation2d.fromDegrees(180))).withTimeout(100),


                // SCORING FOURTH SAMPLE
                new ParallelCommandGroup(
                    new DriveToPosition(m_mecanumDrive, new Pose2d(128, 137, Rotation2d.fromDegrees(223))).withTimeout(2900), // was 3000
                    new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1500)
                ),

                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.LEFT_BUMPER_PRESSED).withTimeout(700),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(700),

                new DriveToPosition(m_mecanumDrive, new Pose2d(120, 120, Rotation2d.fromDegrees(217))).withTimeout(200),
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(700)
        );
    }
}
