package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.CommandBase;
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

        // Starting Position:
        addCommands(
                // SCORING PRELOAD SPECIMEN
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(1),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(1),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1),

                // Hooks preloaded specimen onto bar
                new ParallelCommandGroup(
                        new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN).withTimeout(1000),
                        new DriveToPosition(m_mecanumDrive, new Pose2d(3, 102, Rotation2d.fromDegrees(90))).withTimeout(1450)   // was 1750
                ),

                // Deposits preloaded specimen onto bar
                new DriveToPosition(m_mecanumDrive, new Pose2d(3, 135, Rotation2d.fromDegrees(90))).withTimeout(600),

                new ParallelCommandGroup(
                        new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(600),    // was 900
                        new DriveToPosition(m_mecanumDrive, new Pose2d(3, 135, Rotation2d.fromDegrees(90))).withTimeout(600)
                ),

                new ParallelCommandGroup(
                        new DriveToPosition(m_mecanumDrive, new Pose2d(3, 135, Rotation2d.fromDegrees(90))).withTimeout(400)
                ),

                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_OFF).withTimeout(1),




                // GOING FOR SECOND SPECIMEN
                new ParallelCommandGroup(
                        new DriveToPosition(m_mecanumDrive, new Pose2d(-53, 109, Rotation2d.fromDegrees(-140))).withTimeout(2000) // was 2200  // moves to second sample
                ),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(900), // was 1200 // collects second sample
                new DriveToPosition(m_mecanumDrive, new Pose2d(-67, 97, Rotation2d.fromDegrees(-142))).withTimeout(2000), // was 2500  // moves to second sample

                new ParallelCommandGroup(
                        new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(1000), // was 1500
                        new DriveToPosition(m_mecanumDrive, new Pose2d(-124, 124, Rotation2d.fromDegrees(-215))).withTimeout(2500)    // moves to basket
                ),

                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(800)


        );
    }

}

