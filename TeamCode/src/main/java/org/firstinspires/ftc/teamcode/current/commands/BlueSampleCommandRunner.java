package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Lift2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;

public class BlueSampleCommandRunner extends SequentialCommandGroup {

    private Mecanum2025 m_mecanumDrive;
    private Arm2025 m_armSubsystem;
    private Lift2025 m_liftSubsystem;

    public BlueSampleCommandRunner(Mecanum2025 mecanum2025, Arm2025 armSubsystem, Lift2025 liftSubsystem) {
        m_mecanumDrive = mecanum2025;
        m_armSubsystem = armSubsystem;
        m_liftSubsystem = liftSubsystem;

        // Starting Position:
        addCommands(
                // SCORING PRELOADED SAMPLE
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLAPSED_INTO_ROBOT).withTimeout(1),

                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(1), // ensure the arm stays put
                // Hooks preloaded specimen onto bar
                new ParallelCommandGroup(
                        new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1500),
                        new DriveToPosition(m_mecanumDrive, new Pose2d(127, 132, Rotation2d.fromDegrees(219))).withTimeout(3100)    // moves to basket
                ),
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.LEFT_BUMPER_PRESSED).withTimeout(700),

                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(700),

                new DriveToPosition(m_mecanumDrive, new Pose2d(110, 115, Rotation2d.fromDegrees(219))).withTimeout(200),
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(500),  // retracts arm
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1), // intakes second sample





                // SCORING SECOND SECOND SAMPLE
                new ParallelCommandGroup(
                        new DriveToPosition(m_mecanumDrive, new Pose2d(59, 105, Rotation2d.fromDegrees(153))).withTimeout(2000) // was 2000  // moves to second sample
                ),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(1100), // was 1200 // collects second sample
                new DriveToPosition(m_mecanumDrive, new Pose2d(81, 90, Rotation2d.fromDegrees(153))).withTimeout(2000), // was 2500  // moves to second sample

                new ParallelCommandGroup(
                        new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1000), // was 1500
                        new DriveToPosition(m_mecanumDrive, new Pose2d(127, 132, Rotation2d.fromDegrees(219))).withTimeout(3000)    // moves to basket
                ),
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.LEFT_BUMPER_PRESSED).withTimeout(700),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(800),

                new DriveToPosition(m_mecanumDrive, new Pose2d(111, 115, Rotation2d.fromDegrees(219))).withTimeout(200), // was 300
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(700),  // retracts arm



                // SCORING THIRD SAMPLE
                new ParallelCommandGroup(
                        new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(1), // intakes third sample
                        new DriveToPosition(m_mecanumDrive, new Pose2d(81, 102, Rotation2d.fromDegrees(152))).withTimeout(2200)   // moves near third sample
                ),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(1000),  // was 1200 // collects second sample
                new DriveToPosition(m_mecanumDrive, new Pose2d(105, 85, Rotation2d.fromDegrees(152))).withTimeout(2200),   // collects third sample

                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(1000), // was 1500
                new ParallelCommandGroup(
                        new DriveToPosition(m_mecanumDrive, new Pose2d(127, 132, Rotation2d.fromDegrees(225))).withTimeout(3000)    // moves to basket
                ),

                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.LEFT_BUMPER_PRESSED).withTimeout(700),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(700),


                new DriveToPosition(m_mecanumDrive, new Pose2d(111, 115, Rotation2d.fromDegrees(219))).withTimeout(200),  // backs away from basket
                new ArmCommand(m_armSubsystem, m_liftSubsystem, ArmCommand.ArmPosition.RIGHT_BUMPER_PRESSED).withTimeout(400),  // retracts arm



                // DRIVE TO BAR
                new DriveToPosition(m_mecanumDrive, new Pose2d(91, 19, Rotation2d.fromDegrees(0))).withTimeout(1500),  // moves close to bar
                new DriveToPosition(m_mecanumDrive, new Pose2d(35, 19, Rotation2d.fromDegrees(0))).withTimeout(3000)  // rotates to face bar

        );
    }

}
