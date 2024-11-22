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
                new DriveToPosition(m_mecanumDrive, new Pose2d(3,120,Rotation2d.fromDegrees(90))).withTimeout(3000),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW).withTimeout(10),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN).withTimeout(4000),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(20),
                new DriveToPosition(m_mecanumDrive, new Pose2d(3,102, Rotation2d.fromDegrees(90))).withTimeout(2000),
                new DriveToPosition(m_mecanumDrive, new Pose2d(3,130,Rotation2d.fromDegrees(90))).withTimeout(3000),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(2000),

                // Arm is roughly 40
                // 0 makes the arm face down on the Netlify
//                new DriveToPosition(m_mecanumDrive, new Pose2d(-130, 100, Rotation2d.fromDegrees(270))).withTimeout(3000),
//                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT).withTimeout(1000),
//                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT).withTimeout(20), //At this point, you have 5 seconds to place a specimen into the robot to continue the auto
//                new DriveToPosition(m_mecanumDrive, new Pose2d(-130, 115, Rotation2d.fromDegrees(270))).withTimeout(3000),
//                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN).withTimeout(1000),
//                new DriveToPosition(m_mecanumDrive, new Pose2d(-130, 110, Rotation2d.fromDegrees(180))).withTimeout(2000),
//                new DriveToPosition(m_mecanumDrive, new Pose2d(3, 130, Rotation2d.fromDegrees(90))).withTimeout(2000),
//                new DriveToPosition(m_mecanumDrive, new Pose2d(3, 100, Rotation2d.fromDegrees(90))).withTimeout(2000),
//                new DriveToPosition(m_mecanumDrive, new Pose2d(3, 130, Rotation2d.fromDegrees(90))).withTimeout(2000),
//                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(2000),
//                //new DriveToPosition(m_mecanumDrive, new Pose2d(3, 110, Rotation2d.fromDegrees(90))).withTimeout(2000),
//                new DriveToPosition(m_mecanumDrive, new Pose2d(3, 140, Rotation2d.fromDegrees(90))).withTimeout(3000),
//                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLAPSED_INTO_ROBOT).withTimeout(1000),
//                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_OFF).withTimeout(20),
//                new DriveToPosition(m_mecanumDrive, new Pose2d(140, 140, Rotation2d.fromDegrees(90))),
                //new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(2000),

                //End of current auto
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_OFF)

        );
    }


}
