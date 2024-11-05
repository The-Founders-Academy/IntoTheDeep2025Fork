package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;
import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;

public class ParallelDriveToPositionCommands extends SequentialCommandGroup {

    public Arm2025 m_armSubsystem;
    public Mecanum2025 m_mecanumDrive;

    public ParallelDriveToPositionCommands(Mecanum2025 mecanum2025, Arm2025 armSubsystem) {

        m_mecanumDrive = mecanum2025;
        m_armSubsystem = armSubsystem;


        addCommands(
                // Drive Commands (assuming you start at 0,0)
                new DriveToPosition(m_mecanumDrive, new Pose2d(0,0, Rotation2d.fromDegrees(90))).withTimeout(2000),
                new DriveToPosition(m_mecanumDrive, new Pose2d(50,0, Rotation2d.fromDegrees(180))).withTimeout(2000),

                // Arm Commands
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN).withTimeout(1000),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN).withTimeout(2000),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT).withTimeout(1000),
                new IntakeCommand(m_armSubsystem, IntakeCommand.IntakeSetting.INTAKE_OFF).withTimeout(1000),
                new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLAPSED_INTO_ROBOT).withTimeout(1000),

                // Drives back to original position, with original rotation
                new DriveToPosition(m_mecanumDrive, new Pose2d(0,0, Rotation2d.fromDegrees(0))).withTimeout(2000)

        );
    }


}
