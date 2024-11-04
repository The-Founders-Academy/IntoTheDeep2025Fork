package org.firstinspires.ftc.teamcode.current.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.current.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.current.commands.ParallelDriveToPositionCommands;
import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;
import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.shared.util.CommandGamepad;


@TeleOp
public class DriveToPositionTest extends CommandOpMode {
    private Mecanum2025 m_mecanumDrive;
    private CommandGamepad m_driver;

    private Arm2025 m_armSubsystem;


    @Override
    public void initialize() {
        m_driver = new CommandGamepad(gamepad1, 0, 0);
        MecanumConfigs mecanumConfigs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        m_mecanumDrive = new Mecanum2025(hardwareMap, mecanumConfigs, new Pose2d(0, 0, Rotation2d.fromDegrees(0)), BaseMecanumDrive.Alliance.RED);  // sets initial pose to 0, 0, 0
        m_armSubsystem = new Arm2025(hardwareMap);

//        m_driver.buttonA().whenPressed(new DriveToPosition(m_mecanumDrive, new Pose2d(15, 15, m_mecanumDrive.getHeading())).withTimeout(2000)); // Move 15 cm forward without changing heading
        m_driver.buttonX().whenPressed(new DriveToPosition(m_mecanumDrive, new Pose2d(m_mecanumDrive.getPose().getX(), m_mecanumDrive.getPose().getY(), Rotation2d.fromDegrees(90))).withTimeout(2000)); // return to starting position
        m_driver.buttonY().whenPressed(new DriveToPosition(m_mecanumDrive, new Pose2d(0, 0, Rotation2d.fromDegrees(90))).withTimeout(2000)); // Rotate to 90 deg

        m_driver.buttonB().whenPressed(new InstantCommand(() -> {
            m_mecanumDrive.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        }, m_mecanumDrive));

        m_driver.buttonA().whenPressed(new ParallelDriveToPositionCommands(m_mecanumDrive, m_armSubsystem));
    }
}