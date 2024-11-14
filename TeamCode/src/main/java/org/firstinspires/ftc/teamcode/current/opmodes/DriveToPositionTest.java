package org.firstinspires.ftc.teamcode.current.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.current.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.current.commands.AutoCommandRunner;
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
        m_mecanumDrive = new Mecanum2025(hardwareMap, mecanumConfigs, new Pose2d(0, 0, Rotation2d.fromDegrees(0)), BaseMecanumDrive.Alliance.RED);
        // sets initial pose, should reflect where robot is physically placed
        m_armSubsystem = new Arm2025(hardwareMap);


        m_driver.buttonB().whenPressed(new AutoCommandRunner(m_mecanumDrive, m_armSubsystem));


    }
}