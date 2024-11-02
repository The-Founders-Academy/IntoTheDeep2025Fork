package org.firstinspires.ftc.teamcode.current.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.current.commands.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;
import org.firstinspires.ftc.teamcode.shared.util.CommandGamepad;


@TeleOp
public class Drive2025 extends CommandOpMode {
    private CommandGamepad m_driver;
    private Mecanum2025 m_mecanumDrive;


    @Override
    public void initialize() {

        MecanumConfigs configs = new MecanumConfigs().runMode(MotorEx.RunMode.RawPower);
        m_mecanumDrive = new Mecanum2025(hardwareMap, configs, new Pose2d(0, 0, Rotation2d.fromDegrees(90)), BaseMecanumDrive.Alliance.RED);
        m_driver = new CommandGamepad(gamepad1, 0.2, 0.2);
        m_mecanumDrive.setDefaultCommand(new DriverRelativeDrive(m_mecanumDrive, m_driver));
    }

}