package org.firstinspires.ftc.teamcode.current.autos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.current.commands.AlternateBlueSampleCommandRunner;
import org.firstinspires.ftc.teamcode.current.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Lift2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;
import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;

@Autonomous
public class TestAuto extends CommandOpMode {

    private Mecanum2025 m_mecanumDrive;
    private Arm2025 m_armSubsystem;
    private Lift2025 m_liftSubsystem;

    @Override
    public void initialize() {
        MecanumConfigs mecanumConfigs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        // Pose is set to Blue alliance, closer to yellow samples. Control click https://screamingeagles2025.netlify.app/ to see exactly where it's positioned
        m_mecanumDrive = new Mecanum2025(hardwareMap, mecanumConfigs, new Pose2d(0, 0, Rotation2d.fromDegrees(0)), BaseMecanumDrive.Alliance.BLUE); // y was 159.3
        m_armSubsystem = new Arm2025(hardwareMap);
        m_liftSubsystem = new Lift2025(hardwareMap);


        CommandScheduler.getInstance().schedule(new DriveToPosition(m_mecanumDrive, new Pose2d(0, 0, Rotation2d.fromDegrees(270))).withTimeout(10000));
    }
}
