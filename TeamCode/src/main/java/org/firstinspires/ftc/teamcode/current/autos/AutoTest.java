package org.firstinspires.ftc.teamcode.current.autos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.current.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.current.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;
import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;


@Autonomous
public class AutoTest extends CommandOpMode {

    private Mecanum2025 m_mecanumDrive;
    private Arm2025 m_armSubsystem;

    @Override
    public void initialize() {
        MecanumConfigs mecanumConfigs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        // Pose is set to Blue alliance, closer to blue samples. Control click https://screamingeagles2025.netlify.app/ to see exactly where it's positioned
        m_mecanumDrive = new Mecanum2025(hardwareMap, mecanumConfigs, new Pose2d(-70, 300, Rotation2d.fromDegrees(270)), BaseMecanumDrive.Alliance.RED);
        m_armSubsystem = new Arm2025(hardwareMap);

        // Moves to Blue Human Player zone
        CommandScheduler.getInstance().schedule(  new DriveToPosition(m_mecanumDrive, new Pose2d(-300, 300, m_mecanumDrive.getHeading())).withTimeout(2000));
        CommandScheduler.getInstance().schedule(  new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLAPSED_INTO_ROBOT).withTimeout(2000));

        // Moves to left side of ascent zone
        CommandScheduler.getInstance().schedule(  new DriveToPosition(m_mecanumDrive, new Pose2d(-100, 50,Rotation2d.fromDegrees(180))).withTimeout(2000));
        CommandScheduler.getInstance().schedule(  new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_ATTACH_HANGING_HOOK).withTimeout(2000));
        CommandScheduler.getInstance().schedule(  new ArmCommand(m_armSubsystem, ArmCommand.ArmPosition.ARM_COLLAPSED_INTO_ROBOT).withTimeout(2000));
    }

}
