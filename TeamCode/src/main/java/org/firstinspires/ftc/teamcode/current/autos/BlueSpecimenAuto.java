package org.firstinspires.ftc.teamcode.current.autos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.current.commands.BlueSpecimenCommandRunner;
import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;
import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;


@Autonomous
public class BlueSpecimenAuto extends CommandOpMode {

    private Mecanum2025 m_mecanumDrive;
    private Arm2025 m_armSubsystem;

    @Override
    public void initialize() {
        MecanumConfigs mecanumConfigs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        // Pose is set to Blue alliance, closer to blue samples. Control click https://screamingeagles2025.netlify.app/ to see exactly where it's positioned
        m_mecanumDrive = new Mecanum2025(hardwareMap, mecanumConfigs, new Pose2d(-42.38, 161, Rotation2d.fromDegrees(270)), BaseMecanumDrive.Alliance.BLUE);
        m_armSubsystem = new Arm2025(hardwareMap);


        CommandScheduler.getInstance().schedule(  new BlueSpecimenCommandRunner(m_mecanumDrive, m_armSubsystem));

    }

}
