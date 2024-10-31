package org.firstinspires.ftc.teamcode.current.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.current.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.current.commands.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.current.commands.WristCommand;
import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;
import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.shared.util.CommandGamepad;

import java.util.Collections;
import java.util.Set;

@TeleOp()
public class

DriveAndArm2025 extends CommandOpMode {
    private Mecanum2025 m_mecanumDrive;
    private CommandGamepad m_driver;
    private Arm2025 armSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();


        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
        reset();
    }

    @Override
    public void initialize() {

        MecanumConfigs configs = new MecanumConfigs().runMode(MotorEx.RunMode.RawPower);

        m_mecanumDrive = new Mecanum2025(hardwareMap, configs, new Pose2d(0, 0, Rotation2d.fromDegrees(90)), BaseMecanumDrive.Alliance.RED);
        armSubsystem = new Arm2025(hardwareMap);

        m_driver = new CommandGamepad(gamepad1, 0.2, 0.2);
        m_mecanumDrive.setDefaultCommand(new DriverRelativeDrive(m_mecanumDrive, m_driver));









          m_driver.buttonA().whenPressed(new InstantCommand(() -> armSubsystem.setWristPosition(armSubsystem.getWRIST_FOLDED_OUT())));
          m_driver.buttonA().whenPressed(new InstantCommand(() -> armSubsystem.setArmPosition(armSubsystem.getARM_COLLECT())));
          //m_driver.buttonA().whenPressed(new InstantCommand(() -> armSubsystem.setWristPosition(0.5)));

          m_driver.buttonB().whenPressed(new InstantCommand(() -> armSubsystem.setArmPosition(armSubsystem.getARM_CLEAR_BARRIER())));
          //m_driver.buttonB().whenPressed(new InstantCommand(() -> armSubsystem.setWristPosition(0.8333)));

          m_driver.buttonX().whenPressed(new InstantCommand(() -> armSubsystem.setArmPosition(armSubsystem.getARM_SCORE_SAMPLE_IN_LOW())));
          m_driver.buttonX().whenPressed(new InstantCommand(() -> armSubsystem.setWristPosition(armSubsystem.getWRIST_FOLDED_OUT())));

          m_driver.dpadDown().whenPressed(new InstantCommand(() -> armSubsystem.setArmPosition(armSubsystem.getARM_COLLAPSED_INTO_ROBOT())));
          m_driver.dpadDown().whenPressed(new InstantCommand(() -> armSubsystem.setWristPosition(armSubsystem.getWRIST_FOLDED_IN())));


          m_driver.dpadUp().whenPressed(new InstantCommand(() -> armSubsystem.setArmPosition(armSubsystem.getARM_ATTACH_HANGING_HOOK())));

          m_driver.dpadRight().whenPressed(new InstantCommand(() -> armSubsystem.setArmPosition(armSubsystem.getARM_SCORE_SPECIMEN())));
          m_driver.dpadRight().whenPressed(new InstantCommand(() -> armSubsystem.setWristPosition(armSubsystem.getWRIST_FOLDED_IN())));


    }

    public void reset() {
        // Any code necessary to reset robot

    }
}