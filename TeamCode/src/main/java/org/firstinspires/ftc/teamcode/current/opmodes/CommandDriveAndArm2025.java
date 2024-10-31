package org.firstinspires.ftc.teamcode.current.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.button.Trigger;
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
import org.firstinspires.ftc.teamcode.current.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.current.commands.WristCommand;
import org.firstinspires.ftc.teamcode.current.subsytems.Arm2025;
import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;
import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.shared.util.CommandGamepad;

@TeleOp()
public class CommandDriveAndArm2025 extends CommandOpMode {

    private Mecanum2025 m_mecanumDrive;
    private CommandGamepad m_driver;
    private CommandGamepad m_operator;

    private Arm2025 armSubsystem;
    @Override
    public void initialize() {
        MecanumConfigs configs = new MecanumConfigs().runMode(MotorEx.RunMode.RawPower);

        m_mecanumDrive = new Mecanum2025(hardwareMap, configs, new Pose2d(0, 0, Rotation2d.fromDegrees(90)), BaseMecanumDrive.Alliance.RED);
        armSubsystem = new Arm2025(hardwareMap);

        m_driver = new CommandGamepad(gamepad1, 0.2, 0.2);
        m_operator = new CommandGamepad(gamepad2, 0.2, 0.2);

        m_mecanumDrive.setDefaultCommand(new DriverRelativeDrive(m_mecanumDrive, m_driver));

        // Operator Commands
        m_operator.buttonA().whenPressed(new ArmCommand(armSubsystem, ArmCommand.ArmPosition.ARM_COLLECT));
        m_operator.buttonB().whenPressed(new ArmCommand(armSubsystem, ArmCommand.ArmPosition.ARM_CLEAR_BARRIER));
        m_operator.buttonX().whenPressed(new ArmCommand(armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SAMPLE_IN_LOW));

        m_operator.dpadLeft().whenPressed(new ArmCommand(armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN));
        m_operator.dpadUp().whenPressed(new ArmCommand(armSubsystem, ArmCommand.ArmPosition.ARM_SCORE_SPECIMEN));
        m_operator.dpadRight().whenPressed(new ArmCommand(armSubsystem, ArmCommand.ArmPosition.ARM_WINCH_ROBOT));
        m_operator.dpadDown().whenPressed(new ArmCommand(armSubsystem, ArmCommand.ArmPosition.ARM_COLLAPSED_INTO_ROBOT));

        m_operator.leftBumper().whenPressed(new IntakeCommand(armSubsystem, IntakeCommand.IntakeSetting.INTAKE_COLLECT));
        m_operator.rightBumper().whenPressed(new IntakeCommand(armSubsystem, IntakeCommand.IntakeSetting.INTAKE_DEPSOSIT));
        m_operator.buttonY().whenPressed(new IntakeCommand(armSubsystem, IntakeCommand.IntakeSetting.INTAKE_OFF));

//        new Trigger(() -> m_driver.leftTrigger() > 0.25).whenActive(new ArmCommand(armSubsystem, ArmCommand.ArmPosition.LEFT_TRIGGER_PRESSED));
//        new Trigger(() -> m_driver.rightTrigger() > 0.25).whenActive(new ArmCommand(armSubsystem, ArmCommand.ArmPosition.RIGHT_TRIGGER_PRESSED));

        // buttons created in CommandGamepad class
        m_operator.getLeftTriggerActive().whenActive(new ArmCommand(armSubsystem, ArmCommand.ArmPosition.LEFT_TRIGGER_PRESSED));
        m_operator.getrightTriggerActive().whenActive(new ArmCommand(armSubsystem, ArmCommand.ArmPosition.RIGHT_TRIGGER_PRESSED));

    }
}
