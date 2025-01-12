package org.firstinspires.ftc.teamcode.current.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class LiftMotorTest extends LinearOpMode {

    MotorEx liftMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        liftMotor = hardwareMap.get(MotorEx.class, "lift");
        liftMotor.setRunMode(Motor.RunMode.PositionControl);

        liftMotor.setPositionCoefficient(0.05);
        double Kp = liftMotor.getPositionCoefficient();
        liftMotor.set(0); // sets speed to zero

        liftMotor.setTargetPosition(1500);  // in ticks, figure out inpertick
        liftMotor.setPositionTolerance(15); // hopefully in ticks and not cm

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                while (!liftMotor.atTargetPosition()) {
                    liftMotor.set(0.75); // sets speed
                }
                liftMotor.stopMotor();
            }
        }



    }
}
