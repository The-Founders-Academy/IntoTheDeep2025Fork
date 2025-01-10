package org.firstinspires.ftc.teamcode.current.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class LiftMotorTest extends LinearOpMode {

    @Config
    public static class TestParams {
        public static double LIFT_SCORING_IN_HIGH_BASKET = 0.1;
    }

    public DcMotorEx liftMotor = null; // The lift motor

    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;    // 3.204498269896194

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    double liftPosition = LIFT_COLLAPSED;

    @Override
    public void runOpMode() {
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                liftMotor.setTargetPosition((int) (10));

                ((DcMotorEx) liftMotor).setVelocity(2100);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            } else if (gamepad1.left_bumper) {
                liftMotor.setTargetPosition((int) (0));

                ((DcMotorEx) liftMotor).setVelocity(2100);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            telemetry.addData("Lift Current Position", liftMotor.getCurrentPosition());
            telemetry.addData("Lift Target Position", liftMotor.getTargetPosition());
            telemetry.update();
        }
    }
}
