package org.firstinspires.ftc.teamcode.current.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LiftMotorTest extends LinearOpMode {

    @Config
    public static class TestParams {

        public static double LIFT_SCORING_IN_HIGH_BASKET = 0.1;
    }
    public DcMotor liftMotor = null; //the lift motor

    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    double liftPosition = LIFT_COLLAPSED;

    @Override
    public void runOpMode() throws InterruptedException {
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {


            if (gamepad1.right_bumper) {
                 liftPosition = TestParams.LIFT_SCORING_IN_HIGH_BASKET;

            } else if (gamepad1.left_bumper) {
                liftPosition = LIFT_COLLAPSED;
            }



            if (liftPosition > TestParams.LIFT_SCORING_IN_HIGH_BASKET){
                liftPosition = TestParams.LIFT_SCORING_IN_HIGH_BASKET;
            }
            //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
            if (liftPosition < 0){
                liftPosition = 0;
            }

            liftMotor.setTargetPosition((int) (liftPosition));

            ((DcMotorEx) liftMotor).setVelocity(2100);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("lift current position", liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
