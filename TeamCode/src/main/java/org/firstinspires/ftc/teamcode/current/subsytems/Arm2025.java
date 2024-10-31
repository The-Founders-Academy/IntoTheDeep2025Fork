package org.firstinspires.ftc.teamcode.current.subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm2025 extends SubsystemBase {

    private final Servo wrist;
    private final DcMotor armMotor;
    private final CRServo intake;

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 239.5 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 220 * ARM_TICKS_PER_DEGREE;    // was 230
    final double ARM_SCORE_SPECIMEN = 151.5 * ARM_TICKS_PER_DEGREE;       // was 148
    final double ARM_SCORE_SAMPLE_IN_LOW = 140 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;


    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;


    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    public double armPosition = ARM_COLLAPSED_INTO_ROBOT;


    public Arm2025(final HardwareMap hardwareMap) {

        wrist = hardwareMap.get(Servo.class, "wrist");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(CRServo.class, "intake");
    }

    public double getARM_TICKS_PER_DEGREE() {
        return ARM_TICKS_PER_DEGREE;
    }

    public double getARM_COLLAPSED_INTO_ROBOT() {
        return ARM_COLLAPSED_INTO_ROBOT;
    }

    public double getARM_COLLECT() {
        return ARM_COLLECT;
    }

    public double getARM_CLEAR_BARRIER() {
        return ARM_CLEAR_BARRIER;
    }

    public double getARM_SCORE_SPECIMEN() {
        return ARM_SCORE_SPECIMEN;
    }

    public double getARM_SCORE_SAMPLE_IN_LOW() {
        return ARM_SCORE_SAMPLE_IN_LOW;
    }

    public double getARM_ATTACH_HANGING_HOOK() {
        return ARM_ATTACH_HANGING_HOOK;
    }

    public double getARM_WINCH_ROBOT() {
        return ARM_WINCH_ROBOT;
    }

    public double getWRIST_FOLDED_IN() {
        return WRIST_FOLDED_IN;
    }

    public double getWRIST_FOLDED_OUT() {
        return WRIST_FOLDED_OUT;
    }

    public double getINTAKE_COLLECT() { return INTAKE_COLLECT; }
    public double getINTAKE_OFF() { return INTAKE_OFF; }
    public double getINTAKE_DEPOSIT() { return INTAKE_DEPOSIT; }

    public void setArmPosition(double armPosition) {
        armMotor.setTargetPosition((int) (armPosition));

        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setArmPower(double power) {
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(power);
    }

    public void setWristPosition(double wristPosition) {
        wrist.setPosition(wristPosition);
    }

    public void setIntake(double intakeSpeed) {
        // Same as motor, speed is between -1 and 1
        intake.setPower(intakeSpeed);
    }

}

