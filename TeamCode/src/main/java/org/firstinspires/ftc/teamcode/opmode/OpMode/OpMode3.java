package org.firstinspires.ftc.teamcode.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp(name = "OpMode3")
public class OpMode3 extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    private DcMotor arm = null;
    private DcMotor rail = null;
    private Servo claw = null;

    private static final int ARM_POSITION_INIT = 300;
    private static final int ARM_POSITION_INTAKE = 450;
    private static final int ARM_POSITION_BASKET = 2500;

    private static final double CLAW_CLOSE = 0.5;
    private static final double CLAW_OPEN = 0;

    private enum RobotState {
        INIT,
        INTAKE,
        BASKET
    }

    private RobotState currentState = RobotState.INIT;
    private boolean clawState = false;
    private boolean previousA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        arm = robot.motorarm;
        claw = robot.servoclaw;
        rail = robot.motorlinearRail;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // --- CONTROLS ---
            // A = toggle claw
            // B = toggle linear rail
            // X = claw to INIT position
            // Y = claw to INTAKE position

            if (gamepad1.x) {
                currentState = RobotState.INIT;
            } else if (gamepad1.y) {
                currentState = RobotState.INTAKE;
            }

            if (gamepad1.a && !previousA) {
                clawState = !clawState;
                claw.setPosition(clawState ? CLAW_CLOSE : CLAW_OPEN);
            }
            previousA = gamepad1.a;

            int armTarget = 0;
            switch (currentState) {
                case INIT:
                    armTarget = ARM_POSITION_INIT;
                    break;
                case INTAKE:
                    armTarget = ARM_POSITION_INTAKE;
                    break;
            }

            arm.setTargetPosition(armTarget);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);

            double horizontal = -gamepad1.right_stick_x * 0.6;
            double vertical = gamepad1.right_stick_y * 0.6;
            double turn = -gamepad1.left_stick_x * 0.6;

            double flPower = vertical + turn + horizontal;
            double frPower = vertical - turn - horizontal;
            double blPower = vertical + turn - horizontal;
            double brPower = vertical - turn + horizontal;

            double max = Math.max(1.0,
                    Math.max(Math.abs(flPower),
                            Math.max(Math.abs(frPower),
                                    Math.max(Math.abs(blPower), Math.abs(brPower)))));

            robot.setDrivePower(flPower / max, frPower / max, blPower / max, brPower / max);

            telemetry.addData("Claw", clawState ? "Closed" : "Open");
            telemetry.addData("Arm Target", armTarget);
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
