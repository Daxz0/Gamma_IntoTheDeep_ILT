package org.firstinspires.ftc.teamcode.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.RobotConfig;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp(name = "OpMode3")
public class OpMode3 extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    private enum RobotState {
        INIT,
        INTAKE,
        BASKET,
        RELEASE
    }

    private RobotState currentState = RobotState.INIT;
    private boolean clawState = false;
    private boolean previousA = false;
    private boolean railUp = false;
    private boolean previousB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        DcMotor arm = robot.motorarm;
        Servo claw = robot.servoclaw;
        DcMotor rail = robot.motorlinearRail;

        rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // --- CONTROLS ---
            // A = toggle claw
            // B = toggle linear rail
            // DOWN-DPAD = lower linear rail
            // UP-DPAD = raise linear rail

            // --- STATE CONTROLS ---
            // X = INIT state
            // Y = INTAKE state
            // L-BUMPER = BASKET state
            // R-BUMPER = RELEASE state

            if (gamepad1.x) {
                currentState = RobotState.INIT;
            } else if (gamepad1.y) {
                currentState = RobotState.INTAKE;
            } else if (gamepad1.left_bumper) {
                currentState = RobotState.BASKET;
            } else if (gamepad1.right_bumper) {
                currentState = RobotState.RELEASE;
            }

            // --- CLAW TOGGLE ---
            if (gamepad1.a && !previousA) {
                clawState = !clawState;
                claw.setPosition(clawState ? RobotConfig.CLAW_CLOSE : RobotConfig.CLAW_OPEN);
            }
            previousA = gamepad1.a;

            // --- RAIL TOGGLE ---
            if (gamepad1.b && !previousB) {
                railUp = !railUp;

                rail.setTargetPosition((int) (railUp ? RobotConfig.RAIL_POSITION_UP : RobotConfig.RAIL_POSITION_DOWN));
                rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rail.setPower(RobotConfig.RAIL_POWER);
            }
            previousB = gamepad1.b;

            if (gamepad1.dpad_down) {
                rail.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rail.setPower(RobotConfig.LINEAR_SLIDE_DOWN_POWER);
            } else if (gamepad1.dpad_up) {
                rail.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rail.setPower(RobotConfig.LINEAR_SLIDE_UP_POWER);
            }

            // --- ARM POSITION SETTING ---
            int armTarget = 0;
            switch (currentState) {
                case INIT:
                    armTarget = RobotConfig.ARM_POSITION_INIT;
                    rail.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rail.setPower(RobotConfig.LINEAR_SLIDE_DOWN_POWER);
                    break;
                case INTAKE:
                    armTarget = RobotConfig.ARM_POSITION_INTAKE;
                    break;
                case BASKET:
                    armTarget = RobotConfig.ARM_POSITION_BASKET;
                    break;
                case RELEASE:
                    armTarget = RobotConfig.ARM_POSITION_RELEASE;
                    break;
            }

            arm.setTargetPosition(armTarget);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);

            // --- DRIVE CONTROL ---
            double horizontal = -gamepad1.right_stick_x * 0.6;
            double vertical = gamepad1.right_stick_y * 0.6;
            double turn = -gamepad1.left_stick_x * 0.6;

            double flPower = vertical + turn + horizontal;
            double frPower = vertical - turn - horizontal;
            double blPower = vertical + turn - horizontal;
            double brPower = vertical - turn + horizontal;

            double max = Math.max(1.0, Math.max(Math.abs(flPower),
                    Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower)))));

            robot.setDrivePower(flPower / max, frPower / max, blPower / max, brPower / max);

            // --- TELEMETRY ---
            telemetry.addData("Claw", clawState ? "Closed" : "Open");
            telemetry.addData("Arm Target", armTarget);
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Rail Direction", railUp ? "Up" : "Down");
            telemetry.update();
        }
    }
}
