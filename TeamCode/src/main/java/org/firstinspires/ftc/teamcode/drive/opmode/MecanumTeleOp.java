package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {

        DcMotor Fl = hardwareMap.dcMotor.get("Fl");
        DcMotor Bl = hardwareMap.dcMotor.get("Bl");
        DcMotor Fr = hardwareMap.dcMotor.get("Fr");
        DcMotor Br = hardwareMap.dcMotor.get("Br");
        DcMotor Lr = hardwareMap.dcMotor.get("Lr");
        DcMotor Ll = hardwareMap.dcMotor.get("Ll");
        Servo Geoff = hardwareMap.servo.get("Geoff");

        //Fr.setDirection(DcMotorSimple.Direction.REVERSE);
        //Br.setDirection(DcMotorSimple.Direction.REVERSE);

        Lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double IntegralSum = 0;
        double kp = 0.01;
        double ki = 0.0001;
        double kd = 0.0085;

        double lastError = 0;

        double target = 0;

        double lastClick = 0;

        int cones = 1;

        double servoPos = 0.5;

        double EncoderSlidesFix = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double rx = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double Flp = (y - x + rx) / denominator;
            double Blp = (y + x + rx) / denominator;
            double Frp = (y - x - rx) / denominator;
            double Brp = (y + x - rx) / denominator;

            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                Fr.setPower(Frp * 0.25);
                Fl.setPower(Flp * 0.25);
                Br.setPower(Brp * 0.25);
                Bl.setPower(Blp * 0.25);
            } else if (gamepad1.right_bumper) {
                Fr.setPower(Frp * 0.5);
                Fl.setPower(Flp * 0.5);
                Br.setPower(Brp * 0.5);
                Bl.setPower(Blp * 0.5);
            } else {
                Fr.setPower(Frp);
                Fl.setPower(Flp);
                Br.setPower(Brp);
                Bl.setPower(Blp);
            }


            if (gamepad2.b) {
                target = 2100 + EncoderSlidesFix;
            } else if (gamepad2.x) {
                target = 1300 + EncoderSlidesFix;
            } else if (gamepad2.y) {
                target = 2950 + EncoderSlidesFix;
            } else if (gamepad2.a) {
                target = (cones * 100);
            }
            double error = target - Ll.getCurrentPosition();

            IntegralSum += error;

            if (error < 1) {
                IntegralSum = 0;
            }
            double derivative = (error - lastError);
            lastError = error;

            double Power = (error * kp) + (IntegralSum * ki) + (derivative * kd);

            if (Power > 1) {
                Power = 1;
            } else if (Power < -1) {
                Power = -1;
            }

            if (gamepad2.dpad_up && lastClick == 0) {
                cones += 1;
                lastClick = 1;
            } else if (gamepad2.dpad_down && lastClick == 0) {
                cones -= 1;
                lastClick = 1;
            } else if(gamepad2.dpad_right && lastClick == 0) {
                EncoderSlidesFix += 100;
                lastClick = 1;
            } else if(gamepad2.dpad_left && lastClick == 0) {
                EncoderSlidesFix -= 100;
                lastClick = 1;
            }else if (gamepad2.right_bumper && lastClick == 0){
                if (servoPos == 0.5) {
                    servoPos = 0.8;
                } else {
                    servoPos = 0.5;
                }
                lastClick = 1;
            }else if(!(gamepad2.dpad_up || gamepad2.dpad_down|| gamepad2.dpad_left || gamepad2.dpad_right|| gamepad2.right_bumper)){
                lastClick = 0;
            }

            Geoff.setPosition(servoPos);

            telemetry.addLine("Encoder Fix: " + EncoderSlidesFix);
            telemetry.addLine("Cones: " + cones);
            telemetry.addLine("Position: " + Ll.getCurrentPosition());
            telemetry.addLine("Power: " + Power);
            telemetry.addLine("Target:" + target);
            telemetry.addLine("servo: " + Geoff.getPosition());
            telemetry.update();

            Ll.setPower(Power);
            Lr.setPower(-Power);

        }

    }
}

