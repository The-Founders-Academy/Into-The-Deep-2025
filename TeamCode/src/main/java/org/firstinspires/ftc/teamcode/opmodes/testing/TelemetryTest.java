package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mecanum.BaseMecanumDrive;

@TeleOp
public class TelemetryTest extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        double left_x = gamepad1.left_stick_x;
        double left_y = -gamepad1.left_stick_y;     // inverting values to make positive up
        double left_x_reduced = gamepad1.left_stick_x / 3;
        double left_y_reduced = -gamepad1.left_stick_y / 3;

        double right_x = gamepad1.right_stick_x;
        double right_y = -gamepad1.right_stick_y;   // inverting values to make positive up
        double right_x_reduced = gamepad1.right_stick_x / 3;
        double right_y_reduced = -gamepad1.right_stick_y / 3;

        boolean right_bumper = gamepad1.right_bumper;


        telemetry.addData("Left X: ", left_x);
        telemetry.addData("Left Y: ", left_y);
        telemetry.addData("Left X Reduced: ", left_x_reduced);
        telemetry.addData("Left Y Reduced: ", left_y_reduced);
        telemetry.addData("Right X: ", right_x);
        telemetry.addData("Right Y: ", right_y);
        telemetry.addData("Right X Reduced: ", right_x_reduced);
        telemetry.addData("Right Y Reduced: ", right_y_reduced);
        telemetry.addData("Right Bumper pressed: ", right_bumper);

    }
}
