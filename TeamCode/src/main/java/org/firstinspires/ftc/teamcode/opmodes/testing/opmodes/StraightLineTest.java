package org.firstinspires.ftc.teamcode.opmodes.testing.opmodes;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class StraightLineTest extends OpMode {
    MotorEx fL;
    MotorEx fR;
    MotorEx bL;
    MotorEx bR;

    @Override
    public void init() {
        fL = new MotorEx(hardwareMap, "fL");
        fR = new MotorEx(hardwareMap, "fR");
        bL = new MotorEx(hardwareMap, "bL");
        bR = new MotorEx(hardwareMap, "bR");
    }

    @Override
    public void loop() {                        // Left motors are inverted as they are flipped on robot
        if(gamepad1.y) {            // forward
            fL.set(-1);
            fR.set(1);
            bL.set(-1);
            bR.set(1);
        } else if(gamepad1.a) {     // backward
            fL.set(1);
            fR.set(-1);
            bL.set(1);
            bR.set(-1);

        } else if(gamepad1.x) {     // left
            fL.set(1);
            fR.set(1);
            bL.set(-1);
            bR.set(-1);

        } else if(gamepad1.b) {     // right
            fL.set(-1);
            fR.set(-1);
            bL.set(1);
            bR.set(1);

        } else {                   // stop
            fL.stopMotor();
            fR.stopMotor();
            bL.stopMotor();
            bR.stopMotor();
        }
    }
}
