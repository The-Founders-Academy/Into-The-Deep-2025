package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.commands.RobotRelativeDrive;
import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum2025;


    @TeleOp
    public class RobotDrive2025 extends CommandOpMode {

        private CommandGamepad m_driver;
        private Mecanum2025 m_mecanumDrive;

        @Override
        public void initialize() {

            MecanumConfigs configs = new MecanumConfigs().runMode(MotorEx.RunMode.RawPower);
            m_mecanumDrive = new Mecanum2025(hardwareMap, configs, new Pose2d(0, 0, Rotation2d.fromDegrees(90)), BaseMecanumDrive.Alliance.RED);
            m_driver = new CommandGamepad(gamepad1, 0.2, 0.2);
            m_mecanumDrive.setDefaultCommand(new RobotRelativeDrive(m_mecanumDrive, m_driver));
        }
    }


