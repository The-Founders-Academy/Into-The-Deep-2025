package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum2025;

public class RobotRelativeDrive extends CommandBase {
    private Mecanum2025 m_mecanum;
    private CommandGamepad m_driver;

    public RobotRelativeDrive(Mecanum2025 mecanum, CommandGamepad driver) {
        m_mecanum = mecanum;
        m_driver = driver;
        addRequirements(m_mecanum, m_driver);
    }

    @Override
    public void execute() {
        m_mecanum.moveRobotRelative(m_driver.getLeftY(), -m_driver.getLeftX(), -m_driver.getRightX());
    }

}

