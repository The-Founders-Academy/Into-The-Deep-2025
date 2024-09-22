package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;

public class Mecanum2025 extends BaseMecanumDrive {
    @Config
    public static class Mecanum2025Params {     // FTC Dashboard
        public static double TrackWidthCentimeters = 36;  // distance between odo pods
        public static double DeadWheelRadiusCentimeters = 2.4;
        public static double DeadWheelEncoderRes = 2000.0; // resolution of deadwheels
        public static double PerpendicularOffsetCentimeters = -20.32; // distance of third pod from center of bot

    }


    private HolonomicOdometry m_odometry;
    private Pose2d m_robotPose;     // absolute coordinates and rotation
    private Encoder m_left;
    private Encoder m_right;
    private Encoder m_perpendicular;
    private double m_initialangledegrees;
    public Mecanum2025(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose, Alliance alliance) {
        super(hardwareMap, mecanumConfigs, initialPose, alliance);

        m_odometry = new HolonomicOdometry(
                m_left::getDistance,        // calls functions to get distance
                m_right::getDistance,
                m_perpendicular::getDistance,
                Mecanum2025Params.TrackWidthCentimeters,
                Mecanum2025Params.PerpendicularOffsetCentimeters

        );
        Pose2d Pose2dadjustedinitialpose = new Pose2d(initialPose.getX(), initialPose.getY(), 0);

        m_odometry.updatePose(Pose2dadjustedinitialpose);   // gets position but not rotation
        m_robotPose = initialPose;
        m_initialangledegrees = initialPose.getRotation().getDegrees();

        //  m_left = m_frontLeft.encoder;
        //TODO figure out what motor each encoder corresponds to
    }


    @Override
    public Rotation2d getHeading() {
        return null;
    }

    @Override
    public Pose2d getPose() {
        return m_robotPose; // define later
    }

    @Override
    public void resetPose(Pose2d pose) {
        m_robotPose = pose;

        //TODO set heading to zero
    }

    @Override
    public void setTargetPose(Pose2d pose2d) {

    }

    @Override
    public boolean atTargetPose() {
        return false;
    }

    @Override
    public void moveWithPID() {

    }

    @Override
    public void resetPIDs() {

    }

    @Override
    public void tunePIDs() {

    }

    @Override
    public void periodic() {
        m_odometry.updatePose();
        double currentAngle = m_initialangledegrees - m_odometry.getPose().getRotation().getDegrees();              // initial degrees minus minus clockwise cords
        m_robotPose = new Pose2d(m_odometry.getPose().getTranslation(), Rotation2d.fromDegrees(currentAngle));      // updating robot pose
    }
}
