package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Mecanum2025 extends BaseMecanumDrive {
    @Config
    public static class Mecanum2025Params {     // FTC Dashboard
        public static double TrackWidthCentimeters = 36;  // distance between odo pods
        public static double DeadWheelRadiusCentimeters = 2.4;
        public static double DeadWheelEncoderRes = 2000.0; // resolution of deadwheels
        public static double PerpendicularOffsetCentimeters = -20.32; // distance of third pod from center of bot

        public static PIDCoefficients TranslationX = new PIDCoefficients(0,0,0);     // D should always be zero
        public static PIDCoefficients TranslationY = new PIDCoefficients(0,0,0);
        public static PIDCoefficients TranslationRotation = new PIDCoefficients(0,0,0);
    }


    private HolonomicOdometry m_odometry;
    private Pose2d m_robotPose;     // absolute coordinates and rotation
    private Encoder m_left;
    private Encoder m_right;
    private Encoder m_perpendicular;
    private double m_initialangledegrees;
    private double encoderRadiusCentimeters = 2.4;
    private double ticksPerRevolution = 2000.0;

    public Mecanum2025(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose, Alliance alliance) {
        super(hardwareMap, mecanumConfigs, initialPose, alliance);

        m_odometry = new HolonomicOdometry(
                m_left::getDistance,        // calls functions to get distance
                m_right::getDistance,
                m_perpendicular::getDistance,
                Mecanum2025Params.TrackWidthCentimeters,
                Mecanum2025Params.PerpendicularOffsetCentimeters

        );
        Pose2d Pose2dadjustedinitialpose = new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(0));

        m_odometry.updatePose(Pose2dadjustedinitialpose);   // gets position but not rotation
        m_robotPose = initialPose;
        m_initialangledegrees = initialPose.getRotation().getDegrees();

        double cm_per_tick = 2 * Math.PI * encoderRadiusCentimeters / ticksPerRevolution;
        m_left = m_frontRight.encoder.setDistancePerPulse(cm_per_tick);
        m_left.setDirection(MotorEx.Direction.REVERSE);
        m_right = m_frontLeft.encoder.setDistancePerPulse(cm_per_tick);
        m_right.setDirection(MotorEx.Direction.REVERSE);
        m_perpendicular = m_backLeft.encoder.setDistancePerPulse(cm_per_tick);
    }


    @Override
    public Rotation2d getHeading() {
        return m_odometry.getPose().getRotation().times(-1); // Converting clockwise rotation to counterclockwise
    }

    @Override
    public Pose2d getPose() {
        return m_robotPose;
    }

    @Override
    public void resetPose(Pose2d pose) {
        m_robotPose = pose;
    }

    public void setTargetPose(Pose2d pose2d) {              // getting target x and y coords + rotation based on function input
        m_translationXController.setSetPoint(pose2d.getX());
        m_translationYController.setSetPoint(pose2d.getY());
        m_rotationController.setSetPoint(pose2d.getRotation().getDegrees());
    }

    @Override
    public boolean atTargetPose() {

        boolean atTarget = m_translationXController.atSetPoint() && m_translationYController.atSetPoint();
        boolean atRotation = m_rotationController.atSetPoint();

        return atTarget && atRotation;
    }


    public void moveWithPID() {
        double vX = MathUtil.clamp(m_translationXController.calculate(m_robotPose.getX()),
                -m_mecanumConfigs.getMaxRobotSpeedMps(),
                m_mecanumConfigs.getMaxRobotSpeedMps());
        double vY = MathUtil.clamp(m_translationYController.calculate(m_robotPose.getY()),
                -m_mecanumConfigs.getMaxRobotSpeedMps(),
                m_mecanumConfigs.getMaxRobotSpeedMps());

        // Do some angle wrapping to ensure the shortest path is taken to get to the rotation target
        double normalizedRotationRad = m_robotPose.getHeading();
        if(normalizedRotationRad < 0) {
            normalizedRotationRad = m_robotPose.getHeading() + 2 * Math.PI; // Normalize to [0, 2PI]
        }

        double vOmega = MathUtil.clamp(m_rotationController.calculate(normalizedRotationRad),
                -m_mecanumConfigs.getMaxRobotRotationRps(),
                m_mecanumConfigs.getMaxRobotRotationRps());

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vY, -vX, vOmega, getHeading()); // Transform the x and y coordinates to account for differences between global field coordinates and driver field coordinates
        move(speeds);
    }


    public void resetPIDs() {
        m_translationXController.clearTotalError();
        m_translationYController.clearTotalError();
        m_rotationController.clearTotalError();
    }


    public void tunePIDs() {
        m_translationXController.setPID(Mecanum2025Params.TranslationX.p, Mecanum2025Params.TranslationX.i, Mecanum2025Params.TranslationX.d);
        m_translationYController.setPID(Mecanum2025Params.TranslationY.p, Mecanum2025Params.TranslationY.i, Mecanum2025Params.TranslationY.d);
        m_rotationController.setPID(Mecanum2025Params.TranslationRotation.p, Mecanum2025Params.TranslationRotation.i, Mecanum2025Params.TranslationRotation.d);
    }

    @Override
    public void periodic() {
        tunePIDs();
        m_odometry.updatePose();
        double currentAngle = m_initialangledegrees - m_odometry.getPose().getRotation().getDegrees();              // initial degrees minus minus clockwise cords
        m_robotPose = new Pose2d(m_odometry.getPose().getTranslation(), Rotation2d.fromDegrees(currentAngle));      // updating robot pose

        TelemetryPacket driveInformation = new TelemetryPacket();
        driveInformation.put("Left Encoder", m_left.getDistance());
        driveInformation.put("Left Encoder", m_right.getDistance());
        driveInformation.put("Left Encoder", m_perpendicular.getDistance());
        //TODO GO TO FTC DASHBOARD TO VIEW
        FtcDashboard.getInstance().sendTelemetryPacket(driveInformation);
    }
}
