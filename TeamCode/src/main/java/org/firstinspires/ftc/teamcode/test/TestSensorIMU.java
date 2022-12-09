package org.firstinspires.ftc.teamcode.test;

import android.graphics.drawable.GradientDrawable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.subsystems.MovementSystem;

/*
 * TestSensorIMU - OpMode dedicado a leitura de informações
 * detalhadas como: Posição, Aceleração, Velocidade Angular, Orientação 
 * e etc.. todas providas pelo sensor embutido no próprio controlhub
 * 
 * para outros exemplos: https://stemrobotics.cs.pdx.edu/node/7265.html 
 */

@TeleOp(name = "TestSensorIMU", group = "Test")
@Disabled
public class TestSensorIMU extends LinearOpMode {
    private DestemidosHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DestemidosHardware(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while(opModeIsActive()) {
            MovementSystem.controleOmnidirecionalClassico(gamepad1, robot);

            // pegando informações do imu
            Orientation robotOrientation = robot.sensorIMU.getRobotOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
            );

            AngularVelocity robotAngVel = robot.sensorIMU.getRobotAngularVelocity(AngleUnit.DEGREES);

            YawPitchRollAngles robotAngles = robot.sensorIMU.getRobotYawPitchRollAngles();

            telemetry.addData("Sensor IMU - Velocidade Angular: ", robotAngVel);
            telemetry.addData("Sensor IMU - Orientação em Graus: ", "X: %d / Y: %d / Z: %d",
                    robotOrientation.firstAngle,
                    robotOrientation.secondAngle,
                    robotOrientation.thirdAngle
            );
            telemetry.addData("Sensor IMU - Ângulo das Rotações: ", "Yaw: %d / Pitch: %d / Row: %d",
                    robotAngles.getYaw(AngleUnit.DEGREES),
                    robotAngles.getPitch(AngleUnit.DEGREES),
                    robotAngles.getRoll(AngleUnit.DEGREES)
            );
        }
    }
}
