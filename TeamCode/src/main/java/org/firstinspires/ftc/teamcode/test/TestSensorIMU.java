package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private BNO055IMU sensorIMU;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DestemidosHardware(hardwareMap);
        sensorIMU = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // configurando o sensorIMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "Sensor IMU: ";

        // carragando tudo isso
        sensorIMU.initialize(parameters);

        waitForStart();
        while(opModeIsActive()) {
            MovementSystem.controleOmnidirecionalClassico(gamepad1, robot);

            telemetry.addData("Sensor IMU - AngularOrientation: ", sensorIMU.getAngularOrientation());
            telemetry.addData("Sensor IMU - AngularVelocity: ", sensorIMU.getAngularVelocity());
            telemetry.addData("Sensor IMU - Acceletarion: ", sensorIMU.getAcceleration());
            telemetry.addData("Sensor IMU - Position: ", sensorIMU.getPosition());
        }
    }
}
