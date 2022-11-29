package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
public class TestSensorIMU extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DestemidosHardware robot = new DestemidosHardware(hardwareMap);
        BNO055IMU sensorIMU = hardwareMap.get(BNO055IMU.class, "imu");

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
            // utilizamos o controle padrão de jogo sem a garra
            MovementSystem.controleOmnidirecionalClassico(gamepad1, robot);

            // enviando tudo para a telemetria
            telemetry.addData("Sensor IMU Status: ", sensorIMU.getSystemStatus());
            telemetry.addData("Sensor IMU - AngularOrientation: ", sensorIMU.getAngularOrientation());
            telemetry.addData("Sensor IMU - AngularVelocity: ", sensorIMU.getAngularVelocity());
            telemetry.addData("Sensor IMU - Acceletarion: ", sensorIMU.getAcceleration());
            telemetry.addData("Sensor IMU - Position: ", sensorIMU.getPosition());
        }
    }
}
