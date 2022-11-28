package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.MovementModule;
import org.firstinspires.ftc.teamcode.robots.BaseRobot;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

/*
 * TestSensorIMU - OpMode dedicado a leitura de informações
 * detalhadas como: Posição, Aceleração, Velocidade Angular, Orientação 
 * e etc.. todas providas pelo sensor embutido no próprio controlhub
 * 
 * para outros exemplos: https://stemrobotics.cs.pdx.edu/node/7265.html 
 */

@TeleOp(name = "TestSensorIMU", group = "Test")
public class TestSensorIMU extends OpMode {
    
    // Componentes do robô
    private final BaseRobot robot = new BaseRobot();
    private BNO055IMU sensorIMU; // sensor giroscópio embutido no controlhub

    @Override
    public void init() {

        // setup padrão do robô
        robot.inicializarHardware(hardwareMap);
        robot.configEncoders(DcMotor.RunMode.RUN_USING_ENCODER, robot.motores);
        sensorIMU = hardwareMap.get(BNO055IMU.class, "imu");

        // configurando o sensorIMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "Sensor IMU: ";

        // carragando tudo isso
        sensorIMU.initialize(parameters);
    }

    @Override
    public void loop() {
        // utilizamos o controle padrão de jogo sem a garra 
        MovementModule.controleOmnidirecional(gamepad1, robot.motores, 0.80);

        // enviando tudo para a telemetria
        telemetry.addData("Sensor IMU Status: ", sensorIMU.getSystemStatus());
        telemetry.addData("Sensor IMU - AngularOrientation: ", sensorIMU.getAngularOrientation());
        telemetry.addData("Sensor IMU - AngularVelocity: ", sensorIMU.getAngularVelocity());
        telemetry.addData("Sensor IMU - Acceletarion: ", sensorIMU.getAcceleration());
        telemetry.addData("Sensor IMU - Position: ", sensorIMU.getPosition());

        // e também enviamos o log dos motores
        RobotLogger.debugMotorPower(telemetry, robot.motores);
    }
}
