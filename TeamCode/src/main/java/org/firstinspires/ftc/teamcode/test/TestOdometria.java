package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.modules.OdometryModule;
import org.firstinspires.ftc.teamcode.robots.MecanumRobot;
import org.firstinspires.ftc.teamcode.utils.Pose2D;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

/* (ramalho):
 * TestOdometria - Este OpMode tem como intuito, ser mais uma "prova de conceito"
 * do que ser algo realmente utilizavél para as competições da FTC
 * 
 * Espero que possamos levar para frente esse sistema em algum momento, mas de
 * acordo com a nova temporada (POWER PLAY - 2022/2023), temos prioridades mais
 * importantes do que esse pequeno exemplo
 */

@Autonomous(name = "TestOdometria", group = "Test")
public class TestOdometria extends LinearOpMode {

    // Componentes do robô
    private final MecanumRobot robot = new MecanumRobot();
    private BNO055IMU sensorIMU;

    // Módulos
    private final OdometryModule odometry = new OdometryModule();

    // um verdadeiro quebra-galho, mas com o uso apenas interno
    private void ZerarMotores()
    {
        robot.motores[0].setPower(0.0);
        robot.motores[1].setPower(0.0);
        robot.motores[2].setPower(0.0);
        robot.motores[3].setPower(0.0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // setup padrão do robô
        robot.inicializarHardware(hardwareMap);
        robot.inicializarExtraHardware(hardwareMap);
        sensorIMU = hardwareMap.get(BNO055IMU.class, "imu");

        // direção e encoders alterados
        robot.motores[0].setDirection(DcMotorEx.Direction.REVERSE);
        robot.motores[1].setDirection(DcMotorEx.Direction.REVERSE);
        robot.configEncoders(DcMotor.RunMode.RUN_USING_ENCODER, robot.motores);

        // configurando o sensorIMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "Sensor IMU: ";

        // carrega tudo
        sensorIMU.initialize(parameters);
        telemetry.addData("Sensor IMU Status: ", "Inicializado com Sucesso!");
        
        // reset necessário para toda vez que formos iniciar um round
        robot.resetRodasEncoder();

        waitForStart();
        while (opModeIsActive())
        {
            // informa a posição do robô
            Position currentPosition = sensorIMU.getPosition();
            Orientation currentOrientation = sensorIMU.getAngularOrientation();
            telemetry.addData("Sensor IMU - Posição: ",
                    "X: %.2f, Y: %.2f Theta: &.2f", currentPosition.x, currentPosition.y,currentOrientation.firstAngle);

            RobotLogger.debugMotorPosition(telemetry, robot.motores);
            RobotLogger.debugMotorPower(telemetry, robot.motores);
            telemetry.update();

            // define a posição inicial do robô
            Position inicio = sensorIMU.getPosition();
            Pose2D poseInicial = new Pose2D(inicio.x, inicio.y, 0.0);

            // posição 1: rotacionar para o estacionamnete
            Pose2D pose1 = new Pose2D(-10.0, 10.0, 180.0);

            /*
            * (ramalho): a idéia é transformar da posição "inicio" até a "pose1" em um intervalo curto de 5 segundos,
            * porém ele termina antes e "trava" na ultima pose até o fim dos tempo.
            *
            * Uma possivel melhor implementação, é atrvés de comandos que contenham esses laços e cada comando seria
            * resposnvél apenas por uma ação e por checkar se a trasição ocorreu corretamente
            */
            while ( opModeIsActive() && (robot.timer.seconds() < 5.0) ) 
            {
                odometry.odometria2(poseInicial, pose1, robot.motores);
            } // (ramalho): não tivemos mais tempo de testar, então ficamos só na imaginação :p

            // gire pra ficar de costas pra parede

            // talvez ir um pouco para o lado?

            // girar o pato

            // fim
            ZerarMotores();
        }
    }
}
