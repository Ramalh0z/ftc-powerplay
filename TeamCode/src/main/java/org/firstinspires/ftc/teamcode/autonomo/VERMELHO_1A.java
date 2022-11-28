package org.firstinspires.ftc.teamcode.autonomo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.AutonomousModule;
import org.firstinspires.ftc.teamcode.robots.MecanumRobot;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

@Autonomous(name = "(DE LADO) VERMELHO 1A", group = "Autonomo")
public class VERMELHO_1A extends LinearOpMode {
    // base padrão do robô
    private final MecanumRobot robot = new MecanumRobot();
    private final AutonomousModule autonomousModule = new AutonomousModule();

    @Override
    public void runOpMode() {
        // carregar todos os dispositivos
        robot.inicializarHardware(hardwareMap);
        robot.inicializarExtraHardware(hardwareMap);
        robot.resetRodasEncoder();
        robot.configEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER, robot.motores);

        // reseta o contador e espera o start
        robot.timer.reset();
        waitForStart();

        // OBS: esse opmode requer que o robô fique perpendicular pra parede (de frente pro armazém)

        // encosta no carrosel
        autonomousModule.MoverTras(robot.motores, 0.5);
        sleep(900);
        autonomousModule.ZerarMotores(robot.motores);

        // gira os patos
        autonomousModule.LigarMotorPato(robot.atuadores[3]);
        sleep(2400);
        robot.atuadores[3].setPower(0.0);

        // rotaciona em direção ao pêndulo especifico
        autonomousModule.MoverEsquerda(robot.motores, 0.5);
        sleep(300);
        autonomousModule.ZerarMotores(robot.motores);

        // vá entregar o elemento de jogo no pêndulo
        autonomousModule.MoverBraço(robot.atuadores, 1.0);

        // agora move em direção ao pêndulo
        autonomousModule.MoverFrente(robot.motores, 0.5);
        sleep(2000);
        autonomousModule.ZerarMotores(robot.motores);

        // *pow* (som do bloco caindo?)
        autonomousModule.SoltarObjeto(robot.atuadores[2], 0.5);
        sleep(800);
        robot.atuadores[2].setPower(0.0);

        // volta bem devagarin
        autonomousModule.MoverTras(robot.motores, 0.5);
        sleep(700);
        autonomousModule.ZerarMotores(robot.motores);

        // finaliza no estacionamento
        autonomousModule.RotacionarEsquerda(robot.motores, 1.0);
        sleep(600);
        autonomousModule.ZerarMotores(robot.motores);
    }
}