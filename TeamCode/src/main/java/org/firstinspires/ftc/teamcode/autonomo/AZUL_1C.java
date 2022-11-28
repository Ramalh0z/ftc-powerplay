package org.firstinspires.ftc.teamcode.autonomo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.AutonomousModule;
import org.firstinspires.ftc.teamcode.robots.MecanumRobot;

@Autonomous(name = "AZUL_1C", group = "Autonomo")
public class AZUL_1C extends LinearOpMode {
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

        // encosta no carrosel
        autonomousModule.RotacionarDireita(robot.motores, 0.5);
        sleep(800);
        autonomousModule.ZerarMotores(robot.motores);

        // gira os patos
        autonomousModule.LigarMotorPato(robot.atuadores[3]);
        sleep(2400);
        robot.atuadores[3].setPower(0.0);

        // vira pra ficar de frente no pêndulo
        autonomousModule.RotacionarEsquerda(robot.motores, 0.5);
        sleep(800);
        autonomousModule.ZerarMotores(robot.motores);

        // vá entregar o elemento de jogo no pêndulo
        autonomousModule.MoverBraço(robot.atuadores, 0.3);

        // agora move em direção ao pêndulo
        autonomousModule.MoverFrente(robot.motores, 0.5);
        sleep(2000);
        autonomousModule.ZerarMotores(robot.motores);

        // *pow* (som do bloco caindo?)
        autonomousModule.SoltarObjeto(robot.atuadores[2], 0.5);
        sleep(800);
        robot.atuadores[2].setPower(0.0);

        // abaixa o braço
        autonomousModule.MoverBraço(robot.atuadores, 0.0);

        // sai de perto do pêndulo
        autonomousModule.MoverTras(robot.motores, 0.5);
        sleep(200);
        autonomousModule.ZerarMotores(robot.motores);

        // depois de entregar, vai de lado pra parede
        autonomousModule.RotacionarDireita(robot.motores, 0.5);
        sleep(1000);
        autonomousModule.ZerarMotores(robot.motores);

        // finaliza no estacionamento
        autonomousModule.MoverFrente(robot.motores, 0.5);
        sleep(800);
        autonomousModule.ZerarMotores(robot.motores);
    }
}