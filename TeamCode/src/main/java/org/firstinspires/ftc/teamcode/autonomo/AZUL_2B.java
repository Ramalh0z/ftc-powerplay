package org.firstinspires.ftc.teamcode.autonomo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.AutonomousModule;
import org.firstinspires.ftc.teamcode.robots.MecanumRobot;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

@Autonomous(name = "AZUL_2B", group = "Autonomo")
public class AZUL_2B extends LinearOpMode {
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

        // rotaciona pra direita, ficando de frente pro pêndulo
        autonomousModule.MoverDireita(robot.motores, 0.5);
        sleep(400);
        autonomousModule.ZerarMotores(robot.motores);

        // levanta o braço de antemão
        double controlPower = MathUtils.Lerp(0.0, 1.0, 100);
        autonomousModule.MoverBraço(robot.atuadores, controlPower);

        // vá em direção ao pêndulo
        autonomousModule.MoverFrente(robot.motores,0.5);
        sleep(1000);
        autonomousModule.ZerarMotores(robot.motores);

        // *pow* (som do bloco caindo?)
        autonomousModule.SoltarObjeto(robot.atuadores[2], 0.5);
        sleep(500);
        robot.atuadores[2].setPower(0.0);

        // volta bem devagarin
        autonomousModule.MoverTras(robot.motores, 0.5);
        sleep(1400);
        autonomousModule.ZerarMotores(robot.motores);

        // se alinha com a parede
        autonomousModule.MoverEsquerda(robot.motores, 0.6);
        sleep(500);
        autonomousModule.ZerarMotores(robot.motores);

        // encosta mesmo na parede
        autonomousModule.MoverTras(robot.motores, 0.5);
        sleep(800);
        autonomousModule.ZerarMotores(robot.motores);

        // entra no armazém
        autonomousModule.RotacionarEsquerda(robot.motores, 1.0);
        sleep(1500);
        autonomousModule.ZerarMotores(robot.motores);

        // fica um pouquinho pra frente, por via das dúvidas
        autonomousModule.MoverFrente(robot.motores,1.0);
        sleep(500);
        autonomousModule.ZerarMotores(robot.motores);
    }
}