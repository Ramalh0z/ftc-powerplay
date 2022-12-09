package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotConfiguration;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

/*
 * MovementSystem - O Módulo mais antigo que temos,
 * que é responsável por agrupar os algoritmos de movimentação
 * mais avançados que temos e alguns usamos desde a temporada
 * ULTIMATE GOAL 2020/2021
 * 
 * TODO (ramalho): estudar mais o uso das funções relacionadas a
 * velocidade do robô própriamente dita, para não ficarmos 
 * dependendo da bateria constantemente
 */

public final class MovementSystem {

    public static void controleTank(Gamepad driver, DestemidosHardware robot) {
        double joystick_y  = -driver.left_stick_y;
        double giro        = -driver.right_stick_x;

        double denominador = Math.max( Math.abs(joystick_y) + Math.abs(giro), 1);

        double direitaFrentePower   = (joystick_y - giro) / denominador;
        double direitaTrasPower     = (joystick_y - giro) / denominador;
        double esquerdaFrentePower  = (joystick_y + giro) / denominador;
        double esquerdaTrasPower    = (joystick_y + giro) / denominador;

        robot.motorDireitaFrente.setPower(direitaFrentePower);
        robot.motorDireitaTras.setPower(direitaTrasPower);
        robot.motorEsquerdaFrente.setPower(esquerdaFrentePower);
        robot.motorEsquerdaTras.setPower(esquerdaTrasPower);
    }
    // Movimentação padrão e mais refinada do nosso controle, baseada no vídeo
    // extremamente didático do Gavin Ford: https://youtu.be/gnSW2QpkGXQ
    public static void controleOmnidirecionalClassico(Gamepad driver, DestemidosHardware robot)
    {
        double joystick_y  = -driver.left_stick_y  * RobotConfiguration.usoDasRodas;
        double joystick_x  = -driver.left_stick_x  * RobotConfiguration.usoDasRodas; //* RobotConstants.kCorretorJoystickX;
        double giro        = -driver.right_stick_x * RobotConfiguration.usoDasRodas;

        // o denominador sempre será a maior força (valor absoluto) entre os 4 motores, ou equivalente a 1.
        // isso permite que todos mantenham a mesma taxa, mesmo que um motor ultrapasse os limites [-1, 1]
        double denominador = Math.max( Math.abs(joystick_y) + Math.abs(joystick_x) + Math.abs(giro), 1);

        double direitaFrentePower   = (joystick_y - joystick_x - giro) / denominador;
        double direitaTrasPower     = (joystick_y + joystick_x - giro) / denominador;
        double esquerdaFrentePower  = (joystick_y + joystick_x + giro) / denominador;
        double esquerdaTrasPower    = (joystick_y - joystick_x + giro) / denominador;

        robot.motorDireitaFrente.setPower(direitaFrentePower);
        robot.motorDireitaTras.setPower(direitaTrasPower);
        robot.motorEsquerdaFrente.setPower(esquerdaFrentePower);
        robot.motorEsquerdaTras.setPower(esquerdaTrasPower);
    }

    // Controle possivelmente mais preciso, baseado no método 2 do youtuber Gavin Ford
    public static void controleMecanumAvançado(double theta, double direction, double turn, DestemidosHardware robot) {

        double seno         = Math.sin(theta - RobotConstants.kMecanumWheelsAngle);
        double cosseno      = Math.cos(theta - RobotConstants.kMecanumWheelsAngle);

        // neste caso, o denominador vai "normalizar" os valores de cada eixo
        double denominador  = Math.max( Math.abs(seno), Math.abs(cosseno) );

        double direitaFrentePower   = (seno / denominador) - turn;
        double direitaTrasPower     = (cosseno / denominador) - turn;
        double esquerdaFrentePower  = (cosseno / denominador) + turn;
        double esquerdaTrasPower    = (seno / denominador) + turn;

        if(direction + Math.abs(turn) > 1) {
            direitaFrentePower  /= direction + Math.abs(turn);
            direitaTrasPower    /= direction + Math.abs(turn);
            esquerdaFrentePower /= direction + Math.abs(turn);
            esquerdaTrasPower   /= direction + Math.abs(turn);
        }

        robot.motorDireitaFrente.setPower(direction * direitaFrentePower);
        robot.motorDireitaTras.setPower(direction * direitaTrasPower);
        robot.motorEsquerdaFrente.setPower(direction * esquerdaFrentePower);
        robot.motorEsquerdaTras.setPower(direction * esquerdaTrasPower);
    }
    public static void controleMecanumAlternativo(Gamepad driver, DestemidosHardware robot) {
        double direction = Math.hypot(driver.left_stick_x, driver.left_stick_y);
        double robotAngle = Math.atan2(driver.left_stick_y, driver.left_stick_x) - Math.PI / 4;
        double rightX = driver.right_stick_x;

        final double v1 = direction * Math.cos(robotAngle) + rightX;
        final double v2 = direction * Math.sin(robotAngle) - rightX;
        final double v3 = direction * Math.sin(robotAngle) + rightX;
        final double v4 = direction * Math.cos(robotAngle) - rightX;

        robot.motorEsquerdaFrente.setPower(v1);
        robot.motorDireitaFrente.setPower(v2);
        robot.motorEsquerdaTras.setPower(v3);
        robot.motorDireitaTras.setPower(v4);
    }

    // um controle que sempre direciona o robô para onde apontamos no joystick, independente
    // da orentação do robô na arena
    public static void controleFieldOriented(Gamepad driver, DestemidosHardware robot) {
        double joystick_y  = -driver.left_stick_y  * RobotConfiguration.usoDasRodas;
        double joystick_x  = -driver.left_stick_x  * RobotConfiguration.usoDasRodas; //* RobotConstants.kCorretorJoystickX;
        double giro        = -driver.right_stick_x * RobotConfiguration.usoDasRodas;

        double botHeading = robot.sennnsorIMU.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        ).firstAngle;

        double rotationX = joystick_x * Math.cos(botHeading) - joystick_y * Math.sin(botHeading);
        double rotationY = joystick_x * Math.sin(botHeading) + joystick_y * Math.cos(botHeading);

        double denominador = Math.max( Math.abs(joystick_y) + Math.abs(joystick_x) + Math.abs(giro), 1);

        double direitaFrentePower   = (rotationY - rotationX - giro) / denominador;
        double direitaTrasPower     = (rotationY + rotationX - giro) / denominador;
        double esquerdaFrentePower  = (rotationY + rotationX + giro) / denominador;
        double esquerdaTrasPower    = (rotationY - rotationX + giro) / denominador;

        robot.motorDireitaFrente.setPower(direitaFrentePower);
        robot.motorDireitaTras.setPower(direitaTrasPower);
        robot.motorEsquerdaFrente.setPower(esquerdaFrentePower);
        robot.motorEsquerdaTras.setPower(esquerdaTrasPower);
    }

}