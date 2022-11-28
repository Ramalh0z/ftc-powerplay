package org.firstinspires.ftc.teamcode.modules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.MathUtils;

/*
 * MovementModule - O Módulo mais antigo que temos, 
 * que é responsável por agrupar os algoritmos de movimentação
 * mais avançados que temos e alguns usamos desde a temporada
 * ULTIMATE GOAL 2020/2021
 * 
 * TODO (ramalho): estudar mais o uso das funções relacionadas a
 * velocidade do robô própriamente dita, para não ficarmos 
 * dependendo da bateria constantemente
 */

public final class MovementModule {
    public static final double angulo45 = Math.PI / 4;

    // Controle possivelmente mais preciso, e possivel sucessor do esquema original
    // do controlw tank original
    public static void controleMecanumAvançado(Gamepad driver, DcMotorEx[] motores) {
        double joystick_y  = -driver.left_stick_y;
        double joystick_x  = driver.left_stick_x; // imperfeição do movimento lateral
        double giro        = driver.right_stick_x;

        // o ângulo e o módulo do vetor gerados com o joystick
        double theta = Math.atan2(joystick_y, joystick_x);
        double direction = Math.hypot(joystick_x, joystick_y);

        double seno         = Math.sin(theta - angulo45);
        double cosseno      = Math.cos(theta - angulo45);

        // neste caso, o denominador vai "normalizar" os valores de cada eixo
        double denominador  = Math.max( Math.abs(seno), Math.abs(cosseno) );

        motores[0].setPower(direction + (cosseno/denominador) + giro); // Direita-Frente
        motores[1].setPower(direction + (seno/denominador) - giro);    // Direita-Trás
        motores[2].setPower(direction + (seno/denominador) + giro);    // Esquerda-Frente
        motores[3].setPower(direction + (cosseno/denominador) - giro); // Esquerda-Trás

        /*
        // caso o giro ultrapasse o limite padrão
        if( (direction + giro) > 1)
        {
            // (ramalho): esse é o jeito mais simples e prático que consegui fazer
            // precisamos deixar mais eficiente
            double old_m0 = motores[0].getPower();
            double old_m1 = motores[1].getPower();
            double old_m2 = motores[2].getPower();
            double old_m3 = motores[3].getPower();

            motores[0].setPower(old_m0 / (direction + giro)); // Direita-Frente
            motores[1].setPower(old_m1 / (direction + giro)); // Direita-Trás
            motores[2].setPower(old_m2 / (direction + giro)); // Esquerda-Frente
            motores[3].setPower(old_m3 / (direction + giro)); // Esquerda-Trás
        }
        */
    }

    // Movimentação padrão e mais refinada do nosso controle, baseada no vídeo
    // extremamente didático do Gavin Ford: https://youtu.be/gnSW2QpkGXQ 
    public static void controleOmnidirecional(Gamepad driver, @NonNull DcMotorEx[] motores, double escalaDeForça)
    {
        double joystick_y  = driver.left_stick_y * escalaDeForça;
        double joystick_x  = -driver.left_stick_x * escalaDeForça;
        double giro        = -driver.right_stick_x * escalaDeForça;

        // o denominador sempre será a maior força (valor absoluto) entre os 4 motores, ou equivalente a 1.
        // isso permite que todos mantenham a mesma taxa, mesmo que um motor ultrapasse os limites [-1, 1]
        double denominador = Math.max( Math.abs(joystick_y) + Math.abs(joystick_x) + Math.abs(giro), 1);

        motores[0].setPower( (joystick_y - joystick_x - giro) / denominador);    // Direita-Frente
        motores[2].setPower( (joystick_y + joystick_x + giro) / denominador);    // Esquerda-Frente
        motores[1].setPower( (joystick_y + joystick_x - giro) / denominador);    // Direita-Trás
        motores[3].setPower( (joystick_y - joystick_x + giro) / denominador);    // Esquerda-Trás
    }

    // Moviemntação idêntica a Omnidireiconal tradicional, porém utilizando do "setVelocity()"
    public static void controleVelocidadeOmnidirecional(Gamepad driver, @NonNull DcMotorEx[] motores, double escalaVelocidade)
    {
        //OBS: o valor da escala é estipulado pelo o que observamos nos encoders do FTC_TESTBOT;
        // a escala deve ser na casa dos 1000 ~= 1200
        double joystick_y = -driver.left_stick_y  * escalaVelocidade;
        double joystick_x = driver.left_stick_x   * escalaVelocidade;
        double giro       = driver.right_stick_x  * escalaVelocidade;

        double denominador = Math.max( Math.abs(joystick_y) + Math.abs(joystick_x) + Math.abs(giro), 1);

        motores[0].setVelocity( (joystick_y - joystick_x - giro) / denominador); // Direita-Frente
        motores[1].setVelocity( (joystick_y + joystick_x - giro) / denominador); // Direita-Trás
        motores[2].setVelocity( (joystick_y + joystick_x + giro) / denominador); // Esquerda-Frente
        motores[3].setVelocity( (joystick_y - joystick_x + giro) / denominador); // Esquerda-Trás
    }
}