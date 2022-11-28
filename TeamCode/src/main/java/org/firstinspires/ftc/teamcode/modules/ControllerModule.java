package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.MathUtils;

/*
 * Controller Module - Este módulo engloba todos os
 * mecanismos e lógica dos controles, que são independentes
 * do tipo de robô
 * 
 * TODO (ramalho): procurar uma solução melhor para a velocidade
 * da garra, ela engole bem mais forte do que lança
 */

public final class ControllerModule {

    // Método responsável por levantar a garra com velocidade e suavidade, 
    // sem gastar gastar muita corrente no processo
    public static void controlarARMs(Gamepad driver, DcMotorEx[] atuadores, double armPowerFactor) {
        
        // interpolamos o valor da corrente a ser aplicada, de acordo com 
        // os limites de 0.5 e 1.0, levando em conta a porcentagem da força (armPowerFactor)
        // que queremos usar dos 2 atuadores
        double controlPower = MathUtils.Lerp(0.5, 1.0, driver.left_stick_y * armPowerFactor);
        if (driver.left_stick_y > 0.0) {
            atuadores[0].setPower(-controlPower);
            atuadores[1].setPower(-controlPower);
        }
        atuadores[0].setPower(0);
        atuadores[1].setPower(0);
    }

    // Método responsável por controlar os movimentos simples da garra
    public static void controlarGarra(Gamepad driver, DcMotorEx Garra, double soltaPower, double puxaPower) {
        // button X - solta
        Garra.setPower(driver.x ? -soltaPower : 0.0);

        // button A - puxa
        Garra.setPower(driver.a ? puxaPower : 0.0);
    }

    // Método responsável por roatcioanr os patos de acordo com a força pressionada nos gatilhos
    public static void rotacionarPato(Gamepad driver, DcMotorEx Pato) {

        // gatilho direito
        if (driver.right_trigger > 0.05) {
            Pato.setPower(1.0);
        }
        Pato.setPower(0.0);
        
        // gatilho esquerdo
        if (driver.left_trigger > 0.05) {
            Pato.setPower(-1.0);
        }
        Pato.setPower(0.0);
    }
}