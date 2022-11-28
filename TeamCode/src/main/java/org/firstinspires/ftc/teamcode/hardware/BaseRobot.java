package org.firstinspires.ftc.teamcode.robots;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Base Robot - Abstração enxuta do que o robô deve
 * ter para ser no, mínimo, manuseavél em quaisquer
 * que seja o teste.
 */

public class BaseRobot {  
    /*
     * (ramalho): escolhi usar o array de motores pela 
     * praticidade de passar/configurar os motores como parâmetros
     * 
     * porém, o custo disso é que todo método que recebrá esses motores
     * deve seguir essa ordem abaixo:
     * 
     *      0 - Motor Direita-Frente
     *      1 - Motor Direita-Trás
     *      2 - Motor Esquerda-Frente
     *      3 - Motor Esquerda-Trás
     * 
     * isso deve reforçar um comportamento padrão em todos os lugares, e reduz
     * algumas preocupações quando fomos debuggar
     *
     * referência: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
     */
    public final DcMotorEx[] motores = new DcMotorEx[4];
    public final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // Método fundamental para o preparo inical dos motores com suas devidas configs
    // recebe o HardwareMap padrão do OpMode e já desativa os encoders por padrão
    public void inicializarHardware(HardwareMap hardwareMap)
    {
        // mapeando os motores com seus nomes e sequência correta
        motores[0] = hardwareMap.get(DcMotorEx.class,"DF"); // porta 0 - controlHub
        motores[1] = hardwareMap.get(DcMotorEx.class,"DT"); // porta 1 - controlHub
        motores[2] = hardwareMap.get(DcMotorEx.class,"EF"); // porta 2 - controlHub
        motores[3] = hardwareMap.get(DcMotorEx.class,"ET"); // porta 3 - controlHub

        // precisamos inverter apenas os motores da direita
        motores[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motores[1].setDirection(DcMotorSimple.Direction.REVERSE);

        // comportamento dos motores, quando a força for equivalente 0:
        configZeroPowerBehavior(motores);

        // ativando por padrão todos os encoders
        configEncoders(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, motores);
    }

    // Método que itera por todos os motores do array, e define a direção definida
    public void configDirection(DcMotorEx.Direction Direction, @NonNull DcMotorEx[] arrayDeMotores) {
        for(DcMotorEx motor : arrayDeMotores){
            motor.setDirection(Direction);
        }
    }

    // Método que itera por todos os motores do array, e configura o encoder para o modo definido
    public void configEncoders(DcMotorEx.RunMode runMode, @NonNull DcMotorEx[] arrayDeMotores) {
        for (DcMotorEx motor : arrayDeMotores) {
            motor.setMode(runMode);
        }
    }

    // Método que itera por todos os motores do array, e reseta os encoders 
    public void resetRodasEncoder(){
        for(DcMotorEx motor : motores)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    // Método que itera por todos os motores do array, e define o comportamento padrão do
    // motor quando a força dedicada a ele for zero.
    protected void configZeroPowerBehavior(@NonNull DcMotorEx[] arrayDeMotores) {
        for (DcMotorEx motor : arrayDeMotores){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
