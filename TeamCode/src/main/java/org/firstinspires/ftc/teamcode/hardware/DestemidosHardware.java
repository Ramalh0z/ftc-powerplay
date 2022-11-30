package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

// NOTE(ramalho): deixei esse nome, pq já tinha uma classe "robot hardware" nos exemplos
public class DestemidosHardware {

    // Rodas
    public DcMotorEx motorDireitaFrente;
    public DcMotorEx motorDireitaTras;
    public DcMotorEx motorEsquerdaFrente;
    public DcMotorEx motorEsquerdaTras;

    // Atuadores
    public DcMotorEx motorCentro;
    public DcMotorEx motorBraçoA;
    public DcMotorEx motorBraçoB;

    // Servos
    public Servo servoMão;
    public Servo servoGarraA;
    public Servo servoGarraB;


    // utilitário
    public DcMotorEx[] motores;
    public DcMotorEx[] atuadores;

    public DestemidosHardware(@NonNull HardwareMap hardwareMap){

        // mapeando os motores com seus nomes e sequência correta
        motorDireitaFrente  = hardwareMap.get(DcMotorEx.class,"DF"); // porta 0 - controlHub
        motorDireitaTras    = hardwareMap.get(DcMotorEx.class,"DT"); // porta 1 - controlHub
        motorEsquerdaFrente = hardwareMap.get(DcMotorEx.class,"EF"); // porta 3 - expansion
        motorEsquerdaTras   = hardwareMap.get(DcMotorEx.class,"ET"); // porta 3 - controlHub

        motorCentro = hardwareMap.get(DcMotorEx.class,"centro"); // porta 0 - expansion
        motorBraçoA = hardwareMap.get(DcMotorEx.class,"braçoA"); // porta 1 - expansion
        motorBraçoB = hardwareMap.get(DcMotorEx.class,"braçoB"); // porta 2 - expansion

        servoMão    = hardwareMap.get(Servo.class, "mão");    // porta 1 - controlhub
        servoGarraA = hardwareMap.get(Servo.class, "garraA"); // porta 3 - controlhub
        servoGarraB = hardwareMap.get(Servo.class, "garraB"); // porta 5 - controlhub

        // precisamos inverter apenas os motores da esquerda
        motorDireitaFrente.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDireitaTras.setDirection(DcMotorSimple.Direction.FORWARD);
        motorEsquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBraçoA.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBraçoB.setDirection(DcMotorSimple.Direction.FORWARD);

        servoMão.setDirection(Servo.Direction.REVERSE);
        servoGarraA.setDirection(Servo.Direction.REVERSE);
        servoGarraB.setDirection(Servo.Direction.FORWARD);

        // zerando tudo
        resetEncoders(
                motorDireitaFrente,
                motorDireitaTras,
                motorEsquerdaFrente,
                motorEsquerdaTras,
                motorCentro,
                motorBraçoA,
                motorBraçoB);

        // ativando por padrão todos os encoders
        configEncoders(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                motorDireitaFrente,
                motorDireitaTras,
                motorEsquerdaFrente,
                motorEsquerdaTras,
                motorCentro,
                motorBraçoA,
                motorBraçoB);

        // comportamento dos motores, quando a força for equivalente 0:
        configZeroPowerBehavior(
                motorDireitaFrente,
                motorDireitaTras,
                motorEsquerdaFrente,
                motorEsquerdaTras,
                motorCentro
        );

        // inicalizando a lista de motores já configurados
        motores = new DcMotorEx[]{ motorDireitaFrente, motorDireitaTras,
                motorEsquerdaFrente, motorEsquerdaTras };

        atuadores = new DcMotorEx[] {motorCentro, motorBraçoA, motorBraçoB };

    }

    public void configEncoders(DcMotor.RunMode runMode, @NonNull DcMotorEx... motores) {
        for (DcMotorEx motor : motores) {
            motor.setMode(runMode);
        }
    }

    public void resetEncoders(@NonNull DcMotorEx... motores){
        for(DcMotorEx motor : motores) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void configZeroPowerBehavior(@NonNull DcMotorEx... motores) {
        for (DcMotorEx motor : motores) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // reinicia a contagem relativa dos encoders
    public void resetRodasEncoder(){
        motorDireitaFrente.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireitaTras.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsquerdaFrente.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsquerdaTras.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
