package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.List;

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

    // lista com todos os hubs e seus IDs para fácil acesso
    private List<LynxModule> allHubs;
    private static final int CONTROLHUB_ID = 0;
    private static final int EXPANSIONHUB_ID = 0;

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

    // Sensores
    public IMU sennnsorIMU;
    private IMU.Parameters parametrosDoIMU;

    // utilitário
    public DcMotorEx[] motores;
    public DcMotorEx[] atuadores;

    public DestemidosHardware(@NonNull HardwareMap hardwareMap){

        // mapeando os motores com seus nomes e sequência correta
        allHubs = hardwareMap.getAll(LynxModule.class);

        // uma frescurinha que descobri no dia do intersesi
        allHubs.get(CONTROLHUB_ID).setConstant(Color.CYAN);
        allHubs.get(EXPANSIONHUB_ID).setConstant(Color.CYAN);

        // configurando o sensor IMU
        parametrosDoIMU = new IMU.Parameters(
                // NOTE (ramalho): aqui é de acordo com a posição que colocamos o hub no robô
                // então é mais provável variar a direção das entradas USB nas futuras modificações do robô
                // referências: https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html

                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        // configurando o resto do hardware
        sennnsorIMU = hardwareMap.get(IMU.class, "imu");

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
                motorBraçoB
        );

        // ativando por padrão todos os encoders
        configEncoders(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER,
                motorDireitaFrente,
                motorDireitaTras,
                motorEsquerdaFrente,
                motorEsquerdaTras,
                motorCentro,
                motorBraçoA,
                motorBraçoB
        );

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

        // carregando as configurações no IMU
        sennnsorIMU.initialize(parametrosDoIMU);
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

    public void setBulkReadToAuto() {
        for (LynxModule robotHub : allHubs) {
            robotHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void setBulkCacheToManual() {
        for (LynxModule robotHub : allHubs) {
            robotHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    };
}
