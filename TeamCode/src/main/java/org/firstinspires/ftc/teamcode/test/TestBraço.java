package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Test Braço")
public class TestBraço extends LinearOpMode {
    private final DcMotorEx[] atuadores = new DcMotorEx[4];
    private Servo garraA;
    private Servo garraB;
    private Servo mao;

    @Override
    public void runOpMode() throws InterruptedException {

        atuadores[0] = hardwareMap.get(DcMotorEx.class,"centro");   // porta 0 - rev2
        atuadores[1] = hardwareMap.get(DcMotorEx.class,"puxa");     // porta 1 - rev2
        atuadores[2] = hardwareMap.get(DcMotorEx.class,"braçoA");   // porta 2 - rev2
        atuadores[3] = hardwareMap.get(DcMotorEx.class,"braçoB");   // porta 3 - rev2

        // Confgurando os servos
        garraA = hardwareMap.get(Servo.class, "garraA");
        garraB = hardwareMap.get(Servo.class, "garraB");
        mao    = hardwareMap.get(Servo.class, "mão");

        atuadores[2].setDirection(DcMotorEx.Direction.REVERSE);
        mao.setDirection(Servo.Direction.REVERSE);

        //garraA.setDirection(Servo.Direction.REVERSE);
        garraB.setDirection(Servo.Direction.REVERSE);

        for (DcMotorEx atuador : atuadores) {
            atuador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            atuador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        atuadores[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atuadores[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atuadores[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atuadores[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
