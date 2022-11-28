package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.RobotLogger;

import java.util.List;

@TeleOp(name = "FTC_3", group = "TeleOps")
public class FTC_3 extends OpMode {
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final DcMotorEx[] atuadores = new DcMotorEx[3];
    private List<LynxModule> hubs;

    @Override
    public void init() {

        // bulk reads
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub: hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // mapeando os motores e atuadores para cada slot
        motors[0] = hardwareMap.get(DcMotorEx.class,"DF");  // porta 0 - controlHub;
        motors[1] = hardwareMap.get(DcMotorEx.class,"DT");  // porta 1 - controlHub;
        motors[2] = hardwareMap.get(DcMotorEx.class,"EF");  // porta 2 - controlHub;
        motors[3] = hardwareMap.get(DcMotorEx.class,"ET");  // porta 3 - controlHub;

        atuadores[0] = hardwareMap.get(DcMotorEx.class,"centro");  // porta 0 - rev2
        atuadores[1] = hardwareMap.get(DcMotorEx.class,"braço");  // porta 1 - rev2
        atuadores[2] = hardwareMap.get(DcMotorEx.class,"puxa");  // porta 2 - rev2

        // A direção padrão é FORWARD (frente)
        // apenas os motores da direita precisam estar invertidos
        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);

        // apenas um dos CoreHex presentes na garra precisava ficar invertido
        atuadores[1].setDirection(DcMotorEx.Direction.REVERSE);

        // desativando os encoders
        for ( DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (DcMotorEx atuador: atuadores) {
            atuador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void loop() {

        // comando pra conectar via wifi no adb
        // adb connect 192.168.43.1:5555

        double fatorPower = 80;
        double vertical   = -gamepad1.left_stick_y   * (fatorPower / 100);
        double horizontal = gamepad1.left_stick_x    * (fatorPower / 100);
        double giro       = gamepad1.right_stick_x   * (fatorPower / 100);

        // bulk reads
        for (LynxModule hub: hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        motors[0].setPower(vertical + horizontal + giro);
        motors[1].setPower(vertical - horizontal + giro);
        motors[2].setPower(vertical - horizontal - giro);
        motors[3].setPower(vertical + horizontal - giro);

        //Gamepad 2
        double controlPower = gamepad2.left_stick_y * 0.85;
        if (gamepad2.left_stick_y > 0.0) {
            atuadores[0].setPower(-controlPower);
            atuadores[1].setPower(-controlPower);
        } else {
            atuadores[0].setPower(0);
            atuadores[1].setPower(0);
        }

        // button x
        atuadores[2].setPower(gamepad2.x ? -0.7 : 0.0);

        //button a
        atuadores[2].setPower(gamepad2.a ? 0.5 : 0.0);

        // apenas debug
        RobotLogger.debugMotorPower(telemetry, motors);
        RobotLogger.debugMotorPower(telemetry, atuadores);
        RobotLogger.debugControles(telemetry, gamepad1, gamepad2);
    }
}