package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public final class CameraModule {

    // obrigatórios
    protected final String VULFORIA_KEY = ""; // necessário inserir a chave da api da vulforia
    protected static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    protected static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    // para o acesso a camera em geral
    private final Camera camera;
    private CameraManager cameraManager;
    private CameraCaptureSession cameraCaptureSession;
    private WebcamName webcamName;

    // para o reconhecimento de imagem
    private VuforiaLocalizer vulforiaInstance;

    // extras
    private final HardwareMap hardwareMap;

    // TODO: preencher com as instancia de camera e parametros necessários (incerto)
    public CameraModule(Camera cam, CameraManager cameraManager, VuforiaLocalizer vulforiaInstance, HardwareMap hardwareMap){
        this.camera = cam;
        this.cameraManager = cameraManager;
        this.vulforiaInstance = vulforiaInstance;
        this.hardwareMap = hardwareMap;
    }

    // TODO: configurar webcam deve ser o primeiro método chamado
    public void iniciarWebcam(){
        cameraManager = ClassFactory.getInstance().getCameraManager();
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");
    }

    private void iniciarVulforia(){

        // instancia e configuração dos parametros de detecção
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VULFORIA_KEY; // essencial

        parameters.camera = camera;
        parameters.cameraName = webcamName;

        // finalizando aqui, criando a instancia da engine
        vulforiaInstance = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void iniciarTensorFlow(){
        int tfodMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        TFObjectDetector.Parameters parameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        parameters.minResultConfidence = 0.85F;
        parameters.isModelTensorFlow2 = true;

        // finaliza criando a instancia e carrega o modelo do arquivo
        TFObjectDetector tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(parameters, vulforiaInstance);
        tfObjectDetector.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}