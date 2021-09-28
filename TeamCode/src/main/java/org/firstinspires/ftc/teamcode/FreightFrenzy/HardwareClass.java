package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareClass {

    //Declaração dos motores drive
    public DcMotor motorEsquerda, motorDireita, motorEsquerdaTras, motorDireitaTras;

    //Referências de hardware e gyro
    static BNO055IMU imu;
    HardwareMap hwMap   =  null;

    public void hardwareGeral(HardwareMap ahwMap) {
        //Referência hardware
        hwMap = ahwMap;

        /*
         * =============================================================================
         *                                  GYRO
         * =============================================================================
         */

        //Configura o gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        //Imu no Drive Station
        imu = hwMap.get(BNO055IMU.class, "imu");

        //Inicializa os parametros do gyro
        imu.initialize(parameters);

        /*
         * =============================================================================
         *                                  ACIONADORES
         * =============================================================================
         */

        //Pega o nome das variáveis no Dv.
        motorEsquerda = hwMap.get(DcMotor.class, "motor_Esquerda");
        motorEsquerdaTras = hwMap.get(DcMotor.class, "motor_EsquerdaTras");
        motorDireita = hwMap.get(DcMotor.class, "motor_Direita");
        motorDireitaTras = hwMap.get(DcMotor.class, "motor_DireitaTras");

        //Direção dos motores
        motorEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorDireita.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdaTras.setDirection(DcMotor.Direction.REVERSE);
        motorDireitaTras.setDirection(DcMotor.Direction.FORWARD);

        //Reseta os encoder, por que são usados em dois OpModes
        motorEsquerda.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsquerdaTras.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireitaTras.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Coloca para se mexer contando o encoder
        motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsquerdaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDireitaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    }
