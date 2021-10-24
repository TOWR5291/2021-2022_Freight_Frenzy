package club.towr5291.robotconfig;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.libraries.TOWRDashBoard;

public class HardwareArmMotorsFreightFrenzy {

    public ElapsedTime elapse = new ElapsedTime();
    /* Public OpMode members. */
    public DcMotor          liftMotor1          = null;
    //public DcMotor          intakeMotor1        = null;
    public DcMotorEx        flywheelMotor       = null; // Enhanced motor controls
    //public DcMotor          intakeMotor         = null;
    //public Servo            wristServo          = null;
    public Servo            ClawServo           = null;
    //public Servo            rightArmServo       = null;
    //public Servo            rightWristServo     = null;
    //public Servo            rightClampServo     = null;
    //public Servo            leftArmServo        = null;
    //public Servo            leftWristServo      = null;
    //public Servo            leftClampServo      = null;
    //public Servo            ejector             = null;
    //public DistanceSensor   leftFrontDistance   = null;
    //public DistanceSensor   leftBackDistance    = null;
    //public DistanceSensor   rightFrontDistance  = null;
    //public DistanceSensor   rightBackDistance   = null;

    /* local OpMode members. */
    HardwareMap hwMap               =  null;
    private TOWRDashBoard dashBoard = null;

    /* Constructor */
    public HardwareArmMotorsFreightFrenzy(){
        elapse.startTime();
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, TOWRDashBoard dash) {

        this.dashBoard = dash;
        this.hwMap = ahwMap;

        // Define and Initialize Motors
        this.liftMotor1         = hwMap.dcMotor.get("liftMotor1");
        this.flywheelMotor      = (DcMotorEx) hwMap.dcMotor.get("flywheelMotor");
        this.ClawServo          = hwMap.servo.get("ClawServo");
        //this.intakeMotor1       = hwMap.dcMotor.get("intakeMotor1");
        //this.flywheelMotor    = hwMap.dcMotor.get("flywheelMotor");
        //this.intakeMotor        = hwMap.dcMotor.get("intakeMotor");
        //this.wristServo         = hwMap.servo.get("wristServo");
        //this.rightArmServo      = hwMap.servo.get("rightArmServo");
        //this.rightWristServo    = hwMap.servo.get("rightWristServo");
        //this.rightClampServo    = hwMap.servo.get("rightClampServo");
        //this.leftArmServo       = hwMap.servo.get("leftArmServo");
        //this.leftWristServo     = hwMap.servo.get("leftWristServo");
        //this.leftClampServo     = hwMap.servo.get("leftClampServo");
        //this.ejector            = hwMap.servo.get("ejector");

        setHardwareArmDirections();

        liftMotor1.setPower(0);
        setHardwareLiftMotorResetEncoders();
        this.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setHardwareArmDirections(){
        this.liftMotor1.setDirection(DcMotor.Direction.FORWARD);
        this.flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void setHardwareLiftPower(double power){
        liftMotor1.setPower(power);
    }

    public void setHardwareLiftMotorResetEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setHardwareLiftMotorRunUsingEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHardwareLiftMotorRunWithoutEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setHardwareLiftMotorRunToPosition(){
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getLiftMotor1Encoder() {
        return liftMotor1.getCurrentPosition();
    }

    public void allMotorsStop(){
        this.liftMotor1.setPower(0);
        this.flywheelMotor.setPower(0);
    }

    /* public DistanceSensor getLeftFrontDistance() {
        return leftFrontDistance;
    }

    public DistanceSensor getLeftBackDistance() {
        return leftBackDistance;
    }

    public DistanceSensor getRightFrontDistance() {
        return rightFrontDistance;
    }

    public DistanceSensor getRightBackDistance() {
        return rightBackDistance;
    }*/
}
