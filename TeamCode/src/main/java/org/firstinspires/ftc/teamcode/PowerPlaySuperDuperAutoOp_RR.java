package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "3-Cone AutoOp, Red, Right", group = "Linear Opmode")
public class PowerPlaySuperDuperAutoOp_RR extends PowerPlaySuperAutoOp {

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {
        this.placeFirstConeImmediately = true;
        this.coneColor = new double[]{255, 0, 0};
        //this.leftDst *= -1;
        super.runOpMode();
    }

}
