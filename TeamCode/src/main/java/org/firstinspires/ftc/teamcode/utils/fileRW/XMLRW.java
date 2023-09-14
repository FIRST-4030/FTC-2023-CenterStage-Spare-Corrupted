package org.firstinspires.ftc.teamcode.utils.fileRW;

import android.os.Environment;

import java.io.File;

public class XMLRW {

    public static final String defaultDir = Environment.getExternalStorageDirectory().getPath() + "/ROBOT_XML_DATA/";
    private static boolean initialized = false;

    private XMLRW(){}

    public static void init(){
        if (!initialized) {
            File directory = new File(defaultDir);
            if (!directory.exists()) {
                directory.mkdir();
            }
            initialized = true;
        }
    }
}
