package bct.thermalcamera;

import androidx.appcompat.app.AppCompatActivity;

import android.app.AlarmManager;
import android.content.Context;
import android.os.Bundle;
import android.os.Environment;
import android.os.PowerManager;
import android.os.SystemProperties;
import android.util.Log;
import android.view.WindowManager;
import android.widget.FrameLayout;
import android.widget.Toast;
import android.util.Log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;

public class MainActivity extends AppCompatActivity
{
    ThermalCameraMLX90640 thermalsensor;
    ThermalCameraView thermalcameraview;
    ThermalCameraTemperatureOverlayView thermalcameraoverlayview;

    private static final String TAG = "ThermalCamera";

    // Used to load the 'mlx90640' JNI library on application startup.
    static {
        System.loadLibrary("mlx90640-lib");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        boolean rebootdevice = false;

        //Prevent Android from dimming the display
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        try
        {
            thermalsensor = new ThermalCameraMLX90640("/dev/i2c-1", (byte)0x33, 8);

            thermalcameraview = new ThermalCameraView(this.getApplicationContext(), thermalsensor);
            thermalcameraoverlayview = new ThermalCameraTemperatureOverlayView(this.getApplicationContext(), thermalsensor);
            FrameLayout preview = (FrameLayout) findViewById(R.id.cameraframe);
            preview.addView(thermalcameraoverlayview);
            preview.addView(thermalcameraview);

        }
        catch (Exception ex)
        {
            Toast.makeText(this.getApplicationContext(),  ex.getMessage(), Toast.LENGTH_LONG).show();
            System.exit(1);
        }

        //Use our custom splash screen, during the system boot process
        File bootanimationdirectory = Environment.getExternalStoragePublicDirectory("bootanimation");
        if (bootanimationdirectory.exists()) {
            File bootanimation = new File(bootanimationdirectory, "bootanimation.zip");
            if (!bootanimation.exists()) {
                try {
                    final InputStream is = getResources().getAssets().open("bootanimation.zip");
                    FileOutputStream os = new FileOutputStream(bootanimation, false);

                    // Transfer bytes from in to out
                    byte[] buf = new byte[1024];
                    int len;
                    while ((len = is.read(buf)) > 0) {
                        os.write(buf, 0, len);
                    }

                    is.close();
                    os.close();

                } catch (Exception ex) {
                    Toast.makeText(this.getApplicationContext(), "Failed to copy boot animation: " + ex.getMessage(), Toast.LENGTH_LONG).show();
                }

            }
        }

        //Enable KIOS mode if it is not already set
        String kioskmodesetting = SystemProperties.get("persist.sys.kioskmode");
        if(kioskmodesetting == null || kioskmodesetting.equals(new String("")))
        {
            Log.i(TAG, "Enabling KIOSK mode");
            SystemProperties.set("persist.sys.kioskmode", "true");
            rebootdevice = true;
        }

        if(rebootdevice)
        {
            try {
                PowerManager pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
                pm.reboot(null);
            }
            catch(Exception ex)
            {
                Log.e(TAG, "Failed to reboot device: " + ex.getMessage());
            }
        }
    }

}
