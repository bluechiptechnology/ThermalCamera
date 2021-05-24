package bct.thermalcamera;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.PorterDuff;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.Random;

//Class to display a thermal camera image in a frame layout
public class ThermalCameraView  extends SurfaceView implements SurfaceHolder.Callback, Runnable
{
    private static final String TAG = "ThermalCameraView";

    private SurfaceView surfaceview;
    private SurfaceHolder mHolder;
    ThermalCamera camera;

    private boolean surfaceReady = false;
    private boolean drawingActive = false;
    private Thread drawThread;

    //Small palette for representing thermal data graphically
    int iron_palette[] = { 0xFF00000a, 0xFF000014, 0xFF00001e, 0xFF000025, 0xFF00002a, 0xFF00002e, 0xFF000032, 0xFF000036, 0xFF00003a, 0xFF00003e, 0xFF000042, 0xFF000046, 0xFF00004a, 0xFF00004f, 0xFF000052, 0xFF010055, 0xFF010057, 0xFF020059, 0xFF02005c, 0xFF03005e, 0xFF040061, 0xFF040063, 0xFF050065, 0xFF060067, 0xFF070069, 0xFF08006b, 0xFF09006e, 0xFF0a0070, 0xFF0b0073, 0xFF0c0074, 0xFF0d0075, 0xFF0d0076, 0xFF0e0077, 0xFF100078, 0xFF120079, 0xFF13007b, 0xFF15007c, 0xFF17007d, 0xFF19007e, 0xFF1b0080, 0xFF1c0081, 0xFF1e0083, 0xFF200084, 0xFF220085, 0xFF240086, 0xFF260087, 0xFF280089, 0xFF2a0089, 0xFF2c008a, 0xFF2e008b, 0xFF30008c, 0xFF32008d, 0xFF34008e, 0xFF36008e, 0xFF38008f, 0xFF390090, 0xFF3b0091, 0xFF3c0092, 0xFF3e0093, 0xFF3f0093, 0xFF410094, 0xFF420095, 0xFF440095, 0xFF450096, 0xFF470096, 0xFF490096, 0xFF4a0096, 0xFF4c0097, 0xFF4e0097, 0xFF4f0097, 0xFF510097, 0xFF520098, 0xFF540098, 0xFF560098, 0xFF580099, 0xFF5a0099, 0xFF5c0099, 0xFF5d009a, 0xFF5f009a, 0xFF61009b, 0xFF63009b, 0xFF64009b, 0xFF66009b, 0xFF68009b, 0xFF6a009b, 0xFF6c009c, 0xFF6d009c, 0xFF6f009c, 0xFF70009c, 0xFF71009d, 0xFF73009d, 0xFF75009d, 0xFF77009d, 0xFF78009d, 0xFF7a009d, 0xFF7c009d, 0xFF7e009d, 0xFF7f009d, 0xFF81009d, 0xFF83009d, 0xFF84009d, 0xFF86009d, 0xFF87009d, 0xFF89009d, 0xFF8a009d, 0xFF8b009d, 0xFF8d009d, 0xFF8f009c, 0xFF91009c, 0xFF93009c, 0xFF95009c, 0xFF96009b, 0xFF98009b, 0xFF99009b, 0xFF9b009b, 0xFF9c009b, 0xFF9d009b, 0xFF9f009b, 0xFFa0009b, 0xFFa2009b, 0xFFa3009b, 0xFFa4009b, 0xFFa6009a, 0xFFa7009a, 0xFFa8009a, 0xFFa90099, 0xFFaa0099, 0xFFab0099, 0xFFad0099, 0xFFae0198, 0xFFaf0198, 0xFFb00198, 0xFFb00198, 0xFFb10197, 0xFFb20197, 0xFFb30196, 0xFFb40296, 0xFFb50295, 0xFFb60295, 0xFFb70395, 0xFFb80395, 0xFFb90495, 0xFFba0495, 0xFFba0494, 0xFFbb0593, 0xFFbc0593, 0xFFbd0593, 0xFFbe0692, 0xFFbf0692, 0xFFbf0692, 0xFFc00791, 0xFFc00791, 0xFFc10890, 0xFFc10990, 0xFFc20a8f, 0xFFc30a8e, 0xFFc30b8e, 0xFFc40c8d, 0xFFc50c8c, 0xFFc60d8b, 0xFFc60e8a, 0xFFc70f89, 0xFFc81088, 0xFFc91187, 0xFFca1286, 0xFFca1385, 0xFFcb1385, 0xFFcb1484, 0xFFcc1582, 0xFFcd1681, 0xFFce1780, 0xFFce187e, 0xFFcf187c, 0xFFcf197b, 0xFFd01a79, 0xFFd11b78, 0xFFd11c76, 0xFFd21c75, 0xFFd21d74, 0xFFd31e72, 0xFFd32071, 0xFFd4216f, 0xFFd4226e, 0xFFd5236b, 0xFFd52469, 0xFFd62567, 0xFFd72665, 0xFFd82764, 0xFFd82862, 0xFFd92a60, 0xFFda2b5e, 0xFFda2c5c, 0xFFdb2e5a, 0xFFdb2f57, 0xFFdc2f54, 0xFFdd3051, 0xFFdd314e, 0xFFde324a, 0xFFde3347, 0xFFdf3444, 0xFFdf3541, 0xFFdf363d, 0xFFe0373a, 0xFFe03837, 0xFFe03933, 0xFFe13a30, 0xFFe23b2d, 0xFFe23c2a, 0xFFe33d26, 0xFFe33e23, 0xFFe43f20, 0xFFe4411d, 0xFFe4421c, 0xFFe5431b, 0xFFe54419, 0xFFe54518, 0xFFe64616, 0xFFe74715, 0xFFe74814, 0xFFe74913, 0xFFe84a12, 0xFFe84c10, 0xFFe84c0f, 0xFFe94d0e, 0xFFe94d0d, 0xFFea4e0c, 0xFFea4f0c, 0xFFeb500b, 0xFFeb510a, 0xFFeb520a, 0xFFeb5309, 0xFFec5409, 0xFFec5608, 0xFFec5708, 0xFFec5808, 0xFFed5907, 0xFFed5a07, 0xFFed5b06, 0xFFee5c06, 0xFFee5c05, 0xFFee5d05, 0xFFee5e05, 0xFFef5f04, 0xFFef6004, 0xFFef6104, 0xFFef6204, 0xFFf06303, 0xFFf06403, 0xFFf06503, 0xFFf16603, 0xFFf16603, 0xFFf16703, 0xFFf16803, 0xFFf16902, 0xFFf16a02, 0xFFf16b02, 0xFFf16b02, 0xFFf26c01, 0xFFf26d01, 0xFFf26e01, 0xFFf36f01, 0xFFf37001, 0xFFf37101, 0xFFf37201, 0xFFf47300, 0xFFf47400, 0xFFf47500, 0xFFf47600, 0xFFf47700, 0xFFf47800, 0xFFf47a00, 0xFFf57b00, 0xFFf57c00, 0xFFf57e00, 0xFFf57f00, 0xFFf68000, 0xFFf68100, 0xFFf68200, 0xFFf78300, 0xFFf78400, 0xFFf78500, 0xFFf78600, 0xFFf88700, 0xFFf88800, 0xFFf88800, 0xFFf88900, 0xFFf88a00, 0xFFf88b00, 0xFFf88c00, 0xFFf98d00, 0xFFf98d00, 0xFFf98e00, 0xFFf98f00, 0xFFf99000, 0xFFf99100, 0xFFf99200, 0xFFf99300, 0xFFfa9400, 0xFFfa9500, 0xFFfa9600, 0xFFfb9800, 0xFFfb9900, 0xFFfb9a00, 0xFFfb9c00, 0xFFfc9d00, 0xFFfc9f00, 0xFFfca000, 0xFFfca100, 0xFFfda200, 0xFFfda300, 0xFFfda400, 0xFFfda600, 0xFFfda700, 0xFFfda800, 0xFFfdaa00, 0xFFfdab00, 0xFFfdac00, 0xFFfdad00, 0xFFfdae00, 0xFFfeaf00, 0xFFfeb000, 0xFFfeb100, 0xFFfeb200, 0xFFfeb300, 0xFFfeb400, 0xFFfeb500, 0xFFfeb600, 0xFFfeb800, 0xFFfeb900, 0xFFfeb900, 0xFFfeba00, 0xFFfebb00, 0xFFfebc00, 0xFFfebd00, 0xFFfebe00, 0xFFfec000, 0xFFfec100, 0xFFfec200, 0xFFfec300, 0xFFfec400, 0xFFfec500, 0xFFfec600, 0xFFfec700, 0xFFfec800, 0xFFfec901, 0xFFfeca01, 0xFFfeca01, 0xFFfecb01, 0xFFfecc02, 0xFFfecd02, 0xFFfece03, 0xFFfecf04, 0xFFfecf04, 0xFFfed005, 0xFFfed106, 0xFFfed308, 0xFFfed409, 0xFFfed50a, 0xFFfed60a, 0xFFfed70b, 0xFFfed80c, 0xFFfed90d, 0xFFffda0e, 0xFFffda0e, 0xFFffdb10, 0xFFffdc12, 0xFFffdc14, 0xFFffdd16, 0xFFffde19, 0xFFffde1b, 0xFFffdf1e, 0xFFffe020, 0xFFffe122, 0xFFffe224, 0xFFffe226, 0xFFffe328, 0xFFffe42b, 0xFFffe42e, 0xFFffe531, 0xFFffe635, 0xFFffe638, 0xFFffe73c, 0xFFffe83f, 0xFFffe943, 0xFFffea46, 0xFFffeb49, 0xFFffeb4d, 0xFFffec50, 0xFFffed54, 0xFFffee57, 0xFFffee5b, 0xFFffee5f, 0xFFffef63, 0xFFffef67, 0xFFfff06a, 0xFFfff06e, 0xFFfff172, 0xFFfff177, 0xFFfff17b, 0xFFfff280, 0xFFfff285, 0xFFfff28a, 0xFFfff38e, 0xFFfff492, 0xFFfff496, 0xFFfff49a, 0xFFfff59e, 0xFFfff5a2, 0xFFfff5a6, 0xFFfff6aa, 0xFFfff6af, 0xFFfff7b3, 0xFFfff7b6, 0xFFfff8ba, 0xFFfff8bd, 0xFFfff8c1, 0xFFfff8c4, 0xFFfff9c7, 0xFFfff9ca, 0xFFfff9cd, 0xFFfffad1, 0xFFfffad4, 0xFFfffbd8, 0xFFfffcdb, 0xFFfffcdf, 0xFFfffde2, 0xFFfffde5, 0xFFfffde8, 0xFFfffeeb, 0xFFfffeee, 0xFFfffef1, 0xFFfffef4, 0xFFfffff6 };


    public ThermalCameraView(Context context, ThermalCamera cam) throws Exception
    {
        super(context);

        surfaceview = (SurfaceView)findViewById(R.id.ThermalCameraPreview);

        camera = cam;

        mHolder = getHolder();
        mHolder.setFormat(PixelFormat.TRANSPARENT);
        mHolder.setFixedSize(camera.getCameraWidth(),camera.getCameraHeight());
        mHolder.addCallback(this);
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        Log.i(TAG, "surfaceChanged");
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {

        try {
            Log.i(TAG, "surfaceCreated");
            camera.Initialise();
            mHolder = holder;
            StartStream();
        } catch (Exception e) {
            Log.d(TAG, "Error setting camera preview: " + e.getMessage());
        }
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        Log.i(TAG, "surfaceDestroyed");
        StopStream();
        mHolder = null;
    }

    private void tryDrawing(SurfaceHolder holder) {
        Canvas canvas = holder.lockCanvas();
        if (canvas == null) {
            Log.e(TAG, "Cannot draw onto the canvas as it's null");
        } else {
            UpdateThermalImage(canvas);
            holder.unlockCanvasAndPost(canvas);
        }
    }

    private void UpdateThermalImage(final Canvas canvas)
    {
        float minTemp10thsDegree;
        float maxTemp10thsDegree;
        float Difference10thsDegree;
        float arrayincrementper10thdegree = 1;
        int paletteindex = 0;
        int FramebufferIndex = 0;

        //Calculate temperature range in 10ths of a degree to provide a reasonable range when converting temperature to colour.
        camera.DetermineMinMaxTemperature();
        minTemp10thsDegree = camera.getLastBufferMinTemperature() * 10;
        maxTemp10thsDegree = camera.getLastBufferMaxTemperature() * 10;
        Difference10thsDegree = maxTemp10thsDegree - minTemp10thsDegree;

        arrayincrementper10thdegree = (float) iron_palette.length / Difference10thsDegree;

        Paint p = new Paint();
        p.setStyle(Paint.Style.FILL);

        //Convert temperature to colour and draw to canvas
        for(int yIndexCounter = 0; yIndexCounter < camera.getCameraHeight(); yIndexCounter++)
        {
            for(int xIndexCounter = 0; xIndexCounter < camera.getCameraWidth(); xIndexCounter++)
            {
                paletteindex = (int) (((camera.CameraBufferData[FramebufferIndex] * 10) - minTemp10thsDegree) * arrayincrementper10thdegree);
                if(paletteindex >= iron_palette.length)
                {
                    paletteindex = iron_palette.length -1;
                }
                p.setColor(iron_palette[paletteindex]);
                canvas.drawPoint(xIndexCounter, yIndexCounter, p);
                FramebufferIndex++;
            }
        }
    }

    void StartStream()
    {
        if (drawThread != null)
        {
            Log.d(TAG, "draw thread still active..");
            drawingActive = false;
            try
            {
                drawThread.join();
            } catch (InterruptedException e)
            { // do nothing
            }
        }

        surfaceReady = true;

        if (surfaceReady && drawThread == null)
        {
            drawThread = new Thread(this, "Draw thread");
            drawingActive = true;
            drawThread.start();
        }
        Log.d(TAG, "Created");
    }

    void StopStream()
    {

        if (drawThread == null)
        {
            Log.d(TAG, "DrawThread is null");
            return;
        }
        drawingActive = false;
        while (true)
        {
            try
            {
                Log.d(TAG, "Request last frame");
                drawThread.join(5000);
                break;
            } catch (Exception e)
            {
                Log.e(TAG, "Could not join with draw thread");
            }
        }
        drawThread = null;
    }

    @Override
    public void run()
    {
        Log.d(TAG, "Draw thread started");
        long frameStartTime;
        long frameTime;
        int SleepDuration = 1000 / camera.getCameraFrameRate() - 2;

        try
        {
            while (drawingActive)
            {
                frameStartTime = System.nanoTime();
                camera.Capture();
                tryDrawing(mHolder);

                // calculate the time required to draw the frame in ms
                frameTime = (System.nanoTime() - frameStartTime) / 1000000;



                Thread.sleep(SleepDuration);
            }
        } catch (Exception e)
        {
            Log.w(TAG, "Exception while locking/unlocking");
        }
        Log.d(TAG, "Draw thread finished");
    }

}
