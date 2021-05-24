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

import java.text.DecimalFormat;

public class ThermalCameraTemperatureOverlayView extends SurfaceView implements SurfaceHolder.Callback, Runnable
{
    private static final String TAG = "ThermalCameraView";

    private SurfaceView surfaceview;
    private SurfaceHolder mHolder;
    ThermalCamera camera;

    private boolean surfaceReady = false;
    private boolean drawingActive = false;
    private Thread drawThread;

    public ThermalCameraTemperatureOverlayView(Context context, ThermalCamera cam) throws Exception
    {
        super(context);

        surfaceview = (SurfaceView)findViewById(R.id.ThermalCameraTemperatureOverlay);

        camera = cam;

        mHolder = getHolder();
        mHolder.setFormat(PixelFormat.TRANSPARENT);
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
            UpdateRangeAndTemperature(canvas);
            holder.unlockCanvasAndPost(canvas);
        }
    }

    private void UpdateRangeAndTemperature(final Canvas canvas)
    {

        int paletteindex = 0;
        int FramebufferIndex = 0;

        camera.DetermineMinMaxTemperature();

        //Log.i(TAG, "Min: " + camera.getLastBufferMinTemperature() + " Max: " + camera.getLastBufferMaxTemperature()  );

        Paint p = new Paint();
        p.setStyle(Paint.Style.FILL);
        p.setColor(Color.BLACK);

        p.setTextSize(24);
        p.setFakeBoldText(false);

        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(2);

        //Write temperature to screen
        canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
        canvas.drawText(df.format(camera.getLastBufferMiddleFramebufferTemperature()) + "\u00B0" + "C", (canvas.getWidth() / 2) - 30, canvas.getHeight() / 2, p);
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

        try
        {
            while (drawingActive)
            {
                tryDrawing(mHolder);
                Thread.sleep(30);
            }
        } catch (Exception e)
        {
            Log.w(TAG, "Exception while locking/unlocking");
        }
        Log.d(TAG, "Draw thread finished");
    }

}
