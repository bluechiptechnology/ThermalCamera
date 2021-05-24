package bct.thermalcamera;

//base class for thermal camera implementations
public abstract class ThermalCamera
{
    protected int CameraDimensionWidth;
    protected int CameraDimensionHeight;
    protected int CameraFrameRate;
    protected float[] CameraBufferData = null;
    protected float[] CameraBufferColours = null;
    protected float LastBufferUpdateTime;
    protected float LastBufferMaxTemperature;
    protected float LastBufferMinTemperature;
    protected float LastBufferMiddleFramebufferTemperature;


    abstract void Initialise() throws Exception;
    abstract void Capture() throws Exception;

    int getCameraWidth() { return CameraDimensionWidth;}
    int getCameraHeight() { return CameraDimensionHeight;}
    int getCameraFrameRate() { return CameraFrameRate;}

    float getLastBufferMaxTemperature() { return LastBufferMaxTemperature;}
    float getLastBufferMinTemperature() { return LastBufferMinTemperature;}
    float getLastBufferMiddleFramebufferTemperature() { return LastBufferMiddleFramebufferTemperature;}

    void AllocateFrameBuffer() throws Exception
    {
        if(CameraBufferData == null)
        {
            CameraBufferData = new float[CameraDimensionWidth * CameraDimensionHeight];
            if(CameraBufferData == null)
            {
                throw new Exception("Failed to allocate framebuffer for thermal camera");
            }
        }
    }

    void DetermineMinMaxTemperature()
    {
        LastBufferMaxTemperature = -300;
        LastBufferMinTemperature= 300;

        if(CameraBufferData == null)
        {
            LastBufferMaxTemperature = -300;
            LastBufferMinTemperature = -300;
        }
        else
        {

            for(int iIndexCounter = 0; iIndexCounter < CameraBufferData.length; iIndexCounter++)
            {
                if(CameraBufferData[iIndexCounter] < LastBufferMinTemperature)
                {
                    LastBufferMinTemperature = CameraBufferData[iIndexCounter];
                }

                if(CameraBufferData[iIndexCounter] > LastBufferMaxTemperature)
                {
                    LastBufferMaxTemperature = CameraBufferData[iIndexCounter];
                }
            }
        }

        //Record temperature from the middle of the frame
        LastBufferMiddleFramebufferTemperature = CameraBufferData[(CameraDimensionWidth * (CameraDimensionHeight / 2))  + (CameraDimensionWidth / 2)];
    }

}
