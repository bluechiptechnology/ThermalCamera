package bct.thermalcamera;

import java.io.File;
import java.io.FileDescriptor;
import java.io.RandomAccessFile;

//Thermal camera implementation for MLX90640
public class ThermalCameraMLX90640 extends ThermalCamera
{
    byte MLX90640SlaveAddress;
    String I2CBusPath;
    RandomAccessFile I2CBus;

    public ThermalCameraMLX90640(String i2cbusstr, byte i2cslaveaddress, int framerate)
    {
        super();

        I2CBusPath = i2cbusstr;
        MLX90640SlaveAddress = i2cslaveaddress;
        CameraFrameRate = framerate;

        CameraDimensionWidth = 32;
        CameraDimensionHeight = 24;
    }

    public void Initialise() throws Exception
    {
        File temp;
        boolean bReturnCode;

        AllocateFrameBuffer();

        if(I2CBus != null)
        {
            //return if the I2C bus has already been opened.
            return;
        }

        temp = new File(I2CBusPath);
        if(!temp.exists())
        {
            throw new Exception("I2C port " + temp.getName() + " does not exist");
        }
        /* Check access permission */
        if (!temp.canRead() || !temp.canWrite())
        {
            throw new Exception("I2C port " + temp.getName() + " is not accessible");
        }

        try
        {
            I2CBus = new RandomAccessFile(temp, "rw");

            bReturnCode = InitialiseMLX90640(I2CBus.getFD(), MLX90640SlaveAddress, CameraFrameRate);
            if(bReturnCode != true)
            {
                throw new Exception("Failed to initialise MLX90640");
            }
        }
        catch(Exception ex)
        {
            I2CBus = null;
            throw new Exception("Failed to open I2C bus. " + ex.getMessage());
        }


    }

    public void Capture() throws Exception
    {
        if(GetMLX90640Frame(I2CBus.getFD(), MLX90640SlaveAddress, CameraBufferData, CameraBufferData.length) == false)
        {
            throw new Exception("Failed to get MLX90640 frame");
        }
    }

    // JNI
    private native boolean InitialiseMLX90640(FileDescriptor fd,  byte i2cslaveaddress, int framerate);
    private native boolean GetMLX90640Frame(FileDescriptor fd, byte i2cslaveaddress, float framedata[], int length);

}


