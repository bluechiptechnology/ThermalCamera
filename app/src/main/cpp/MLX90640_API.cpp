/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"
#include "MLX90640_API_jni.h"
#include <math.h>
#include <cstring>

void ExtractVDDParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractPTATParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractGainParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractTgcParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractResolutionParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractKsTaParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractKsToParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractAlphaParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractOffsetParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractKtaPixelParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractKvPixelParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractCPParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
void ExtractCILCParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
int ExtractDeviatingPixels(uint16_t *eeData, paramsMLX90640 *mlx90640);
int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2);  
float GetMedian(float *values, int n);
int IsPixelBad(uint16_t pixel,paramsMLX90640 *params);
int ValidateFrameData(uint16_t *frameData);
int ValidateAuxData(uint16_t *auxData);

static const char *TAG="mlx90640_api";

#include "android/log.h"
#define LOGI(fmt, args...) __android_log_print(ANDROID_LOG_INFO,  TAG, fmt, ##args)
#define LOGD(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, TAG, fmt, ##args)
#define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_ERROR, TAG, fmt, ##args)

uint16_t eeMLX90640[832];
float mlx90640To[768];
paramsMLX90640 mlx90640;

int MLX90640_DumpEE(uint8_t slaveAddr, uint16_t *eeData)
{
     return MLX90640_I2CRead(slaveAddr, 0x2400, 832, eeData);
}

int MLX90640_SynchFrame(uint8_t slaveAddr)
{
    uint16_t dataReady = 0;
    uint16_t statusRegister;
    int error = 1;
    
    error = MLX90640_I2CWrite(slaveAddr, 0x8000, 0x0030);
    if(error == -1)
    {
        return error;
    }
    
    while(dataReady == 0)
    {
        error = MLX90640_I2CRead(slaveAddr, 0x8000, 1, &statusRegister);
        if(error != 0)
        {
            return error;
        }    
        dataReady = statusRegister & 0x0008;
    }      
    
   return 0;   
}

int MLX90640_TriggerMeasurement(uint8_t slaveAddr)
{
    int error = 1;
    uint16_t ctrlReg;
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &ctrlReg);
    
    if ( error != 0) 
    {
        return error;
    }    
                                                
    ctrlReg |= 0x8000;
    error = MLX90640_I2CWrite(slaveAddr, 0x800D, ctrlReg);
    
    if ( error != 0)
    {
        return error;
    }    
    
    error = MLX90640_I2CGeneralReset();
    
    if ( error != 0)
    {
        return error;
    }    
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &ctrlReg);
    
    if ( error != 0)
    {
        return error;
    }    
    
    if ((ctrlReg & 0x8000) != 0)
    {
        return -9;
    }
    
    return 0;    
}
    
int MLX90640_GetFrameData(uint8_t slaveAddr, uint16_t *frameData)
{
    uint16_t dataReady = 0;
    uint16_t controlRegister1;
    uint16_t statusRegister;
    int error = 1;
    uint16_t data[64];
    uint8_t cnt = 0;
    
    while(dataReady == 0)
    {
        error = MLX90640_I2CRead(slaveAddr, 0x8000, 1, &statusRegister);
        if(error != 0)
        {
            return error;
        }    
        dataReady = statusRegister & 0x0008;
    }      
    
    error = MLX90640_I2CWrite(slaveAddr, 0x8000, 0x0030);
    if(error == -1)
    {
        return error;
    }
                     
    error = MLX90640_I2CRead(slaveAddr, 0x0400, 768, frameData); 
    if(error != 0)
    {
        return error;
    }                       
    
    error = MLX90640_I2CRead(slaveAddr, 0x0700, 64, data); 
    if(error != 0)
    {
        return error;
    }     
        
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    frameData[832] = controlRegister1;
    frameData[833] = statusRegister & 0x0001;
    
    if(error != 0)
    {
        return error;
    }
    
    error = ValidateAuxData(data);
    if(error == 0)
    {
        for(cnt=0; cnt<64; cnt++)
        {
            frameData[cnt+768] = data[cnt];
        }
    }        
    
    error = ValidateFrameData(frameData);
    if (error != 0)
    {
        return error;
    }
    
    return frameData[833];    
}

int ValidateFrameData(uint16_t *frameData)
{
    uint8_t line = 0;
    
    for(int i=0; i<768; i+=32)
    {
        if((frameData[i] == 0x7FFF) && (line%2 == frameData[833])) return -8;
        line = line + 1;
    }    
        
    return 0;    
}

int ValidateAuxData(uint16_t *auxData)
{
    
    if(auxData[0] == 0x7FFF) return -8;    
    
    for(int i=8; i<19; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    for(int i=20; i<23; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    for(int i=24; i<33; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    for(int i=40; i<51; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    for(int i=52; i<55; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    for(int i=56; i<64; i++)
    {
        if(auxData[i] == 0x7FFF) return -8;
    }
    
    return 0;
    
}
    
int MLX90640_ExtractParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int error = 0;
    
    ExtractVDDParameters(eeData, mlx90640);
    ExtractPTATParameters(eeData, mlx90640);
    ExtractGainParameters(eeData, mlx90640);
    ExtractTgcParameters(eeData, mlx90640);
    ExtractResolutionParameters(eeData, mlx90640);
    ExtractKsTaParameters(eeData, mlx90640);
    ExtractKsToParameters(eeData, mlx90640);
    ExtractCPParameters(eeData, mlx90640);
    ExtractAlphaParameters(eeData, mlx90640);
    ExtractOffsetParameters(eeData, mlx90640);
    ExtractKtaPixelParameters(eeData, mlx90640);
    ExtractKvPixelParameters(eeData, mlx90640);
    ExtractCILCParameters(eeData, mlx90640);
    error = ExtractDeviatingPixels(eeData, mlx90640);  
    
    return error;

}

//------------------------------------------------------------------------------

int MLX90640_SetResolution(uint8_t slaveAddr, uint8_t resolution)
{
    uint16_t controlRegister1;
    int value;
    int error;
    
    value = (resolution & 0x03) << 10;
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    
    if(error == 0)
    {
        value = (controlRegister1 & 0xF3FF) | value;
        error = MLX90640_I2CWrite(slaveAddr, 0x800D, value);        
    }    
    
    return error;
}

//------------------------------------------------------------------------------

int MLX90640_GetCurResolution(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int resolutionRAM;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    if(error != 0)
    {
        return error;
    }    
    resolutionRAM = (controlRegister1 & 0x0C00) >> 10;
    
    return resolutionRAM; 
}

//------------------------------------------------------------------------------

int MLX90640_SetRefreshRate(uint8_t slaveAddr, uint8_t refreshRate)
{
    uint16_t controlRegister1;
    int value;
    int error;
    
    value = (refreshRate & 0x07)<<7;
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    if(error == 0)
    {
        value = (controlRegister1 & 0xFC7F) | value;
        error = MLX90640_I2CWrite(slaveAddr, 0x800D, value);
    }    
    
    return error;
}

//------------------------------------------------------------------------------

int MLX90640_GetRefreshRate(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int refreshRate;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    if(error != 0)
    {
        return error;
    }    
    refreshRate = (controlRegister1 & 0x0380) >> 7;
    
    return refreshRate;
}

//------------------------------------------------------------------------------

int MLX90640_SetInterleavedMode(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int value;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    
    if(error == 0)
    {
        value = (controlRegister1 & 0xEFFF);
        error = MLX90640_I2CWrite(slaveAddr, 0x800D, value);        
    }    
    
    return error;
}

//------------------------------------------------------------------------------

int MLX90640_SetChessMode(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int value;
    int error;
        
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    
    if(error == 0)
    {
        value = (controlRegister1 | 0x1000);
        error = MLX90640_I2CWrite(slaveAddr, 0x800D, value);        
    }    
    
    return error;
}

//------------------------------------------------------------------------------

int MLX90640_GetCurMode(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int modeRAM;
    int error;
    
    error = MLX90640_I2CRead(slaveAddr, 0x800D, 1, &controlRegister1);
    if(error != 0)
    {
        return error;
    }    
    modeRAM = (controlRegister1 & 0x1000) >> 12;
    
    return modeRAM; 
}

//------------------------------------------------------------------------------

void MLX90640_CalculateTo(uint16_t *frameData, const paramsMLX90640 *params, float emissivity, float tr, float *result)
{
    float vdd;
    float ta;
    float ta4;
    float tr4;
    float taTr;
    float gain;
    float irDataCP[2];
    float irData;
    float alphaCompensated;
    uint8_t mode;
    int8_t ilPattern;
    int8_t chessPattern;
    int8_t pattern;
    int8_t conversionPattern;
    float Sx;
    float To;
    float alphaCorrR[4];
    int8_t range;
    uint16_t subPage;
    float ktaScale;
    float kvScale;
    float alphaScale;
    float kta;
    float kv;
    
    subPage = frameData[833];
    vdd = MLX90640_GetVdd(frameData, params);
    ta = MLX90640_GetTa(frameData, params);
    
    ta4 = (ta + 273.15);
    ta4 = ta4 * ta4;
    ta4 = ta4 * ta4;
    tr4 = (tr + 273.15);
    tr4 = tr4 * tr4;
    tr4 = tr4 * tr4;
    taTr = tr4 - (tr4-ta4)/emissivity;
    
    ktaScale = pow(2,(double)params->ktaScale);
    kvScale = pow(2,(double)params->kvScale);
    alphaScale = pow(2,(double)params->alphaScale);
    
    alphaCorrR[0] = 1 / (1 + params->ksTo[0] * 40);
    alphaCorrR[1] = 1 ;
    alphaCorrR[2] = (1 + params->ksTo[1] * params->ct[2]);
    alphaCorrR[3] = alphaCorrR[2] * (1 + params->ksTo[2] * (params->ct[3] - params->ct[2]));
    
//------------------------- Gain calculation -----------------------------------    
    gain = frameData[778];
    if(gain > 32767)
    {
        gain = gain - 65536;
    }
    
    gain = params->gainEE / gain; 
  
//------------------------- To calculation -------------------------------------    
    mode = (frameData[832] & 0x1000) >> 5;
    
    irDataCP[0] = frameData[776];  
    irDataCP[1] = frameData[808];
    for( int i = 0; i < 2; i++)
    {
        if(irDataCP[i] > 32767)
        {
            irDataCP[i] = irDataCP[i] - 65536;
        }
        irDataCP[i] = irDataCP[i] * gain;
    }
    irDataCP[0] = irDataCP[0] - params->cpOffset[0] * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    if( mode ==  params->calibrationModeEE)
    {
        irDataCP[1] = irDataCP[1] - params->cpOffset[1] * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    }
    else
    {
      irDataCP[1] = irDataCP[1] - (params->cpOffset[1] + params->ilChessC[0]) * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    }

    for( int pixelNumber = 0; pixelNumber < 768; pixelNumber++)
    {
        ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2; 
        chessPattern = ilPattern ^ (pixelNumber - (pixelNumber/2)*2); 
        conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);
        
        if(mode == 0)
        {
          pattern = ilPattern; 
        }
        else 
        {
          pattern = chessPattern; 
        }               
        
        if(pattern == frameData[833])
        {    
            irData = frameData[pixelNumber];
            if(irData > 32767)
            {
                irData = irData - 65536;
            }
            irData = irData * gain;
            
            kta = params->kta[pixelNumber]/ktaScale;
            kv = params->kv[pixelNumber]/kvScale;
            irData = irData - params->offset[pixelNumber]*(1 + kta*(ta - 25))*(1 + kv*(vdd - 3.3));
            
            if(mode !=  params->calibrationModeEE)
            {
              irData = irData + params->ilChessC[2] * (2 * ilPattern - 1) - params->ilChessC[1] * conversionPattern; 
            }                       
    
            irData = irData - params->tgc * irDataCP[subPage];
            irData = irData / emissivity;
            
            alphaCompensated = SCALEALPHA*alphaScale/params->alpha[pixelNumber];
            alphaCompensated = alphaCompensated*(1 + params->KsTa * (ta - 25));
                        
            Sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr);
            Sx = sqrt(sqrt(Sx)) * params->ksTo[1];            
            
            To = sqrt(sqrt(irData/(alphaCompensated * (1 - params->ksTo[1] * 273.15) + Sx) + taTr)) - 273.15;                     
                    
            if(To < params->ct[1])
            {
                range = 0;
            }
            else if(To < params->ct[2])   
            {
                range = 1;            
            }   
            else if(To < params->ct[3])
            {
                range = 2;            
            }
            else
            {
                range = 3;            
            }      
            
            To = sqrt(sqrt(irData / (alphaCompensated * alphaCorrR[range] * (1 + params->ksTo[range] * (To - params->ct[range]))) + taTr)) - 273.15;
                        
            result[pixelNumber] = To;
        }
    }
}

//------------------------------------------------------------------------------

void MLX90640_GetImage(uint16_t *frameData, const paramsMLX90640 *params, float *result)
{
    float vdd;
    float ta;
    float gain;
    float irDataCP[2];
    float irData;
    float alphaCompensated;
    uint8_t mode;
    int8_t ilPattern;
    int8_t chessPattern;
    int8_t pattern;
    int8_t conversionPattern;
    float image;
    uint16_t subPage;
    float ktaScale;
    float kvScale;
    float kta;
    float kv;
    
    subPage = frameData[833];
    vdd = MLX90640_GetVdd(frameData, params);
    ta = MLX90640_GetTa(frameData, params);
    
    ktaScale = pow(2,(double)params->ktaScale);
    kvScale = pow(2,(double)params->kvScale);
    
//------------------------- Gain calculation -----------------------------------    
    gain = frameData[778];
    if(gain > 32767)
    {
        gain = gain - 65536;
    }
    
    gain = params->gainEE / gain; 
  
//------------------------- Image calculation -------------------------------------    
    mode = (frameData[832] & 0x1000) >> 5;
    
    irDataCP[0] = frameData[776];  
    irDataCP[1] = frameData[808];
    for( int i = 0; i < 2; i++)
    {
        if(irDataCP[i] > 32767)
        {
            irDataCP[i] = irDataCP[i] - 65536;
        }
        irDataCP[i] = irDataCP[i] * gain;
    }
    irDataCP[0] = irDataCP[0] - params->cpOffset[0] * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    if( mode ==  params->calibrationModeEE)
    {
        irDataCP[1] = irDataCP[1] - params->cpOffset[1] * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    }
    else
    {
      irDataCP[1] = irDataCP[1] - (params->cpOffset[1] + params->ilChessC[0]) * (1 + params->cpKta * (ta - 25)) * (1 + params->cpKv * (vdd - 3.3));
    }

    for( int pixelNumber = 0; pixelNumber < 768; pixelNumber++)
    {
        ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2; 
        chessPattern = ilPattern ^ (pixelNumber - (pixelNumber/2)*2); 
        conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern);
        
        if(mode == 0)
        {
          pattern = ilPattern; 
        }
        else 
        {
          pattern = chessPattern; 
        }
        
        if(pattern == frameData[833])
        {    
            irData = frameData[pixelNumber];
            if(irData > 32767)
            {
                irData = irData - 65536;
            }
            irData = irData * gain;
            
            kta = params->kta[pixelNumber]/ktaScale;
            kv = params->kv[pixelNumber]/kvScale;
            irData = irData - params->offset[pixelNumber]*(1 + kta*(ta - 25))*(1 + kv*(vdd - 3.3));

            if(mode !=  params->calibrationModeEE)
            {
              irData = irData + params->ilChessC[2] * (2 * ilPattern - 1) - params->ilChessC[1] * conversionPattern; 
            }
            
            irData = irData - params->tgc * irDataCP[subPage];
                        
            alphaCompensated = params->alpha[pixelNumber];
            
            image = irData*alphaCompensated;
            
            result[pixelNumber] = image;
        }
    }
}

//------------------------------------------------------------------------------

float MLX90640_GetVdd(uint16_t *frameData, const paramsMLX90640 *params)
{
    float vdd;
    float resolutionCorrection;

    int resolutionRAM;    
    
    vdd = frameData[810];
    if(vdd > 32767)
    {
        vdd = vdd - 65536;
    }
    resolutionRAM = (frameData[832] & 0x0C00) >> 10;
    resolutionCorrection = pow(2, (double)params->resolutionEE) / pow(2, (double)resolutionRAM);
    vdd = (resolutionCorrection * vdd - params->vdd25) / params->kVdd + 3.3;
    
    return vdd;
}

//------------------------------------------------------------------------------

float MLX90640_GetTa(uint16_t *frameData, const paramsMLX90640 *params)
{
    float ptat;
    float ptatArt;
    float vdd;
    float ta;
    
    vdd = MLX90640_GetVdd(frameData, params);
    
    ptat = frameData[800];
    if(ptat > 32767)
    {
        ptat = ptat - 65536;
    }
    
    ptatArt = frameData[768];
    if(ptatArt > 32767)
    {
        ptatArt = ptatArt - 65536;
    }
    ptatArt = (ptat / (ptat * params->alphaPTAT + ptatArt)) * pow(2, (double)18);
    
    ta = (ptatArt / (1 + params->KvPTAT * (vdd - 3.3)) - params->vPTAT25);
    ta = ta / params->KtPTAT + 25;
    
    return ta;
}

//------------------------------------------------------------------------------

int MLX90640_GetSubPageNumber(uint16_t *frameData)
{
    return frameData[833];    

}    

//------------------------------------------------------------------------------
void MLX90640_BadPixelsCorrection(uint16_t *pixels, float *to, int mode, paramsMLX90640 *params)
{   
    float ap[4];
    uint8_t pix;
    uint8_t line;
    uint8_t column;
    
    pix = 0;
    while(pixels[pix] != 0xFFFF)
    {
        line = pixels[pix]>>5;
        column = pixels[pix] - (line<<5);
        
        if(mode == 1)
        {        
            if(line == 0)
            {
                if(column == 0)
                {        
                    to[pixels[pix]] = to[33];                    
                }
                else if(column == 31)
                {
                    to[pixels[pix]] = to[62];                      
                }
                else
                {
                    to[pixels[pix]] = (to[pixels[pix]+31] + to[pixels[pix]+33])/2.0;                    
                }        
            }
            else if(line == 23)
            {
                if(column == 0)
                {
                    to[pixels[pix]] = to[705];                    
                }
                else if(column == 31)
                {
                    to[pixels[pix]] = to[734];                       
                }
                else
                {
                    to[pixels[pix]] = (to[pixels[pix]-33] + to[pixels[pix]-31])/2.0;                       
                }                       
            } 
            else if(column == 0)
            {
                to[pixels[pix]] = (to[pixels[pix]-31] + to[pixels[pix]+33])/2.0;                
            }
            else if(column == 31)
            {
                to[pixels[pix]] = (to[pixels[pix]-33] + to[pixels[pix]+31])/2.0;                
            } 
            else
            {
                ap[0] = to[pixels[pix]-33];
                ap[1] = to[pixels[pix]-31];
                ap[2] = to[pixels[pix]+31];
                ap[3] = to[pixels[pix]+33];
                to[pixels[pix]] = GetMedian(ap,4);
            }                   
        }
        else
        {        
            if(column == 0)
            {
                to[pixels[pix]] = to[pixels[pix]+1];            
            }
            else if(column == 1 || column == 30)
            {
                to[pixels[pix]] = (to[pixels[pix]-1]+to[pixels[pix]+1])/2.0;                
            } 
            else if(column == 31)
            {
                to[pixels[pix]] = to[pixels[pix]-1];
            } 
            else
            {
                if(IsPixelBad(pixels[pix]-2,params) == 0 && IsPixelBad(pixels[pix]+2,params) == 0)
                {
                    ap[0] = to[pixels[pix]+1] - to[pixels[pix]+2];
                    ap[1] = to[pixels[pix]-1] - to[pixels[pix]-2];
                    if(fabs(ap[0]) > fabs(ap[1]))
                    {
                        to[pixels[pix]] = to[pixels[pix]-1] + ap[1];                        
                    }
                    else
                    {
                        to[pixels[pix]] = to[pixels[pix]+1] + ap[0];                        
                    }
                }
                else
                {
                    to[pixels[pix]] = (to[pixels[pix]-1]+to[pixels[pix]+1])/2.0;                    
                }            
            }                      
        } 
        pix = pix + 1;    
    }    
}

//------------------------------------------------------------------------------

void ExtractVDDParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int16_t kVdd;
    int16_t vdd25;
    
    kVdd = eeData[51];
    
    kVdd = (eeData[51] & 0xFF00) >> 8;
    if(kVdd > 127)
    {
        kVdd = kVdd - 256;
    }
    kVdd = 32 * kVdd;
    vdd25 = eeData[51] & 0x00FF;
    vdd25 = ((vdd25 - 256) << 5) - 8192;
    
    mlx90640->kVdd = kVdd;
    mlx90640->vdd25 = vdd25; 
}

//------------------------------------------------------------------------------

void ExtractPTATParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    float KvPTAT;
    float KtPTAT;
    int16_t vPTAT25;
    float alphaPTAT;
    
    KvPTAT = (eeData[50] & 0xFC00) >> 10;
    if(KvPTAT > 31)
    {
        KvPTAT = KvPTAT - 64;
    }
    KvPTAT = KvPTAT/4096;
    
    KtPTAT = eeData[50] & 0x03FF;
    if(KtPTAT > 511)
    {
        KtPTAT = KtPTAT - 1024;
    }
    KtPTAT = KtPTAT/8;
    
    vPTAT25 = eeData[49];
    
    alphaPTAT = (eeData[16] & 0xF000) / pow(2, (double)14) + 8.0f;
    
    mlx90640->KvPTAT = KvPTAT;
    mlx90640->KtPTAT = KtPTAT;    
    mlx90640->vPTAT25 = vPTAT25;
    mlx90640->alphaPTAT = alphaPTAT;   
}

//------------------------------------------------------------------------------

void ExtractGainParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int16_t gainEE;
    
    gainEE = eeData[48];
    if(gainEE > 32767)
    {
        gainEE = gainEE -65536;
    }
    
    mlx90640->gainEE = gainEE;    
}

//------------------------------------------------------------------------------

void ExtractTgcParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    float tgc;
    tgc = eeData[60] & 0x00FF;
    if(tgc > 127)
    {
        tgc = tgc - 256;
    }
    tgc = tgc / 32.0f;
    
    mlx90640->tgc = tgc;        
}

//------------------------------------------------------------------------------

void ExtractResolutionParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    uint8_t resolutionEE;
    resolutionEE = (eeData[56] & 0x3000) >> 12;    
    
    mlx90640->resolutionEE = resolutionEE;
}

//------------------------------------------------------------------------------

void ExtractKsTaParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    float KsTa;
    KsTa = (eeData[60] & 0xFF00) >> 8;
    if(KsTa > 127)
    {
        KsTa = KsTa -256;
    }
    KsTa = KsTa / 8192.0f;
    
    mlx90640->KsTa = KsTa;
}

//------------------------------------------------------------------------------

void ExtractKsToParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int32_t KsToScale;
    int8_t step;
    
    step = ((eeData[63] & 0x3000) >> 12) * 10;
    
    mlx90640->ct[0] = -40;
    mlx90640->ct[1] = 0;
    mlx90640->ct[2] = (eeData[63] & 0x00F0) >> 4;
    mlx90640->ct[3] = (eeData[63] & 0x0F00) >> 8;    
    
    mlx90640->ct[2] = mlx90640->ct[2]*step;
    mlx90640->ct[3] = mlx90640->ct[2] + mlx90640->ct[3]*step;
    mlx90640->ct[4] = 400;
    
    KsToScale = (eeData[63] & 0x000F) + 8;
    KsToScale = 1UL << KsToScale;
    
    mlx90640->ksTo[0] = eeData[61] & 0x00FF;
    mlx90640->ksTo[1] = (eeData[61] & 0xFF00) >> 8;
    mlx90640->ksTo[2] = eeData[62] & 0x00FF;
    mlx90640->ksTo[3] = (eeData[62] & 0xFF00) >> 8;      
    
    for(int i = 0; i < 4; i++)
    {
        if(mlx90640->ksTo[i] > 127)
        {
            mlx90640->ksTo[i] = mlx90640->ksTo[i] - 256;
        }
        mlx90640->ksTo[i] = mlx90640->ksTo[i] / KsToScale;
    } 
    
    mlx90640->ksTo[4] = -0.0002;
}

//------------------------------------------------------------------------------

void ExtractAlphaParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int accRow[24];
    int accColumn[32];
    int p = 0;
    int alphaRef;
    uint8_t alphaScale;
    uint8_t accRowScale;
    uint8_t accColumnScale;
    uint8_t accRemScale;
    float alphaTemp[768];
    float temp;
    

    accRemScale = eeData[32] & 0x000F;
    accColumnScale = (eeData[32] & 0x00F0) >> 4;
    accRowScale = (eeData[32] & 0x0F00) >> 8;
    alphaScale = ((eeData[32] & 0xF000) >> 12) + 30;
    alphaRef = eeData[33];
    
    for(int i = 0; i < 6; i++)
    {
        p = i * 4;
        accRow[p + 0] = (eeData[34 + i] & 0x000F);
        accRow[p + 1] = (eeData[34 + i] & 0x00F0) >> 4;
        accRow[p + 2] = (eeData[34 + i] & 0x0F00) >> 8;
        accRow[p + 3] = (eeData[34 + i] & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 24; i++)
    {
        if (accRow[i] > 7)
        {
            accRow[i] = accRow[i] - 16;
        }
    }
    
    for(int i = 0; i < 8; i++)
    {
        p = i * 4;
        accColumn[p + 0] = (eeData[40 + i] & 0x000F);
        accColumn[p + 1] = (eeData[40 + i] & 0x00F0) >> 4;
        accColumn[p + 2] = (eeData[40 + i] & 0x0F00) >> 8;
        accColumn[p + 3] = (eeData[40 + i] & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 32; i ++)
    {
        if (accColumn[i] > 7)
        {
            accColumn[i] = accColumn[i] - 16;
        }
    }

    for(int i = 0; i < 24; i++)
    {
        for(int j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            alphaTemp[p] = (eeData[64 + p] & 0x03F0) >> 4;
            if (alphaTemp[p] > 31)
            {
                alphaTemp[p] = alphaTemp[p] - 64;
            }
            alphaTemp[p] = alphaTemp[p]*(1 << accRemScale);
            alphaTemp[p] = (alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) + alphaTemp[p]);
            alphaTemp[p] = alphaTemp[p] / pow(2,(double)alphaScale);
            alphaTemp[p] = alphaTemp[p] - mlx90640->tgc * (mlx90640->cpAlpha[0] + mlx90640->cpAlpha[1])/2;
            alphaTemp[p] = SCALEALPHA/alphaTemp[p];
        }
    }
    
    temp = alphaTemp[0];
    for(int i = 1; i < 768; i++)
    {
        if (alphaTemp[i] > temp)
        {
            temp = alphaTemp[i];
        }
    }
    
    alphaScale = 0;
    while(temp < 32767.4)
    {
        temp = temp*2;
        alphaScale = alphaScale + 1;
    } 
    
    for(int i = 0; i < 768; i++)
    {
        temp = alphaTemp[i] * pow(2,(double)alphaScale);        
        mlx90640->alpha[i] = (temp + 0.5);        
        
    } 
    
    mlx90640->alphaScale = alphaScale;      
   
}

//------------------------------------------------------------------------------

void ExtractOffsetParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int occRow[24];
    int occColumn[32];
    int p = 0;
    int16_t offsetRef;
    uint8_t occRowScale;
    uint8_t occColumnScale;
    uint8_t occRemScale;
    

    occRemScale = (eeData[16] & 0x000F);
    occColumnScale = (eeData[16] & 0x00F0) >> 4;
    occRowScale = (eeData[16] & 0x0F00) >> 8;
    offsetRef = eeData[17];
    if (offsetRef > 32767)
    {
        offsetRef = offsetRef - 65536;
    }
    
    for(int i = 0; i < 6; i++)
    {
        p = i * 4;
        occRow[p + 0] = (eeData[18 + i] & 0x000F);
        occRow[p + 1] = (eeData[18 + i] & 0x00F0) >> 4;
        occRow[p + 2] = (eeData[18 + i] & 0x0F00) >> 8;
        occRow[p + 3] = (eeData[18 + i] & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 24; i++)
    {
        if (occRow[i] > 7)
        {
            occRow[i] = occRow[i] - 16;
        }
    }
    
    for(int i = 0; i < 8; i++)
    {
        p = i * 4;
        occColumn[p + 0] = (eeData[24 + i] & 0x000F);
        occColumn[p + 1] = (eeData[24 + i] & 0x00F0) >> 4;
        occColumn[p + 2] = (eeData[24 + i] & 0x0F00) >> 8;
        occColumn[p + 3] = (eeData[24 + i] & 0xF000) >> 12;
    }
    
    for(int i = 0; i < 32; i ++)
    {
        if (occColumn[i] > 7)
        {
            occColumn[i] = occColumn[i] - 16;
        }
    }

    for(int i = 0; i < 24; i++)
    {
        for(int j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            mlx90640->offset[p] = (eeData[64 + p] & 0xFC00) >> 10;
            if (mlx90640->offset[p] > 31)
            {
                mlx90640->offset[p] = mlx90640->offset[p] - 64;
            }
            mlx90640->offset[p] = mlx90640->offset[p]*(1 << occRemScale);
            mlx90640->offset[p] = (offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + mlx90640->offset[p]);
        }
    }
}

//------------------------------------------------------------------------------

void ExtractKtaPixelParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int p = 0;
    int8_t KtaRC[4];
    int8_t KtaRoCo;
    int8_t KtaRoCe;
    int8_t KtaReCo;
    int8_t KtaReCe;
    uint8_t ktaScale1;
    uint8_t ktaScale2;
    uint8_t split;
    float ktaTemp[768];
    float temp;
    
    KtaRoCo = (eeData[54] & 0xFF00) >> 8;
    if (KtaRoCo > 127)
    {
        KtaRoCo = KtaRoCo - 256;
    }
    KtaRC[0] = KtaRoCo;
    
    KtaReCo = (eeData[54] & 0x00FF);
    if (KtaReCo > 127)
    {
        KtaReCo = KtaReCo - 256;
    }
    KtaRC[2] = KtaReCo;
      
    KtaRoCe = (eeData[55] & 0xFF00) >> 8;
    if (KtaRoCe > 127)
    {
        KtaRoCe = KtaRoCe - 256;
    }
    KtaRC[1] = KtaRoCe;
      
    KtaReCe = (eeData[55] & 0x00FF);
    if (KtaReCe > 127)
    {
        KtaReCe = KtaReCe - 256;
    }
    KtaRC[3] = KtaReCe;
  
    ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8;
    ktaScale2 = (eeData[56] & 0x000F);

    for(int i = 0; i < 24; i++)
    {
        for(int j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            split = 2*(p/32 - (p/64)*2) + p%2;
            ktaTemp[p] = (eeData[64 + p] & 0x000E) >> 1;
            if (ktaTemp[p] > 3)
            {
                ktaTemp[p] = ktaTemp[p] - 8;
            }
            ktaTemp[p] = ktaTemp[p] * (1 << ktaScale2);
            ktaTemp[p] = KtaRC[split] + ktaTemp[p];
            ktaTemp[p] = ktaTemp[p] / pow(2,(double)ktaScale1);
            //ktaTemp[p] = ktaTemp[p] * mlx90640->offset[p];
        }
    }
    
    temp = fabs(ktaTemp[0]);
    for(int i = 1; i < 768; i++)
    {
        if (fabs(ktaTemp[i]) > temp)
        {
            temp = fabs(ktaTemp[i]);
        }
    }
    
    ktaScale1 = 0;
    while(temp < 63.4)
    {
        temp = temp*2;
        ktaScale1 = ktaScale1 + 1;
    }    
     
    for(int i = 0; i < 768; i++)
    {
        temp = ktaTemp[i] * pow(2,(double)ktaScale1);
        if (temp < 0)
        {
            mlx90640->kta[i] = (temp - 0.5);
        }
        else
        {
            mlx90640->kta[i] = (temp + 0.5);
        }        
        
    } 
    
    mlx90640->ktaScale = ktaScale1;           
}


//------------------------------------------------------------------------------

void ExtractKvPixelParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    int p = 0;
    int8_t KvT[4];
    int8_t KvRoCo;
    int8_t KvRoCe;
    int8_t KvReCo;
    int8_t KvReCe;
    uint8_t kvScale;
    uint8_t split;
    float kvTemp[768];
    float temp;

    KvRoCo = (eeData[52] & 0xF000) >> 12;
    if (KvRoCo > 7)
    {
        KvRoCo = KvRoCo - 16;
    }
    KvT[0] = KvRoCo;
    
    KvReCo = (eeData[52] & 0x0F00) >> 8;
    if (KvReCo > 7)
    {
        KvReCo = KvReCo - 16;
    }
    KvT[2] = KvReCo;
      
    KvRoCe = (eeData[52] & 0x00F0) >> 4;
    if (KvRoCe > 7)
    {
        KvRoCe = KvRoCe - 16;
    }
    KvT[1] = KvRoCe;
      
    KvReCe = (eeData[52] & 0x000F);
    if (KvReCe > 7)
    {
        KvReCe = KvReCe - 16;
    }
    KvT[3] = KvReCe;
  
    kvScale = (eeData[56] & 0x0F00) >> 8;


    for(int i = 0; i < 24; i++)
    {
        for(int j = 0; j < 32; j ++)
        {
            p = 32 * i +j;
            split = 2*(p/32 - (p/64)*2) + p%2;
            kvTemp[p] = KvT[split];
            kvTemp[p] = kvTemp[p] / pow(2,(double)kvScale);
            //kvTemp[p] = kvTemp[p] * mlx90640->offset[p];
        }
    }
    
    temp = fabs(kvTemp[0]);
    for(int i = 1; i < 768; i++)
    {
        if (fabs(kvTemp[i]) > temp)
        {
            temp = fabs(kvTemp[i]);
        }
    }
    
    kvScale = 0;
    while(temp < 63.4)
    {
        temp = temp*2;
        kvScale = kvScale + 1;
    }    
     
    for(int i = 0; i < 768; i++)
    {
        temp = kvTemp[i] * pow(2,(double)kvScale);
        if (temp < 0)
        {
            mlx90640->kv[i] = (temp - 0.5);
        }
        else
        {
            mlx90640->kv[i] = (temp + 0.5);
        }        
        
    } 
    
    mlx90640->kvScale = kvScale;        
}

//------------------------------------------------------------------------------

void ExtractCPParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    float alphaSP[2];
    int16_t offsetSP[2];
    float cpKv;
    float cpKta;
    uint8_t alphaScale;
    uint8_t ktaScale1;
    uint8_t kvScale;

    alphaScale = ((eeData[32] & 0xF000) >> 12) + 27;
    
    offsetSP[0] = (eeData[58] & 0x03FF);
    if (offsetSP[0] > 511)
    {
        offsetSP[0] = offsetSP[0] - 1024;
    }
    
    offsetSP[1] = (eeData[58] & 0xFC00) >> 10;
    if (offsetSP[1] > 31)
    {
        offsetSP[1] = offsetSP[1] - 64;
    }
    offsetSP[1] = offsetSP[1] + offsetSP[0]; 
    
    alphaSP[0] = (eeData[57] & 0x03FF);
    if (alphaSP[0] > 511)
    {
        alphaSP[0] = alphaSP[0] - 1024;
    }
    alphaSP[0] = alphaSP[0] /  pow(2,(double)alphaScale);
    
    alphaSP[1] = (eeData[57] & 0xFC00) >> 10;
    if (alphaSP[1] > 31)
    {
        alphaSP[1] = alphaSP[1] - 64;
    }
    alphaSP[1] = (1 + alphaSP[1]/128) * alphaSP[0];
    
    cpKta = (eeData[59] & 0x00FF);
    if (cpKta > 127)
    {
        cpKta = cpKta - 256;
    }
    ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8;    
    mlx90640->cpKta = cpKta / pow(2,(double)ktaScale1);
    
    cpKv = (eeData[59] & 0xFF00) >> 8;
    if (cpKv > 127)
    {
        cpKv = cpKv - 256;
    }
    kvScale = (eeData[56] & 0x0F00) >> 8;
    mlx90640->cpKv = cpKv / pow(2,(double)kvScale);
       
    mlx90640->cpAlpha[0] = alphaSP[0];
    mlx90640->cpAlpha[1] = alphaSP[1];
    mlx90640->cpOffset[0] = offsetSP[0];
    mlx90640->cpOffset[1] = offsetSP[1];  
}

//------------------------------------------------------------------------------

void ExtractCILCParameters(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    float ilChessC[3];
    uint8_t calibrationModeEE;
    
    calibrationModeEE = (eeData[10] & 0x0800) >> 4;
    calibrationModeEE = calibrationModeEE ^ 0x80;

    ilChessC[0] = (eeData[53] & 0x003F);
    if (ilChessC[0] > 31)
    {
        ilChessC[0] = ilChessC[0] - 64;
    }
    ilChessC[0] = ilChessC[0] / 16.0f;
    
    ilChessC[1] = (eeData[53] & 0x07C0) >> 6;
    if (ilChessC[1] > 15)
    {
        ilChessC[1] = ilChessC[1] - 32;
    }
    ilChessC[1] = ilChessC[1] / 2.0f;
    
    ilChessC[2] = (eeData[53] & 0xF800) >> 11;
    if (ilChessC[2] > 15)
    {
        ilChessC[2] = ilChessC[2] - 32;
    }
    ilChessC[2] = ilChessC[2] / 8.0f;
    
    mlx90640->calibrationModeEE = calibrationModeEE;
    mlx90640->ilChessC[0] = ilChessC[0];
    mlx90640->ilChessC[1] = ilChessC[1];
    mlx90640->ilChessC[2] = ilChessC[2];
}

//------------------------------------------------------------------------------

int ExtractDeviatingPixels(uint16_t *eeData, paramsMLX90640 *mlx90640)
{
    uint16_t pixCnt = 0;
    uint16_t brokenPixCnt = 0;
    uint16_t outlierPixCnt = 0;
    int warn = 0;
    int i;
    
    for(pixCnt = 0; pixCnt<5; pixCnt++)
    {
        mlx90640->brokenPixels[pixCnt] = 0xFFFF;
        mlx90640->outlierPixels[pixCnt] = 0xFFFF;
    }
        
    pixCnt = 0;    
    while (pixCnt < 768 && brokenPixCnt < 5 && outlierPixCnt < 5)
    {
        if(eeData[pixCnt+64] == 0)
        {
            mlx90640->brokenPixels[brokenPixCnt] = pixCnt;
            brokenPixCnt = brokenPixCnt + 1;
        }    
        else if((eeData[pixCnt+64] & 0x0001) != 0)
        {
            mlx90640->outlierPixels[outlierPixCnt] = pixCnt;
            outlierPixCnt = outlierPixCnt + 1;
        }    
        
        pixCnt = pixCnt + 1;
        
    } 
    
    if(brokenPixCnt > 4)  
    {
        warn = -3;
    }         
    else if(outlierPixCnt > 4)  
    {
        warn = -4;
    }
    else if((brokenPixCnt + outlierPixCnt) > 4)  
    {
        warn = -5;
    } 
    else
    {
        for(pixCnt=0; pixCnt<brokenPixCnt; pixCnt++)
        {
            for(i=pixCnt+1; i<brokenPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->brokenPixels[pixCnt],mlx90640->brokenPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }    
            }    
        }
        
        for(pixCnt=0; pixCnt<outlierPixCnt; pixCnt++)
        {
            for(i=pixCnt+1; i<outlierPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->outlierPixels[pixCnt],mlx90640->outlierPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }    
            }    
        } 
        
        for(pixCnt=0; pixCnt<brokenPixCnt; pixCnt++)
        {
            for(i=0; i<outlierPixCnt; i++)
            {
                warn = CheckAdjacentPixels(mlx90640->brokenPixels[pixCnt],mlx90640->outlierPixels[i]);
                if(warn != 0)
                {
                    return warn;
                }    
            }    
        }    
        
    }    
    
    
    return warn;
       
}

//------------------------------------------------------------------------------

 int CheckAdjacentPixels(uint16_t pix1, uint16_t pix2)
 {
     int pixPosDif;
     
     pixPosDif = pix1 - pix2;
     if(pixPosDif > -34 && pixPosDif < -30)
     {
         return -6;
     } 
     if(pixPosDif > -2 && pixPosDif < 2)
     {
         return -6;
     } 
     if(pixPosDif > 30 && pixPosDif < 34)
     {
         return -6;
     }
     
     return 0;    
 }
 
//------------------------------------------------------------------------------
 
float GetMedian(float *values, int n)
 {
    float temp;
    
    for(int i=0; i<n-1; i++)
    {
        for(int j=i+1; j<n; j++)
        {
            if(values[j] < values[i]) 
            {                
                temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }
    
    if(n%2==0) 
    {
        return ((values[n/2] + values[n/2 - 1]) / 2.0);
        
    } 
    else 
    {
        return values[n/2];
    }
    
 }           

//------------------------------------------------------------------------------

int IsPixelBad(uint16_t pixel,paramsMLX90640 *params)
{
    for(int i=0; i<5; i++)
    {
        if(pixel == params->outlierPixels[i] || pixel == params->brokenPixels[i])
        {
            return 1;
        }    
    }   
    
    return 0;     
}     

//------------------------------------------------------------------------------

JNIEXPORT jboolean JNICALL Java_bct_thermalcamera_ThermalCameraMLX90640_InitialiseMLX90640(JNIEnv * env, jobject thiz, jobject filedescriptor, jbyte i2cslaveaddress, jint framerate)
{
    int status;
    jint fd = -1;
    jclass fdClass = (*env).FindClass("java/io/FileDescriptor");

    //Test EEPROM data
    //uint16_t eeMLX90640[832] = { 0x00AE,0x499A,0x0000,0x2061,0x0005,0x0320,0x03E0,0x1710,0xA224,0x0185,0x0499,0x0000,0x1901,0x0000,0x0000,0xB533,0x4210,0xFFC2,0x0202,0x0202,0xF202,0xF1F2,0xD1E1,0xAFC0,0xFF00,0xF002,0xF103,0xE103,0xE1F5,0xD1E4,0xC1D5,0x91C2,0x8895,0x30D9,0xEDCB,0x110F,0x3322,0x2233,0x0011,0xCCEE,0xFFED,0x1100,0x2222,0x3333,0x2233,0x0022,0xDEF0,0x9ACC,0x15CC,0x2FA4,0x2555,0x9C78,0x7666,0x01C8,0x3B38,0x3534,0x2452,0x0463,0x13BB,0x0623,0xEC00,0x9797,0x9797,0x2AFB,0x00AE,0xFBE0,0x1B70,0xF3BE,0x000E,0xF86E,0x1B7E,0xF3CE,0xFFCE,0xF41E,0x102E,0xEC0E,0xFFDE,0xEC3E,0x139E,0xEF9E,0xFB9E,0xF77E,0x13E0,0xE7EE,0xF7AE,0xF750,0x0C30,0xEBEE,0xF730,0xF010,0x0B50,0xE430,0xF420,0xF370,0x07C0,0xE450,0x0470,0xFBCE,0xFF5C,0x0F90,0x07D0,0xFC3E,0xFF6C,0x0F90,0x03A0,0xFC0E,0xF40C,0x0BF0,0x03A0,0xF41E,0xF78C,0x0B70,0xFF72,0xFF6E,0xF7DE,0x07C0,0xFFA2,0x0330,0xF42E,0x0BC0,0xFF22,0xFC00,0xF75E,0x0410,0x0022,0x0350,0xF3A0,0x0832,0x04DE,0xFBF0,0x1BCE,0xF00E,0xFC5E,0xFC80,0x1BF0,0xF02E,0x0002,0xF81E,0x142E,0xEC9E,0x07DE,0xF09E,0x17CE,0xF3AE,0xFFC0,0xFBB0,0x1080,0xEBFE,0xFFE0,0xFF90,0x1460,0xE4AE,0xFBC0,0xF840,0x0FE0,0xE860,0xF8C0,0xF400,0x0842,0xE4B0,0x0890,0x03BE,0xFF9C,0x0FD0,0x0020,0x0450,0xFFCC,0x0FE0,0x07D0,0x03FE,0xFBEE,0x0C60,0x0B80,0xF86E,0xFB8E,0x1370,0x0782,0x038E,0xF85E,0x0FC2,0x07C2,0x037E,0xF84E,0x0880,0x0392,0x0420,0xF7CE,0x0C42,0xFCB2,0xFFE0,0xF020,0x0490,0x107E,0x03D0,0x1F90,0xFBCE,0x089E,0x0080,0x1820,0xF40E,0x0800,0xFC30,0x141E,0xF06E,0x0400,0xFFA0,0x17CE,0xF7B0,0x07D0,0xFFB0,0x1830,0xF3FE,0x0002,0xFFE0,0x14D0,0xECB0,0xFBE2,0xFCB0,0x13B0,0xECA0,0xF8DE,0xF432,0x0832,0xE8D0,0x1420,0xFF8E,0xFF6E,0x1380,0x0840,0x005E,0xFBEC,0x0FB0,0x0BB2,0xFFFE,0xFBDE,0x0820,0x0BC0,0x0360,0xFB8C,0x0F70,0x0794,0x036E,0xFBFE,0x0FA0,0x0BC4,0x0390,0xF89E,0x0C72,0xFFB2,0xFC70,0xFB7E,0x0470,0xFCB0,0xFFF0,0xF3F0,0x04A0,0x049E,0x03B0,0x1F90,0xF7D0,0x042E,0x0070,0x1F70,0xFBBE,0x0F00,0x03B0,0x142E,0xF01E,0x07B0,0xFFB0,0x1B60,0xF37E,0xFBD0,0xFF90,0x1410,0xF3C0,0xFC00,0x0370,0x1482,0xF030,0xF800,0xFC50,0x13C2,0xF050,0x0070,0xF812,0x0C02,0xEC80,0x00D0,0xFBFE,0xFBCC,0x0810,0xFC60,0xFCB0,0xFBCE,0x0FE0,0x0B40,0xFFFE,0xF05C,0x0840,0x07D0,0xFFD0,0xF79E,0x0FB0,0xF802,0xFFD0,0xF44E,0x0BF0,0xFC32,0x07A0,0xF4BE,0x0C60,0xF822,0x0080,0xF01E,0x0892,0x00B4,0xF850,0xF040,0x04B2,0x085E,0x0782,0x1F70,0xFBEE,0x001E,0x0420,0x1F80,0xFBB0,0x03B0,0x0390,0x17F0,0xF04E,0x0770,0xFFE0,0x1B40,0xF76E,0xFFC0,0xFFB0,0x17E0,0xEC1E,0x03A0,0x03A0,0x10C0,0xEC60,0xFBC2,0xFC80,0x0C00,0xEC60,0x0050,0xF800,0x0802,0xEC90,0x0080,0xF7B0,0xF7AE,0x0410,0xFC32,0xFC50,0xF7BE,0x07F0,0xFFD2,0xFBC0,0xF02E,0x0460,0x0382,0xF410,0xF36E,0x0BA0,0xFBF2,0xFBC0,0xF01C,0x0440,0xFFE2,0xFBE0,0xF0EE,0x08A2,0xF804,0xFCB0,0xEC3E,0x04A2,0x0082,0xF830,0xE830,0x04B2,0x13F0,0x0380,0x1F40,0xFBB0,0x0F90,0x0420,0x17A0,0xF7AE,0x0F40,0xFFE2,0x13AE,0xF03E,0x0F12,0xFF60,0x0F50,0xF340,0x0362,0xFF30,0x1760,0xEFD0,0x0762,0x0360,0x1072,0xEC50,0xF7B2,0xF852,0x07B0,0xE480,0xF820,0xF7C2,0x03C2,0xE490,0x1422,0x03AE,0x036E,0x13C2,0x13B2,0x0440,0xFFCE,0x13D2,0x1362,0x0002,0xFBDE,0x0C40,0x1732,0x0390,0xFF8E,0x1760,0x0B82,0x0750,0x039E,0x1000,0x0F82,0x0B80,0xFCAE,0x1080,0x0BD4,0x0470,0xFBCE,0x0C92,0x0832,0x07E0,0xF7FE,0x0CA2,0x0010,0x0380,0x13D0,0xF7A0,0xFFBE,0x0052,0x1380,0xF770,0xFF70,0xFFA0,0x0FC0,0xF3BE,0x0340,0xFF60,0x0FC0,0xF370,0xFB30,0xFB80,0x0C10,0xE40E,0xFBA0,0xFBB0,0x0C42,0xE860,0xFB92,0xF4A2,0x0B82,0xE850,0xF832,0xFBA2,0x0002,0xE470,0x0022,0xF7A0,0xEFFE,0x0BC0,0x03D2,0xF860,0xF79E,0x0F92,0x0390,0xFFB0,0xF3FE,0x0FC0,0x0762,0xFF70,0xEFFE,0x1380,0x0362,0xFFB0,0xF42E,0x0810,0x07A2,0x07C0,0xF87E,0x0C82,0x0B94,0x0490,0xFB90,0x1062,0x0842,0x07B0,0xEC10,0x0C82,0x0850,0x13E2,0x2360,0x0420,0x0460,0x10B0,0x1FB0,0x03E0,0x0B80,0x0BF0,0x1430,0xFC00,0x0F90,0x0BC2,0x1BA0,0xFFC0,0x07C2,0x0B82,0x1BF0,0xF44E,0x0BB2,0x0FD2,0x14C2,0xF8A0,0x0792,0x0852,0x13E2,0xF850,0x00A0,0x0032,0x0C22,0xF0D0,0xF452,0xEFE0,0xEF7E,0xFC32,0xF072,0xF4C0,0xEBCE,0x03F0,0xFBA2,0xF400,0xE45E,0x0410,0xFFA2,0xF7D0,0xEBBE,0x0BD0,0xFBC2,0xFB80,0xF00E,0x0050,0x03D2,0x03D0,0xF0E0,0x0CA0,0x0384,0x0440,0xF3EE,0x0C52,0x00A2,0x0030,0xEC20,0x04C0,0x1022,0x0FD2,0x1F80,0x03F0,0x0830,0x0C82,0x17E0,0xFFB0,0x0410,0x0432,0x0870,0xF48E,0x0BD0,0x07B2,0x0F90,0xFBB0,0xFFF0,0x07A2,0x1410,0xF410,0x0022,0x0BC2,0x0CE0,0xF850,0xFFB2,0x0490,0x0BC0,0xECC0,0xFC70,0x0012,0x0400,0xF0B2,0x0402,0xF7D0,0xF37E,0x0BF2,0x0022,0xFC90,0xEFFE,0x0FC2,0xFC12,0xF84E,0xE87E,0x0480,0x07E2,0xFFB0,0xF7AE,0x0FC0,0x0002,0x07A0,0xF81E,0x1002,0x0422,0x0FD0,0xF8CE,0x1842,0x07A4,0x0880,0xFBB0,0x0CB0,0x0C62,0x0BF0,0xFBF0,0x10A0,0xF030,0x07D2,0x0BE0,0xF800,0xECA0,0x0482,0x0830,0xFBE0,0xF040,0xFC80,0x0810,0xF030,0xF410,0xF830,0x0BA0,0xF7A0,0xF3D2,0xFFF2,0x0840,0xEFF0,0xF400,0x03B2,0x0872,0xF030,0xEFB2,0x0042,0x03B2,0xEC40,0xFFE0,0xFFE2,0x0012,0xF420,0xF422,0xF7B0,0xE7CE,0x0BD2,0xF080,0x0070,0xEC2E,0x0FE2,0xF850,0x0070,0xF00E,0x0C42,0x0020,0x0030,0xF7AE,0x17B2,0x03D2,0x0400,0xF84E,0x17F0,0x0BE2,0x13A0,0xFC4E,0x1820,0x0792,0x1020,0xFB9E,0x1C10,0x1BC2,0x13C0,0xFBE0,0x2002,0xF040,0x13A2,0x0F80,0xFC30,0xF46E,0x0CC2,0x17B2,0x0010,0xFC10,0x0872,0x1000,0xF8B0,0x07BE,0x0BE2,0x13B0,0xFFE0,0xF410,0x0450,0x0C70,0xF420,0x03C0,0x0F82,0x1060,0xFFE0,0xFB70,0x13D2,0x0F90,0xF820,0xFC40,0x0FA2,0x0BE2,0xFC60,0xF012,0xFB80,0xEB5E,0x0802,0xF420,0x0090,0xF78E,0x13E2,0xFC02,0x0060,0xF40E,0x1090,0x0F90,0x0BD0,0xFBAE,0x1FD2,0x0002,0x0820,0xF85E,0x1800,0x0F82,0x1B60,0xFC3E,0x23C2,0x0B42,0x1BA0,0xFF7E,0x27E0,0x1012,0x1B70,0xFFC0,0x2040,0xFC70,0x1BA2,0x0FA0,0x0BA0,0x0002,0x1432,0x0FE0,0x0010,0xF83E,0x13E0,0x085E,0x07E0,0x005E,0x0842,0x0FEE,0x03D0,0xFC20,0x0FE2,0x1400,0x0780,0x0B90,0x1772,0x1410,0x07B0,0xFB10,0x17F2,0x0B20,0x03F0,0xFC1E,0x17B2,0x07CE,0x0830,0xE050,0xEF80,0xD38E,0x0382,0xEBE0,0xF810,0xDFBE,0x07D0,0xEC10,0xFFC0,0xE01E,0x0BB0,0xF820,0xF810,0xEBBE,0x0BA0,0xFBF0,0x07A0,0xF3EE,0x1B50,0x0752,0x0F30,0xF7EE,0x1B80,0x02F2,0x0FD0,0xF70E,0x13C0,0x0BE0,0x1390,0xF79E,0x1C00 };

    if (fdClass != NULL) {
        jfieldID fdClassDescriptorFieldID = (*env).GetFieldID(fdClass, "descriptor", "I");
        if (fdClassDescriptorFieldID != NULL && filedescriptor != NULL) {
            fd = (*env).GetIntField(filedescriptor, fdClassDescriptorFieldID);
        }
    }

    MLX90640_I2CInit(fd, i2cslaveaddress);

    status = MLX90640_DumpEE(i2cslaveaddress, eeMLX90640);
    if (status != 0)
    {
        LOGE("Failed to load system parameters");
        return JNI_FALSE;
    }

    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    if (status != 0)
    {
        LOGE("Parameter extraction failed");
        return JNI_FALSE;
    }

    if(framerate <= 0)
    {
        MLX90640_SetRefreshRate(i2cslaveaddress, 0x00); //Set rate to 0.05Hz
    }
    else if(framerate < 2)
    {
        MLX90640_SetRefreshRate(i2cslaveaddress, 0x01); //Set rate to 1Hz
    }
    else if(framerate < 4)
    {
        MLX90640_SetRefreshRate(i2cslaveaddress, 0x02); //Set rate to 2Hz
    }
    else if(framerate < 8)
    {
        MLX90640_SetRefreshRate(i2cslaveaddress, 0x03); //Set rate to 4Hz
    }
    else if(framerate < 16)
    {
        MLX90640_SetRefreshRate(i2cslaveaddress, 0x04); //Set rate to 8Hz
    }
    else if(framerate < 32)
    {
        MLX90640_SetRefreshRate(i2cslaveaddress, 0x05); //Set rate to 16Hz
    }
    else if(framerate < 64)
    {
        MLX90640_SetRefreshRate(i2cslaveaddress, 0x06); //Set rate to 32Hz
    }
    else
    {
        MLX90640_SetRefreshRate(i2cslaveaddress, 0x07); //Set rate to 64Hz
    }

    return JNI_TRUE;
}

JNIEXPORT jboolean JNICALL Java_bct_thermalcamera_ThermalCameraMLX90640_GetMLX90640Frame(JNIEnv * env, jobject thiz, jobject filedescriptor, jbyte i2cslaveaddress,  jfloatArray framedata, jint arraylength) {
    jint fd = -1;
    int iIndexCounter;
    jint ret;
    int status;

    //Test frame data
    //uint16_t mlx90640Frame[834] = { 0xFFB3,0xFFAC,0xFFB4,0xFFAA,0xFFB3,0xFFAC,0xFFB6,0xFFA9,0xFFB2,0xFFA8,0xFFB4,0xFFA6,0xFFB1,0xFFA5,0xFFB4,0xFFA2,0xFFB4,0xFFA5,0xFFB4,0xFFA4,0xFFB6,0xFFA7,0xFFB5,0xFFA4,0xFFBA,0xFFA6,0xFFB8,0xFFA5,0xFFB6,0xFFAA,0xFFBD,0xFFA4,0xFFA9,0xFFA8,0xFFA6,0xFFA8,0xFFA9,0xFFA6,0xFFA6,0xFFA5,0xFFAA,0xFFA2,0xFFA5,0xFFA2,0xFFA9,0xFF9F,0xFFA5,0xFFA1,0xFFAD,0xFFA0,0xFFA6,0xFFA2,0xFFAB,0xFFA3,0xFFA8,0xFFA4,0xFFB2,0xFFA2,0xFFAB,0xFFA3,0xFFB0,0xFFA4,0xFFAF,0xFFA3,0xFFB2,0xFFAC,0xFFB4,0xFFAB,0xFFB1,0xFFAC,0xFFB1,0xFFA8,0xFFB1,0xFFA9,0xFFB3,0xFFA5,0xFFB2,0xFFA5,0xFFB1,0xFFA3,0xFFB5,0xFFA5,0xFFB1,0xFFA3,0xFFB6,0xFFA7,0xFFB5,0xFFA2,0xFFB8,0xFFA7,0xFFB7,0xFFA3,0xFFB6,0xFFA8,0xFFB9,0xFFA2,0xFFA7,0xFFA7,0xFFA4,0xFFA7,0xFFA9,0xFFA5,0xFFA4,0xFFA6,0xFFA8,0xFFA3,0xFFA4,0xFFA2,0xFFAC,0xFF9F,0xFFA2,0xFFA3,0xFFAD,0xFFA0,0xFFA4,0xFFA2,0xFFAC,0xFFA2,0xFFA8,0xFFA1,0xFFB1,0xFFA3,0xFFA8,0xFFA3,0xFFAD,0xFFA1,0xFFAC,0xFFA1,0xFFB3,0xFFAD,0xFFB5,0xFFA9,0xFFB2,0xFFAB,0xFFB2,0xFFA8,0xFFB4,0xFFA9,0xFFB1,0xFFA4,0xFFB1,0xFFA6,0xFFB3,0xFFA2,0xFFB5,0xFFA7,0xFFB2,0xFFA3,0xFFB2,0xFFA5,0xFFB4,0xFFA2,0xFFB6,0xFFA5,0xFFB8,0xFFA5,0xFFB4,0xFFA7,0xFFB9,0xFFA0,0xFFAA,0xFFA5,0xFFA4,0xFFA6,0xFFAA,0xFFA4,0xFFA2,0xFFA4,0xFFA9,0xFFA1,0xFFA2,0xFFA3,0xFFAA,0xFFA1,0xFFA3,0xFFA0,0xFFAD,0xFF9F,0xFFA4,0xFFA3,0xFFAB,0xFFA0,0xFFA3,0xFFA1,0xFFAD,0xFF9E,0xFFA9,0xFFA1,0xFFAB,0xFFA0,0xFFAA,0xFF9D,0xFFB0,0xFFAD,0xFFB2,0xFFAA,0xFFB2,0xFFAB,0xFFB3,0xFFA9,0xFFB8,0xFFAA,0xFFB6,0xFFA4,0xFFB2,0xFFA6,0xFFB1,0xFFA4,0xFFB2,0xFFA4,0xFFB2,0xFFA4,0xFFB2,0xFFA7,0xFFB5,0xFFA4,0xFFB2,0xFFA5,0xFFB4,0xFFA2,0xFFB4,0xFFA6,0xFFB9,0xFFA2,0xFFA5,0xFFA4,0xFFA1,0xFFA4,0xFFA6,0xFFA0,0xFFA1,0xFFA3,0xFFA8,0xFFA7,0xFFA1,0xFFAA,0xFFA6,0xFFA1,0xFFA1,0xFFA1,0xFFA8,0xFF9D,0xFFA2,0xFF9F,0xFFA8,0xFF9F,0xFFA2,0xFF9F,0xFFAB,0xFF9E,0xFFA4,0xFFA0,0xFFAB,0xFF9F,0xFFA8,0xFF9B,0xFFAF,0xFFAE,0xFFB3,0xFFA9,0xFFAF,0xFFAB,0xFFB4,0xFFA8,0xFFBB,0xFFAA,0xFFC6,0xFFA6,0xFFC0,0xFFA8,0xFFB6,0xFFA2,0xFFB0,0xFFA5,0xFFB1,0xFFA2,0xFFB3,0xFFA6,0xFFB2,0xFFA2,0xFFB4,0xFFA2,0xFFB4,0xFFA2,0xFFB4,0xFFA6,0xFFB7,0xFFA1,0xFFA3,0xFFA2,0xFF9F,0xFFA1,0xFFA5,0xFFA2,0xFFA2,0xFFA4,0xFFAA,0xFFB2,0xFFA4,0xFFB4,0xFFA8,0xFFAD,0xFFA1,0xFFA6,0xFFA8,0xFF9D,0xFFA2,0xFF9D,0xFFAB,0xFF9D,0xFFA3,0xFF9F,0xFFAD,0xFF9C,0xFFA3,0xFF9F,0xFFAB,0xFF9D,0xFFA6,0xFF9C,0xFFB3,0xFFAD,0xFFB3,0xFFA9,0xFFB4,0xFFAC,0xFFB5,0xFFA9,0xFFC8,0xFFAE,0xFFC8,0xFFAB,0xFFC9,0xFFAE,0xFFC2,0xFFA6,0xFFBD,0xFFA9,0xFFB5,0xFFA8,0xFFB2,0xFFAC,0xFFB0,0xFFA4,0xFFB3,0xFFA4,0xFFB2,0xFFA1,0xFFB2,0xFFA4,0xFFB4,0xFF9C,0xFFA4,0xFFA1,0xFF9F,0xFFA2,0xFFA7,0xFFA1,0xFFA2,0xFFAA,0xFFAD,0xFFB7,0xFFA7,0xFFB8,0xFFAD,0xFFB5,0xFFA3,0xFFB1,0xFFAF,0xFFA8,0xFFAA,0xFFA0,0xFFB2,0xFF9B,0xFFA5,0xFF9D,0xFFAC,0xFF9A,0xFFA4,0xFF9D,0xFFAB,0xFF9C,0xFFA5,0xFF9A,0xFFAE,0xFFAD,0xFFAD,0xFFA8,0xFFB1,0xFFAD,0xFFBB,0xFFAE,0xFFCB,0xFFB2,0xFFCE,0xFFAE,0xFFCB,0xFFB0,0xFFC5,0xFFAB,0xFFC6,0xFFB2,0xFFBD,0xFFB0,0xFFB2,0xFFB1,0xFFB0,0xFFA8,0xFFB3,0xFFAA,0xFFB1,0xFFA4,0xFFB1,0xFFA5,0xFFB4,0xFF9D,0xFF9F,0xFF9E,0xFF98,0xFF9F,0xFFA3,0xFFA0,0xFFA2,0xFFB3,0xFFAC,0xFFBB,0xFFA7,0xFFBF,0xFFAE,0xFFB7,0xFFA5,0xFFB3,0xFFB4,0xFFB0,0xFFAE,0xFFA6,0xFFB6,0xFF9B,0xFFAB,0xFF99,0xFFB4,0xFF9A,0xFFA7,0xFF9D,0xFFAC,0xFF9D,0xFFA5,0xFF9A,0xFFAC,0xFFAD,0xFFB0,0xFFA8,0xFFB1,0xFFAD,0xFFC3,0xFFAF,0xFFCA,0xFFB3,0xFFCD,0xFFAD,0xFFCA,0xFFB0,0xFFC8,0xFFAD,0xFFC6,0xFFB8,0xFFB9,0xFFB1,0xFFB2,0xFFB5,0xFFAF,0xFFAD,0xFFB3,0xFFAF,0xFFB0,0xFFA9,0xFFB0,0xFFA6,0xFFB4,0xFF9D,0xFF9E,0xFF9C,0xFF9C,0xFF9F,0xFFA1,0xFFA4,0xFFA0,0xFFB3,0xFFAB,0xFFB5,0xFFA4,0xFFB9,0xFFAC,0xFFB3,0xFFA7,0xFFB2,0xFFB5,0xFFAF,0xFFB1,0xFF9D,0xFFB7,0xFF9B,0xFFAD,0xFF9A,0xFFB6,0xFF9A,0xFFA8,0xFF9C,0xFFAA,0xFF9B,0xFFA5,0xFF9A,0xFFAE,0xFFAC,0xFFB0,0xFFAB,0xFFB6,0xFFAE,0xFFC0,0xFFAD,0xFFC3,0xFFB0,0xFFC2,0xFFAB,0xFFC4,0xFFB0,0xFFC2,0xFFB0,0xFFC4,0xFFB8,0xFFB2,0xFFB3,0xFFAE,0xFFB4,0xFFAE,0xFFAF,0xFFB0,0xFFAF,0xFFAF,0xFFA5,0xFFB2,0xFFA4,0xFFB3,0xFF9D,0xFF9E,0xFF9A,0xFF9A,0xFF9F,0xFFA1,0xFFA4,0xFF9D,0xFFAF,0xFFA7,0xFFAC,0xFF9F,0xFFAD,0xFFAA,0xFFB0,0xFFA7,0xFFAF,0xFFB5,0xFFA6,0xFFAF,0xFF9A,0xFFB5,0xFF98,0xFFAA,0xFF9B,0xFFB3,0xFF99,0xFFA5,0xFF9A,0xFFAA,0xFF9F,0xFFA5,0xFF9A,0xFFA7,0xFFAC,0xFFAA,0xFFA8,0xFFAA,0xFFAD,0xFFB0,0xFFAB,0xFFB9,0xFFAD,0xFFBF,0xFFAB,0xFFBD,0xFFAF,0xFFC0,0xFFB0,0xFFBA,0xFFB4,0xFFAE,0xFFAF,0xFFAC,0xFFB0,0xFFAC,0xFFAC,0xFFB0,0xFFAE,0xFFB0,0xFFA6,0xFFBB,0xFFA5,0xFFBC,0xFFA0,0xFF96,0xFF96,0xFF92,0xFF99,0xFF99,0xFF98,0xFF97,0xFF9E,0xFFA0,0xFFA0,0xFF9E,0xFFA7,0xFFA5,0xFFA8,0xFFA3,0xFFA9,0xFFAE,0xFF9A,0xFFA5,0xFF97,0xFFAE,0xFF95,0xFFA6,0xFF99,0xFFAF,0xFF98,0xFFA2,0xFFA0,0xFFAB,0xFFA9,0xFFA3,0xFFA6,0xFFA4,0xFFAE,0xFFA7,0xFFA5,0xFFA7,0xFFA7,0xFFA9,0xFFA6,0xFFAC,0xFFA6,0xFFB0,0xFFA3,0xFFB7,0xFFAD,0xFFB7,0xFFA9,0xFFAF,0xFFAB,0xFFA8,0xFFA8,0xFFAC,0xFFAD,0xFFAB,0xFFAA,0xFFAF,0xFFAE,0xFFB6,0xFFA7,0xFFBC,0xFFAB,0xFFC4,0xFFA4,0xFF93,0xFF95,0xFF90,0xFF94,0xFF94,0xFF93,0xFF92,0xFF96,0xFF99,0xFF93,0xFF96,0xFF97,0xFFA0,0xFF9E,0xFF9B,0xFF9E,0xFFA4,0xFF93,0xFF9D,0xFF94,0xFFA9,0xFF96,0xFF9F,0xFF96,0xFFAB,0xFF97,0xFFA1,0xFFA5,0xFFA9,0xFFAA,0xFFA4,0xFFA6,0xFFA4,0xFFAC,0xFFA4,0xFFA6,0xFFA6,0xFFA7,0xFFA6,0xFFA1,0xFFA5,0xFFA6,0xFFA4,0xFFA3,0xFFA7,0xFFA4,0xFFA7,0xFF9F,0xFFAB,0xFFA3,0xFFA9,0xFFA3,0xFFAA,0xFFA7,0xFFAB,0xFFA6,0xFFAE,0xFFAA,0xFFB7,0xFFA6,0xFFBB,0xFFAA,0xFFBB,0xFFA3,0xFF87,0xFF8A,0xFF84,0xFF8C,0xFF8A,0xFF8B,0xFF86,0xFF8B,0xFF8B,0xFF89,0xFF85,0xFF8B,0xFF8F,0xFF89,0xFF8A,0xFF8B,0xFF91,0xFF8A,0xFF8C,0xFF8D,0xFF9A,0xFF8B,0xFF95,0xFF8E,0xFF9E,0xFF93,0xFF98,0xFF9D,0xFF9E,0xFF9D,0xFF9A,0xFF99,0x4DFA,0x1A56,0x7FFF,0x1A56,0x7FFF,0x1A55,0x7FFF,0x1A55,0xFFB9,0xCE07,0x1584,0xD653,0xFFF9,0x0009,0x0000,0xFFFD,0x1976,0x03FD,0x0297,0x7FFF,0x1976,0x03FD,0x0297,0x7FFF,0x0001,0x0001,0x0001,0x0001,0x0001,0x0001,0x0001,0x0001,0x0695,0x7FFF,0x1A56,0x7FFF,0x1A56,0x7FFF,0x1A55,0x7FFF,0xFFBD,0xF57A,0xCEF2,0xD8E0,0x0009,0xFFFD,0xFFFC,0x0000,0x00ED,0x0046,0x2AD6,0x0035,0x00EE,0x0046,0x2AD6,0x0035,0x0001,0x0001,0x0001,0x0001,0x0001,0x0001,0x0001,0x0001,0x1901,0x0000 };
    uint16_t mlx90640Frame[834];

    jclass fdClass = (*env).FindClass("java/io/FileDescriptor");

    if (fdClass != NULL) {
        jfieldID fdClassDescriptorFieldID = (*env).GetFieldID(fdClass, "descriptor", "I");
        if (fdClassDescriptorFieldID != NULL && filedescriptor != NULL) {
            fd = (*env).GetIntField(filedescriptor, fdClassDescriptorFieldID);
        }
    }

    jfloat * framedataptr = (*env).GetFloatArrayElements(framedata, 0);

#define TA_SHIFT 8

    status = MLX90640_GetFrameData(i2cslaveaddress, mlx90640Frame);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, framedataptr);

    (*env).ReleaseFloatArrayElements( framedata, framedataptr, 0);

    return JNI_TRUE;
}