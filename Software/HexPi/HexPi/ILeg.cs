/**********************************************************************************************//**
 * @file    ileg.cs
 *
 * @brief   Implements the ileg class.
 **************************************************************************************************/

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;

namespace HexPi
{
    /**********************************************************************************************//**
     * @class   ILeg
     *
     * @brief   A represents a leg of the hexapod.
     *
     * @author  Alexander Miller
     * @date    11.08.2016
     **************************************************************************************************/

    class ILeg
    {
        #region FIELDS

        protected enum modes : byte {NORMAL,TERRAIN};

        /** @brief   The device. */
        I2cDevice device = null;

        byte mode = 0;



        /** @brief   The offset of the first angle. */
        protected double alphaOff = 0;

        /** @brief   The offset of the second angle. */
        protected double betaOff = 0;

        /** @brief   The offset of the third angle. */
        protected double gammaOff = 0;

        /** @brief   The control variable of the calculations. */
        protected double t = 0;

        /** @brief   The offset for the control variable. */
        protected double tOffset = 0;

        /** @brief   The x position of the TCP. */
        protected double xPos = 0;

        /** @brief   The y position of the TCP. */
        protected double yPos = 0;

        /** @brief   The z position of the TCP. */
        protected double zPos = 0;

        /** @brief   The leg-position x-offset */
        protected double xOffset = 0;

        /** @brief   The leg-position y-offset */
        protected double yOffset = 0;

        /** @brief   The rotation. */
        protected double rotation = 0;

        /** @brief   The rotation of the xy-axis at xy-movement. */
        protected double xyRotation = 0;

        protected double id = 0;

        //******

        #endregion FIELDS

        #region CONSTANTS

        /** @brief   The step size for x coordinate in mm. */
        protected const double stepSizeX = 20;

        /** @brief   The step size for y coordinate in mm. */
        protected const double stepSizeY = 20;

        /** @brief   The step size for z coordinate in mm. */
        protected const double stepSizeZ = 30;

        /** @brief   The step size for rotation in mm. */
        protected const double stepSizeR = 30;

        /** @brief   The period. */
        protected const int period = 100;

        protected const int lift = period / 2 + 10;
        protected const int sense = period / 2 + 40;
        

        /** @brief   The height of the first joint. */
        protected const double zOffset = 88;

        /** @brief   The distance between the first and second joint in mm. */
        protected const double A1 = 52;

        /** @brief   The lenght of the upper leg in mm. */
        protected const double A2 = 69;

        /** @brief   The lenght of the lower leg in mm. */
        protected const double A3 = 88;


        #endregion CONSTANTS

        #region PROPERTIES

        /**********************************************************************************************//**
        * @property    public double XPos
        *
        * @brief   Gets or sets the x position.
        *
        * @return  The x coordinate position.
        **************************************************************************************************/

        public double XPos
        {
            get
            {
                return xPos;
            }

            set
            {
                xPos = value;
            }
        }

        /**********************************************************************************************//**
         * @property    public double YPos
         *
         * @brief   Gets or sets the y position.
         *
         * @return  The y coordinate position.
         **************************************************************************************************/

        public double YPos
        {
            get
            {
                return yPos;
            }

            set
            {
                yPos = value;
            }
        }

        /**********************************************************************************************//**
         * @property    public double ZPos
         *
         * @brief   Gets or sets the z position.
         *
         * @return  The z coordinate position.
         **************************************************************************************************/

        public double ZPos
        {
            get
            {
                return zPos;
            }

            set
            {
                zPos = value;
                if (zPos > stepSizeZ)
                {
                    zPos = stepSizeZ;
                }
            }
        }

        /**********************************************************************************************//**
         * @property    public double StepSizeX
         *
         * @brief   Gets the step size for x.
         *
         * @return  The step size x coordinate.
         **************************************************************************************************/

        public double StepSizeX
        {
            get
            {
                return stepSizeX;
            }
        }

        /**********************************************************************************************//**
         * @property    public double StepSizeY
         *
         * @brief   Gets the step size for y.
         *
         * @return  The step size y coordinate.
         **************************************************************************************************/

        public double StepSizeY
        {
            get
            {
                return stepSizeY;
            }
        }

        /**********************************************************************************************//**
         * @property    public double StepSizeZ
         *
         * @brief   Gets the step size for z.
         *
         * @return  The step size z coordinate.
         **************************************************************************************************/

        public double StepSizeZ
        {
            get
            {
                return stepSizeZ;
            }
        }

       

        #endregion PROPERTIES

        #region ABSTRACT_FUNCTIONS

        #endregion ABSTRACT_FUNKTIONS

        #region FUNCTIONS


        /**********************************************************************************************//**
         * @fn  public ILeg(int tOffset, int aOff, int bOff, int cOff , double rotation)
         *
         * @brief   Constructor.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   tOffset     The control offset.
         * @param   aOff        The offset for alpha.
         * @param   bOff        The offset for beta.
         * @param   cOff        The offset for gamma.
         * @param   rotation    The angle of the leg path in rotation.
         **************************************************************************************************/

        public ILeg(int tOffset, int aOff, int bOff, int cOff, double rotation, int address,int xOff,int yOff)
        {
            this.tOffset = tOffset;
            t = this.tOffset;

            alphaOff = aOff;
            betaOff = bOff;
            gammaOff = cOff;

            xOffset = xOff;
            yOffset = yOff;
            

            this.rotation = (rotation / 180) * Math.PI;

            init(address);



        }



        /**********************************************************************************************//**
* @fn  public async void init()
*
* @brief   Initializes this device.
*
* @author  Alexander Miller
* @date    11.08.2016
**************************************************************************************************/

        public async void init(int address)
        {
            try
            {
                id = address;
                I2cConnectionSettings settings = new I2cConnectionSettings(address); // Address
                settings.BusSpeed = I2cBusSpeed.FastMode;
                settings.SharingMode = I2cSharingMode.Shared;
                string aqs = I2cDevice.GetDeviceSelector("I2C1");
                DeviceInformationCollection dis = await DeviceInformation.FindAllAsync(aqs);
                device = await I2cDevice.FromIdAsync(dis[0].Id, settings);
            }
            catch
            {
                Debug.WriteLine("Error: I2C init failed!");
            }
        }


        /**********************************************************************************************//**
         * @fn  public override void calcPositionR(double increment)
         *
         * @brief   Calculates the leg position in a rotational movement.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   increment   Amount to increment by.
         **************************************************************************************************/

        public void calcPositionR(double increment,bool terrainmode)
        {

            t = ((t + increment) + period) % period;
            if (t <= period / 2)
            {
                xPos = -4 * ((stepSizeR * Math.Cos(rotation)) / period) * t + (stepSizeR * Math.Cos(rotation));
                yPos = -4 * ((stepSizeR * Math.Sin(rotation)) / period) * t + (stepSizeR * Math.Sin(rotation));
            }
            else
            {
                xPos = 4 * ((stepSizeR * Math.Cos(rotation)) / period) * (t - period / 2) - (stepSizeR * Math.Cos(rotation));
                yPos = 4 * ((stepSizeR * Math.Sin(rotation)) / period) * (t - period / 2) - (stepSizeR * Math.Sin(rotation));
            }
            calcPositionZ(terrainmode);
        }
        //******

        /**********************************************************************************************//**
         * @fn  public void sendData()
         *
         * @brief   Send Motor positions.
         *
         * @author  Alexander Miller
         * @date    01.03.2017
         *
         **************************************************************************************************/

        public void sendData(byte[] data)
        {
            try
            {
                if (device != null)
                {
                    device.Write(data);
                }
                else
                {
                    //Debug.WriteLine("Error: I2C write failed!");
                }

            }
            catch (Exception e)
            {
                Debug.WriteLine("Error: I2C hat write failed!" + e.Message);
            }

        }


        /**********************************************************************************************//**
         * @fn  public void calcPositionCenter()
         *
         * @brief   Resets the TCP position and the control variable.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        public void calcPositionCenter()
        {
            t = tOffset;
            xPos = 0.0;
            yPos = 0.0;
        }

        /**********************************************************************************************//**
         * @fn  public void calcPositionX(double increment)
         *
         * @brief   Calculates the TCP position for movement in x direction.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   increment   Amount to increment by.
         **************************************************************************************************/

        public void calcPositionXY(double x, double y, Boolean terrainmode)
        {
            int frame = 10;



            t = (t + Math.Sqrt(x * x + y * y)) % period;

            //if t is equal to period/2 +- frame
            if ((t>= period/2-frame) && t<=period/2+frame)
            {
                xyRotation = Math.Atan2(-y,-x);
            }

            if (terrainmode)
            {

                if (mode != (byte)ILeg.modes.TERRAIN)
                {
                    mode = (byte)ILeg.modes.TERRAIN;
                    sendMode(mode);
                }

                //if (t <= period / 2)
                //{

                //    xPos = -4 * ((stepSizeR * Math.Cos(xyRotation)) / period) * t + (stepSizeR * Math.Cos(xyRotation));
                //    yPos = -4 * ((stepSizeR * Math.Sin(xyRotation)) / period) * t + (stepSizeR * Math.Sin(xyRotation));
                //}
                //else if (t > period / 2 && t <= lift)
                //{
                //    xPos = -stepSizeR * Math.Cos(xyRotation);
                //    yPos = -stepSizeR * Math.Sin(xyRotation);
                //}
                //else if (t > lift && t <= sense)
                //{
                //    xPos = 4 * ((stepSizeR * Math.Cos(xyRotation)) / (sense - lift)) * (t - lift) - (stepSizeR * Math.Cos(xyRotation));
                //    yPos = 4 * ((stepSizeR * Math.Sin(xyRotation)) / (sense - lift)) * (t - lift) - (stepSizeR * Math.Sin(xyRotation));
                //}
                //else if (t > sense)
                //{
                //    xPos = stepSizeR * Math.Cos(xyRotation);
                //    yPos = stepSizeR * Math.Sin(xyRotation);

                //}
                calcPositionZ(terrainmode);
            }
            else
            {
                if (mode != (byte)ILeg.modes.NORMAL)
                {
                    mode = (byte)ILeg.modes.NORMAL;
                    sendMode(mode);
                }

                if (t <= period / 2)
                {
                    xPos = -4 * ((stepSizeR * Math.Cos(xyRotation)) / period) * t + (stepSizeR * Math.Cos(xyRotation));
                    yPos = -4 * ((stepSizeR * Math.Sin(xyRotation)) / period) * t + (stepSizeR * Math.Sin(xyRotation));
                }
                else
                {
                    xPos = 4 * ((stepSizeR * Math.Cos(xyRotation)) / period) * (t - period / 2) - (stepSizeR * Math.Cos(xyRotation));
                    yPos = 4 * ((stepSizeR * Math.Sin(xyRotation)) / period) * (t - period / 2) - (stepSizeR * Math.Sin(xyRotation));
                }
                calcPositionZ(terrainmode);
            }

            



            if (xPos > stepSizeX)
            {
                xPos = stepSizeX;
            }
            else if (xPos < -stepSizeX)
            {
                xPos = -stepSizeX;
            }
            if (yPos > stepSizeY)
            {
                yPos = stepSizeY;
            }
            else if (yPos < -stepSizeY)
            {
                yPos = -stepSizeY;
            }


            
        }


        protected void calcPositionZ(Boolean terrainmode)
        {
            if (terrainmode)
            {
                if (t <= period / 2)
                {

                    zPos = 0;
                   
                }
                else if (t > period / 2 && t <= lift)
                {
                    zPos = 2 * stepSizeZ / (lift - period / 2) * (t - period / 2);
                }
                else if (t > lift && t <= sense)
                {
                    zPos = stepSizeZ;
                }
                else if (t > sense)
                {
                    zPos = -2 * stepSizeZ / (sense - period / 2) * (t - sense)+stepSizeZ;

                }
            }
            else
            {
                if (t <= period / 2)
                {
                    zPos = 0;

                }
                else
                {
                    zPos = -1 * (stepSizeZ * 16 / (period * period)) * (t - 3 * period / 4) * (t - 3 * period / 4) + stepSizeZ;
                }

            }

            



            if (zPos > stepSizeZ)
            {
                zPos = stepSizeZ;
            }
            else if (zPos < -stepSizeZ)
            {
                zPos = -stepSizeZ;
            }
        }


        

        /**********************************************************************************************//**
         * @fn  public void calcData()
         *
         * @brief   Calculates the data for the LegController.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        public void calcData()
        {

            byte[] data = new byte[4];
            data[0] = 3;
            data[1] = (Byte)XPos;
            data[2] = (Byte)YPos;
            data[3] = (Byte)ZPos;
            sendData(data);
        }

        public void sendMode(byte m)
        {
            byte[] data = new byte[2];
            data[0]=6;
            data[1] = m;
            sendData(data);
        }

        public void sendCalibrationData()
        {
            //Write calibration data
            byte[] data = new byte[4];
            data[0] = 4;
            data[3] = (byte)alphaOff;
            data[2] = (byte)betaOff;
            data[1] = (byte)gammaOff;
            Debug.WriteLine("Writing calibration data!");
        }

        public void calcPose(double x, double y, double z, double a, double b)
        {
            
            double tempX = xOffset;
            double tempY = yOffset;
            double tempZ = zOffset;
            double sA = Math.Sin(z);
            double sB = Math.Sin(y);
            double sC = Math.Sin(x);
            double cA = Math.Cos(z);
            double cB = Math.Cos(y);
            double cC = Math.Cos(x);
            




            double newX = tempX * (cA * cB) + tempY * (cA * sB * sC - sA * cC) + tempZ * (sA * sC + cA * sB * cC);
            double newY = tempX * (sA * cB) + tempY * (cA*cC+sA*sB*sC) + tempZ * (sA*sB*cC-cA*sC);
            double newZ = tempX * (-sB) + tempY * (cB*sC) + tempZ * (cB*cC);

            xPos = newX - tempX + a;
            yPos = newY - tempY ;
            zPos = newZ - tempZ + b;
       
            //Debug.WriteLine("Ytemp: " + tempY + " Y: " + newY);
            Debug.WriteLine("ID: " + id + "X: " + xPos + " Y: " + yPos + " Z: " + zPos);

        }

        #endregion FUNCTIONS

    }
}
