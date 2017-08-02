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
     * @class   Leg
     *
     * @brief   A represents a leg of the hexapod.
     *
     * @author  Alexander Miller
     * @date    11.08.2016
     **************************************************************************************************/

    class Leg
    {
        #region Objects
        /** @brief   The device. */
        I2cDevice device = null;

        #endregion Objects

        #region FIELDS

        private double stepSizeTurn = 30;

        /** @brief   The offset of the first angle. */
        private double alphaOff = 0;

        /** @brief   The offset of the second angle. */
        private double betaOff = 0;

        /** @brief   The offset of the third angle. */
        private double gammaOff = 0;

        /** @brief   The control variable of the calculations. */
        private double t = 0;

        /** @brief   The offset for the control variable. */
        private double tOffset = 0;

        /** @brief   The x position of the TCP. */
        private double xPos = 0;

        /** @brief   The y position of the TCP. */
        private double yPos = 0;

        /** @brief   The z position of the TCP. */
        private double zPos = 0;

        /** @brief   The leg-position x-offset */
        private double xOffset = 0;

        /** @brief   The leg-position y-offset */
        private double yOffset = 0;

        /** @brief   The rotation. */
        private double rRotation = 0;

        /** @brief   The rotation of the xy-axis at xy-movement. */
        private double xyRotation = 0;

        private byte id = 0;


        #endregion FIELDS

        #region CONSTANTS

        ///** @brief   The step size for x coordinate in mm. */
        //private const double stepSizeX = 30;

        ///** @brief   The step size for y coordinate in mm. */
        //private const double stepSizeY = 20;

        /** @brief   The step size for z coordinate in mm. */
        private const double stepSizeZ = 30;

        /** @brief   The step size for rotation in mm. */
        private const double stepSizeXY = 30;

        /** @brief   The period. */
        private const int period = 100;

        private const int lift = period / 2 + 10;
        private const int sense = period / 2 + 40;
        private const int frame = 1;

        /** @brief   The height of the first joint. */
        private const double zOffset = 88;

        /** @brief   The distance between the first and second joint in mm. */
        private const double A1 = 52;

        /** @brief   The lenght of the upper leg in mm. */
        private const double A2 = 69;

        /** @brief   The lenght of the lower leg in mm. */
        private const double A3 = 88;


        #endregion CONSTANTS

        #region PROPERTIES

        /**********************************************************************************************//**
        * @property    public double XPos
        *
        * @brief   Gets or sets the x position.
        *
        * @return  The x coordinate position.
        **************************************************************************************************/

        public int XPos
        {
            get
            {
                return (int)xPos;
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

        public int YPos
        {
            get
            {
                return (int)yPos;
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

        public int ZPos
        {
            get
            {
                return (int)zPos;
            }

            set
            {

                zPos = value;
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
                return stepSizeXY;
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
                return stepSizeXY;
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

        public Leg(int tOffset, int aOff, int bOff, int cOff, double rotation, byte address, int xOff, int yOff)
        {
            this.tOffset = tOffset;
            t = this.tOffset;

            alphaOff = aOff;
            betaOff = bOff;
            gammaOff = cOff;

            xOffset = xOff;
            yOffset = yOff;


            this.rRotation = (rotation / 180) * Math.PI;

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

        public async void init(byte address)
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
         * @fn  public override void calcPositionRotate(double increment)
         *
         * @brief   Calculates the leg position in a rotational movement.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   increment   Amount to increment by.
         **************************************************************************************************/

        public void calcPositionRotate(double increment, byte mode)
        {
            if (mode == (byte)Controller.modes.TERRAIN)
            {
                t = ((t + increment / 3)+period) % period;
            }
            else
            {
                t = ((t + increment) + period) % period;
            }

            calcXY(stepSizeXY,rRotation,mode);

            //t = ((t + increment) + period) % period;
            //if (t <= period / 2)
            //{
            //    xPos = -4 * ((stepSizeR * Math.Cos(rotation)) / period) * t + (stepSizeR * Math.Cos(rotation));
            //    yPos = -4 * ((stepSizeR * Math.Sin(rotation)) / period) * t + (stepSizeR * Math.Sin(rotation));
            //}
            //else
            //{
            //    xPos = 4 * ((stepSizeR * Math.Cos(rotation)) / period) * (t - period / 2) - (stepSizeR * Math.Cos(rotation));
            //    yPos = 4 * ((stepSizeR * Math.Sin(rotation)) / period) * (t - period / 2) - (stepSizeR * Math.Sin(rotation));
            //}
            //calcZ(mode);
        }
        //******

        public void calcPositionTurn(double x, double a, byte mode)
        {
            a = -a;

            //t = ((t + a / Math.Abs(a) * x) + period) % period;
            t = ((t + x) + period) % period;


            //if t is equal to period*0.25 or period*0.75 +- frame
            if ((t >= period * 0.75 - frame) && (t <= period * 0.75 + frame) || (t >= period * 0.25 - frame) && (t <= period * 0.25 + frame))
            {
                //xyRotation = Math.Atan2(-y, -x);
                
                    double rad = a / Math.Abs(a) * ((1000 - 500 * Math.Abs(a)) - Math.Abs(yOffset));

                    double w = Math.Atan2(xOffset, rad);

                if (!double.IsNaN(w))
                {
                    xyRotation = w;
                }
                else
                {
                    xyRotation = 0;
                }
                    
                if (Math.Sign(a)==-1)
                {
                    xyRotation -= Math.PI;
                }
                Debug.WriteLine("W=" + xyRotation*180/Math.PI);

                if (!double.IsNaN(rad))
                {
                    if (yOffset < 0)
                    {
                        if (rad > yOffset)
                        {
                            stepSizeTurn = stepSizeXY;
                        }
                        else
                        {
                            stepSizeTurn = stepSizeXY * (Math.Abs(rad) - Math.Abs(yOffset)) / (Math.Abs(rad) + Math.Abs(yOffset));
                        }
                    }
                    if (yOffset > 0)
                    {
                        if (rad < yOffset)
                        {
                            stepSizeTurn = stepSizeXY;
                        }
                        else
                        {
                            stepSizeTurn = stepSizeXY * (Math.Abs(rad) - Math.Abs(yOffset)) / (Math.Abs(rad) + Math.Abs(yOffset));
                        }
                    }
                }
                else
                {
                    stepSizeTurn = stepSizeXY;
                }
            

                    if (stepSizeTurn > stepSizeXY)
                    {
                        stepSizeTurn = stepSizeXY;
                    }
                    else if (stepSizeTurn < 0)
                    {
                        stepSizeTurn = 0;
                    }

                
            }

            calcXY(stepSizeTurn,xyRotation,mode);
        }

        /**********************************************************************************************//**
         * @fn  public void calcPositionWalk(double increment)
         *
         * @brief   Calculates the TCP position for movement in x direction.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   increment   Amount to increment by.
         **************************************************************************************************/

        public void calcPositionWalk(double x, double y, byte mode)
        {





            if (mode == (byte)Controller.modes.TERRAIN)
            {
                t = (t + Math.Sqrt(x * x + y * y) / 3) % period;
            }
            else
            {
                t = (t + Math.Sqrt(x * x + y * y)) % period;
            }


            //if t is equal to period*0.25 or period*0.75 +- frame
            if ((t >= period * 0.75 - frame) && (t <= period * 0.75 + frame) || (t >= period * 0.25 - frame) && (t <= period * 0.25 + frame))
            {
                xyRotation = Math.Atan2(y, x);
            }

            calcXY(stepSizeXY,xyRotation,mode);

            //if (mode == (byte)Controller.modes.TERRAIN)
            //{

            //    if (t <= period / 2)
            //    {

            //        xPos = -4 * ((stepSizeXY * Math.Cos(xyRotation)) / period) * t + (stepSizeXY * Math.Cos(xyRotation));
            //        yPos = -4 * ((stepSizeXY * Math.Sin(xyRotation)) / period) * t + (stepSizeXY * Math.Sin(xyRotation));
            //    }
            //    else if (t > period / 2 && t <= lift)
            //    {
            //        xPos = -stepSizeXY * Math.Cos(xyRotation);
            //        yPos = -stepSizeXY * Math.Sin(xyRotation);
            //    }
            //    else if (t > lift && t <= sense)
            //    {
            //        xPos = 4 * ((stepSizeXY * Math.Cos(xyRotation)) / (sense - lift)) * (t - lift) - (stepSizeXY * Math.Cos(xyRotation));
            //        yPos = 4 * ((stepSizeXY * Math.Sin(xyRotation)) / (sense - lift)) * (t - lift) - (stepSizeXY * Math.Sin(xyRotation));
            //    }
            //    else if (t > sense)
            //    {
            //        xPos = stepSizeXY * Math.Cos(xyRotation);
            //        yPos = stepSizeXY * Math.Sin(xyRotation);

            //    }
            //    calcZ(mode);
            //}
            //else
            //{

            //    if (t <= period / 2)
            //    {
            //        xPos = -4 * ((stepSizeXY * Math.Cos(xyRotation)) / period) * t + (stepSizeXY * Math.Cos(xyRotation));
            //        yPos = -4 * ((stepSizeXY * Math.Sin(xyRotation)) / period) * t + (stepSizeXY * Math.Sin(xyRotation));
            //    }
            //    else
            //    {
            //        xPos = 4 * ((stepSizeXY * Math.Cos(xyRotation)) / period) * (t - period / 2) - (stepSizeXY * Math.Cos(xyRotation));
            //        yPos = 4 * ((stepSizeXY * Math.Sin(xyRotation)) / period) * (t - period / 2) - (stepSizeXY * Math.Sin(xyRotation));
            //    }
            //    calcZ(mode);
            //}





            //if (xPos > stepSizeXY)
            //{
            //    xPos = stepSizeXY;
            //}
            //else if (xPos < -stepSizeXY)
            //{
            //    xPos = -stepSizeXY;
            //}
            //if (yPos > stepSizeXY)
            //{
            //    yPos = stepSizeXY;
            //}
            //else if (yPos < -stepSizeXY)
            //{
            //    yPos = -stepSizeXY;
            //}



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
            zPos = 0.0;
        }

        /**********************************************************************************************//**
         * @fn  public void calcXY(double stepsize, double rotation, byte mode)
         *
         * @brief   Calculates the TCP position for movement in xy direction.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   increment   Amount to increment by.
         **************************************************************************************************/

        public void calcXY(double stepsize, double rotation, byte mode)
        {

            if (mode == (byte)Controller.modes.TERRAIN)
            {

                if (t <= period / 2)
                {

                    xPos = -4 * ((stepsize * Math.Cos(rotation)) / period) * t + (stepsize * Math.Cos(rotation));
                    yPos = -4 * ((stepsize * Math.Sin(rotation)) / period) * t + (stepsize * Math.Sin(rotation));
                }
                else if (t > period / 2 && t <= lift)
                {
                    xPos = -stepsize * Math.Cos(rotation);
                    yPos = -stepsize * Math.Sin(rotation);
                }
                else if (t > lift && t <= sense)
                {
                    xPos = 2 * ((stepsize * Math.Cos(rotation)) / (sense - lift)) * (t - lift) - (stepsize * Math.Cos(rotation));
                    yPos = 2 * ((stepsize * Math.Sin(rotation)) / (sense - lift)) * (t - lift) - (stepsize * Math.Sin(rotation));
                }
                else if (t > sense)
                {
                    xPos = stepsize * Math.Cos(rotation);
                    yPos = stepsize * Math.Sin(rotation);

                }
                
            }
            else
            {

                if (t <= period / 2)
                {
                    xPos = -4 * ((stepsize * Math.Cos(rotation)) / period) * t + (stepsize * Math.Cos(rotation));
                    yPos = -4 * ((stepsize * Math.Sin(rotation)) / period) * t + (stepsize * Math.Sin(rotation));
                }
                else
                {
                    xPos = 4 * ((stepsize * Math.Cos(rotation)) / period) * (t - period / 2) - (stepsize * Math.Cos(rotation));
                    yPos = 4 * ((stepsize * Math.Sin(rotation)) / period) * (t - period / 2) - (stepsize * Math.Sin(rotation));
                }
                
            }
            calcZ(mode);




            if (xPos > stepSizeXY)
            {
                xPos = stepSizeXY;
            }
            else if (xPos < -stepSizeXY)
            {
                xPos = -stepSizeXY;
            }
            if (yPos > stepSizeXY)
            {
                yPos = stepSizeXY;
            }
            else if (yPos < -stepSizeXY)
            {
                yPos = -stepSizeXY;
            }



        }


        private void calcZ(byte mode)
        {

            if (mode == (byte)Controller.modes.TERRAIN)
            {
                if (t <= period / 2)
                {

                    zPos = 0;

                }

                else if (t > period / 2 && t <= lift)
                {
                    zPos = stepSizeZ;
                }
                else if (t > lift && t <= sense)
                {
                    zPos = stepSizeZ;
                }
                else if (t > sense)
                {
                    zPos = 0;
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

        public void calcDataTerrain()
        {

            byte[] data = new byte[4];
            data[0] = 6;
            data[1] = (Byte)XPos;
            data[2] = (Byte)YPos;
            data[3] = (Byte)ZPos;
            sendData(data);
        }

        public void setColor(ushort hue)
        {
            byte[] byteArray = BitConverter.GetBytes(hue);

            byte[] data = new byte[3];
            data[0] = 1;
            data[1] = byteArray[1];
            data[2] = byteArray[0];

            sendData(data);
        }

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

        public void calcPose(double yaw, double pitch, double roll, double a, double b)
        {

            double tempX = xOffset - xPos;
            double tempY = yOffset - yPos;
            double tempZ = zOffset - zPos;
            double sA = Math.Sin(yaw);
            double sB = Math.Sin(pitch);
            double sC = Math.Sin(roll);
            double cA = Math.Cos(yaw);
            double cB = Math.Cos(pitch);
            double cC = Math.Cos(roll);





            double newX = tempX * (cA * cB) + tempY * (cA * sB * sC - sA * cC) + tempZ * (sA * sC + cA * sB * cC);
            double newY = tempX * (sA * cB) + tempY * (cA * cC + sA * sB * sC) + tempZ * (sA * sB * cC - cA * sC);
            double newZ = tempX * (-sB) + tempY * (cB * sC) + tempZ * (cB * cC);

            xPos += newX - tempX + a;
            yPos += newY - tempY;
            zPos += newZ - tempZ + b;

        }

        #endregion FUNCTIONS

    }
}
