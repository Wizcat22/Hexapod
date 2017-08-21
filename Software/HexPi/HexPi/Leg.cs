/**********************************************************************************************//**
 * @file    Leg.cs
 *
 * @brief   Implements the Leg class.
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

    /**
     * @class   Leg
     *
     * @brief   Represents a leg of the hexapod.
     *
     * @author  Alexander Miller
     * @date    13.08.2017
     */

    class Leg
    {
        #region Objects
        /** @brief   The i2c-device. */
        I2cDevice device = null;

        #endregion Objects

        #region FIELDS

        /** @brief   The step size while turn */
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

        /** @brief   The angle of the movement line while rotating around center. */
        private double rRotation = 0;

        /** @brief   The rotation of the movement line at xy-movement. */
        private double xyRotation = 0;

        /** @brief   The i2c identifier */
        private byte id = 0;


        #endregion FIELDS

        #region CONSTANTS

        /** @brief   The step size for z coordinate in mm. */
        private const double stepSizeZ = 40;

        /** @brief   The step size for rotation in mm. */
        private const double stepSizeXY = 30;

        /** @brief   The period. */
        private const int period = 100;

        /** @brief   The section to lift the leg in terrain mode */
        private const int lift = period / 2 + 10;
        /** @brief   The section to sense the ground in terrain mode */
        private const int sense = period / 2 + 40;
        /** @brief   The frame around 0/0/0 to change the rotation angle */
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


        /**
         * @property    public int XPos
         *
         * @brief   Gets or sets the x position
         *
         * @return  The x coordinate.
         */

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



        /**
         * @property    public int YPos
         *
         * @brief   Gets or sets the y position
         *
         * @return  The y coordinate.
         */

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



        /**
         * @property    public int ZPos
         *
         * @brief   Gets or sets the z position
         *
         * @return  The z coordinate.
         */

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



        /**
         * @property    public double StepSizeX
         *
         * @brief   Gets the step size for x movement
         *
         * @return  The step size for x in mm.
         */

        public double StepSizeX
        {
            get
            {
                return stepSizeXY;
            }
        }



        /**
         * @property    public double StepSizeY
         *
         * @brief   Gets the step size for y movement
         *
         * @return  The step size for y in mm.
         */

        public double StepSizeY
        {
            get
            {
                return stepSizeXY;
            }
        }



        /**
         * @property    public double StepSizeZ
         *
         * @brief   Gets the step size for z movement
         *
         * @return  The step size for z in mm.
         */

        public double StepSizeZ
        {
            get
            {
                return stepSizeZ;
            }
        }



        #endregion PROPERTIES

        #region FUNCTIONS




        /**
         * @fn  public Leg(int tOffset, int aOff, int bOff, int cOff, double rotation, byte address, int xOff, int yOff)
         *
         * @brief   Constructor
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   tOffset     The control offset.
         * @param   aOff        The offset for alpha.
         * @param   bOff        The offset for beta.
         * @param   cOff        The offset for gamma.
         * @param   rotation    The angle of the leg path in rotation.
         * @param   address     The i2c address.
         * @param   xOff        The x offset from the center of the body.
         * @param   yOff        The y offset from the center of the body.
         */

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




        /**
         * @fn  public async void init(byte address)
         *
         * @brief   Initializes this device
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   address The i2c address.
         */

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



        /**
         * @fn  public void calcPositionRotate(double increment, byte mode)
         *
         * @brief   Calculates the tcp position while rotating
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   increment   Amount to increment by.
         * @param   mode        The mode.
         */

        public void calcPositionRotate(double increment, byte mode)
        {
            //if in terrain mode-> walk slower
            if (mode == (byte)Controller.modes.TERRAIN)
            {
                t = ((t + Math.Abs(increment)) + period) % period;
            }
            else
            {
                t = ((t + Math.Abs(increment)) + period) % period;
            }

            if ((t >= period * 0.75 - frame) && (t <= period * 0.75 + frame) || (t >= period * 0.25 - frame) && (t <= period * 0.25 + frame))
            {
                if (increment >= 0)
                {
                    xyRotation = rRotation;
                }
                else
                {
                    xyRotation = rRotation + Math.PI;
                }

            }


            //calc tcp xy position based on rRotation 
            calcXY(stepSizeXY, xyRotation, mode);
        }

        /**
         * @fn  public void calcPositionTurn(double x, double a, byte mode)
         *
         * @brief   Calculates the tcp position while turning
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   x       Amount to increment by.
         * @param   a       factor for the radius of the turning motion.
         * @param   mode    The mode.
         */

        public void calcPositionTurn(double x, double a, byte mode)
        {
            a = -a;

            t = ((t + Math.Abs(x)) + period) % period;


            //if t is equal to period*0.25 or period*0.75 +- frame
            if ((t >= period * 0.75 - frame) && (t <= period * 0.75 + frame) || (t >= period * 0.25 - frame) && (t <= period * 0.25 + frame))
            {

                //calc the radius of the circle
                double rad = a / Math.Abs(a) * ((1000 - 500 * Math.Abs(a)) - Math.Abs(yOffset));

                //calc angle of the movement path
                double w = Math.Atan2(xOffset, rad);

                //check for errors
                if (!double.IsNaN(w))
                {
                    xyRotation = w;
                }
                else
                {
                    xyRotation = 0;
                }

                //adjust the angle of the movement path (turn it by 180°)
                if (Math.Sign(a) == -1)
                {
                    xyRotation -= Math.PI;
                }
                if (x < 0)
                {
                    xyRotation -= Math.PI;
                }

                //if angle is valid
                if (!double.IsNaN(rad))
                {
                    //if leg is on left side
                    if (yOffset < 0)
                    {
                        //if center of circle is on the right side
                        if (rad > yOffset)
                        {
                            stepSizeTurn = stepSizeXY;
                        }
                        //if center of circle is on the left side
                        else
                        {
                            //adapt the stepsize based on distance to the right side
                            stepSizeTurn = stepSizeXY * (Math.Abs(rad) - Math.Abs(yOffset)) / (Math.Abs(rad) + Math.Abs(yOffset));
                        }
                    }
                    //if leg is on the right side
                    if (yOffset > 0)
                    {
                        //if center of circle is on the left side
                        if (rad < yOffset)
                        {
                            stepSizeTurn = stepSizeXY;
                        }
                        //if center of circle is on the right side
                        else
                        {
                            //adapt the stepsize based on distance to the left side
                            stepSizeTurn = stepSizeXY * (Math.Abs(rad) - Math.Abs(yOffset)) / (Math.Abs(rad) + Math.Abs(yOffset));
                        }
                    }
                }
                else
                {
                    stepSizeTurn = stepSizeXY;
                }

                //check boundaries
                if (stepSizeTurn > stepSizeXY)
                {
                    stepSizeTurn = stepSizeXY;
                }
                else if (stepSizeTurn < 0)
                {
                    stepSizeTurn = 0;
                }


            }
            //calc tcp position based on calculatet angle and stepsize
            calcXY(stepSizeTurn, xyRotation, mode);
        }


        /**
         * @fn  public void calcPositionWalk(double x, double y, byte mode)
         *
         * @brief   Calculates the TCP position while moving.
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   x       Amount to increment x by.
         * @param   y       Amount to increment y by.
         * @param   mode    The mode.
         */

        public void calcPositionWalk(double x, double y, byte mode)
        {



            //if in terrain mode-> walk slower
            if (mode == (byte)Controller.modes.TERRAIN)
            {
                t = (t + Math.Sqrt(x * x + y * y)) % period;
            }
            else
            {
                t = (t + Math.Sqrt(x * x + y * y)) % period;
            }
            //Debug.WriteLine(Math.Sqrt(x * x + y * y));

            //if t is equal to period*0.25 or period*0.75 +- frame
            if ((t >= period * 0.75 - frame) && (t <= period * 0.75 + frame) || (t >= period * 0.25 - frame) && (t <= period * 0.25 + frame))
            {
                //calc angle based on the inputs
                xyRotation = Math.Atan2(y, x);
            }
            //calc tcp position
            calcXY(stepSizeXY, xyRotation, mode);


        }


        /**
         * @fn  public void calcPositionCenter()
         *
         * @brief   Resets the TCP position and the control variable.
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

        public void calcPositionCenter()
        {
            t = tOffset;
            xPos = 0.0;
            yPos = 0.0;
            zPos = 0.0;
        }


        /**
         * @fn  public void calcXY(double stepsize, double rotation, byte mode)
         *
         * @brief    Calculates the TCP position for movement in xy direction.
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   stepsize    The stepsize.
         * @param   rotation    The rotation.
         * @param   mode        The mode.
         */

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



            //check boundaries
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

        /**
         * @fn  private void calcZ(byte mode)
         *
         * @brief   Calculates the z-position of the tcp
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   mode    The mode.
         */

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
                else if (t <= period * 0.6)
                {
                    zPos = -1 * (stepSizeZ * 100 / (period * period)) * (t - 0.6 * period) * (t - 0.6 * period) + stepSizeZ;
                }
                else if (t <= period)
                {
                    zPos = -1 * (stepSizeZ * 6.25 / (period * period)) * (t - 0.6 * period) * (t - 0.6 * period) + stepSizeZ;
                }
                //else if (t <= 3 * period / 4)
                //{
                //    zPos = -1 * (stepSizeZ * 16 / (period * period)) * (t - 3 * period / 4) * (t - 3 * period / 4) + stepSizeZ;
                //}
                //else if (t <= period)
                //{
                //    zPos = (stepSizeZ * 16 / (period * period)) * (t - period) * (t - period);
                //}
                else
                {
                    zPos = 0;
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






        /**
         * @fn  public void calcData()
         *
         * @brief   Calculates the data to send over i2c (Command = Set TCP position)
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

        public void calcData()
        {



            byte[] data = new byte[4];
            //set tcp position command
            data[0] = 3;
            //tcp position
            data[1] = (Byte)XPos;
            data[2] = (Byte)YPos;
            data[3] = (Byte)ZPos;

            //send data over i2c
            sendData(data);

        }

        /**
         * @fn  public void calcDataTerrain()
         *
         * @brief   Calculates the data to send over i2c (Command = Set TCP position with ground sensing)
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

        public void calcDataTerrain()
        {

            byte[] data = new byte[4];
            //set tcp position with ground sensing
            data[0] = 6;
            data[1] = (Byte)XPos;
            data[2] = (Byte)YPos;
            data[3] = (Byte)ZPos;
            sendData(data);
        }

        /**
         * @fn  public void setColor(ushort hue)
         *
         * @brief   Sets the color of the status led on the legcontroller
         *
         * @author  Alex
         * @date    13.08.2017
         *
         * @param   hue The hue value of the HSV color.
         */

        public void setColor(ushort hue)
        {
            byte[] byteArray = BitConverter.GetBytes(hue);

            byte[] data = new byte[3];
            //set led color command
            data[0] = 1;
            data[1] = byteArray[1];
            data[2] = byteArray[0];

            sendData(data);
        }



        /**
         * @fn  public void sendData(byte[] data)
         *
         * @brief   Sends a data array over i2c
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   data    The data.
         */

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

        /**
         * @fn  public void sendCalibrationData()
         *
         * @brief   Sends the calibration data
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

        public void sendCalibrationData()
        {

            byte[] data = new byte[4];
            //Write calibration data
            data[0] = 4;
            data[3] = (byte)alphaOff;
            data[2] = (byte)betaOff;
            data[1] = (byte)gammaOff;
            Debug.WriteLine("Writing calibration data!");
            sendData(data);
        }

        /**
         * @fn  public int readLegHeight()
         *
         * @brief   Reads leg height
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @return  The leg height in mm.
         */

        public int readLegHeight()
        {
            try
            {
                byte[] height = new byte[1];
                if (device != null)
                {
                    //read 1 byte
                    device.Read(height);
                    //return leg hight (signed byte!)
                    return (sbyte)height[0];
                }
                else
                {
                    //Debug.WriteLine("Error: I2C write failed!");
                    return 0;
                }

            }
            catch (Exception e)
            {
                Debug.WriteLine("Error: I2C hat read failed!" + e.Message);
            }
            return 0;
        }

        /**
         * @fn  public void calcPose(double yaw, double pitch, double roll, double a, double b, double c)
         *
         * @brief   Calculates the pose
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   yaw     The yaw.
         * @param   pitch   The pitch.
         * @param   roll    The roll.
         * @param   a       Distance to move along the x axis.
         * @param   b       Distance to move along the y axis.
         * @param   c       Distance to move along the z axis.
         */

        public void calcPose(double yaw, double pitch, double roll, double a, double b, double c)
        {

            //TCP coordinate in global coordinate system ( 0/0/0 = center of body ; +x = front; +z = top ; +y = left)
            double tempX = xOffset - xPos;
            double tempY = yOffset - yPos;
            double tempZ = zOffset - zPos;
            double sA = Math.Sin(yaw);
            double sB = Math.Sin(pitch);
            double sC = Math.Sin(roll);
            double cA = Math.Cos(yaw);
            double cB = Math.Cos(pitch);
            double cC = Math.Cos(roll);




            //Roll-Pitch-Yaw-Rotation matrix
            double newX = tempX * (cA * cB) + tempY * (cA * sB * sC - sA * cC) + tempZ * (sA * sC + cA * sB * cC);
            double newY = tempX * (sA * cB) + tempY * (cA * cC + sA * sB * sC) + tempZ * (sA * sB * cC - cA * sC);
            double newZ = tempX * (-sB) + tempY * (cB * sC) + tempZ * (cB * cC);

            //Set leg position in local coordinate system
            xPos += newX - tempX + a;
            yPos += newY - tempY + b;
            zPos += newZ - tempZ + c;

        }

        #endregion FUNCTIONS

    }
}
