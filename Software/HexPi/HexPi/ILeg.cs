/**********************************************************************************************//**
 * @file    ileg.cs
 *
 * @brief   Implements the ileg class.
 **************************************************************************************************/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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

    abstract class ILeg
    {
        //Fields
        
        /** @brief   The TCP offset of the x coordinate. */
        protected double xOff = 0.0;

        /** @brief   The TCP offset of the y coordinate. */
        protected double yOff = 0.0;

        /** @brief   The TCP offset of the z coordinate. */
        protected double zOff = 0.0;

        /** @brief   The height of the first joint minus the actual TCP z position . */
        protected double L1 = 0;

        /** @brief   The distance of the second joint to the TCP. */
        protected double L2 = 0;

        /** @brief   The lenght of the direct connection of the second joint and the TCP. */
        protected double L3 = 0;
       
        /** @brief   The angle of the first joint. */
        protected double alpha = 0;

        /** @brief   The angle of the second joint. */
        protected double beta = 0;

        /** @brief   The angle of the third joint. */
        protected double gamma = 0;

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

        /** @brief   The data for the first motor. */
        protected byte motorData0 = 0;
        
        /** @brief   The data for the second motor. */
        protected byte motorData1 = 0;

        /** @brief   The data for the third motor. */
        protected byte motorData2 = 0;

        /** @brief   The rotation. */
        protected double rotation = 0;
        //******

        //Constants
        
        /** @brief   The step size for x coordinate in mm. */
        protected const double stepSizeX = 30;

        /** @brief   The step size for y coordinate in mm. */
        protected const double stepSizeY = 30;

        /** @brief   The step size for z coordinate in mm. */
        protected const double stepSizeZ = 30;

        /** @brief   The step size for rotation in mm. */
        protected const double stepSizeR = 30;

        /** @brief   The period. */
        protected const double period = 100;

        /** @brief   The height of the first joint. */
        protected const double zOffset = 95;

        /** @brief   The distance between the first and second joint in mm. */
        protected const double A1 = 30;

        /** @brief   The lenght of the upper leg in mm. */
        protected const double A2 = 65;

        /** @brief   The lenght of the lower leg in mm. */
        protected const double A3 = 95;
        //******

        //Properties

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

        /**********************************************************************************************//**
         * @property    public double Xoff
         *
         * @brief   Gets or sets the x offset.
         *
         * @return  The XOFF.
         **************************************************************************************************/

        public double Xoff
        {
            get
            {
                return xOff;
            }

            set
            {
                xOff = value;
            }
        }

        /**********************************************************************************************//**
         * @property    public double Yoff
         *
         * @brief   Gets or sets the y offset.
         *
         * @return  The yoff.
         **************************************************************************************************/

        public double Yoff
        {
            get
            {
                return yOff;
            }

            set
            {
                yOff = value;
            }
        }

        /**********************************************************************************************//**
         * @property    public double Zoff
         *
         * @brief   Gets or sets the z offset.
         *
         * @return  The zoff.
         **************************************************************************************************/

        public double Zoff
        {
            get
            {
                return zOff;
            }

            set
            {
                zOff = value;
                if (zOff > 2 * stepSizeZ / 3)
                {
                    zOff = 2 * stepSizeZ / 3;
                }
                else if (zOff < -2 * stepSizeZ / 3)
                {
                    zOff = -2 * stepSizeZ / 3;
                }
            }
        }
        //******

        //Abstract functions

        /**********************************************************************************************//**
         * @fn  public abstract void inverseKinematics();
         *
         * @brief   Inverse kinematics.
         *          This function calculates the motorangles based on the TCP position and lenght of the leg.
         *          The calculations are different for the right and left side of the robot.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        public abstract void inverseKinematics();

        /**********************************************************************************************//**
         * @fn  public abstract void calcPositionR(double increment);
         *
         * @brief   Calculates the leg position in a rotational movement.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   increment   Amount to increment by.
         **************************************************************************************************/

        public abstract void calcPositionR(double increment);
        //******


        //Functions

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

        public void calcPositionX(double increment)
        {
            t = ((t + increment) % period + period) % period;
            yPos = 0.0;
            if (t <= period / 2)
            {
                xPos = 4 * stepSizeX / period * t - stepSizeX + xOff;
            }
            else
            {
                xPos = -4 * stepSizeX / period * (t - period / 2) + stepSizeX + xOff;
            }
            calcPositionZ(false);
        }

        /**********************************************************************************************//**
         * @fn  public void calcPositionY(double increment)
         *
         * @brief   Calculates the TCP position for movement in y direction.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   increment   Amount to increment by.
         **************************************************************************************************/

        public void calcPositionY(double increment)
        {
            t = ((t + increment) % period + period) % period;
            xPos = 0.0; //+ xoff;
            if (t <= period / 2)
            {
                yPos = 4 * stepSizeY / period * t - stepSizeY + yOff;
            }
            else
            {
                yPos = -4 * stepSizeY / period * (t - period / 2) + stepSizeY + yOff;
            }
            calcPositionZ(false);
        }

        /**********************************************************************************************//**
         * @fn  public void calcPositionZ(bool off)
         *
         * @brief   Calculates the TCP position for movement in z direction.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   off Turns the offset of.
         **************************************************************************************************/

        public void calcPositionZ(bool off)
        {
            if(off)
            {
                t = 0.0;
            }
            if (t <= period / 2)
            {
                zPos = 0 + zOff;

            }
            else
            {
                zPos = -1 * ((stepSizeZ - Math.Abs(zOff)) * 16 / (period * period)) * (t - 3 * period / 4) * (t - 3 * period / 4) + stepSizeZ - Math.Abs(zOff) + zOff;
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
         * @fn  public byte getMotorData(int n)
         *
         * @brief   Gets motor data.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   n   Chooses the motor.
         *
         * @return  The motor data.
         **************************************************************************************************/

        public byte getMotorData(int n)
        {
            switch (n)
            {
                case 0: return motorData0;
                case 1: return motorData1;
                case 2: return motorData2;
                default: return 100;
            }

        }

        /**********************************************************************************************//**
         * @fn  public void calcData()
         *
         * @brief   Calculates the data for the Servo Hat.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        public void calcData()
        {
            motorData0 = (byte)(1.38888888888 * alpha + 187.5);
            motorData1 = (byte)(1.38888888888 * beta + 187.5);
            motorData2 = (byte)(1.38888888888 * gamma + 187.5);
        }
        //******
    }
}
