/**********************************************************************************************//**
 * @file    rightleg.cs
 *
 * @brief   Implements the rightleg class.
 **************************************************************************************************/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HexPi
{
    sealed class RightLeg : ILeg
    {
        //Functions

        /**********************************************************************************************//**
         * @fn  public RightLeg(int tOffset, int aOff, int bOff, int cOff , double rotation)
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

        public RightLeg(int tOffset, int aOff, int bOff, int cOff, double rotation)
        {
            this.tOffset = tOffset;
            t = this.tOffset;

            alphaOff = aOff;
            betaOff = bOff;
            gammaOff = cOff;

            this.rotation = (rotation/180) * Math.PI;
        }

        /**********************************************************************************************//**
         * @fn  public override void inverseKinematics()
         *
         * @brief   Inverse kinematics.
         *          This function calculates the motorangles based on the TCP position and lenght of the leg.
         *          The calculations are different for the right and left side of the robot.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/
        public override void inverseKinematics()
        {
            //ALPHA
            alpha = Math.Atan2(xPos, A1 + A2 + yPos);

            //BETA
            L1 = zOffset - zPos;
            L2 = A2 + yPos;
            L3 = Math.Sqrt(L1 * L1 + L2 * L2);

            beta = Math.Acos(L1 / L3);
            beta = beta + Math.Acos((A2 * A2 - A3 * A3 + L3 * L3) / (2 * A2 * L3));

            //GAMMA
            gamma = Math.Acos((A3 * A3 - L3 * L3 + A2 * A2) / (2 * A3 * A2));

            //RAD TO DEG
            alpha = (alpha * 180 / Math.PI - alphaOff) * -1;
            beta = (beta * 180 / Math.PI - betaOff - 90) * 1;
            gamma = (gamma * 180 / Math.PI - gammaOff - 90) * -1;

            //Debug.WriteLine("DEBUG: " + alpha + " :: " + beta + " :: " + gamma);

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

        public override void calcPositionR(double increment)
        {

            t = ((t - increment) % period + period) % period;
            if (t <= period / 2)
            {
                xPos = 4 * ((stepSizeR * Math.Cos(rotation)) / period) * t - (stepSizeR * Math.Cos(rotation));
                yPos = 4 * ((stepSizeR * Math.Sin(rotation)) / period) * t - (stepSizeR * Math.Sin(rotation)); ;
            }
            else
            {
                xPos = -4 * ((stepSizeR * Math.Cos(rotation)) / period) * (t - period / 2) + (stepSizeR * Math.Cos(rotation));
                yPos = -4 * ((stepSizeR * Math.Sin(rotation)) / period) * (t - period / 2) + (stepSizeR * Math.Sin(rotation));
            }
            calcPositionZ(false);
        }
        //******
    }
}
