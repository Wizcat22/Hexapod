///**********************************************************************************************//**
// * @file    rightleg.cs
// *
// * @brief   Implements the rightleg class.
// **************************************************************************************************/

//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;

//namespace HexPi
//{
//    sealed class RightLeg : ILeg
//    {
//        //Functions

//        /**********************************************************************************************//**
//         * @fn  public RightLeg(int tOffset, int aOff, int bOff, int cOff , double rotation)
//         *
//         * @brief   Constructor.
//         *
//         * @author  Alexander Miller
//         * @date    11.08.2016
//         *
//         * @param   tOffset     The control offset.
//         * @param   aOff        The offset for alpha.
//         * @param   bOff        The offset for beta.
//         * @param   cOff        The offset for gamma.
//         * @param   rotation    The angle of the leg path in rotation.
//         **************************************************************************************************/

//        public RightLeg(int tOffset, int aOff, int bOff, int cOff, double rotation, int address)
//        {
//            this.tOffset = tOffset;
//            t = this.tOffset;

//            alphaOff = aOff;
//            betaOff = bOff;
//            gammaOff = cOff;

//            this.rotation = (rotation/180) * Math.PI;

//                init(address);
            

            
//        }

       
//        /**********************************************************************************************//**
//         * @fn  public override void calcPositionR(double increment)
//         *
//         * @brief   Calculates the leg position in a rotational movement.
//         *
//         * @author  Alexander Miller
//         * @date    11.08.2016
//         *
//         * @param   increment   Amount to increment by.
//         **************************************************************************************************/

//        public override void calcPositionR(double increment)
//        {

//            t = ((t - increment) % period + period) % period;
//            if (t <= period / 2)
//            {
//                xPos = 4 * ((stepSizeR * Math.Cos(rotation)) / period) * t - (stepSizeR * Math.Cos(rotation));
//                yPos = 4 * ((stepSizeR * Math.Sin(rotation)) / period) * t - (stepSizeR * Math.Sin(rotation)); ;
//            }
//            else
//            {
//                xPos = -4 * ((stepSizeR * Math.Cos(rotation)) / period) * (t - period / 2) + (stepSizeR * Math.Cos(rotation));
//                yPos = -4 * ((stepSizeR * Math.Sin(rotation)) / period) * (t - period / 2) + (stepSizeR * Math.Sin(rotation));
//            }
//            calcPositionZ();
//        }
//        //******
//    }
//}
