using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HexPi
{
    sealed class RightLeg : ILeg
    {
        

        public override void inverseKinematics()
        {
            //ALPHA
            alpha = Math.Atan2(xPos, A1 + A2 + yPos);

            //BETA
            L1 = zOffset - zPos;
            L2 = A2 + yPos;
            b = Math.Sqrt(L1 * L1 + L2 * L2);

            beta = Math.Acos(L1 / b);
            beta = beta + Math.Acos((A2 * A2 - A3 * A3 + b * b) / (2 * A2 * b));

            //GAMMA
            gamma = Math.Acos((A3 * A3 - b * b + A2 * A2) / (2 * A3 * A2));

            //RAD TO DEG
            alpha = alpha * 180 / Math.PI;
            beta = (beta * 180 / Math.PI - 90) * 1;
            gamma = (gamma * 180 / Math.PI - 90) * -1;

            //Debug.WriteLine("DEBUG: " + alpha + " :: " + beta + " :: " + gamma);

        }

        //protected override void calcPositionZ()
        //{
        //    if (t <= period / 2)
        //    {
        //        zPos = 0 + zoff;
        //    }
        //    else
        //    {
        //        zPos = -1 * (stepSizeZ * 16 / (period * period)) * (t - period / 2 - period / 4) * (t - period / 2 - period / 4) + stepSizeZ + zoff;
        //    }
        //}

        public override void calcPositionR(double increment)
        {
            t = ((t + increment) % period + period) % period;
            yPos = 0.0;
            if (t <= period / 2)
            {
                xPos = 4 * stepSizeX / period * t - stepSizeX;
            }
            else
            {
                xPos = -4 * stepSizeX / period * (t - period / 2) + stepSizeX;
            }
            calcPositionZ();
        }
    }
}
