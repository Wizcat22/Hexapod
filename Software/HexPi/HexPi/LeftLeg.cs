using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HexPi
{
    sealed class LeftLeg : ILeg
    {
        //Functions
        public LeftLeg(int tOffset, int aOff, int bOff, int cOff , double rotation)
        {
            this.tOffset = tOffset;
            t = this.tOffset;

            alphaOff = aOff;
            betaOff = bOff;
            gammaOff = cOff;

            this.rotation = (rotation / 180) * Math.PI;
        }

        public override void inverseKinematics()
        {
            //ALPHA
            alpha = Math.Atan2(xPos, A1 + A2 + yPos);

            //BETA
            L1 = zOffset - zPos;
            L2 = A2 - yPos;
            b = Math.Sqrt(L1 * L1 + L2 * L2);

            beta = Math.Acos(L1 / b);
            beta = beta + Math.Acos((A2 * A2 - A3 * A3 + b * b) / (2 * A2 * b));

            //GAMMA
            gamma = Math.Acos((A3 * A3 - b * b + A2 * A2) / (2 * A3 * A2));

            //RAD TO DEG
            alpha = (alpha * 180 / Math.PI - alphaOff) * 1;
            beta = (beta * 180 / Math.PI - betaOff - 90) * -1;
            gamma = (gamma * 180 / Math.PI - gammaOff - 90) * 1;

            //Debug.WriteLine("DEBUG: " + alpha + " :: " + beta + " :: " + gamma);

        }

        public override void calcPositionR(double increment)
        {
            
            t = ((t - increment) % period + period) % period;
            if (t <= period / 2)
            {
                xPos = 4 * ((stepSizeR * Math.Cos(rotation)) / period) * t - (stepSizeR * Math.Cos(rotation));
                yPos = 4 * ((stepSizeR * Math.Sin(rotation)) / period) * t - (stepSizeR * Math.Sin(rotation));
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
