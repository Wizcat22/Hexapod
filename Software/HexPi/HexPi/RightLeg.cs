using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HexPi
{
    class RightLeg : ILeg
    {
        private double zOffset = 95;
        private double A1 = 30;
        private double A2 = 60;
        private double A3 = 95;
        private double L1 = 0;
        private double L2 = 0;
        private double b = 0;
        private double alpha = 0;
        private double beta = 0;
        private double gamma = 0;
        private double t = 0;
        private double period = 100;
        private double stepSizeX = 30;
        private double stepSizeY = 30;
        private double stepSizeZ = 30;
        private double xPos = 0;
        private double yPos = 0;
        private double zPos = 0;
        private byte motorData0 = 0;
        private byte motorData1 = 0;
        private byte motorData2 = 0;




        public override void InverseKinematics()
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

        public void calcPositions(int direction, double increment)
        {
            //Stop motion
            if (direction == 0)
            {
                t = 0.0;
                xPos = 0.0;
                yPos = 0.0;
            }
            //Motion in x-direction
            else if (direction == 1)
            {
                t = (t + increment) % period;
                yPos = 0.0;
                if (t <= period / 2)
                {
                    xPos = 4 * stepSizeX / period * t - stepSizeX;
                }
                else
                {
                    xPos = -4 * stepSizeX / period * (t - period / 2) + stepSizeX;
                }


            }
            //Motion in y-direction
            else if (direction == 2)
            {
                t = (t + increment) % period;
                xPos = 0.0;
                if (t <= period / 2)
                {
                    yPos = 4 * stepSizeY / period * t - stepSizeY;
                }
                else
                {
                    yPos = -4 * stepSizeY / period * (t - period / 2) + stepSizeY;
                }

            }
        }

        public void calcData()
        {
            motorData0 = (byte)(1.38888888888 * alpha + 187.5);
            motorData1 = (byte)(1.38888888888 * beta + 187.5);
            motorData2 = (byte)(1.38888888888 * gamma + 187.5);
        }

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
    }
}
