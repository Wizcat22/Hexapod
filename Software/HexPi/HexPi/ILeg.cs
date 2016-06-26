using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HexPi
{
    abstract class ILeg
    {
        protected double zOffset = 95;
        protected double A1 = 30;
        protected double A2 = 60;
        protected double A3 = 95;
        protected double L1 = 0;
        protected double L2 = 0;
        protected double b = 0;
        protected double alpha = 0;
        protected double beta = 0;
        protected double gamma = 0;
        protected double t = 0;
        protected double tOffset = 0;
        protected double period = 100;
        protected double stepSizeX = 30;
        protected double stepSizeY = 30;
        protected double stepSizeZ = 30;
        protected double xPos = 0;
        protected double yPos = 0;
        protected double zPos = 0;
        protected byte motorData0 = 0;
        protected byte motorData1 = 0;
        protected byte motorData2 = 0;


        public double XPos
        {
            get
            {
                return xPos;
            }
        }

        public double YPos
        {
            get
            {
                return yPos;
            }
        }

        public double ZPos
        {
            get
            {
                return zPos;
            }
        }

        public void inverseKinematics()
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
            t = (t + increment) % period;

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
            //Motion in z-direction
            if (t <= period / 2)
            {
                zPos = 0;
            }
            else
            {
                zPos = -1 * stepSizeZ / (period * period) * (t - (period / 2)) * (t - (period / 2)) + stepSizeZ;
            }



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

        public abstract void calcData();
        
    }
}
