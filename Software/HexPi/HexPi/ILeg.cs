using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HexPi
{
    abstract class ILeg
    {
        //Fields
        protected double xOff = 0.0;
        protected double yOff = 0.0;
        protected double zOff = 0.0;
        protected double L1 = 0;
        protected double L2 = 0;
        protected double b = 0;
        protected double alpha = 0;
        protected double beta = 0;
        protected double gamma = 0;
        protected double alphaOff = 0;
        protected double betaOff = 0;
        protected double gammaOff = 0;
        protected double t = 0;
        protected double tOffset = 0;
        protected double xPos = 0;
        protected double yPos = 0;
        protected double zPos = 0;
        protected byte motorData0 = 0;
        protected byte motorData1 = 0;
        protected byte motorData2 = 0;
        //******

        //Constants
        protected const double stepSizeX = 30;
        protected const double stepSizeY = 30;
        protected const double stepSizeZ = 30;
        protected const double period = 100;
        protected const double zOffset = 95;
        protected const double A1 = 30;
        protected const double A2 = 65;
        protected const double A3 = 95;
        //******

        //Properties
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

        public double StepSizeX
        {
            get
            {
                return stepSizeX;
            }
        }

        public double StepSizeY
        {
            get
            {
                return stepSizeY;
            }
        }

        public double StepSizeZ
        {
            get
            {
                return stepSizeZ;
            }
        }

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
        public abstract void inverseKinematics();

        public abstract void calcPositionR(double increment);
        //******


        //Functions
        public void calcPositionCenter()
        {
            t = tOffset;
            xPos = 0.0;
            yPos = 0.0;
        }

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
            calcPositionZ();
        }

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
            calcPositionZ();
        }

        protected void calcPositionZ()
        {
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

        public void calcData()
        {
            motorData0 = (byte)(1.38888888888 * alpha + 187.5);
            motorData1 = (byte)(1.38888888888 * beta + 187.5);
            motorData2 = (byte)(1.38888888888 * gamma + 187.5);
        }
        //******
    }
}
