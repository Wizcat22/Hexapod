using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HexPi
{
    abstract class ILeg
    {
        protected const double zOffset = 95;
        protected double xoff = 0.0;
        protected double yoff = 0.0;
        protected double zoff = 0.0;
        protected const double A1 = 30;
        protected const double A2 = 65;
        protected const double A3 = 95;
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
                return xoff;
            }

            set
            {
                xoff = value;
            }
        }

        public double Yoff
        {
            get
            {
                return yoff;
            }

            set
            {
                yoff = value;
            }
        }

        public double Zoff
        {
            get
            {
                return zoff;
            }

            set
            {
                zoff = value;
                if (zoff>stepSizeZ-10)
                {
                    zoff = stepSizeZ - 10;
                }
                else if(zoff < -stepSizeZ+10)
                {
                    zoff = -stepSizeZ + 10;
                }
            }
        }

        public abstract void inverseKinematics();





        public void calcPositionCenter(double increment)
        {
            t = ((t + increment) % period + period) % period;
            t = tOffset;
            xPos = 0.0;
            yPos = 0.0;
        }

        public void calcPositionX(double increment)
        {
            t = ((t + increment) % period + period) % period;
            yPos = 0.0; //+ (bodyWidth/2) - Math.Abs(yoff);
            if (t <= period / 2)
            {
                xPos = 4 * stepSizeX / period * t - stepSizeX + xoff;
            }
            else
            {
                xPos = -4 * stepSizeX / period * (t - period / 2) + stepSizeX + xoff;
            }
            calcPositionZ();
        }

        public void calcPositionY(double increment)
        {
            t = ((t + increment) % period + period) % period;
            xPos = 0.0; //+ xoff;
            if (t <= period / 2)
            {
                yPos = 4 * stepSizeY / period * t - stepSizeY + yoff;
            }
            else
            {
                yPos = -4 * stepSizeY / period * (t - period / 2) + stepSizeY + yoff;
            }
            calcPositionZ();
        }

        protected void calcPositionZ()
        {
            if (t <= period / 2)
            {
                zPos = 0 + zoff;

            }
            else
            {
                zPos = -1 * (stepSizeZ * 16 / (period * period)) * (t - period / 2 - period / 4) * (t - period / 2 - period / 4) + stepSizeZ + zoff;
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

        public abstract void calcPositionR(double increment);

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

    }
}
