using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HexPi
{
    sealed class RightLeg : ILeg
    {
        public override void calcData()
        {
            motorData0 = (byte)(1.38888888888 * alpha + 187.5);
            motorData1 = (byte)(1.38888888888 * beta + 187.5);
            motorData2 = (byte)(1.38888888888 * gamma + 187.5);
        }      
    }
}
