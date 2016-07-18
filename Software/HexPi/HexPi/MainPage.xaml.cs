using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;

// Die Vorlage "Leere Seite" ist unter http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409 dokumentiert.

namespace HexPi
{
    /// <summary>
    /// Eine leere Seite, die eigenständig verwendet oder zu der innerhalb eines Rahmens navigiert werden kann.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        Controller control;
        DispatcherTimer timer;
        DispatcherTimer timer2;
        public MainPage()
        {
            this.InitializeComponent();
            control = new Controller();
            control.init();

            timer = new DispatcherTimer();
            timer.Interval = System.TimeSpan.FromMilliseconds(10);
            timer.Tick += updateData;
            timer.Start();


        }

        private void updateData(object sender, object e)
        {
            tBlockLeg1X.Text = "X: " + control.getGuiData(0, 0);
            tBlockLeg1Y.Text = "Y: " + control.getGuiData(0, 1);
            tBlockLeg1Z.Text = "Z: " + control.getGuiData(0, 2);

            tBlockLeg2X.Text = "X: " + control.getGuiData(1, 0);
            tBlockLeg2Y.Text = "Y: " + control.getGuiData(1, 1);
            tBlockLeg2Z.Text = "Z: " + control.getGuiData(1, 2);

            tBlockLeg3X.Text = "X: " + control.getGuiData(2, 0);
            tBlockLeg3Y.Text = "Y: " + control.getGuiData(2, 1);
            tBlockLeg3Z.Text = "Z: " + control.getGuiData(2, 2);

            tBlockLeg4X.Text = "X: " + control.getGuiData(3, 0);
            tBlockLeg4Y.Text = "Y: " + control.getGuiData(3, 1);
            tBlockLeg4Z.Text = "Z: " + control.getGuiData(3, 2);

            tBlockLeg5X.Text = "X: " + control.getGuiData(4, 0);
            tBlockLeg5Y.Text = "Y: " + control.getGuiData(4, 1);
            tBlockLeg5Z.Text = "Z: " + control.getGuiData(4, 2);

            tBlockLeg6X.Text = "X: " + control.getGuiData(5, 0);
            tBlockLeg6Y.Text = "Y: " + control.getGuiData(5, 1);
            tBlockLeg6Z.Text = "Z: " + control.getGuiData(5, 2);

            tBlockBodyX.Text = "X: " + control.X;
            tBlockBodyY.Text = "Y: " + control.Y;
            tBlockBodyZ.Text = "Z: " + control.Z;
            tBlockBodyR.Text = "R: " + control.R;


        }
    }
}
