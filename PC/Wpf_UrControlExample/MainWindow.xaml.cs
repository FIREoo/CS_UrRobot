using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using UrRobot.Socket;
using UrRobot.Coordinates;
using System.Net;
using System.Net.Sockets;

namespace Wpf_UrControlExample
{
    /// <summary>
    /// MainWindow.xaml 的互動邏輯
    /// </summary>
    public partial class MainWindow : Window
    {
        UrSocketControl UR = new UrSocketControl();
        public MainWindow()
        {
            InitializeComponent();
        }

        private void Btn_startServer_Click(object sender, RoutedEventArgs e)
        {

            UR.stopServer();
            UR.startServer("auto", 888);
        }

        private void Btn_gripper_Click(object sender, RoutedEventArgs e)
        {
            UR.goGripper(int.Parse(tb_grip.Text), int.Parse(tb_grip_f.Text), int.Parse(tb_grip_s.Text));
            UR.goGripper(255, 0, 200);
        }

        private void Btn_goPos_Click(object sender, RoutedEventArgs e)
        {
            UR.goPosition(new URCoordinates(0.45.M(), 0.0.M(), 0.14.M(), 3.14.rad(), 0.rad(), 0.rad()));
        }

        private void Btn_goJoint_Click(object sender, RoutedEventArgs e)
        {
            UR.goJoint(new URJoint(3.14.rad(), -1.57.rad(), 0.rad(), -1.57.rad(), 0.rad(), 0.rad()));
        }

        private void Btn_recordj_Click(object sender, RoutedEventArgs e)
        {
            UR.startRecord();
        }

        private void Btn_robotStop_Click(object sender, RoutedEventArgs e)
        {
            UR.Stop();

        }

        private void Btn_servoj1_Click(object sender, RoutedEventArgs e)
        {
            UR.goTrack(new URJoint(-1.5.rad(), -3.0.rad(), 1.1.rad(), -1.2.rad(), -1.7.rad(), 7.8.rad()));
        }

        private void Btn_servoj2_Click(object sender, RoutedEventArgs e)
        {
            UR.goTrack(new URJoint(-1.5.rad(), -3.0.rad(), 2.2.rad(), -2.3.rad(), -1.6.rad(), 7.8.rad()));
        }

        private void Btn_record_Click(object sender, RoutedEventArgs e)
        {
            UR.startRecord();
        }

        private void Btn_write_Click(object sender, RoutedEventArgs e)
        {
            UR.Record_joint();
        }

        private void Btn_endRecord_Click(object sender, RoutedEventArgs e)
        {
            UR.endRecord();
        }

        private void Btn_goFile_Click(object sender, RoutedEventArgs e)
        {
            UR.goFile("Path\\record.path");
        }

        private void Btn_Rmovep_Click(object sender, RoutedEventArgs e)
        {
            UR.goRelativePosition();
        }

        private void Btn_Rmovej_Click(object sender, RoutedEventArgs e)
        {
            UR.goRelativeJoint(j6: 1.rad());
        }
        //client
        UrSocketControl.Client URc = new UrSocketControl.Client();
        private void Btn_connectClient_Click(object sender, RoutedEventArgs e)
        {

            if (!URc.ClientConnect("192.168.1.104"))
                Console.WriteLine("~~~");

            URc.Client_RTDE();
        }

        private void Btn_clientPos_Click(object sender, RoutedEventArgs e)
        {
            Task.Run(() =>
            {
                while (true)
                    this.Dispatcher.Invoke((Action)(() => { lb_clientPos.Content = URc.getPosition().ToString("(3)", "0.00"); }));
            });
        }

        private void Btn_clientForce_Click(object sender, RoutedEventArgs e)
        {
            Task.Run(() =>
            {
                while (true)
                {
                    this.Dispatcher.Invoke((Action)(() => { lb_clientForce.Content = URc.getFilterForce().ToString("3","0"); }));
                }
            });
        }
    }
}
