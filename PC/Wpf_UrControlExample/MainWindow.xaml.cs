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
            UR.startServer("192.168.1.103",888);
        }

        private void Btn_gripper_Click(object sender, RoutedEventArgs e)
        {
            UR.goGripper(int.Parse(tb_grip.Text), int.Parse(tb_grip_f.Text), int.Parse(tb_grip_s.Text));
            UR.goGripper(255,0,200);
        }

        private void Btn_goPos_Click(object sender, RoutedEventArgs e)
        {
            UR.goPosition(new URCoordinates(0.45f, 0.0f, 0.14f, 3.14f, 0, 0));
            //UR.goPosition(new URCoordinates(0.45.M(), 0.0.M(), 0.14.M(), 3.14f.rad(), 0, 0));
        }

        private void Btn_goJoint_Click(object sender, RoutedEventArgs e)
        {
           UR.goJoint(3.14f,-1.57f,0,-1.57f,0,0);
        }

        private void Btn_recordj_Click(object sender, RoutedEventArgs e)
        {
            UR.startRecord();
        }

        private void Btn_robotStop_Click(object sender, RoutedEventArgs e)
        {
            UR.Stop();

        }
    }
}
