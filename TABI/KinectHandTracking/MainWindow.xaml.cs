using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
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

namespace KinectHandTracking
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Members

        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        IList<Body> _bodies;

        //Sisi Kanan
        Vector SR = new Vector(), ER = new Vector(), WR = new Vector(), HR = new Vector();
        Vector SRER = new Vector(), ERWR = new Vector(), WRHR = new Vector();
        private double deltaHRx = 0, deltaHRy = 0;
        private double tempHRx = 0, tempHRy = 0;
        private double alphaHR = 0;
        private int[] kuantKanan = new int[99];

        //Sisi Kiri
        Vector SL = new Vector(), EL = new Vector(), WL = new Vector(), HL = new Vector();
        Vector SLEL = new Vector(), ELWL = new Vector(), WLHL = new Vector();
        private double deltaHLx = 0, deltaHLy = 0;
        private double tempHLx = 0, tempHLy = 0;
        private double alphaHL = 0;
        private int[] kuantKiri = new int[99];

        //Sisi Tengah
        private double Ny = 0, SMy = 0;
        Vector SS = new Vector();
        Vector HRHL = new Vector();

        //Posisi
        private string tanganKanan = "", tanganKiri = "";

        //angel
        double SSSRER = new double();
        double SRERWR = new double();
        double ERWRHR = new double();
        double SSSLEL = new double();
        double SLELWL = new double();
        double ELWLHL = new double();

        double DisHRHL = new double();

        //Flag
        private int flag1 = 0, flag2 = 0, flag3 = 0, flag4 = 0;
        private int i = 0, j = 0;
        private int statusAmbil = 0;
        private string namaGerakan = "";

        #endregion

        #region Constructor

        public MainWindow()
        {
            InitializeComponent();
        }

        #endregion

        private void InitStaticFeatures()
        {
            Vector SRER = new Vector();
            Vector ERWR = new Vector();
            Vector WRHR = new Vector();
            Vector SLEL = new Vector();
            Vector ELWL = new Vector();
            Vector WLHL = new Vector();
            Vector HRHL = new Vector();
            double SSSRER = new double();
            double SRERWR = new double();
            double ERWRHR = new double();
            double SSSLEL = new double();
            double SLELWL = new double();
            double ELWLHL = new double();
            double DisHRHL = new double();
        }

        #region Event handlers

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.GetDefault();

            if (_sensor != null)
            {
                _sensor.Open();
                statusDetail.Content = "Idle";
                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
                
            }
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            if (_reader != null)
            {
                _reader.Dispose();
            }

            if (_sensor != null)
            {
                _sensor.Close();
            }
        }

        async void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();

            #region Acquire Frame Color
            // Color
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    camera.Source = frame.ToBitmap();
                }
            }
            #endregion

            // Body
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    canvas.Children.Clear();
                    _bodies = new Body[frame.BodyFrameSource.BodyCount];
                    frame.GetAndRefreshBodyData(_bodies);

                    //Deklarasi Variable  untuk write data 
                    var csv = new StringBuilder();
                    string filePath = "E:\\Kuliah\\TUGAS AKHIR\\TABI\\Data\\DataSet\\dataori.csv";
                    string imagePath = "";

                    foreach (var body in _bodies)
                    {
                        if (body != null)
                        {
                            if (body.IsTracked)
                            {
                                //deteksi Joint
                                Joint handRight = body.Joints[JointType.HandRight];
                                Joint handLeft = body.Joints[JointType.HandLeft];
                                Joint wristRight = body.Joints[JointType.WristRight];
                                Joint wristLeft = body.Joints[JointType.WristLeft];
                                Joint elbowRight = body.Joints[JointType.ElbowRight];
                                Joint elbowLeft = body.Joints[JointType.ElbowLeft];
                                Joint shoulderRight = body.Joints[JointType.ShoulderRight];
                                Joint shoulderLeft = body.Joints[JointType.ShoulderLeft];
                                Joint spineMid = body.Joints[JointType.SpineMid];
                                Joint spineShoulder = body.Joints[JointType.SpineShoulder];
                                Joint neck = body.Joints[JointType.Neck];

                                //Coordinate mapping
                                foreach (Joint joint in body.Joints.Values)
                                {
                                    if (joint.TrackingState == TrackingState.Tracked)
                                    {
                                        // 3D space point
                                        CameraSpacePoint jointPosition = joint.Position;
                                        // 2D space point
                                        Point point = new Point();

                                        ColorSpacePoint colorPoint = _sensor.CoordinateMapper.MapCameraPointToColorSpace(jointPosition);
                                        point.X = float.IsInfinity(colorPoint.X) ? 0 : colorPoint.X;
                                        point.Y = float.IsInfinity(colorPoint.Y) ? 0 : colorPoint.Y;

                                        Ellipse ellipse = new Ellipse
                                        {
                                            Fill = Brushes.Yellow,
                                            Width = 30,
                                            Height = 30
                                        };

                                        Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
                                        Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2);

                                        canvas.Children.Add(ellipse);
                                    }
                                }

                                #region Notif State

                                // Find the hand states
                                string rightHandState = "-";
                                string leftHandState = "-";

                                switch (body.HandRightState)
                                {
                                    case HandState.Open:
                                        rightHandState = "Open";
                                        break;
                                    case HandState.Closed:
                                        rightHandState = "Closed";
                                        break;
                                    case HandState.Lasso:
                                        rightHandState = "Lasso";
                                        break;
                                    case HandState.Unknown:
                                        rightHandState = "Unknown";
                                        break;
                                    case HandState.NotTracked:
                                        rightHandState = "Not tracked";
                                        break;
                                    default:
                                        break;
                                }

                                switch (body.HandLeftState)
                                {
                                    case HandState.Open:
                                        leftHandState = "Open";
                                        break;
                                    case HandState.Closed:
                                        leftHandState = "Closed";
                                        break;
                                    case HandState.Lasso:
                                        leftHandState = "Lasso";
                                        break;
                                    case HandState.Unknown:
                                        leftHandState = "Unknown";
                                        break;
                                    case HandState.NotTracked:
                                        leftHandState = "Not tracked";
                                        break;
                                    default:
                                        break;
                                }
                                // buat nampilkan di xaml
                                
                                 tblRightHandState.Content = rightHandState;
                                 tblLeftHandState.Content = leftHandState;

                                #endregion

                                #region Untuk Membuat Dataset Baru
                                // Koordinat Tangan
                                //Koordinat setelah filter .xx
                                SS.X = Math.Round((spineShoulder.Position.X), 2); SS.Y = Math.Round((spineShoulder.Position.Y), 2);

                                SR.X = Math.Round((shoulderRight.Position.X), 2); SR.Y = Math.Round((shoulderRight.Position.Y), 2);
                                ER.X = Math.Round((elbowRight.Position.X), 2); ER.Y = Math.Round((elbowRight.Position.Y), 2);
                                WR.X = Math.Round((wristRight.Position.X), 2); WR.Y = Math.Round((wristRight.Position.Y), 2);
                                HR.X = Math.Round((handRight.Position.X), 2); HR.Y = Math.Round((handRight.Position.Y), 2);

                                SL.X = Math.Round((shoulderLeft.Position.X), 2); SL.Y = Math.Round((shoulderLeft.Position.Y), 2);
                                EL.X = Math.Round((elbowLeft.Position.X), 2); EL.Y = Math.Round((elbowLeft.Position.Y), 2);
                                WL.X = Math.Round((wristLeft.Position.X), 2); WL.Y = Math.Round((wristLeft.Position.Y), 2);
                                HL.X = Math.Round((handLeft.Position.X), 2); HL.Y = Math.Round((handLeft.Position.Y), 2);
                                


                                // buat nampilkan di xaml
                                xR.Content = HR.X;
                                yR.Content = HR.Y;
                                xL.Content = HL.X;
                                yL.Content = HL.Y;



                                if (flag1 == 0)
                                {
                                    deltaHRx = HR.X; deltaHRy = HR.Y;
                                    deltaHLx = HL.X; deltaHLy = HL.Y;
                                    flag1 = 1;
                                }
                                else
                                {
                                    deltaHRx = HR.X - tempHRx; deltaHRy = HR.Y - tempHRy;
                                    deltaHLx = HL.X - tempHLx; deltaHLy = HL.Y - tempHLy;
                                }
                                tempHRx = HR.X; tempHRy = HR.Y;
                                tempHLx = HL.X; tempHLy = HL.Y;

                                // kondisi idle
                                

                                if (i < 40 && statusAmbil != 0)
                                {
                                    if (flag2 == 0 && statusAmbil == 1)
                                    {
                                        await Task.Delay(5000);
                                        flag2 = 1;
                                    }

                                    if (flag2 == 0 && statusAmbil == 2)
                                    {
                                        await Task.Delay(3000);
                                        flag2 = 1;
                                    }

                                    #region Status Frame
                                    if (i < 38)
                                    {
                                        ambilData.Content = (i + 1).ToString();
                                    }
                                    else ambilData.Content = "Done";
                                    #endregion

                                    #region Identifikasi Posisi
                                    if (i == 1)
                                    {
                                        Ny = (neck.Position.Y);
                                        SMy = (spineMid.Position.Y);
                                    }
                                    #endregion

                                    #region Frame Tengah untuk Posisi
                                    if (i == 20)
                                    {
                                        if (HL.Y > Ny) tanganKiri = "Kepala";
                                        else if (HL.Y < SMy) tanganKiri = "Perut";
                                        else tanganKiri = "Dada";

                                        if (HR.Y > Ny) tanganKanan = "Kepala";
                                        else if (HR.Y < SMy) tanganKanan = "Perut";
                                        else tanganKanan = "Dada";
                                    }
                                    #endregion

                                    #region Ekstraksi Fitur Dinamis
                                    // tangan kanan
                                    if (deltaHRx == 0 && deltaHRy == 0)
                                    {
                                        alphaHR = -1;
                                    }
                                    else if (deltaHRx >= 0 && deltaHRy >= 0)
                                    {
                                        alphaHR = (Math.Atan(deltaHRy / deltaHRx)) * (180 / Math.PI);
                                    }
                                    else if (deltaHRx < 0)
                                    {
                                        alphaHR = (Math.Atan(deltaHRy / deltaHRx)) * (180 / Math.PI) + 180;
                                    }
                                    else
                                    {
                                        alphaHR = (Math.Atan(deltaHRy / deltaHRx)) * (180 / Math.PI) + 360;
                                    }

                                    // tangan kiri
                                    if (deltaHLx == 0 && deltaHLy == 0)
                                    {
                                        alphaHL = -1;
                                    }
                                    else if (deltaHLx >= 0 && deltaHLy >= 0)
                                    {
                                        alphaHL = (Math.Atan(deltaHLy / deltaHLx)) * (180 / Math.PI);
                                    }
                                    else if (deltaHRx < 0)
                                    {
                                        alphaHL = (Math.Atan(deltaHLy / deltaHLx)) * (180 / Math.PI) + 180;
                                    }
                                    else
                                    {
                                        alphaHL = (Math.Atan(deltaHLy / deltaHLx)) * (180 / Math.PI) + 360;
                                    }
                                    #endregion

                                    #region Fitur Dinamis
                                    // tangan kanan 
                                    if (alphaHR >= 315) { kuantKanan[i] = 8; }
                                    else if (alphaHR >= 270) { kuantKanan[i] = 7; }
                                    else if (alphaHR >= 225) { kuantKanan[i] = 6; }
                                    else if (alphaHR >= 180) { kuantKanan[i] = 5; }
                                    else if (alphaHR >= 135) { kuantKanan[i] = 4; }
                                    else if (alphaHR >= 90) { kuantKanan[i] = 3; }
                                    else if (alphaHR >= 45) { kuantKanan[i] = 2; }
                                    else if (alphaHR >= 0) { kuantKanan[i] = 1; }
                                    else if (alphaHR < 0) { kuantKanan[i] = 0; }

                                    // tangan kiri
                                    if (alphaHL >= 315) { kuantKiri[i] = 8; }
                                    else if (alphaHL >= 270) { kuantKiri[i] = 7; }
                                    else if (alphaHL >= 225) { kuantKiri[i] = 6; }
                                    else if (alphaHL >= 180) { kuantKiri[i] = 5; }
                                    else if (alphaHL >= 135) { kuantKiri[i] = 4; }
                                    else if (alphaHL >= 90) { kuantKiri[i] = 3; }
                                    else if (alphaHL >= 45) { kuantKiri[i] = 2; }
                                    else if (alphaHL >= 0) { kuantKiri[i] = 1; }
                                    else if (alphaHL < 0) { kuantKiri[i] = 0; }
                                    #endregion

                                    #region Ekstraksi Fitur Statis 
                                    if (1 < i && i < 38)
                                    {

                                        SRER += ER - SR;
                                        ERWR += WR - ER;
                                        WRHR += HR - WR;
                                        SLEL += EL - SL;
                                        ELWL += WL - EL;
                                        WLHL += HL - WL;
                                        HRHL += HL - HR;


                                        Vector v1, v2;
                                        double res;

                                        //SS-SR-ER
                                        v1 = SS - SR;
                                        v2 = ER - SR;
                                        res = Vector.AngleBetween(v1, v2);
                                        SSSRER += (double)res * Math.PI / 180;

                                        //SR-ER-WR
                                        v1 = SR - ER;
                                        v2 = WR - ER;
                                        res = Vector.AngleBetween(v1, v2);
                                        SRERWR += (double)res * Math.PI / 180;

                                        //ER-WR-HR
                                        v1 = ER - WR;
                                        v2 = HR - WR;
                                        res = Vector.AngleBetween(v1, v2);
                                        ERWRHR += (double)res * Math.PI / 180;

                                        //SS-SL-EL
                                        v1 = SS - SL;
                                        v2 = EL - SL;
                                        res = Vector.AngleBetween(v1, v2);
                                        SSSLEL += (double)res * Math.PI / 180;

                                        //SL-EL-WL
                                        v1 = SL - EL;
                                        v2 = WL - EL;
                                        res = Vector.AngleBetween(v1, v2);
                                        SLELWL += (double)res * Math.PI / 180;

                                        //EL-WL-HL
                                        v1 = EL - WL;
                                        v2 = HL - WL;
                                        res = Vector.AngleBetween(v1, v2);
                                        ELWLHL += (double)res * Math.PI / 180;

                                        //Distance HR - HL
                                        DisHRHL += Math.Sqrt((HR.X - HL.X) * (HR.X - HL.X) + (HR.Y - HL.Y) * (HR.Y - HL.Y));   // sqrt from dotproduct = length
                                    }
                                    #endregion
                                    
                                    #region data
                                    if (i == 38)
                                    {
                                        #region data statis
                                        SRER /= 36;
                                        ERWR /= 36;
                                        WRHR /= 36;
                                        SLEL /= 36;
                                        ELWL /= 36;
                                        WLHL /= 36;
                                        HRHL /= 36;
                                        SSSRER /= 36;
                                        SRERWR /= 36;
                                        ERWRHR /= 36;
                                        SSSLEL /= 36;
                                        SLELWL /= 36;
                                        ELWLHL /= 36;
                                        DisHRHL /= 36;

                                        var stringSRER = SRER.ToString(); var stringERWR = ERWR.ToString(); var stringWRHR = WRHR.ToString();
                                        var stringSLEL = SLEL.ToString(); var stringELWL = ELWL.ToString(); var stringWLHL = WLHL.ToString();
                                        var stringHRHL = HRHL.ToString();
                                        var stringSSSRER = SSSRER.ToString(); var stringSRERWR = SRERWR.ToString(); var stringERWRHR = ERWRHR.ToString();
                                        var stringSSSLEL = SSSLEL.ToString(); var stringSLELWL = SLELWL.ToString(); var stringELWLHL = ELWLHL.ToString();
                                        var stingDisHRHL = DisHRHL.ToString();
                                        #endregion

                                        #region data dinamis
                                        var stringkuant4 = kuantKiri[4].ToString(); var stringkuant6 = kuantKiri[6].ToString(); var stringkuant8 = kuantKiri[8].ToString();
                                        var stringkuant10 = kuantKiri[10].ToString(); var stringkuant12 = kuantKiri[12].ToString(); var stringkuant14 = kuantKiri[14].ToString();
                                        var stringkuant16 = kuantKiri[16].ToString(); var stringkuant18 = kuantKiri[18].ToString(); var stringkuant20 = kuantKiri[20].ToString();
                                        var stringkuant22 = kuantKiri[22].ToString(); var stringkuant24 = kuantKiri[24].ToString(); var stringkuant26 = kuantKiri[26].ToString();
                                        var stringkuant28 = kuantKiri[28].ToString(); var stringkuant30 = kuantKiri[30].ToString(); var stringkuant32 = kuantKiri[32].ToString();
                                        var stringkuant34 = kuantKiri[34].ToString(); var stringkuant36 = kuantKiri[36].ToString(); var stringkuant38 = kuantKiri[38].ToString();

                                        var stringkuant43 = kuantKanan[4].ToString(); var stringkuant45 = kuantKanan[6].ToString(); var stringkuant47 = kuantKanan[8].ToString();
                                        var stringkuant49 = kuantKanan[10].ToString(); var stringkuant51 = kuantKanan[12].ToString(); var stringkuant53 = kuantKanan[14].ToString();
                                        var stringkuant55 = kuantKanan[16].ToString(); var stringkuant57 = kuantKanan[18].ToString(); var stringkuant59 = kuantKanan[20].ToString();
                                        var stringkuant61 = kuantKanan[22].ToString(); var stringkuant63 = kuantKanan[24].ToString(); var stringkuant65 = kuantKanan[26].ToString();
                                        var stringkuant67 = kuantKanan[28].ToString(); var stringkuant69 = kuantKanan[30].ToString(); var stringkuant71 = kuantKanan[32].ToString();
                                        var stringkuant73 = kuantKanan[34].ToString(); var stringkuant75 = kuantKanan[36].ToString(); var stringkuant77 = kuantKanan[38].ToString();
                                        
                                        var stringtangankiri = tanganKiri; var stringtangankanan = tanganKanan;
                                        var stringnamagerakan = namaGerakan;
                                        
                                        #endregion

                                        #region Training & Testing
                                        if (statusAmbil == 1)
                                        {
                                            var newLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},{19},{20},{21},{22},{23},{24},{25},{26},{27},{28},{29},{30},{31},{32},{33},{34},{35},{36},{37},{38},{39},{40},{41},{42},{43},{44},{45},{46},{47},{48},{49},{50},{51},{52}",
                                                    stringkuant4, stringkuant6, stringkuant8, stringkuant10, stringkuant12, stringkuant14, stringkuant16,
                                                    stringkuant18, stringkuant20, stringkuant22, stringkuant24, stringkuant26, stringkuant28, stringkuant30,
                                                    stringkuant32, stringkuant34, stringkuant36, stringkuant38, stringkuant43, stringkuant45, stringkuant47,
                                                    stringkuant49, stringkuant51, stringkuant53, stringkuant55, stringkuant57, stringkuant59, stringkuant61,
                                                    stringkuant63, stringkuant65, stringkuant67, stringkuant69, stringkuant71, stringkuant73, stringkuant75,
                                                    stringkuant77,
                                                    stringtangankiri, stringtangankanan,
                                                    stringSRER, stringERWR, stringWRHR, stringSLEL, stringELWL, stringWLHL, stringHRHL,
                                                    stringSSSRER, stringSRERWR, stringERWRHR, stringSSSLEL, stringSLELWL, stringELWLHL, stingDisHRHL,
                                                    stringnamagerakan
                                                    );
                                            
                                            // masukkan data dalam baris 
                                            csv.AppendLine(newLine);

                                            flag3++;
                                        }
                                        else if (statusAmbil == 2)
                                        {
                                            // Testing Data
                                            // Statis or Dinamis
                                            flag4 = 0;
                                            for (j = 2; j < 39; j = j+2)
                                                // menentukan Statis atau Dinamis
                                            {
                                                if (kuantKanan[j] == 0)
                                                {
                                                    flag4++;
                                                }
                                            }
                                            if (flag4 > 9)
                                            {
                                                // Statis 
                                                #region Statis
                                                if (ELWL.X < 0.03)
                                                {
                                                    if (SRER.X < 0.11)
                                                    {
                                                        if (SLEL.X < -0.02)
                                                        {
                                                            if (DisHRHL < 0.47)
                                                            {
                                                                outputText.Content = "Hamba";
                                                            }
                                                            else if (DisHRHL >= 0.47)
                                                            {
                                                                outputText.Content = "Jendral";
                                                            }
                                                        }
                                                        else if (SLEL.X >= -0.02)
                                                        {
                                                            outputText.Content = "Wadah";
                                                        }
                                                    }
                                                    else if (SRER.X >= 0.11)
                                                    {
                                                        if (SLEL.X < -0.04)
                                                        {
                                                            if (SRERWR < -0.84)
                                                            {
                                                                outputText.Content = "Hai";
                                                            }
                                                            else if (SRERWR >= -0.84)
                                                            {
                                                                outputText.Content = "Hormat";
                                                            }
                                                        }
                                                        else if (SLEL.X >= -0.04)
                                                        {
                                                            if (SRERWR < -0.84)
                                                            {
                                                                outputText.Content = "Hai";
                                                            }
                                                            else if (SRERWR >= -0.84)
                                                            {
                                                                outputText.Content = "Ketua";
                                                            }
                                                        }
                                                    }
                                                }
                                                else if (ELWL.X >= 0.03)
                                                {
                                                    if (ERWR.Y < -0.03)
                                                    {
                                                        if (SLEL.X < -0.1)
                                                        {
                                                            outputText.Content = "Ada";
                                                        }
                                                        else if (SLEL.X >= -0.1)
                                                        {
                                                            outputText.Content = "Gang";
                                                        }
                                                    }
                                                    else if (ERWR.Y >= -0.03)
                                                    {
                                                        if (SRER.Y < -0.12)
                                                        {
                                                            outputText.Content = "Botol";
                                                        }
                                                        else if (SRER.Y >= -0.12)
                                                        {
                                                            outputText.Content = "Geledeg";
                                                        }
                                                    }
                                                }

                                                imagePath = "E:\\Kuliah\\TUGAS AKHIR\\TABI\\GambarIsyarat\\Statis\\";

                                                #endregion
                                            }
                                            else if (flag4 <= 9)
                                            {
                                                // Dinamis
                                                #region dinamis

                                                if (kuantKiri[20] < 6)
                                                {
                                                    if (kuantKiri[24] < 5.5)
                                                    {
                                                        if (kuantKiri[34] < 0.5)
                                                        {
                                                            if (kuantKanan[18] < 2)
                                                            {
                                                                if (kuantKanan[8] < 0.5)
                                                                {
                                                                    outputText.Content = "Samping";
                                                                }
                                                                else if (kuantKanan[8] >= 0.5)
                                                                {
                                                                    outputText.Content = "Sempit";
                                                                }
                                                            }
                                                            else if (kuantKanan[18] >= 2)
                                                            {
                                                                if (kuantKanan[26] < 4.5)
                                                                {
                                                                    if (kuantKanan[36] < 2)
                                                                    {
                                                                        if (kuantKanan[18] < 6.5)
                                                                        {
                                                                            outputText.Content = "Topeng";
                                                                        }
                                                                        else if (kuantKanan[18] >= 6.5)
                                                                        {
                                                                            outputText.Content = "Samping";
                                                                        }
                                                                    }
                                                                    else if (kuantKanan[36] >= 2)
                                                                    {
                                                                        if (kuantKiri[32] < 0.5)
                                                                        {
                                                                            if (kuantKiri[18] < 0.5)
                                                                            {
                                                                                outputText.Content = "Topeng";
                                                                            }
                                                                            else if (kuantKiri[18] >= 0.5)
                                                                            {
                                                                                outputText.Content = "Bingung";
                                                                            }
                                                                        }
                                                                        else if (kuantKiri[32] >= 0.5)
                                                                        {
                                                                            outputText.Content = "Bingung";
                                                                        }
                                                                    }
                                                                }
                                                                else if (kuantKanan[26] >= 4.5)
                                                                {
                                                                    if (kuantKiri[16] < 3.5)
                                                                    {
                                                                        outputText.Content = "Sempit";
                                                                    }
                                                                    else if (kuantKiri[16] >= 3.5)
                                                                    {
                                                                        outputText.Content = "Bingkai";
                                                                    }
                                                                }
                                                            }
                                                        }
                                                        else if (kuantKiri[34] >= 0.5)                              //20
                                                        {
                                                            if (tanganKiri == "Perut")
                                                            {
                                                                if (kuantKanan[26] < 4.5)
                                                                {
                                                                    if (kuantKiri[30] < 0.5)
                                                                    {
                                                                        if (kuantKanan[32] < 2)
                                                                        {
                                                                            if (tanganKanan == "Kepala")
                                                                            {
                                                                                outputText.Content = "Topeng";
                                                                            }
                                                                            else if (tanganKanan == "Perut")
                                                                            {
                                                                                outputText.Content = "Samping";
                                                                            }
                                                                            else if (tanganKanan == "Dada")
                                                                            {
                                                                                outputText.Content = "Topeng";
                                                                            }
                                                                        }
                                                                        else if (kuantKanan[32] >= 2)
                                                                        {
                                                                            outputText.Content = "Bingung";
                                                                        }
                                                                    }
                                                                    else if (kuantKanan[30] >= 0.5)
                                                                    {
                                                                        if (kuantKanan[32] < 3)                    //30
                                                                        {
                                                                            outputText.Content = "Topeng";
                                                                        }
                                                                        else if (kuantKanan[32] >= 3)
                                                                        {
                                                                            outputText.Content = "Sempit";
                                                                        }
                                                                    }
                                                                }
                                                                else if (kuantKanan[26] >= 4.5)
                                                                {
                                                                    if (kuantKiri[20] < 0.5)
                                                                    {
                                                                        if (kuantKiri[18] < 2)
                                                                        {
                                                                            outputText.Content = "Sempit";
                                                                        }
                                                                        else if (kuantKiri[18] >= 0.5)
                                                                        {
                                                                            outputText.Content = "Bingung";
                                                                        }
                                                                    }
                                                                    else if (kuantKiri[20] >= 0.5)
                                                                    {
                                                                        outputText.Content = "Sempit";
                                                                    }
                                                                }
                                                            }
                                                            else if (tanganKiri == "Dada")
                                                            {
                                                                outputText.Content = "Rujuk";
                                                            }
                                                            else if (tanganKiri == "Kepala")
                                                            {
                                                                outputText.Content = "Kijang";
                                                            }
                                                        }
                                                    }
                                                    else if (kuantKiri[24] >= 5.5)
                                                    {
                                                        if (kuantKiri[18] < 2.5)                                    //40
                                                        {
                                                            if (kuantKanan[14] < 3)
                                                            {
                                                                outputText.Content = "Kijang";
                                                            }
                                                            else if (kuantKanan[14] >= 3)
                                                            {
                                                                outputText.Content = "Bola";
                                                            }
                                                        }
                                                        else if (kuantKanan[18] >= 2.5)
                                                        {
                                                            outputText.Content = "Badan";
                                                        }
                                                    }
                                                }
                                                else if (kuantKiri[20] >= 6)
                                                {
                                                    if (kuantKanan[30] < 3)
                                                    {
                                                        if (kuantKanan[36] < 3)
                                                        {
                                                            if (kuantKanan[32] < 4.5)
                                                            {
                                                                if (kuantKanan[28] < 1.5)
                                                                {
                                                                    if (kuantKanan[24] < 2)
                                                                    {
                                                                        outputText.Content = "Besar";
                                                                    }
                                                                    else if (kuantKanan[24] >= 2)           //50
                                                                    {
                                                                        outputText.Content = "Kijang";
                                                                    }
                                                                }
                                                                else if (kuantKanan[28] >= 1.5)
                                                                {
                                                                    if (kuantKanan[6] < 1)
                                                                    {
                                                                        outputText.Content = "Kijang";
                                                                    }
                                                                    else if (kuantKanan[6] >= 1)
                                                                    {
                                                                        outputText.Content = "Besar";
                                                                    }
                                                                }
                                                            }
                                                            else if (kuantKanan[32] >= 4.5)
                                                            {
                                                                outputText.Content = "Kijang";
                                                            }
                                                        }
                                                        else if (kuantKanan[36] >= 3)
                                                        {
                                                            if (kuantKanan[26] < 4)
                                                            {
                                                                outputText.Content = "Kijang";
                                                            }
                                                            else if (kuantKanan[26] >= 4)
                                                            {
                                                                if (kuantKiri[22] < 7.5)
                                                                {
                                                                    outputText.Content = "Bingkai";
                                                                }
                                                                else if (kuantKiri[22] >= 7.5)
                                                                {
                                                                    outputText.Content = "Badan";
                                                                }
                                                            }
                                                        }
                                                    }
                                                    else if (kuantKanan[30] >= 3)                          //60
                                                    {
                                                        if (kuantKiri[12] < 7.5)
                                                        {
                                                            if (kuantKanan[34] < 6.5)
                                                            {
                                                                if (kuantKiri[14] < 0.5)
                                                                {
                                                                    outputText.Content = "Bingung";
                                                                }
                                                                else if (kuantKiri[14] >= 0.5)
                                                                {
                                                                    if (kuantKiri[22] < 4)
                                                                    {
                                                                        outputText.Content = "Rujuk";
                                                                    }
                                                                    else if (kuantKiri[22] >= 4)
                                                                    {
                                                                        outputText.Content = "Bola";
                                                                    }
                                                                }
                                                            }
                                                            else if (kuantKanan[34] >= 6.5)
                                                            {
                                                                if (kuantKanan[18] < 7.5)
                                                                {
                                                                    outputText.Content = "Badan";
                                                                }
                                                                else if (kuantKanan[18] >= 7.5)
                                                                {
                                                                    outputText.Content = "Bingung";
                                                                }
                                                            }
                                                        }
                                                        else if (kuantKiri[12] >= 7.5)                      //70
                                                        {
                                                            if (kuantKiri[14] < 5.5)
                                                            {
                                                                if (kuantKanan[20] < 7.5)
                                                                {
                                                                    outputText.Content = "Badan";
                                                                }
                                                                else if (kuantKanan[20] >= 7.5)
                                                                {
                                                                    outputText.Content = "Bola";
                                                                }
                                                            }
                                                            else if (kuantKiri[14] >= 5.5)
                                                            {
                                                                if (tanganKiri == "Perut")
                                                                {
                                                                    outputText.Content = "Bola";
                                                                }
                                                                else if (tanganKiri == "Dada")
                                                                {
                                                                    outputText.Content = "Bingkai";
                                                                }
                                                                else if (tanganKiri == "Kepala")
                                                                {
                                                                    outputText.Content = "Kijang";
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                                imagePath = "E:\\Kuliah\\TUGAS AKHIR\\TABI\\GambarIsyarat\\Dinamis\\";
                                                #endregion
                                            }
                                            // Testing Data
                                            string imageFullPath = imagePath + outputText.Content + ".bmp";
                                            if (File.Exists(imageFullPath))
                                                outputImage.Source = (ImageSource)new ImageSourceConverter().ConvertFrom(imageFullPath);
                                        }

                                        if (flag3 < 15)
                                        {
                                            i = -1;
                                            flag2 = 0;
                                            InitStaticFeatures();
                                            ambil.Content = flag3.ToString();

                                        }
                                        else
                                        {
                                            statusDetail.Content = "Data Created";
                                            ambil.Content = "-";
                                        }
                                        flag1 = 0;
                                        #endregion
                                    }
                                    #endregion
                                    i++;
                                }
                                #endregion
                            }
                        }
                    }
                    if (statusAmbil == 1)
                    {
                        File.AppendAllText(filePath, csv.ToString());
                    }
                }
            }
        }
        #endregion

        private void OneTestButton_Click(object sender, RoutedEventArgs e)
        {
            statusDetail.Content = "Testing Data";
            statusAmbil = 2;
            flag2 = 0;
            flag3 = 0;
            i = 0;
            InitStaticFeatures();
        }

        private void createButton_click(object sender, RoutedEventArgs e)
        {
            statusDetail.Content = "Create Dataset";
            statusAmbil = 1;
            flag2 = 0;
            flag3 = 0;
            i = 0;
            InitStaticFeatures();

            namaGerakan = fileName.Text;
        }

        private void stopButton_Click(object sender, RoutedEventArgs e)
        {
            statusDetail.Content = "Idle";
            statusAmbil = 0;
            flag2 = 0;
            ambilData.Content = "Done";
            outputText.Content = "";
        }
    }
}
