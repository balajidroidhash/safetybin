namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Forms;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Controls;
    using System.Collections.ObjectModel;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Wpf.Controls;
    using Microsoft.Kinect.Input;
    using System.Data.SqlClient;
    using System.Data;
    using System.Configuration;

    public sealed class MyListBoxItem
    {
        public string Field1 { get; set; }
        public string Field2 { get; set; }
    }

    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        private KinectSensor kinectSensor = null;
        private ColorFrameReader colorFrameReader = null;
        private WriteableBitmap colorBitmap = null;
        private CoordinateMapper coordinateMapper = null;
        private BodyFrameReader bodyFrameReader = null;
        private Body[] bodies = null;
        private String url;
        private string currentObject;

        public SqlConnection con = new SqlConnection(ConfigurationManager.ConnectionStrings["DB"].ConnectionString);
        public SqlCommand cmd = new SqlCommand();
        public SqlDataAdapter da = new SqlDataAdapter();
        public DataTable dt = new DataTable();

        public ObservableCollection<MyListBoxItem> Images { get; set; }

        public MainWindow()
        {
            this.kinectSensor = KinectSensor.GetDefault();
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;
            this.kinectSensor.Open();
            this.DataContext = this;
            this.InitializeComponent();

            KinectRegion.SetKinectRegion(this, kinectRegion);
            App app = ((App)System.Windows.Application.Current);
            app.KinectRegion = kinectRegion;
            this.kinectRegion.KinectSensor = KinectSensor.GetDefault();
            con.Open();
            con.Close();
        }

        public ImageSource imageSource
        {
            get
            {
                return this.colorBitmap;
            }
        }

        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();

                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.colorBitmap.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                        }

                        this.colorBitmap.Unlock();
                    }
                }
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {                
                foreach (Body body in this.bodies)
                {
                    IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                    CameraSpacePoint shoulderleftPosition = joints[JointType.ShoulderLeft].Position;
                    CameraSpacePoint shoulderrightPosition = joints[JointType.ShoulderRight].Position;
                    CameraSpacePoint neckPosition = joints[JointType.Neck].Position;
                    CameraSpacePoint hipleftPosition = joints[JointType.HipLeft].Position;
                    CameraSpacePoint hiprightPosition = joints[JointType.HipRight].Position;
                    CameraSpacePoint spinebasePosition = joints[JointType.SpineBase].Position;
                    CameraSpacePoint handrightPosition = joints[JointType.HandRight].Position;
                    CameraSpacePoint handleftPosition = joints[JointType.HandLeft].Position;

                    ColorSpacePoint shoulderleftSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(shoulderleftPosition);
                    ColorSpacePoint shoulderrightSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(shoulderrightPosition);
                    ColorSpacePoint neckSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(neckPosition);
                    ColorSpacePoint hipleftSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(hipleftPosition);
                    ColorSpacePoint hiprightSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(hiprightPosition);
                    ColorSpacePoint spinebaseSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(spinebasePosition);
                    ColorSpacePoint handrightSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(handrightPosition);
                    ColorSpacePoint handleftSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(handleftPosition);

                    double shoulderWidth = (shoulderrightSpacePoint.X - shoulderleftSpacePoint.X) / 2;

                    double hipWidth = (hiprightSpacePoint.X - hipleftSpacePoint.X) * 2.5;

                    if (shoulderWidth > 0 && !double.IsInfinity(shoulderWidth))
                    {
                        imgNecklace.Width = shoulderWidth;

                        imgLongNecklace.Width = shoulderWidth;

                        imgDoubleNecklace.Width = shoulderWidth;
                    }

                    if (hipWidth > 0 && !double.IsInfinity(hipWidth))
                    {
                        imgOttiyanam.Width = hipWidth;
                    }

                    if (!double.IsInfinity(neckSpacePoint.X) && !double.IsInfinity(neckSpacePoint.Y) && !double.IsInfinity(shoulderWidth))
                    {
                        Canvas.SetLeft(imgNecklace, neckSpacePoint.X - (shoulderWidth / 2));
                        Canvas.SetTop(imgNecklace, neckSpacePoint.Y);

                        Canvas.SetLeft(imgLongNecklace, neckSpacePoint.X - (shoulderWidth / 2));
                        Canvas.SetTop(imgLongNecklace, neckSpacePoint.Y);

                        Canvas.SetLeft(imgDoubleNecklace, neckSpacePoint.X - (shoulderWidth / 2));
                        Canvas.SetTop(imgDoubleNecklace, neckSpacePoint.Y);
                    }

                    if (!double.IsInfinity(spinebaseSpacePoint.X) && !double.IsInfinity(spinebaseSpacePoint.Y) && !double.IsInfinity(hipWidth))
                    {
                        Canvas.SetLeft(imgOttiyanam, spinebaseSpacePoint.X - (hipWidth / 2));
                        Canvas.SetTop(imgOttiyanam, spinebaseSpacePoint.Y - 200);
                    }
                }
            }
        }

        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {

        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            currentObject = "n";

            Images = new ObservableCollection<MyListBoxItem>();
            dt = new DataTable();
            cmd = new SqlCommand("select path,material from M_Items where type='Necklace'", con);
            con.Open();
            da.SelectCommand = cmd;
            da.Fill(dt);
            con.Close();

            for (int i = 0; i <= dt.Rows.Count - 1; i++)
            {
                Images.Add(new MyListBoxItem { Field1 = dt.Rows[i].ItemArray[0].ToString(), Field2 = dt.Rows[i].ItemArray[1].ToString() });
            }

            Panel.DataContext = Images;
            Panel.Visibility = Visibility.Visible;
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            currentObject = "l";

            Images = new ObservableCollection<MyListBoxItem>();
            dt = new DataTable();
            cmd = new SqlCommand("select path,material from M_Items where type='Long Necklace'", con);
            con.Open();
            da.SelectCommand = cmd;
            da.Fill(dt);
            con.Close();

            for (int i = 0; i <= dt.Rows.Count - 1; i++)
            {
                Images.Add(new MyListBoxItem { Field1 = dt.Rows[i].ItemArray[0].ToString(), Field2 = dt.Rows[i].ItemArray[1].ToString() });
            }

            Panel.DataContext = Images;
            Panel.Visibility = Visibility.Visible;
        }

        private void Button_Click_2(object sender, RoutedEventArgs e)
        {
            currentObject = "d";

            Images = new ObservableCollection<MyListBoxItem>();
            dt = new DataTable();
            cmd = new SqlCommand("select path,material from M_Items where type='Double Necklace'", con);
            con.Open();
            da.SelectCommand = cmd;
            da.Fill(dt);
            con.Close();

            for (int i = 0; i <= dt.Rows.Count - 1; i++)
            {
                Images.Add(new MyListBoxItem { Field1 = dt.Rows[i].ItemArray[0].ToString(), Field2 = dt.Rows[i].ItemArray[1].ToString() });
            }

            Panel.DataContext = Images;
            Panel.Visibility = Visibility.Visible;
        }

        private void Button_Click_3(object sender, RoutedEventArgs e)
        {
            currentObject = "o";

            Images = new ObservableCollection<MyListBoxItem>();
            dt = new DataTable();
            cmd = new SqlCommand("select path,material from M_Items where type='Ottiyanam'", con);
            con.Open();
            da.SelectCommand = cmd;
            da.Fill(dt);
            con.Close();

            for (int i = 0; i <= dt.Rows.Count - 1; i++)
            {
                Images.Add(new MyListBoxItem { Field1 = dt.Rows[i].ItemArray[0].ToString(), Field2 = dt.Rows[i].ItemArray[1].ToString() });
            }

            Panel.DataContext = Images;
            Panel.Visibility = Visibility.Visible;
        }

        private void Items_Click(object sender, RoutedEventArgs e)
        {
            System.Windows.Controls.Button btnSeletedItem = sender as System.Windows.Controls.Button;
            Image selectedItemImage = btnSeletedItem.Content as Image;

            BitmapSource source = new BitmapImage(new Uri(selectedItemImage.Source.ToString()));

            switch (currentObject)
            {
                case "n":
                    imgNecklace.Source = source;
                    break;
                case "l":
                    imgLongNecklace.Source = source;
                    break;
                case "d":
                    imgDoubleNecklace.Source = source;
                    break;
                case "o":
                    imgOttiyanam.Source = source;
                    break;
            }

            Panel.Visibility = Visibility.Hidden;
        }
    }
}