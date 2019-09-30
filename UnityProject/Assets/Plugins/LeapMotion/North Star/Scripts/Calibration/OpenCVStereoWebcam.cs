using OpenCvSharp;
using System.Runtime.InteropServices;
using UnityEngine;
using Intel.RealSense;
using System.Linq;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Threading;
using System.Threading.Tasks;

namespace Leap.Unity.AR.Testing
{
    using RSConfig = Intel.RealSense.Config;
    public class RSVideoCapture
    {
        private Pipeline m_pipe;
        private RSConfig m_cfg;
        private ConcurrentQueue<(VideoFrame left, VideoFrame right)> m_frames;
        private CancellationTokenSource m_cts;

        private void Frame2Mat(VideoFrame frame, ref Mat img)
        {
            frame.CopyTo(img.Data);
        }
        public int FrameWidth { get;} = 848;
        public int FrameHeight { get; } = 800;
        public bool IsOpened { get; private set; }

        public RSVideoCapture(int deviceNumber, Context ctx)
        {
            m_cts = new CancellationTokenSource();
            m_frames = new ConcurrentQueue<(VideoFrame, VideoFrame)>();
            var devices = ctx.QueryDevices();
            var device = devices[deviceNumber];
            m_pipe = new Pipeline(ctx);
            m_cfg = new RSConfig();
            m_cfg.EnableDevice(device.Info.GetInfo(CameraInfo.SerialNumber));
            m_pipe.Start(m_cfg);
            IsOpened = true;
            Task.Factory.StartNew(Capture, m_cts.Token);
        }

        private void Capture()
        {
            while (!m_cts.IsCancellationRequested)
            {
                FrameSet frames = m_pipe.WaitForFrames(5000);
                if (frames.Count < 2)
                    continue;
                m_frames.Enqueue((frames[0].As<VideoFrame>(), frames[1].As<VideoFrame>()));
                //frames?.Dispose();
            }
        }

        public bool Read(ref Mat left, ref Mat right)
        {
            if (!m_frames.IsEmpty)
            {
                if (m_frames.TryDequeue(out (VideoFrame left, VideoFrame right) result))
                {
                    Frame2Mat(result.left, ref left);
                    Frame2Mat(result.right, ref right);
                    return true;
                }
            }
            return false;
            
        }

        public void Release()
        {
            m_cts.Cancel();
            m_pipe.Stop();
        }
    }

    public class OpenCVStereoWebcam : MonoBehaviour
    {
        public int deviceNumber = 0;
        public Renderer leftDisplay, rightDisplay;
        public bool updateScreenAutomatically = true;

        //[System.NonSerialized]
        //public byte[] leftData, rightData;
        [System.NonSerialized]
        public Mat leftImage, rightImage;

        //public VideoCapture cap;
        public RSVideoCapture cap;
        private static Context m_ctx;
        Texture2D leftTexture, rightTexture;

        static OpenCVStereoWebcam()
        {
            m_ctx = new Context();
        }

        void Start()
        {
            //cap = new OpenCvSharp.VideoCapture(deviceNumber);
            ////cap.Fps = 30;
            //cap.FrameWidth = 1280;
            //cap.FrameHeight = 480;
            //cap.AutoExposure = 0;
            //cap.Exposure = -4;
            cap = new RSVideoCapture(deviceNumber, m_ctx);
            leftImage = new Mat(cap.FrameHeight, cap.FrameWidth, MatType.CV_8UC1);
            rightImage = new Mat(cap.FrameHeight, cap.FrameWidth, MatType.CV_8UC1);
            //leftData = new byte[cap.FrameHeight * cap.FrameWidth];
            //rightData = new byte[cap.FrameHeight * cap.FrameWidth];
        }

        void Update()
        {
            if (Time.frameCount % 2 == deviceNumber) return;
            if (cap == null)
            {
                DebugEx.LogWarning("cap is null");
                return;
            }
            //Should multithread the cap.Read() operation, it can take anywhere from 6 to 25ms
            if (cap.Read(ref leftImage, ref rightImage))
            {
                if (leftImage.Channels() > 1)
                    Cv2.CvtColor(leftImage, leftImage, ColorConversionCodes.BGR2GRAY);
                // Get the Left Image Data
                //Marshal.Copy(leftImage.Data, leftData, 0, cap.FrameHeight * cap.FrameWidth);
                // Display the Left Image Texture
                if (updateScreenAutomatically) updateScreen(leftImage, true);
                // Get the Right Image Data
                if (rightImage.Channels() > 1)
                    Cv2.CvtColor(rightImage, rightImage, ColorConversionCodes.BGR2GRAY);
                //Marshal.Copy(rightImage.Data, rightData, 0, cap.FrameHeight * cap.FrameWidth);
                // Display the Right Image Texture
                if (updateScreenAutomatically) updateScreen(rightImage, false);
            }
        }
        private void OnDestroy()
        {
            cap.Release();
            //leftImage.Release();
            //rightImage.Release();
        }

        public void updateScreen(Mat image, bool isLeft)
        {
            Renderer display = isLeft ? leftDisplay : rightDisplay;

            // Display the Right Image Texture
            if (display != null)
            {
                if (isLeft)
                {
                    fillTexture(image, ref leftTexture);
                }
                else
                {
                    fillTexture(image, ref rightTexture);
                }
                display.material.mainTexture = isLeft ? leftTexture : rightTexture;
            }
        }

        public void changeDeviceNumber(int newDeviceNumber)
        {
            if (cap.IsOpened) cap.Release();
            cap = null;
            deviceNumber = newDeviceNumber;
            cap = new RSVideoCapture(newDeviceNumber, m_ctx);
        }

        public static void fillTexture(Mat input, ref Texture2D output, byte[] bytes = null)
        {
            bool textureGood = (output != null &&
                                output.width == input.Width && output.height == input.Height &&
                              ((input.Type() == MatType.CV_8UC3 && output.format == TextureFormat.RGB24) ||
                               (input.Type() == MatType.CV_8UC1 && output.format == TextureFormat.R8)));
            if (!textureGood)
            {
                TextureFormat format = (input.Type() == MatType.CV_8UC3) ? TextureFormat.RGB24 : TextureFormat.R8;
                output = new Texture2D(input.Width, input.Height, format, false);
            }
            if (bytes == null)
            {
                output.LoadRawTextureData(input.Data, input.Width * input.Height * ((input.Type() == MatType.CV_8UC3) ? 3 : 1));
            }
            else
            {
                output.LoadRawTextureData(bytes);
            }
            output.Apply();
        }
    }
}