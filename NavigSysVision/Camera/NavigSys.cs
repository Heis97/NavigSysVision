using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using Emgu.CV;
using Emgu.CV.Aruco;
using System.Drawing;
using System.ComponentModel;
using System.Drawing.Design;

using System.Text.Json;


using Emgu.CV.Dnn;
using Accord.Math;
using System.Reflection;
using System.Text.Json;
using System.Text.Json.Serialization;
using Accord.Statistics;

namespace opengl3
{

    

    public class ToolDef
    {
        [JsonPropertyName("ind")] public int Ind { get; set; }
        [JsonPropertyName("err")] public double Err { get; set; }
        [JsonPropertyName("points")] public double[][] Points { get; set; } = Array.Empty<double[]>();

        [JsonIgnore]
        public Point3d_GL[] Local =>
            Points.Select(p => new Point3d_GL(p[0], p[1], p[2])).ToArray();

        
        double[] dists = new double[1];


        Matrix<double> matrix_frame = null;
        public void comp_charachter_dists()
        {
            if (Local == null) return;
            if (Local.Length <2) return;

            dists = new double[Local.Length-1];
            for(int i = 0; i < dists.Length; i++)
            {
                dists[i] = (Local[i] - Local[Local.Length - 1]).magnitude();
            }
        }
        public Point3d_GL[] find_tool(Point3d_GL[] ps)
        {
            if (ps.Length < Local.Length) return null;
            var ps_out = new Point3d_GL[dists.Length+1];
            int ind_main_p = -1;
            for(int i = 0; i < ps.Length; i++)
            {
                var cur_dists = new double[ps.Length];
                Array.Fill(cur_dists, -1);

                bool one_dist_have = false;
                for (int j = 0; j < ps.Length; j++)
                {
                    if(i!=j)
                    {
                        cur_dists[j] = (ps[j] - ps[i]).magnitude();

                        if(Math.Abs(cur_dists[j] - dists[0]) < Err)
                        {
                            one_dist_have = true;
                        }
                    }
                }

                if(one_dist_have)
                {
                    ind_main_p = i;
                    var dist_have = new int[dists.Length];
                    Array.Fill(dist_have, -1);

                    for (int j = 0; j < cur_dists.Length; j++)
                    {
                        for (int k = 0; k < dists.Length; k++)
                        {
                            if (Math.Abs(cur_dists[j] - dists[k]) < Err && cur_dists[j]>0)
                            {
                                dist_have[k] = j;
                            }
                        }
                    }

                    var all_dist_have = true;
                    for (int k = 0; k < dists.Length; k++)
                    {
                        
                        if( dist_have[k]>=0)
                        {
                            ps_out[k] = ps[dist_have[k]];
                        }
                        else
                        {
                            all_dist_have = false;
                        }
                    }
                    if(all_dist_have)
                    {
                        ps_out[ps_out.Length-1] = ps[ind_main_p];
                        return ps_out;
                    }
                }
            }

            return null;
        }
    }

    public class ToolLibrary
    {
        [JsonPropertyName("tools")] public List<ToolDef> Tools { get; set; } = new();

        static readonly JsonSerializerOptions opts = new() { WriteIndented = true };

        public static ToolLibrary Load(string path)
        {
            var lib = JsonSerializer.Deserialize<ToolLibrary>(File.ReadAllText(path), opts)!;
            foreach (var t in lib.Tools)
            {
                t.Points = ToArr(Point3d_GL.ToLocal(t.Local));
                t.comp_charachter_dists();
            }
                
            return lib;
        }

        public void Save(string path) => File.WriteAllText(path, JsonSerializer.Serialize(this, opts));

        public static double[][] ToArr(Point3d_GL[] ps) =>
            ps.Select(p => new[] { Math.Round(p.x, 3), Math.Round(p.y, 3), Math.Round(p.z, 3) }).ToArray();

        public  Point3d_GL[][] tools_ps(Point3d_GL[] ps)
        {
            var tools_ps_arr = new Point3d_GL[Tools.Count][];
            for(int i = 0; i < Tools.Count; i++)
            {
                tools_ps_arr[i] = Tools[i].find_tool(ps);
            }
            return tools_ps_arr;
        }


    }

    



    public class NavigVision
    {
        VideoCapture[] videoCaptures = new VideoCapture[4];
        volatile IntPtr[] camera_ind_ptr = new IntPtr[] { (IntPtr)0, (IntPtr)0, (IntPtr)0, (IntPtr)0, (IntPtr)0, (IntPtr)0, (IntPtr)0 };
        private Size cameraSize = new Size(1280, 720);
        volatile Mat[] mat_global = new Mat[6];
        int cam1_ind = 0;
        int cam2_ind = 1;
        int cams_num = 2;
        readonly object mat_lock = new object();
        TCPserver _TCPserver1;
        Thread server_thread1;
        StereoCamera stereo_cam;

        ToolLibrary navig_tools;
        public NavigVision()
        {
            navig_tools = new ToolLibrary();
            /*var tool1 = new ToolDef();
            tool1.Ind = 1;
            tool1.Points = new double[][] { new double[] { 0, 0, 0 }, new double[] { 0, 10, 0 }, new double[] { 10, 0, 0 } };
            tools.Tools.Add(tool1);
            tools.Save("tools_navig.json");*/

            navig_tools = ToolLibrary.Load("tools_navig.json");
            navig_tools.Save("tools_navig.json");
            for (int i = 0; i < mat_global.Length; i++) mat_global[i] = new Mat();

            var conf1 = "cam1_cam_cal041126a.txt";
            var conf2 = "cam2_cam_cal041126a.txt";
            Console.WriteLine("loadScanner");
            var cam1 = CameraCV.load_camera(conf1);
            var cam2 = CameraCV.load_camera(conf2);
            Console.WriteLine("load_camera");

            stereo_cam = new StereoCamera(new CameraCV[] { cam1, cam2 });

            var stereo_cal_1 = "cam_cal041126a";
            var frms_stereo = FrameLoader.loadImages_stereoCV(@"cam1\" + stereo_cal_1, @"cam2\" + stereo_cal_1,  FrameType.Pattern, false);
            stereo_cam.calibrate_stereo(frms_stereo,PatternType.Mesh,chess_size,markSize);
        }
        private void videoStart(int number, int num_imb = -1)
        {
            var capture = new VideoCapture(number, VideoCapture.API.DShow);

            var num_vid = num_imb;
            if (num_vid < 0) num_vid = number;
            videoCaptures[num_vid] = capture;
            capture.Set(CapProp.FrameWidth, cameraSize.Width);
            capture.Set(CapProp.FrameHeight, cameraSize.Height);
            capture.Set(CapProp.Exposure, -7);
            capture.Set(CapProp.FourCC, VideoWriter.Fourcc('M', 'J', 'P', 'G'));
            //capture.Set(CapProp.Fps, 30);
            cameraSize.Width = (int)capture.Get(CapProp.FrameWidth);
            cameraSize.Height = (int)capture.Get(CapProp.FrameHeight);
            //videoCaptures[scanner_config.Cam1_ind]?.Set(CapProp.AutoExposure, 1);
            var fps = capture.Get(CapProp.Fps);
            Console.WriteLine("number: " + number + "; num_vid: " + num_vid + "; " + cameraSize.Width.ToString() + " " + cameraSize.Height.ToString() + " " + fps);
            //capture.SetCaptureProperty(CapProp.Contrast, 30);
            camera_ind_ptr[num_vid] = capture.Ptr;

            capture.ImageGrabbed += capturingVideo;
            capture.Start();
        }

        void drawCameras(VideoCapture cap)
        {
            int ind_cam = Array.IndexOf(camera_ind_ptr, cap.Ptr);
            if (ind_cam < 0 || ind_cam >= cams_num) return;

            var frame = new Mat();
            if (!cap.Retrieve(frame) || frame.IsEmpty) { frame.Dispose(); return; }

            lock (mat_lock)
            {
                mat_global[ind_cam].Dispose();
                mat_global[ind_cam] = frame;
            }

        }
        Size chess_size = new Size(11,8);
        float markSize = 18.37f;
        void capturingVideo(object sender, EventArgs e)
        {
            drawCameras((VideoCapture)sender);
        }
        void calib_cam()
        {
            var frms_1 = FrameLoader.loadImages_diff(@"cam1\cam_cal041126a", FrameType.Pattern, PatternType.Mesh);//
            var cam1 = new CameraCV(frms_1, chess_size, markSize, null);
            cam1.save_camera("cam1_cam_cal041126a.txt");
        }
        private void con_navig_sys()
        {

           // calib_cam();

            videoStart(cam1_ind, 0);
            videoStart(cam2_ind, 1);
            System.Timers.Timer timer = new System.Timers.Timer(10);

            _TCPserver1 = new TCPserver(62000);
            server_thread1 = new Thread(_TCPserver1.startServer);
            server_thread1.Start();

        }
        string cur_save_fold = "cam_cal041126a";
        int glob_im_coubter = 0;


        Point3d_GL[][] find_scene(Point3d_GL[] ps3d, ToolLibrary navig_tools)
        {
            if (ps3d == null) return null;
            if(ps3d.Length <2) return null;

            var ps_tools  = navig_tools.tools_ps(ps3d);

            //find_matrixes and send
            return ps_tools;
        }

        public void Run()
        {
            con_navig_sys();
            var mats = new Mat[cams_num];
            var mats_show = new Mat[cams_num];
            var mats_find = new Mat[cams_num];
            for (int i = 0; i < cams_num; i++)
            {
                CvInvoke.NamedWindow("cam" + i);
                mats[i] = new Mat();
                mats_show[i] = new Mat();
                mats_find[i] = new Mat();
            }
                
            
            while (true)
            {
                var ps_all_2d = new System.Drawing.PointF[cams_num][];
                for (int i = 0; i < cams_num; i++)
                {
                    bool has_frame = false;
                    lock (mat_lock)
                    {
                        if (!mat_global[i].IsEmpty)
                        {
                            mats_show[i].Dispose();
                            mats_show[i] = mat_global[i].Clone();
                            has_frame = true;
                        }
                    }
                    if (has_frame)
                    {
                        mats[i] = mats_show[i].Clone();
                        //mats_find[i] = FindCircles.findCircles_v4(mats_show[i], ref ps_all_2d[i], new Size(5, 5), false);
                        FindCircles.findCircles_v3(mats_show[i], ref ps_all_2d[i], new Size(5, 5), false); mats_find[i] = mats_show[i].Clone();
                    }
                        
                }

                var ps3d = stereo_cam.comp_points_3d_not_matched(ps_all_2d[0], ps_all_2d[1]);

                //draw_3d_ps(ps3d, ref mats_find,0, 255,0, 3,false);

                var ps_all = find_scene(ps3d, navig_tools);
                if (ps_all != null)
                {
                    if(ps_all.Length > 0)
                    {
                        draw_3d_ps(ps_all[0], ref mats_find,255,0,0,2);
                    }
                }
                
                for (int i = 0; i < cams_num; i++)
                {
                    if (mats_find[i] != null && !mats_find[i].IsEmpty)
                    {
                        CvInvoke.Imshow("cam" + i, mats_find[i]);
                        //CvInvoke.Imshow("cam" + i, mats_show[i]);
                       
                       // mats_show[i].Dispose();
                    }
                }

                //send
                int key = CvInvoke.WaitKey(10);
                if (key == 's')
                {
                    UtilOpenCV.saveImage(mats[0], mats[1], glob_im_coubter.ToString(), cur_save_fold);
                    glob_im_coubter++;
                }
                if (key == 27) break;  // Esc — выход
            }

            foreach (var c in videoCaptures) { c?.Stop(); c?.Dispose(); }
            CvInvoke.DestroyAllWindows();
        }

        void draw_3d_ps(Point3d_GL[] ps3d, ref Mat[] mats_find, int r, int g, int b,int size, bool text = true)
        {
            if (ps3d != null && ps3d.Length > 0 && stereo_cam.R != null)
            {
                var cs0 = stereo_cam.cameraCVs[0].matrixCS;
                var pr0 = stereo_cam.cameraCVs[0].project_world(ps3d, cs0);
                var pr1 = stereo_cam.cameraCVs[1].project_world(ps3d, cs0 * stereo_cam.R);
                if (pr0.Length > 0) UtilOpenCV.drawPointsF(mats_find[0], pr0, r, g, b, size, text);
                if (pr1.Length > 0) UtilOpenCV.drawPointsF(mats_find[1], pr1, r, g, b, size, text);

            }
        }

        private void timer_navig_processing_Tick(object sender, EventArgs e)
        {
            //var ps3d = navig_system.navigation_processing_get_points3d_3cam(navig_system.points2d_cams[0], navig_system.points2d_cams[1], navig_system.points2d_cams[2]);    //navyg_sys_info(camera_only)
            //navig_system.navigation_processing_get_scene(ps3d, true);
        }

    }

}
