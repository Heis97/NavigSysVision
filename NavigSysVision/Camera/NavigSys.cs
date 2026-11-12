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
using System.Text;

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

        public Matrix<double> FitPose(Point3d_GL[] measured, out double rms)
        {
            var l = Local; int n = l.Length;
            var cl = new Point3d_GL(); var cw = new Point3d_GL();
            for (int k = 0; k < n; k++) { cl = cl + l[k]; cw = cw + measured[k]; }
            cl = cl / n; cw = cw / n;

            var H = new Matrix<double>(3, 3);
            for (int k = 0; k < n; k++)
            {
                var a = l[k] - cl; var b = measured[k] - cw;
                double[] av = { a.x, a.y, a.z }, bv = { b.x, b.y, b.z };
                for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) H[r, c] += av[r] * bv[c];
            }

            var S = new Matrix<double>(3, 1); var U = new Matrix<double>(3, 3); var Vt = new Matrix<double>(3, 3);
            CvInvoke.SVDecomp(H, S, U, Vt, SvdFlag.Default);
            var R = Vt.Transpose() * U.Transpose();
            if (CvInvoke.Determinant(R) < 0)                 // защита от зеркального решения
            {
                for (int c = 0; c < 3; c++) Vt[2, c] *= -1;
                R = Vt.Transpose() * U.Transpose();
            }

            var M = UtilMatr.eye_matr(4);
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++) M[r, c] = R[r, c];
                M[r, 3] = (r == 0 ? cw.x : r == 1 ? cw.y : cw.z)
                        - (R[r, 0] * cl.x + R[r, 1] * cl.y + R[r, 2] * cl.z);
            }

            double s = 0;
            for (int k = 0; k < n; k++) s += Math.Pow((Transform(M, l[k]) - measured[k]).magnitude(), 2);
            rms = Math.Sqrt(s / n);
            return M;
        }
        public static Point3d_GL Transform(Matrix<double> M, Point3d_GL p) => new Point3d_GL(
            M[0, 0] * p.x + M[0, 1] * p.y + M[0, 2] * p.z + M[0, 3],
            M[1, 0] * p.x + M[1, 1] * p.y + M[1, 2] * p.z + M[1, 3],
            M[2, 0] * p.x + M[2, 1] * p.y + M[2, 2] * p.z + M[2, 3]);
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
        //Size cameraSize = new Size(1280, 720);

        Size cameraSize = new Size(1920, 1080);
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

            var conf1 = "cam1_cam_cal121126_2a.txt";
            var conf2 = "cam2_cam_cal121126_2a.txt";
            Console.WriteLine("loadScanner");
            var cam1 = CameraCV.load_camera(conf1);
            var cam2 = CameraCV.load_camera(conf2);
            Console.WriteLine("load_camera");

            stereo_cam = new StereoCamera(new CameraCV[] { cam1, cam2 });

            var stereo_cal_1 = "cam_cal121126_2";
            var frms_stereo = FrameLoader.loadImages_stereoCV(@"cam1\" + stereo_cal_1, @"cam2\" + stereo_cal_1,  FrameType.Pattern, false);
            stereo_cam.calibrate_stereo(frms_stereo,PatternType.Mesh,chess_size,markSize);

            //cs0_inv = new Matrix<double>(4, 4);
            //CvInvoke.Invert(stereo_cam.cameraCVs[0].matrixCS, cs0_inv, DecompMethod.LU);
        }
        private void videoStart(int number, int num_imb = -1)
        {
            var capture = new VideoCapture(number, VideoCapture.API.DShow);

            var num_vid = num_imb;
            if (num_vid < 0) num_vid = number;
            videoCaptures[num_vid] = capture;
            capture.Set(CapProp.FrameWidth, cameraSize.Width);
            capture.Set(CapProp.FrameHeight, cameraSize.Height);
            capture.Set(CapProp.Exposure, -12);
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
                frame_counter[ind_cam]++;
            }

        }
        long[] frame_counter = new long[6];   // номер последнего кадра каждой камеры (меняется под mat_lock)
        /// <summary>сколько кадров усредняется при сохранении по клавише 's' (1 - сохранять один кадр, как раньше)</summary>
        int save_frames_n = 10;
        Size chess_size = new Size(11,8);
        float markSize = 18.37f;//18.37f;
        void capturingVideo(object sender, EventArgs e)
        {
            drawCameras((VideoCapture)sender);
        }
        void calib_cam()
        {
            var frms_1 = FrameLoader.loadImages_diff(@"cam2\cam_cal121126_2", FrameType.Pattern, PatternType.Mesh);//
            var cam1 = new CameraCV(frms_1, chess_size, markSize, null);
            cam1.save_camera("cam2_cam_cal121126_2a.txt");
        }
        private void con_navig_sys()
        {

            //calib_cam();

            videoStart(cam1_ind, 0);
            videoStart(cam2_ind, 1);
            System.Timers.Timer timer = new System.Timers.Timer(10);

            _TCPserver1 = new TCPserver(62000);
            server_thread1 = new Thread(_TCPserver1.startServer);
            server_thread1.Start();

        }
        string cur_save_fold = "cam_cal121126_3";
        int glob_im_coubter = 0;

        bool fon_inited = false;
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
            var mats_fon = new Mat[cams_num];
            for (int i = 0; i < cams_num; i++)
            {
                CvInvoke.NamedWindow("cam" + i, WindowFlags.AutoSize);
                mats[i] = new Mat();
                mats_show[i] = new Mat();
                mats_find[i] = new Mat();
                mats_fon[i] = new Mat();
            }
                
            
            // накопление кадров для усреднённого сохранения (клавиша 's')
            bool burst_active = false;
            int burst_iters = 0;
            var burst_acc = new Mat[cams_num];
            var burst_cnt = new int[cams_num];
            var burst_last_counter = new long[cams_num];
            var burst_ps_first = new System.Drawing.PointF[cams_num][];
            var burst_ps_last = new System.Drawing.PointF[cams_num][];
            var cur_counter = new long[cams_num];

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
                            if(fon_inited)
                            {
                                mats_show[i]-=mats_fon[i];
                            }
                            cur_counter[i] = frame_counter[i];
                            has_frame = true;
                        }
                    }
                    if (has_frame)
                    {
                        mats[i] = mats_show[i].Clone();
                        //mats_find[i] = FindCircles.findCircles_v3(mats_show[i], ref ps_all_2d[i], chess_size,false,false);
                        FindCircles.findCircles_v3(mats_show[i], ref ps_all_2d[i], chess_size, false,false); mats_find[i] = mats_show[i].Clone();

                        // усреднённое сохранение: берём только новые кадры камеры
                        if (burst_active && burst_cnt[i] < save_frames_n && cur_counter[i] != burst_last_counter[i])
                        {
                            burst_last_counter[i] = cur_counter[i];
                            var f32 = new Mat();
                            mats[i].ConvertTo(f32, DepthType.Cv32F);
                            if (burst_cnt[i] == 0)
                            {
                                burst_acc[i] = f32;
                                burst_ps_first[i] = ps_all_2d[i] == null ? null : (System.Drawing.PointF[])ps_all_2d[i].Clone();
                            }
                            else
                            {
                                CvInvoke.Add(burst_acc[i], f32, burst_acc[i]);
                                f32.Dispose();
                            }
                            burst_ps_last[i] = ps_all_2d[i] == null ? null : (System.Drawing.PointF[])ps_all_2d[i].Clone();
                            burst_cnt[i]++;
                        }
                    }
                        
                }

                var ps3d = stereo_cam.comp_points_3d_not_matched(ps_all_2d[0], ps_all_2d[1]);
                var msg = new StringBuilder();

                foreach (var tool in navig_tools.Tools)
                {
                    var meas = (ps3d != null && ps3d.Length >= 3) ? tool.find_tool(ps3d) : null;
                    Matrix<double>? M = null; double rms = -1;
                    if (meas != null)
                    {
                        M = tool.FitPose(meas, out rms);
                        if (rms > tool.Err) M = null;      // точки сопоставлены неверно — инструмент не принимаем
                    }

                    if (M != null)
                    {
                        draw_3d_ps(meas, ref mats_find, 255, 0, 0, 2);
                        draw_tool_frame(M, mats_find, tool.Ind);
                    }
                    msg.Append(PoseLine(frame_id, tool.Ind, M, rms));
                }

                if (_TCPserver1.connected) _TCPserver1.pushBuffer(msg.ToString());
                frame_id++;

                var ps_all = find_scene(ps3d, navig_tools);
                if (ps_all != null)
                {
                    if(ps_all.Length > 0)
                    {
                        draw_3d_ps(ps_all[0], ref mats_find,255,0,0,2);
                    }
                }
                
                if (burst_active)
                {
                    burst_iters++;
                    bool burst_done = true;
                    for (int i = 0; i < cams_num; i++) burst_done &= burst_cnt[i] >= save_frames_n;
                    if (burst_done)
                    {
                        var avg = new Mat[cams_num];
                        for (int i = 0; i < cams_num; i++)
                        {
                            avg[i] = new Mat();
                            burst_acc[i].ConvertTo(avg[i], DepthType.Cv8U, 1.0 / burst_cnt[i]);
                            burst_acc[i].Dispose();
                            // сдвиг шаблона за время накопления: если заметно больше ~0.1 пикс, шаблон или камера двигались
                            Console.WriteLine("cam" + (i + 1) + ": усреднено кадров " + burst_cnt[i] + "; " + burst_shift(burst_ps_first[i], burst_ps_last[i]));
                        }
                        UtilOpenCV.saveImage(avg[0], cams_num > 1 ? avg[1] : null, glob_im_coubter.ToString(), cur_save_fold);
                        glob_im_coubter++;
                        for (int i = 0; i < cams_num; i++) avg[i].Dispose();
                        burst_active = false;
                    }
                    else if (burst_iters > 100 * save_frames_n)
                    {
                        Console.WriteLine("сохранение отменено: камеры не выдали " + save_frames_n + " новых кадров");
                        for (int i = 0; i < cams_num; i++) burst_acc[i]?.Dispose();
                        burst_active = false;
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
                    Console.WriteLine("s_pressed");
                    if (save_frames_n <= 1)
                    {
                        UtilOpenCV.saveImage(mats[0], mats[1], glob_im_coubter.ToString(), cur_save_fold);
                        glob_im_coubter++;
                    }
                    else if (!burst_active)
                    {
                        // начинаем накопление save_frames_n кадров с каждой камеры; сохранение - когда наберутся
                        burst_active = true;
                        burst_iters = 0;
                        for (int i = 0; i < cams_num; i++)
                        {
                            burst_cnt[i] = 0;
                            burst_acc[i] = null;
                            burst_last_counter[i] = cur_counter[i];
                            burst_ps_first[i] = null;
                            burst_ps_last[i] = null;
                        }
                        Console.WriteLine("накопление " + save_frames_n + " кадров, не двигайте шаблон...");
                    }
                }
                /*else if (key == 'f')
                {
                    fon_inited = true;
                    mats_fon[0] = mats[0].Clone();
                    mats_fon[1] = mats[1].Clone();
                    //UtilOpenCV.saveImage(mats[0], mats[1], glob_im_coubter.ToString(), cur_save_fold);

                }
                else if (key == 'd')
                {
                    fon_inited = false;
                }*/
                if (key == 27) break;  // Esc — выход
            }

            foreach (var c in videoCaptures) { c?.Stop(); c?.Dispose(); }
            CvInvoke.DestroyAllWindows();
        }
        /// <summary>смещение центров шаблона между первым и последним кадром накопления, пикс</summary>
        static string burst_shift(System.Drawing.PointF[] first, System.Drawing.PointF[] last)
        {
            if (first == null || last == null || first.Length == 0 || first.Length != last.Length) return "шаблон не найден";
            if ((first[0].IsEmpty && first[first.Length - 1].IsEmpty) || (last[0].IsEmpty && last[last.Length - 1].IsEmpty)) return "шаблон не найден";
            double s = 0, max = 0;
            for (int k = 0; k < first.Length; k++)
            {
                double dx = last[k].X - first[k].X;
                double dy = last[k].Y - first[k].Y;
                double d = Math.Sqrt(dx * dx + dy * dy);
                s += d * d;
                if (d > max) max = d;
            }
            return "сдвиг шаблона rms " + Math.Round(Math.Sqrt(s / first.Length), 3) + " max " + Math.Round(max, 3) + " пикс";
        }

        long frame_id = 0;   // поле класса

        static string PoseLine(long frame, int ind, Matrix<double>? M, double rms)
        {
            var ci = System.Globalization.CultureInfo.InvariantCulture;
            if (M == null) return $"P;{frame};{ind};0\n";                 // инструмент не виден
            var sb = new StringBuilder($"P;{frame};{ind};1;{rms.ToString("F3", ci)}");
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 4; c++)
                    sb.Append(';').Append(M[r, c].ToString(r < 3 && c < 3 ? "F6" : "F3", ci));
            return sb.Append('\n').ToString();
        }
        void draw_3d_ps(Point3d_GL[] ps3d, ref Mat[] mats_find, int r, int g, int b,int size, bool text = true)
        {
            if (ps3d != null && ps3d.Length > 0 && stereo_cam.R != null)
            {
                var cs0 = new Matrix<double>(new double[4, 4] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } });
                var pr0 = stereo_cam.cameraCVs[0].project_world(ps3d, cs0);
                var pr1 = stereo_cam.cameraCVs[1].project_world(ps3d, cs0 * stereo_cam.R);
                if (pr0.Length > 0) UtilOpenCV.drawPointsF(mats_find[0], pr0, r, g, b, size, text);
                if (pr1.Length > 0) UtilOpenCV.drawPointsF(mats_find[1], pr1, r, g, b, size, text);

            }
        }
        void draw_tool_frame(Matrix<double> M, Mat[] mats, int ind, double len = 40)
        {
            if (stereo_cam.R == null) return;
            var axes = new[] { new Point3d_GL(0, 0, 0), new Point3d_GL(len, 0, 0),
                       new Point3d_GL(0, len, 0), new Point3d_GL(0, 0, len) }
                       .Select(p => ToolDef.Transform(M, p)).ToArray();

            var cs0 = new Matrix<double>(new double[4, 4] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } });
            var pr = new[] { stereo_cam.cameraCVs[0].project_world(axes, cs0),
                     stereo_cam.cameraCVs[1].project_world(axes, cs0 * stereo_cam.R) };
            var col = new[] { new MCvScalar(0, 0, 255), new MCvScalar(0, 255, 0), new MCvScalar(255, 0, 0) }; // X красная, Y зелёная, Z синяя (BGR)

            for (int i = 0; i < 2; i++)
            {
                if (pr[i].Length < 4 || mats[i] == null || mats[i].IsEmpty) continue;
                var o = Point.Round(pr[i][0]);
                for (int a = 1; a <= 3; a++) CvInvoke.Line(mats[i], o, Point.Round(pr[i][a]), col[a - 1], 2);
                CvInvoke.PutText(mats[i], "T" + ind, new Point(o.X + 6, o.Y - 6),
                                 FontFace.HersheySimplex, 0.6, new MCvScalar(255, 255, 0), 2);
            }
        }
        private void timer_navig_processing_Tick(object sender, EventArgs e)
        {
            //var ps3d = navig_system.navigation_processing_get_points3d_3cam(navig_system.points2d_cams[0], navig_system.points2d_cams[1], navig_system.points2d_cams[2]);    //navyg_sys_info(camera_only)
            //navig_system.navigation_processing_get_scene(ps3d, true);
        }

    }

}
