using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using Accord.Statistics.Distributions.Univariate;


namespace opengl3
{
    //public class Pa
   public enum PatternType { Chess,Mesh};
    public class CameraCV
    {
        public Matrix<double> cameramatrix;
        public Matrix<double> cameramatrix_inv;

        public Matrix<double> cameramatrix_opt;
        public Matrix<double> cameramatrix_opt_inv;

        public Matrix<double> distortmatrix;
        public Frame[] frames;
        public System.Drawing.PointF[][] corners;
        public System.Drawing.PointF[] last_corners;
        public PointF im_centr;
        public MCvPoint3D32f[][] objps;
        public Mat[] tvecs;
        public Mat[] rvecs;
        public Mat cur_t;
        public Mat cur_r;
        public float[] pos;
        public Mat mapx;
        public Mat mapy;
        public Matrix<double> prjmatrix;
        public Matrix<double> prjmatrix_inv;
        public Matrix<double> matrixCS;
        public Matrix<double> matrixSC;
        public Size image_size;
        public Size pattern_size;
        public Rectangle newRoI;
        public float mark_size = 10f;

        public List<PointF[]> scan_points = new List<PointF[]>();

        void init_vars()
        {
            cur_t = new Mat();
            cur_r = new Mat();
            matrixCS = new Matrix<double>(4, 4);
            matrixSC = new Matrix<double>(4, 4);
            cameramatrix_inv = invMatrix(cameramatrix); 
            mapx = new Mat();
            mapy = new Mat();
            newRoI = new Rectangle();
            if(cameramatrix == null) { return; }
            var matr = CvInvoke.GetOptimalNewCameraMatrix(cameramatrix, distortmatrix, image_size, 1, image_size, ref newRoI);
            /* prin.t(cameramatrix);
             Console.WriteLine("optim");
             prin.t(matr);*/
            cameramatrix_opt = UtilMatr.toMatrix(matr);
            cameramatrix_opt_inv = invMatrix(cameramatrix_opt);
            //cameramatrix =
            prin.t("cameramatrix_opt");
            prin.t(cameramatrix_opt);
            prin.t("cameramatrix_opt_inv");
            prin.t(cameramatrix_opt_inv);


            im_centr = new PointF((float)cameramatrix_opt[0, 2], (float)cameramatrix_opt[1, 2]);
            CvInvoke.InitUndistortRectifyMap(cameramatrix_opt, distortmatrix, null, matr, image_size, DepthType.Cv32F,1, mapx, mapy);//_opt

            pos = new float[3] { 0, 0, 0 };
        }
        public CameraCV(Matrix<double> _cameramatrix, Matrix<double> _distortmatrix, Size im_size)
        {
            cameramatrix = _cameramatrix;
            distortmatrix = _distortmatrix;
            image_size = im_size;
            pattern_size = new Size(6, 7);
            
            init_vars();
        }
        public CameraCV(Frame[] _frames, Size _size, float markSize, MCvPoint3D32f[][] obp_inp)
        {
            calibrateCam(_frames, _size, markSize, obp_inp);
            //calibrateCamFish(_frames, _size, markSize, obp_inp);
            pattern_size = _size;
            init_vars();
        }

        

        void setPos()
        {
            pos[0] = (float)matrixCS[0, 3];
            pos[1] = (float)matrixCS[1, 3];
            pos[2] = (float)matrixCS[2, 3];
        }


        static public Matrix<double> invMatrix(Matrix<double> matr)
        {
            if (matr == null) return null;
            var inv_matr = new Matrix<double>(matr.Size);
            CvInvoke.Invert(matr, inv_matr, DecompMethod.LU);
            return inv_matr;
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="cur_r"></param>
        /// <param name="cur_t"></param>
        /// <returns>matr Cam->Scene,matr Scene->Cam</returns>
        static public Matrix<double>[] assemblMatrix(Mat cur_r, Mat cur_t)
        {

            var r = new Matrix<double>(3, 3);
            CvInvoke.Rodrigues(cur_r, r);
            var data_t = (double[,])cur_t.GetData();
            var data_mx = new double[4, 4]
            {
                {r[0,0] ,r[0,1],r[0,2],data_t[0,0]},
                {r[1,0] ,r[1,1],r[1,2],data_t[1,0]},
                {r[2,0] ,r[2,1],r[2,2],data_t[2,0]},
                {0,0,0,1 }
            };

            var matrSC = new Matrix<double>(data_mx);
            var matrCS = new Matrix<double>(4, 4);
            CvInvoke.Invert(matrSC, matrCS, DecompMethod.LU);
            return new Matrix<double>[] { matrCS, matrSC };

        }
        public float[] compPos(MCvPoint3D32f[] points3D, System.Drawing.PointF[] points2D)
        {
            
           // prin.t(points3D);
           // prin.t(points2D);
           //var distort = new Matrix<double>(new double[,] { { 0, 0, 0, 0, 0 } });
            CvInvoke.SolvePnP(points3D, points2D, cameramatrix, distortmatrix, cur_r, cur_t);//_opt

            var matrs = assemblMatrix(cur_r, cur_t);
            matrixCS = matrs[0];
            matrixSC = matrs[1];
            
            /*prin.t(matrixCS);
            prin.t(matrixSC);
            var data_mx = new double[4, 4]
            {
                {0 ,1,0,100 },
                {1 ,0,0,26},
                {0 ,0,-1,154},
                {0,0,0,1 }
            };
            matrixCS = new Matrix<double>(data_mx);
            CvInvoke.Invert(matrixCS, matrixSC, DecompMethod.LU);*/

            //var points_cam = PointCloud.fromLines(PointF.toPointF( points2D), this, LaserSurface.zeroFlatInCam(this.matrixSC));
            //var points3d_cur = PointCloud.camToScene(points_cam, this.matrixCS);
            /*prin.t(this.matrixSC);
            prin.t(this.matrixCS);
            Console.WriteLine("points3d_cur");
            foreach (var p in points3d_cur) Console.WriteLine(p);*/
            setPos();
            return pos;
        }

        /// <summary>СКО ошибки репроекции последнего compPos по шаблону, пикс</summary>
        public double last_reproj_err = -1;

        /// <summary>
        /// 3d-координаты всех точек сетки по 4 угловым точкам
        /// (corners: индексы 0, w-1, w*(h-1), w*h-1 упорядоченного массива, как в UtilOpenCV.takeGabObp)
        /// </summary>
        static MCvPoint3D32f[] gridObp(MCvPoint3D32f[] corners, Size size_patt)
        {
            int w = size_patt.Width;
            int h = size_patt.Height;
            var ps = new MCvPoint3D32f[w * h];
            var p0 = corners[0];
            var pw = corners[1];
            var ph = corners[2];
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    float u = w > 1 ? (float)i / (w - 1) : 0;
                    float v = h > 1 ? (float)j / (h - 1) : 0;
                    ps[j * w + i] = new MCvPoint3D32f(
                        p0.X + (pw.X - p0.X) * u + (ph.X - p0.X) * v,
                        p0.Y + (pw.Y - p0.Y) * u + (ph.Y - p0.Y) * v,
                        p0.Z + (pw.Z - p0.Z) * u + (ph.Z - p0.Z) * v);
                }
            }
            return ps;
        }

        /// <summary>СКО ошибки репроекции для текущих cur_r, cur_t, пикс</summary>
        double reprojErr(MCvPoint3D32f[] points3D, System.Drawing.PointF[] points2D)
        {
            var pr = CvInvoke.ProjectPoints(points3D, cur_r, cur_t, cameramatrix, distortmatrix);
            if (pr == null || pr.Length != points2D.Length || pr.Length == 0) return -1;
            double s = 0;
            for (int i = 0; i < pr.Length; i++)
            {
                double dx = pr[i].X - points2D[i].X;
                double dy = pr[i].Y - points2D[i].Y;
                s += dx * dx + dy * dy;
            }
            return Math.Sqrt(s / pr.Length);
        }

        public bool compPos(Mat _mat, PatternType patternType,Size pattern_size,float mark = -1,bool centr = false)
        {
            var mat = _mat.Clone();
            
            Size size_patt = pattern_size;
            float markSize = this.mark_size;
            if (mark > 0)
            {
                markSize = mark;
            }

            var x = markSize * (size_patt.Height - 1);
            var y = markSize * (size_patt.Width - 1);

            /* var points3d = new MCvPoint3D32f[]
                {
                    new MCvPoint3D32f(0,0,0),
                    new MCvPoint3D32f(x,0,0),
                    new MCvPoint3D32f(0,y,0),
                    new MCvPoint3D32f(x,y,0)
                };*/
            MCvPoint3D32f[] points3d = null;
            if (centr)
            {
                points3d = new MCvPoint3D32f[]
                {
                    new MCvPoint3D32f(x/2,y/2,0),
                    new MCvPoint3D32f(-x/2,y/2,0),
                    new MCvPoint3D32f(x/2,-y/2,0),
                    new MCvPoint3D32f(-x/2,-y/2,0)
                };
            }
            else 
            {
                /*points3d = new MCvPoint3D32f[]
                {
                    new MCvPoint3D32f(x,y,0),
                    new MCvPoint3D32f(0,y,0),
                    new MCvPoint3D32f(x,0,0),
                    new MCvPoint3D32f(0,0,0)
                };*/

                /* points3d = new MCvPoint3D32f[]
                    {
                        new MCvPoint3D32f(0,y,0), new MCvPoint3D32f(0,0,0),
                        new MCvPoint3D32f(x,y,0), new MCvPoint3D32f(x,0,0)
                    };*/

                points3d = new MCvPoint3D32f[]
                {
                    new MCvPoint3D32f(0,0,0), new MCvPoint3D32f(0,y,0),
                    new MCvPoint3D32f(x,0,0), new MCvPoint3D32f(x,y,0)
                };
            }

            var len = size_patt.Width * size_patt.Height;
            var cornF = new System.Drawing.PointF[len];
            //CvInvoke.Imshow("mat", mat);
            //CvInvoke.WaitKey(200);
            var matDraw = FindCircles.findCircles_v3(mat,ref cornF, size_patt);
            if (matDraw == null)
            {
                return false;
            }
            // шаблон не найден или не упорядочен: findCircles_v3 в этом случае оставляет массив нулей
            if (cornF == null || cornF.Length != len) return false;
            if (cornF[0].IsEmpty && cornF[len - 1].IsEmpty) return false;

            var mat_p1 = UtilOpenCV.drawPointsF(_mat, cornF, 255, 0, 0);
            last_corners = (System.Drawing.PointF[])cornF.Clone();

            // поза считается по всем точкам сетки, а не по 4 угловым:
            // 3d-координаты каждой точки - интерполяция между теми же 4 углами points3d,
            // поэтому система координат шаблона остаётся прежней
            var points3d_all = gridObp(points3d, size_patt);
            var pos_cur = compPos(points3d_all, cornF);
            last_reproj_err = reprojErr(points3d_all, cornF);
            /*Console.WriteLine(pos_cur[0] + " " + pos_cur[1] + " " + pos_cur[2] + " ");
            UtilOpenCV.drawPoints(matDraw, points2d, points3d, 255, 0, 255, 1);
            CvInvoke.Imshow("pos "+ pos_cur[0] + " " + pos_cur[1] + " " + pos_cur[2] + " ", matDraw);//points2d[0].X
            CvInvoke.WaitKey();*/
            mat = null;
            matDraw = null;

            GC.Collect();
            return true;
            
        }
        public Point3d_GL point3DfromCam(PointF _p)
        {
            using var src = new VectorOfPointF(new[] { new System.Drawing.PointF(_p.X, _p.Y) });
            using var dst = new VectorOfPointF();
            CvInvoke.UndistortPoints(src, dst, cameramatrix, distortmatrix); // исходная матрица, не _opt
            return new Point3d_GL(dst[0].X, dst[0].Y, 1);
        }

        public System.Drawing.PointF[] project_world(Point3d_GL[] ps, Matrix<double> cam_to_world)
        {
            if (ps == null || cam_to_world == null) return Array.Empty<System.Drawing.PointF>();
            var valid = ps.Where(p => p.exist).ToArray();
            if (valid.Length == 0) return Array.Empty<System.Drawing.PointF>();

            var w2c = new Matrix<double>(4, 4);
            CvInvoke.Invert(cam_to_world, w2c, DecompMethod.LU);

            var rot = w2c.GetSubRect(new Rectangle(0, 0, 3, 3)).Clone();
            var rvec = new Mat(); CvInvoke.Rodrigues(rot, rvec);
            var tvec = new Matrix<double>(new[] { w2c[0, 3], w2c[1, 3], w2c[2, 3] });

            var obj = valid.Select(p => new MCvPoint3D32f((float)p.x, (float)p.y, (float)p.z)).ToArray();
            return CvInvoke.ProjectPoints(obj, rvec, tvec, cameramatrix, distortmatrix);
        }

        public PointF point2DfromCam(Point3d_GL _p)//_opt
        {
            if(_p.z==0) return new PointF(0,0);
            var p_priv = _p / _p.z;
            var p = (cameramatrix_opt * p_priv);
            return new PointF(p.x,p.y);
        }
        public Mat undist(Mat mat)
        {
            var mat_ret = new Mat();
            CvInvoke.Remap(mat, mat_ret, mapx, mapy, Inter.Linear);
            //-----------------------------
            //CvInvoke.Undistort(mat, mat_ret, cameramatrix, distortmatrix, cameramatrix_opt);
            //mat_ret = new Mat(mat_ret, newRoI);
            /*CvInvoke.Imshow("mat", mat);
            CvInvoke.Imshow("mat_und", mat_ret);
            
            
            CvInvoke.Imshow("mat_und_cut", mat_ret);
            CvInvoke.WaitKey();*/

            return mat_ret;
        }
        void calibrateCam(Frame[] frames, Size size, float markSize, MCvPoint3D32f[][] obp_inp)
        {
            if (frames== null) { return; }
            if (frames.Length == 0) { return; }

            this.frames = frames;
            this.mark_size = markSize;
            Matrix<double> _cameramatrix = new Matrix<double>(3, 3);
            Matrix<double> _distortmatrix = new Matrix<double>(5, 1);

            var objps = new List<MCvPoint3D32f[]>();
            var corners = new List<System.Drawing.PointF[]>();

            var obp = new MCvPoint3D32f[size.Width * size.Height];

            int ind = 0;
            for (int j = 0; j < size.Height; j++)
            {
                for (int i = 0; i < size.Width; i++)
                {
                    obp[ind] = new MCvPoint3D32f(-markSize * (float)i, markSize * (float)j, 0.0f);
                    ind++;
                }
            }


            Console.WriteLine("fr len: " + frames.Length);
            int ind_fr = 0;
            int i_c = 0;
            foreach (var frame in frames)
            {
                if(i_c%1==0)
                {
                    var corn2 = findPoints(frame, size);
                    //prin.t(corn2);
                    //Console.WriteLine("___________________");
                    //corn2 = makeNoise(corn2,0.5f);
                    if (corn2 == null)
                    {
                        Console.WriteLine("NOT:");
                    }
                    else
                    {
                        objps.Add(obp);
                        corners.Add(corn2);
                    }
                    ind_fr++;
                    Console.WriteLine(frame.name);

                }

                i_c++;
            }



            var rvecs = new Mat[corners.Count];
            var tvecs = new Mat[corners.Count];


            this.objps = objps.ToArray();
            this.corners = corners.ToArray();
            this.image_size = frames[0].im.Size;
            if (obp_inp != null)
            {
                this.objps = obp_inp;
            }
            Console.WriteLine("objp: " + this.objps.Length);
            Console.WriteLine("corn: " + this.corners.Length);
            //prin.t("_______________");
            //UtilOpenCV.printMatch(this.objps, this.corners);
            /* prin.t(this.objps);
             prin.t("_______________");
             prin.t(this.corners);*/


            var err = CvInvoke.CalibrateCamera(this.objps, this.corners, this.image_size, _cameramatrix, _distortmatrix, CalibType.Default, new MCvTermCriteria(3000, 0.0001), out rvecs, out tvecs);

            Console.WriteLine("err: " + err);
            Console.WriteLine("t,r_len: " + tvecs.Length + " " + rvecs.Length);
            

            this.tvecs = tvecs;
            this.rvecs = rvecs;
            foreach (var v in tvecs)
            {
                //prin.t(v);
            }

            

            //computeDistortionMaps(ref mapx, ref mapy, _cameramatrix, _distortmatrix, frames[0].im.Size);
            
            //var und_pic = new Mat();
            //CvInvoke.Remap(frames[0].im, und_pic, mapx, mapy, Inter.Linear);
            //_cameramatrix[0, 0] *= 2.25;
            //_cameramatrix[1, 1] *= 2.25;
            cameramatrix = _cameramatrix;
            
            distortmatrix = _distortmatrix;
            
            var objpsn = new MCvPoint3D32f[0][];
            var cornersn = new System.Drawing.PointF[0][];
            //normalyseData(this.objps, this.corners, 640, 10, out objpsn, out cornersn);
            //calibrateCameraTest(this.objps, this.corners);
            prin.t("cameramatrix:");
            prin.t(cameramatrix);
            prin.t("distortmatrix:");
            prin.t(distortmatrix);

        }

        static System.Drawing.PointF[] makeNoise(System.Drawing.PointF[] ps, float ampl)
        {
            for(int i = 0; i < ps.Length; i++)
            {
                ps[i].X +=(float) NormalDistribution.Random(0, ampl);
                ps[i].Y += (float)NormalDistribution.Random(0, ampl);
            }
            

            return ps;
        }
        public  static System.Drawing.PointF[] findPoints(Mat mat, Size size_patt)
        {
            var gray = mat.ToImage<Gray, byte>();
            var corn = new VectorOfPointF();
            var ret = CvInvoke.FindChessboardCorners(gray, size_patt, corn);
            if (ret == true)
            {
                CvInvoke.CornerSubPix(gray, corn, new Size(5, 5), new Size(-1, -1), new MCvTermCriteria(30, 0.001));
                CvInvoke.DrawChessboardCorners(mat, size_patt, corn, ret);
              //  CvInvoke.Imshow("asda", mat);
                //CvInvoke.WaitKey();
                var corn2 = corn.ToArray();
                return corn2;
                /*if(obp_inp!=null)
                    {               
                        UtilOpenCV.drawMatches(mat1, corn2, UtilMatr.toPointF(obp_inp[ind_fr]), 255, 0, 0, 3);                       
                    }
                    else
                    {
                        CvInvoke.DrawChessboardCorners(mat1, size, corn, ret);
                        //UtilOpenCV.drawMatches(mat1, corn2, UtilMatr.toPointF(obp), 255, 0, 0, 3);
                    }
                    UtilOpenCV.drawMatches(mat1, corn2, UtilMatr.toPointF(obp_inp[ind_fr]), 255, 0, 0, 3);
                    CvInvoke.Imshow("asda", mat1);
                     CvInvoke.WaitKey();*/


                //Console.WriteLine(frame.name);
            }
            else
            {
                Console.WriteLine("not find patt");
                return null;
            }
        }

        public static Mat findPoints_chess(Mat mat, Size size_patt)
        {
            var gray = mat.ToImage<Gray, byte>();
            var corn = new VectorOfPointF();
            if(mat.IsEmpty) return mat;
            var ret = CvInvoke.FindChessboardCorners(gray, size_patt, corn,CalibCbType.AdaptiveThresh);
            if (ret == true)
            {
               // ret = CvInvoke.FindChessboardCorners(gray, size_patt, corn, CalibCbType.AdaptiveThresh);
                CvInvoke.CornerSubPix(gray, corn, new Size(5, 5), new Size(-1, -1), new MCvTermCriteria(30, 0.001));
                CvInvoke.DrawChessboardCorners(mat, size_patt, corn, ret);
                //var corn2 = corn.ToArray();
            }
            return mat;
        }
        public static System.Drawing.PointF[] findPoints(Frame frame, Size size_patt)
        {
            
            if (frame.frameType == FrameType.MarkBoard)
            {
                return findPoints(frame.im, size_patt);
            }
            else if (frame.frameType == FrameType.Pattern)
            {
                var len = size_patt.Width * size_patt.Height;
                var cornF = new System.Drawing.PointF[len];
                var f_c = FindCircles.findCircles_v3(frame.im,ref cornF, size_patt);
                
                if(f_c != null)
                {
                   // CvInvoke.Imshow("asda", f_c);
                   // CvInvoke.WaitKey();
                }
               // cornF = PointF.addNoise(cornF, 1.0f);
                //mat = null;
                //Console.WriteLine(" cornF");
                //prin.t(cornF);
                //Console.WriteLine(" corn2");
                if (f_c == null)
                    return null;
                
                if (cornF == null)          
                    return null;            
                if (cornF.Length != len)
                    return null;
                else
                    return cornF;
            }
            else
            {
                return null;
            }

        }

        public static Mat findPoints_circle(Mat mat, Size size_patt)
        {

           
            var len = size_patt.Width * size_patt.Height;
            var cornF = new System.Drawing.PointF[len];
            Console.WriteLine("size " + size_patt.Width + " " + size_patt.Height);
            var f_c = FindCircles.findCircles(mat, ref cornF, size_patt);
            return f_c;

        }
        static public CameraCV load_camera(string path)
        {
            var settings = Settings_loader.load_data(path);

            var cameramatrix = (Matrix<double>)settings[0];
            var distortionmatrix = (Matrix<double>)settings[1];
            var size = (Size)settings[2];
            return new CameraCV(cameramatrix, distortionmatrix, size);
        }

        public void save_camera(string path)
        {
            Settings_loader.save_file(path, new object[] { cameramatrix, distortmatrix, image_size });
        }




        void normalyseData(MCvPoint3D32f[][] points3d, System.Drawing.PointF[][] points2d, float k2d, float k3d, out MCvPoint3D32f[][] npoints3d, out System.Drawing.PointF[][] npoints2d)
        {
            npoints3d = (MCvPoint3D32f[][])points3d.Clone();
            for (int i = 0; i < npoints3d.Length; i++)
            {
                npoints3d[i] = (MCvPoint3D32f[])points3d[i].Clone();
            }
            npoints2d = (System.Drawing.PointF[][])points2d.Clone();
            for (int i = 0; i < npoints3d.Length; i++)
            {
                for (int j = 0; j < npoints3d[i].Length; j++)
                {
                    Console.WriteLine("_____________________");
                    Console.WriteLine(points3d[i][j].X + " " + points3d[i][j].Y);
                    npoints3d[i][j] = new MCvPoint3D32f(points3d[i][j].X / k3d, points3d[i][j].Y / k3d, points3d[i][j].Z / k3d);
                    Console.WriteLine(npoints3d[i][j].X + " " + npoints3d[i][j].Y);
                }
            }
            Console.WriteLine("|||||||||||||||||||||||||||||||||||||");
            for (int i = 0; i < npoints2d.Length; i++)
            {
                for (int j = 0; j < npoints2d[i].Length; j++)
                {
                    //Console.WriteLine("_____________________");
                    //Console.WriteLine(points2d[i][j].X + " " + points2d[i][j].Y);
                    npoints2d[i][j] = new System.Drawing.PointF(npoints2d[i][j].X / k2d, npoints2d[i][j].Y / k2d);
                    //Console.WriteLine(npoints2d[i][j].X + " " + npoints2d[i][j].Y);
                }
            }

        }
        void calibrateCamFish(Frame[] frames, Size size, float markSize, MCvPoint3D32f[][] obp_inp)
        {
            this.frames = frames;

            Matrix<double> _cameramatrix = new Matrix<double>(3, 3);
            Matrix<double> _distortmatrix = new Matrix<double>(4, 1);

            var objps = new List<MCvPoint3D32f[]>();
            var corners = new List<System.Drawing.PointF[]>();
            size = new Size(size.Height, size.Width);
            var obp = new MCvPoint3D32f[size.Width * size.Height];

            int ind = 0;
            for (int j = 0; j < size.Height; j++)
            {
                for (int i = 0; i < size.Width; i++)
                {
                    obp[ind] = new MCvPoint3D32f(markSize * ((float)i - size.Width / 2), markSize * ((float)j - size.Height / 2), 0.0f);
                    ind++;
                }
            }

            Console.WriteLine("FISH______________________________: " + frames.Length);
            Console.WriteLine("fr len: " + frames.Length);
            int ind_fr = 0;
            foreach (var frame in frames)
            {
                var corn2 = findPoints(frame, size);
                var im1 = frame.im.Clone();
                //UtilOpenCV.drawMatches(im1, UtilOpenCV.removeFromNegative( obp), corn2, 1, 0, 0);
                //CvInvoke.Imshow("yk"+ind_fr.ToString(), im1);
                if (corn2 == null)
                {
                    Console.WriteLine("NOT:");
                    Console.WriteLine(frame.name);
                }
                else
                {
                    objps.Add(obp);
                    corners.Add(corn2);
                }
                ind_fr++;
            }

            //CvInvoke.WaitKey();

            var rvecs = new Mat();
            var tvecs = new Mat();


            this.objps = objps.ToArray();
            this.corners = corners.ToArray();

            var p3d = new VectorOfVectorOfPoint3D32F(objps.GetRange(0, 10).ToArray());
            var p2d = new VectorOfVectorOfPointF(corners.GetRange(0, 10).ToArray());

            this.image_size = frames[0].im.Size;
            if (obp_inp != null)
            {
                this.objps = obp_inp;
            }
            Console.WriteLine("objp: " + this.objps.Length);
            Console.WriteLine("corn: " + this.corners.Length);
            //prin.t("_______________");
            //UtilOpenCV.printMatch(this.objps, this.corners);
            /* prin.t(this.objps);
             prin.t("_______________");
             prin.t(this.corners);*/
            //var err = CvInvoke.CalibrateCamera(this.objps, this.corners, this.image_size, _cameramatrix, _distortmatrix, CalibType.Default, new MCvTermCriteria(100, 0.01), out rvecs, out tvecs);


            Fisheye.Calibrate(p3d, p2d, this.image_size, _cameramatrix, _distortmatrix, rvecs, tvecs, Fisheye.CalibrationFlag.Default, new MCvTermCriteria(30, 0.01));

            //Console.WriteLine("err: " + err);
            //Console.WriteLine("t,r_len: " + tvecs.Length + " " + rvecs.Length);
            var newRoI = new Rectangle();

            //this.tvecs = tvecs;
            //this.rvecs = rvecs;
            prin.t(tvecs);
            var matrP = new Matrix<double>(3, 3);
            matrP[0, 0] = 1;
            matrP[1, 1] = 1;
            matrP[2, 2] = 1;
            var matrR = new Matrix<double>(3, 3);
            Fisheye.EstimateNewCameraMatrixForUndistortRectify(_cameramatrix, _distortmatrix, this.image_size, matrR, matrP);
            var mapx = new Mat();
            var mapy = new Mat();
            /* var matr = CvInvoke.GetOptimalNewCameraMatrix(_cameramatrix, _distortmatrix, frames[0].im.Size, 1, frames[0].im.Size, ref newRoI);

             //computeDistortionMaps(ref mapx, ref mapy, _cameramatrix, _distortmatrix, frames[0].im.Size);
             mapx = new Mat();
             mapy = new Mat();
             CvInvoke.InitUndistortRectifyMap(_cameramatrix, _distortmatrix, null, matr, frames[0].im.Size, DepthType.Cv32F, mapx, mapy);

             var und_pic = new Mat();
             CvInvoke.Remap(frames[0].im, und_pic, mapx, mapy, Inter.Linear);
             cameramatrix = _cameramatrix;
             cameramatrix_inv = invMatrix(cameramatrix);
             distortmatrix = _distortmatrix;*/

            Fisheye.InitUndistortRectifyMap(_cameramatrix, _distortmatrix, matrR, matrP, frames[0].im.Size, DepthType.Cv32F,1, mapx, mapy);
            //prin.t(mapx);
            prin.t(" matrR:");
            prin.t(matrR);
            prin.t(" matrP:");
            prin.t(matrP);
            var im = UtilOpenCV.mapToMat(UtilOpenCV.mapToFloat(mapx));
            CvInvoke.Imshow("mapx", im);
            var und_pic = new Mat();
            CvInvoke.Remap(frames[0].im, und_pic, mapx, mapy, Inter.Linear);
            CvInvoke.Imshow("orig", frames[0].im);
            CvInvoke.Imshow("undist", und_pic);
            // CvInvoke.WaitKey();
            prin.t("Fish cameramatrix:");
            prin.t(_cameramatrix);
            prin.t("Fish distortmatrix:");
            prin.t(_distortmatrix);

        }


    }
}
