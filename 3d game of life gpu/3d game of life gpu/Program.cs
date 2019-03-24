using System;
using Alea;
using Alea.CSharp;
using Alea.cuRAND;
using SharpDX;
using SharpDX.Direct3D9;
using SharpDX.Windows;
using D3D9Device = SharpDX.Direct3D9.DeviceEx;
using CUDADevice = Alea.Device;
using System.Diagnostics;
using System.Reflection;
using SharpDX.Mathematics.Interop;
using System.Windows.Forms;

namespace _3d_game_of_life_gpu
{
    class Program
    {
        private const int Width = 80;
        private const int Height = 80;
        private const int Depth = 80;
        private const int CubesPerUnit = 160;
        private const float PixelsPerUnit = 624;
        private const int FacePointNum = 18;
        private const int EdgePointNum = 24;
        private const int Total = Width * Height * Depth;
        private const int updateMilliseconds = 10;
        private const int inverseCubeDensity = 6;
        private const int rotationPeriodMilliseconds = 3000;
        private const float mouseSensitivity = 1f / CubesPerUnit;
        private static bool isMouseDown;
        private static int2 mousePosition = new int2(-1, 0);
        private static Vector3 upDirection = new Vector3(0, 1, 0);
        private static Vector3 cameraPosition = new Vector3(0.5f, 0.5f, 1.0f);
        private static Vector3 lightPosition = new Vector3(0.6350852961f, 0.6350852961f, 0.6350852961f);
        private static readonly dim3 BlockSize = new dim3(8, 8, 8);
        private static readonly dim3 GridSize = new dim3(Width / BlockSize.x, Height / BlockSize.y, Depth / BlockSize.z);
        private static readonly LaunchParam launchParam = new LaunchParam(GridSize, BlockSize);
        private const RngType rngType = RngType.PSEUDO_DEFAULT;
        private static Stopwatch clock;
        private static int deviceId;
        private static Gpu gpu;
        private static IntPtr faceVertexBufferResource;
        private static IntPtr boundaryVertexBufferResource;
        private static IntPtr existOldVertexBufferResource;
        private static IntPtr existNewVertexBufferResource;
        private static VertexBuffer faceVertices;
        private static VertexBuffer boundaryVertices;
        private static VertexBuffer existOldVertices;
        private static VertexBuffer existNewVertices;
        private static VertexDeclaration faceDeclaration;
        private static VertexDeclaration edgeDeclaration;
        private static D3D9Device device;
        private static RenderForm form;
        private static int3 visibleCorner;
        private static Material matFace;
        private static Material matBoundary;
        private static int currentMilliseconds;
        private static int offsetMilliseconds;
        private static Light light;
        public static float Color(float r, float g, float b)
        {
            return (0x010000 * b + 0x000100 * g + 0x000001 * r);
        }
        public static void FillRandom(deviceptr<bool> existNew, uint[] randomUnformatted)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            int j = blockIdx.y * blockDim.y + threadIdx.y;
            int k = blockIdx.z * blockDim.z + threadIdx.z;
            existNew[Index(i, j, k)] = randomUnformatted[Index(i, j, k)] % inverseCubeDensity == 0;
        }
        public static void MakeBoundary(deviceptr<float3> boundary)
        {
            AddCubeEdges(boundary,
                new float3(
                    Scale(0, Width),
                    Scale(0, Height),
                    Scale(0, Depth)),
                new float3(
                    Scale(Width, Width),
                    Scale(Height, Height),
                    Scale(Depth, Depth)));
        }
        public static void CopyNewToOld(deviceptr<bool> existOld, deviceptr<bool> existNew)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            int j = blockIdx.y * blockDim.y + threadIdx.y;
            int k = blockIdx.z * blockDim.z + threadIdx.z;
            existOld[Index(i, j, k)] = existNew[Index(i, j, k)];
        }
        public static void UpdateGame(deviceptr<bool> existOld, deviceptr<bool> existNew)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            int j = blockIdx.y * blockDim.y + threadIdx.y;
            int k = blockIdx.z * blockDim.z + threadIdx.z;
            int nearbyAlive = 0;
            for (int dx = -1; dx < 2; dx++)
            {
                for (int dy = -1; dy < 2; dy++)
                {
                    for (int dz = -1; dz < 2; dz++)
                    {
                        if (dx != 0 || dy != 0 || dz != 0)
                        {
                            nearbyAlive += existOld[Index(i + dx, j + dy, k + dz)] ? 1 : 0;
                        }
                    }
                }
            }
            existNew[Index(i, j, k)] = (existOld[Index(i, j, k)] && nearbyAlive >= 4 && nearbyAlive <= 5) || (nearbyAlive >= 5 && nearbyAlive <= 5);
        }
        public static float Scale(int unscaled, int maxVal) => ((float)unscaled - (maxVal / 2)) / CubesPerUnit * 2;
        public static void MakeFaces(deviceptr<float3> face, deviceptr<bool> exist, int3 visibleCorner)
        {
            int x = blockIdx.x * blockDim.x + threadIdx.x;
            int y = blockIdx.y * blockDim.y + threadIdx.y;
            int z = blockIdx.z * blockDim.z + threadIdx.z;
            for (int i = 0; i < FacePointNum; i++)
            {
                face[FaceIndex(x, y, z, 2 * i)] = new float3(0, 0, 0);
                face[FaceIndex(x, y, z, 2 * i + 1)] = new float3(0, 0, 1);
            }
            if (!exist[Index(x, y, z)])
                return;
            float scaledXLower = Scale(x, Width);
            float scaledYLower = Scale(y, Height);
            float scaledZLower = Scale(z, Depth);
            float scaledXHigher = Scale(x + 1, Width);
            float scaledYHigher = Scale(y + 1, Height);
            float scaledZHigher = Scale(z + 1, Depth);
            float3 origin = new float3(scaledXLower, scaledYLower, scaledZLower);
            float3 X = new float3(scaledXHigher, scaledYLower, scaledZLower);
            float3 Y = new float3(scaledXLower, scaledYHigher, scaledZLower);
            float3 Z = new float3(scaledXLower, scaledYLower, scaledZHigher);
            float3 XY = new float3(scaledXHigher, scaledYHigher, scaledZLower);
            float3 XZ = new float3(scaledXHigher, scaledYLower, scaledZHigher);
            float3 YZ = new float3(scaledXLower, scaledYHigher, scaledZHigher);
            float3 XYZ = new float3(scaledXHigher, scaledYHigher, scaledZHigher);
            int index = 0;
            if (visibleCorner.z == 0)
                AddFace(face, new int4(x, y, z, index++), origin, Y, XY, X, new float3(0, 0, -1));
            if (visibleCorner.y == 0)
                AddFace(face, new int4(x, y, z, index++), origin, X, XZ, Z, new float3(0, -1, 0));
            if (visibleCorner.x == 0)
                AddFace(face, new int4(x, y, z, index++), origin, Z, YZ, Y, new float3(-1, 0, 0));
            if (visibleCorner.z == 1)
                AddFace(face, new int4(x, y, z, index++), XZ, XYZ, YZ, Z, new float3(0, 0, 1));
            if (visibleCorner.y == 1)
                AddFace(face, new int4(x, y, z, index++), YZ, XYZ, XY, Y, new float3(0, 1, 0));
            if (visibleCorner.x == 1)
                AddFace(face, new int4(x, y, z, index++), XY, XYZ, XZ, X, new float3(1, 0, 0));
            int packedVisiblePoint = visibleCorner.x * 4 + visibleCorner.y * 2 + visibleCorner.z;
        }
        public static void AddFace(deviceptr<float3> face, int4 arrayPos, float3 BL, float3 TL, float3 TR, float3 BR, float3 normal)
        {
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w)] = BL;
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w + 1)] = normal;
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w + 2)] = TL;
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w + 3)] = normal;
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w + 4)] = BR;
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w + 5)] = normal;
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w + 6)] = BR;
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w + 7)] = normal;
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w + 8)] = TL;
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w + 9)] = normal;
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w + 10)] = TR;
            face[FaceIndex(arrayPos.x, arrayPos.y, arrayPos.z, 12 * arrayPos.w + 11)] = normal;
        }
        public static void AddCubeEdges(deviceptr<float3> edge, float3 visibleLocation, float3 hiddenLocation)
        {
            edge[0] = new float3(visibleLocation.x, visibleLocation.y, visibleLocation.z);
            edge[1] = new float3(visibleLocation.x, visibleLocation.y, hiddenLocation.z);
            edge[2] = new float3(visibleLocation.x, visibleLocation.y, visibleLocation.z);
            edge[3] = new float3(visibleLocation.x, hiddenLocation.y, visibleLocation.z);
            edge[4] = new float3(visibleLocation.x, visibleLocation.y, visibleLocation.z);
            edge[5] = new float3(hiddenLocation.x, visibleLocation.y, visibleLocation.z);
            edge[6] = new float3(visibleLocation.x, visibleLocation.y, hiddenLocation.z);
            edge[7] = new float3(visibleLocation.x, hiddenLocation.y, hiddenLocation.z);
            edge[8] = new float3(visibleLocation.x, visibleLocation.y, hiddenLocation.z);
            edge[9] = new float3(hiddenLocation.x, visibleLocation.y, hiddenLocation.z);
            edge[10] = new float3(visibleLocation.x, hiddenLocation.y, visibleLocation.z);
            edge[11] = new float3(visibleLocation.x, hiddenLocation.y, hiddenLocation.z);
            edge[12] = new float3(visibleLocation.x, hiddenLocation.y, visibleLocation.z);
            edge[13] = new float3(hiddenLocation.x, hiddenLocation.y, visibleLocation.z);
            edge[14] = new float3(hiddenLocation.x, visibleLocation.y, visibleLocation.z);
            edge[15] = new float3(hiddenLocation.x, visibleLocation.y, hiddenLocation.z);
            edge[16] = new float3(hiddenLocation.x, visibleLocation.y, visibleLocation.z);
            edge[17] = new float3(hiddenLocation.x, hiddenLocation.y, visibleLocation.z);
            edge[18] = new float3(hiddenLocation.x, hiddenLocation.y, visibleLocation.z);
            edge[19] = new float3(hiddenLocation.x, hiddenLocation.y, hiddenLocation.z);
            edge[20] = new float3(hiddenLocation.x, visibleLocation.y, hiddenLocation.z);
            edge[21] = new float3(hiddenLocation.x, hiddenLocation.y, hiddenLocation.z);
            edge[22] = new float3(visibleLocation.x, hiddenLocation.y, hiddenLocation.z);
            edge[23] = new float3(hiddenLocation.x, hiddenLocation.y, hiddenLocation.z);
        }
        public static int FaceIndex(int x, int y, int z, int i) => ((x * Height + y) * Depth + z) * FacePointNum * 2 + i;
        public static int EdgeIndex(int x, int y, int z, int i) => ((x * Height + y) * Depth + z) * EdgePointNum + i;
        public static int Index(int x, int y, int z)
        {
            return (((x + Width) % Width) * Height + ((y + Height) % Height)) * Depth + ((z + Depth) % Depth);
        }
        unsafe public static deviceptr<T> DoMemoryStuff<T>(IntPtr resourceRef)
        {
            CUDAInterop.cuSafeCall(CUDAInterop.cuGraphicsMapResources(1, &resourceRef, IntPtr.Zero));
            IntPtr resourcePtr = IntPtr.Zero;
            IntPtr resourceSize = IntPtr.Zero;
            CUDAInterop.cuSafeCall(CUDAInterop.cuGraphicsResourceGetMappedPointer(&resourcePtr, &resourceSize, resourceRef));
            return new deviceptr<T>(resourcePtr);
        }
        unsafe public static void Setup()
        {
            IntPtr existNewVertexBufferResourceRef = existNewVertexBufferResource;
            IntPtr boundaryVertexBufferResourceRef = boundaryVertexBufferResource;
            deviceptr<bool> existNew = DoMemoryStuff<bool>(existNewVertexBufferResourceRef);
            deviceptr<float3> boundary = DoMemoryStuff<float3>(boundaryVertexBufferResourceRef);
            uint[] randomUnformatted = new uint[Total];
            using (Generator rng = Generator.CreateGpu(gpu, rngType))
            {
                rng.SetPseudoRandomGeneratorSeed((ulong)currentMilliseconds);
                rng.Generate(randomUnformatted);
            }
            gpu.Launch(FillRandom, launchParam, existNew, randomUnformatted);
            gpu.Launch(MakeBoundary, launchParam, boundary);
            CUDAInterop.cuSafeCall(CUDAInterop.cuGraphicsUnmapResources(1u, &existNewVertexBufferResourceRef, IntPtr.Zero));
            CUDAInterop.cuSafeCall(CUDAInterop.cuGraphicsUnmapResources(1u, &boundaryVertexBufferResourceRef, IntPtr.Zero));
        }
        unsafe public static void Update()
        {
            IntPtr existOldVertexBufferResourceRef = existOldVertexBufferResource;
            IntPtr existNewVertexBufferResourceRef = existNewVertexBufferResource;
            deviceptr<bool> existOld = DoMemoryStuff<bool>(existOldVertexBufferResourceRef);
            deviceptr<bool> existNew = DoMemoryStuff<bool>(existNewVertexBufferResourceRef);
            gpu.Launch(CopyNewToOld, launchParam, existOld, existNew);
            gpu.Launch(UpdateGame, launchParam, existOld, existNew);
            CUDAInterop.cuSafeCall(CUDAInterop.cuGraphicsUnmapResources(1u, &existOldVertexBufferResourceRef, IntPtr.Zero));
            CUDAInterop.cuSafeCall(CUDAInterop.cuGraphicsUnmapResources(1u, &existNewVertexBufferResourceRef, IntPtr.Zero));
        }
        unsafe public static void Draw()
        {
            IntPtr faceVertexBufferResourceRef = faceVertexBufferResource;
            IntPtr existNewVertexBufferResourceRef = existNewVertexBufferResource;
            deviceptr<bool> existNew = DoMemoryStuff<bool>(existNewVertexBufferResourceRef);
            deviceptr<float3> face = DoMemoryStuff<float3>(faceVertexBufferResourceRef);
            gpu.Launch(MakeFaces, launchParam, face, existNew, visibleCorner);
            CUDAInterop.cuSafeCall(CUDAInterop.cuGraphicsUnmapResources(1u, &faceVertexBufferResourceRef, IntPtr.Zero));
            CUDAInterop.cuSafeCall(CUDAInterop.cuGraphicsUnmapResources(1u, &existNewVertexBufferResourceRef, IntPtr.Zero));
        }
        unsafe static IntPtr RegisterVerticesResource(VertexBuffer vertices)
        {
            IntPtr res = IntPtr.Zero;
            CUDAInterop.cuSafeCall(CUDAInterop.cuGraphicsD3D9RegisterResource(&res, vertices.NativePointer, 0));
            return res;
        }
        static void UnregisterVerticesResource(IntPtr res)
        {
            CUDAInterop.cuSafeCall(CUDAInterop.cuGraphicsUnregisterResource(res));
        }
        unsafe static void Main(string[] args)
        {
            clock = Stopwatch.StartNew();
            deviceId = CUDADevice.Default.Id;
            gpu = Gpu.Get(deviceId);
            gpu.Context.SetCurrent();
            var iconStream = Assembly.GetExecutingAssembly().GetManifestResourceStream(typeof(Program), "Icon.ico");
            form = new RenderForm("3D Game of Life") { WindowState = FormWindowState.Maximized, Icon = new System.Drawing.Icon(iconStream) };
            device = new D3D9Device(
                new Direct3DEx(),
                deviceId,
                DeviceType.Hardware,
                form.Handle,
                CreateFlags.HardwareVertexProcessing,
                new PresentParameters(form.ClientSize.Width, form.ClientSize.Height));
            form.KeyDown += KeyPressed;
            form.MouseDown += MouseDown;
            form.MouseMove += MouseMove;
            form.MouseUp += MouseUp;
            VertexElement[] face = new VertexElement[]
            {
                new VertexElement(0, 0, DeclarationType.Float3, DeclarationMethod.Default, DeclarationUsage.Position, 0),
                new VertexElement(0, 12, DeclarationType.Float3, DeclarationMethod.Default, DeclarationUsage.Normal, 0),
                VertexElement.VertexDeclarationEnd
            };
            VertexElement[] edge = new VertexElement[]
            {
                new VertexElement(0, 0, DeclarationType.Float3, DeclarationMethod.Default, DeclarationUsage.Position, 0),
                VertexElement.VertexDeclarationEnd
            };
            device.SetRenderState(RenderState.Lighting, true);
            device.SetRenderState(RenderState.ShadeMode, ShadeMode.Flat);
            faceDeclaration = new VertexDeclaration(device, face);
            edgeDeclaration = new VertexDeclaration(device, edge);
            faceVertices = new VertexBuffer(device, sizeof(float3) * 2 * Total * FacePointNum, Usage.WriteOnly, VertexFormat.Position, Pool.Default);
            boundaryVertices = new VertexBuffer(device, sizeof(float3) * EdgePointNum, Usage.WriteOnly, VertexFormat.Normal, Pool.Default);
            existOldVertices = new VertexBuffer(device, sizeof(bool) * Total, Usage.WriteOnly, VertexFormat.None, Pool.Default);
            existNewVertices = new VertexBuffer(device, sizeof(bool) * Total, Usage.WriteOnly, VertexFormat.None, Pool.Default);
            faceVertexBufferResource = RegisterVerticesResource(faceVertices);
            boundaryVertexBufferResource = RegisterVerticesResource(boundaryVertices);
            existOldVertexBufferResource = RegisterVerticesResource(existOldVertices);
            existNewVertexBufferResource = RegisterVerticesResource(existNewVertices);
            Setup();
            int prevMilliseconds = 0;
            light = new Light()
            {
                Type = LightType.Point,
                Specular = new RawColor4(1, 1, 1, 1),
                Range = 1000,
                Position = lightPosition,
                Diffuse = new RawColor4(1, 1, 1, 1),
                Attenuation1 = 1,
                Attenuation0 = 0,
                Attenuation2 = 0,
                Ambient = new RawColor4(1, 1, 1, 1),
                Direction = new RawVector3(1, 1, 1)
            };
            matFace = new Material()
            {
                Diffuse = new RawColor4(0.7f, 0.7f, 0.7f, 1),
                Ambient = new RawColor4(0.1f, 0.1f, 0.1f, 1),
                Specular = new RawColor4(0.8f, 0.8f, 0.8f, 1),
                Emissive = new RawColor4(0, 0, 0, 0),
                Power = 1
            };
            matBoundary = new Material()
            {
                Diffuse = new RawColor4(1, 1, 1, 1),
                Ambient = new RawColor4(1, 1, 1, 1),
                Specular = new RawColor4(1, 1, 1, 1),
                Emissive = new RawColor4(1, 1, 1, 1),
                Power = 1
            };
            device.SetLight(0, ref light);
            device.EnableLight(0, true);
            Matrix proj = Matrix.OrthoLH(form.Width / PixelsPerUnit, form.Height / PixelsPerUnit, 0, 100);
            device.SetTransform(TransformState.Projection, proj);
            RenderLoop.Run(form, () =>
            {
                if (clock.IsRunning)
                {
                    currentMilliseconds = (int)(clock.Elapsed.TotalMilliseconds);
                    if (prevMilliseconds / updateMilliseconds != (currentMilliseconds + offsetMilliseconds) / updateMilliseconds)
                        Update();
                    prevMilliseconds = currentMilliseconds + offsetMilliseconds;
                    Draw();
                    Render();
                }
            });

            UnregisterVerticesResource(faceVertexBufferResource);
            UnregisterVerticesResource(existOldVertexBufferResource);
            UnregisterVerticesResource(existNewVertexBufferResource);
            UnregisterVerticesResource(boundaryVertexBufferResource);
            faceDeclaration.Dispose();
            edgeDeclaration.Dispose();
            faceVertices.Dispose();
            boundaryVertices.Dispose();
            existOldVertices.Dispose();
            existNewVertices.Dispose();
            device.Dispose();
            form.Dispose();
        }
        private static void MouseUp(object sender, MouseEventArgs e)
        {
            isMouseDown = false;
        }
        private static void MouseMove(object sender, MouseEventArgs e)
        {
            if (!isMouseDown)
                return;
            float2 offset = new float2((mousePosition.x - e.X) * mouseSensitivity, (e.Y - mousePosition.y) * mouseSensitivity);
            float C = DeviceFunction.Cos(offset.x);
            float S = DeviceFunction.Sin(offset.x);
            float t = 1 - C;
            Matrix3x3 rotationTransformHorizontal = new Matrix3x3
            {
                M11 = t * upDirection.X * upDirection.X + C,
                M12 = t * upDirection.X * upDirection.Y - S * upDirection.Z,
                M13 = t * upDirection.X * upDirection.Z + S * upDirection.Y,
                M21 = t * upDirection.X * upDirection.Y + S * upDirection.Z,
                M22 = t * upDirection.Y * upDirection.Y + C,
                M23 = t * upDirection.Y * upDirection.Z - S * upDirection.X,
                M31 = t * upDirection.X * upDirection.Z - S * upDirection.Y,
                M32 = t * upDirection.Y * upDirection.Z + S * upDirection.X,
                M33 = t * upDirection.Z * upDirection.Z + C
            };
            cameraPosition = Vector3.Transform(cameraPosition, rotationTransformHorizontal);
            C = DeviceFunction.Cos(offset.y);
            S = DeviceFunction.Sin(offset.y);
            t = 1 - C;
            Vector3 perpindicularAxis = Vector3.Normalize(Vector3.Cross(upDirection, cameraPosition));
            Matrix3x3 rotationTransformVertical = new Matrix3x3
            {
                M11 = t * perpindicularAxis.X * perpindicularAxis.X + C,
                M12 = t * perpindicularAxis.X * perpindicularAxis.Y - S * perpindicularAxis.Z,
                M13 = t * perpindicularAxis.X * perpindicularAxis.Z + S * perpindicularAxis.Y,
                M21 = t * perpindicularAxis.X * perpindicularAxis.Y + S * perpindicularAxis.Z,
                M22 = t * perpindicularAxis.Y * perpindicularAxis.Y + C,
                M23 = t * perpindicularAxis.Y * perpindicularAxis.Z - S * perpindicularAxis.X,
                M31 = t * perpindicularAxis.X * perpindicularAxis.Z - S * perpindicularAxis.Y,
                M32 = t * perpindicularAxis.Y * perpindicularAxis.Z + S * perpindicularAxis.X,
                M33 = t * perpindicularAxis.Z * perpindicularAxis.Z + C
            };
            cameraPosition = Vector3.Transform(cameraPosition, rotationTransformVertical);
            upDirection = Vector3.Transform(upDirection, rotationTransformVertical);
            mousePosition = new int2(e.X, e.Y);
            Draw();
        }
        private static void MouseDown(object sender, MouseEventArgs e)
        {
            mousePosition = new int2(e.X, e.Y);
            isMouseDown = true;
            }
        private static unsafe void Render()
        {
            light.Position = lightPosition;
            device.SetLight(0, ref light);
            visibleCorner = new int3(cameraPosition.X > 0 ? 1 : 0, cameraPosition.Y > 0 ? 1 : 0, cameraPosition.Z > 0 ? 1 : 0);
            Matrix view = Matrix.LookAtLH(
                              cameraPosition,
                              new Vector3(0.0f, 0.0f, 0.0f),
                              upDirection);
            device.SetTransform(TransformState.View, view);
            device.Clear(ClearFlags.Target | ClearFlags.ZBuffer, new ColorBGRA(0, 0, 0, 1), 1.0f, 0);
            device.BeginScene();
            device.VertexDeclaration = faceDeclaration;
            device.Material = matFace;
            device.SetStreamSource(0, faceVertices, 0, sizeof(float3) * 2);
            device.DrawPrimitives(PrimitiveType.TriangleList, 0, Total * FacePointNum);
            device.VertexDeclaration = edgeDeclaration;
            device.Material = matBoundary;
            device.SetStreamSource(0, boundaryVertices, 0, sizeof(float3));
            device.DrawPrimitives(PrimitiveType.LineList, 0, EdgePointNum);
            device.EndScene();
            device.Present();
        }
        private static unsafe void KeyPressed(object sender, KeyEventArgs e)
        {
            switch (e.KeyCode)
            {
                case Keys.Space:
                    if (clock.IsRunning)
                        clock.Stop();
                    else
                        clock.Start();
                    break;
                case Keys.R:
                case Keys.F5:
                    Setup();
                    break;
                case Keys.Right:
                    if (clock.IsRunning)
                        break;
                    Update();
                    Draw();
                    Render();
                    break;
            }
        }
    }
}