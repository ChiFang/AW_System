#define rtAGV_TEST_0

#define rtAGV_TEST_1

#define rtAGV_DEBUG_PRINT

#define HT_Lee_DEBUG

#define ThreadDelay

using System;

using rtAGV_Common;
using PLC_Control;
using rtAGV_Navigate;
using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.IO;
using System.Text;


namespace rtAGV_Sys
{
    public struct rtCarCFG
    {
        /** \brief Define: 兩輪中心到後輪距離 */
        public const int CAR_LENGTH = 1500;

        /** \brief Define: 兩輪距離 */
        public const int CAR_WIDTH_WHEEL = 1140;

        /** \brief 兩輪中心到後輪距離 */
        public int lLength;

        /** \brief 兩輪距離 */
        public int lWidth;

        public void LoadDefault()
        {
            lLength = CAR_LENGTH;
            lWidth = CAR_WIDTH_WHEEL;
        }
    }

    public struct rtAGV_CFG
    {
        /** \brief Goods Infomation of each one >> 貨架設定*/
        public rtWarehousingInfo[][] atWarehousingCfg;

        /** \brief Map 地圖設定 */
        public rtAGV_MAP tMapCfg;

        /** \brief Region Cfg 區域範圍設定*/
        public ROI[] atRegionCfg;

        /** \brief Spec of AGV car 車身規格設定 */
        public rtCarCFG tCarCfg;

        /** \brief 車輪馬達控制參數設定設定 */
        public rtMotor_Cfg tMotorCtrlCfg;

        public void LoadDefault()
        {
            atWarehousingCfg = new rtWarehousingInfo[0][];
            atRegionCfg = new ROI[0];

            tMapCfg.Init();
            tCarCfg.LoadDefault();
            tMotorCtrlCfg.LoadDefault();
        }
    }

    public struct rtAGV_SensorData
    {
        /** \brief 光達資料 */
        public double[] aeLiDarData;

        /** \brief 定位座標 */
        public rtVector tPosition;

        /** \brief 定位方向 */
        public double eDirection;

        /** \brief Left Wheel Speed */
        public double eLeftWheelSpeed;

        /** \brief Right Wheel Speed */
        public double eRightWheelSpeed;

        /** \brief Fork Input Data */
        public rtForkCtrl_Data tForkInputData;
    }

    public struct rtAGV_Data
    {
        /** \brief 有障礙物狀況時需要的flag */
        public bool bEmergency;

        /** \brief 導航算出來的路徑 */
        public rtPath_Info[] atPathInfo;

        /** \brief Motor data 馬達相關數值 */
        public rtMotorCtrl CMotor;

        /** \brief Fork data 貨叉相關數值 */
        public rtForkCtrl CFork;

        /** \brief 當下車子資訊 */
        public rtCarData tCarInfo;

        /** \brief Output Data: AGV Status */
        public byte ucAGV_Status;

        /** \brief InOutput Data: Sensor Data */
        public rtAGV_SensorData tSensorData;

        public void Init()
        {
            bEmergency = false;
            atPathInfo = new rtPath_Info[0];
            CMotor = new rtMotorCtrl();
            CFork = new rtForkCtrl();
            tCarInfo.Init();
            ucAGV_Status = (byte)rtAGV_Control.rtAGVStatus.STANDBY;
        }
    }

    public class rtSensorReader
    {

        public rtSensorReader()
        {

        }

        public static void rtReadSensorData(rtAGV_Control a_tAGV_Control)
        {

        }
    }

    public class rtAGV_Control
    {
        public enum rtAGVCmd { CMD_STOP = 0x00, CMD_DELIVER = 0x01, CMD_CONTINUE = 0x02, CMD_PAUSE = 0x03, CMD_RESET = 0x04, CMD_LOAD = 0x05, CMD_UNLOAD = 0x06 };

        public enum rtAGVStatus { STANDBY, PAUSE, STOP, EMERGENCY_STOP, MOVE_TO_SRC, LOAD, MOVE_TO_DEST, UNLOAD, MOVE_TO_PARK, ERROR_NO_CFG };

        /** \brief Define: CMD shift bits */
        public const ushort CMD = 56;

        /** \brief Define: SRC_REGION shift bits */
        public const ushort SRC_REGION = 48;

        /** \brief Define: SRC_POSITION shift bits */
        public const ushort SRC_POSITION = 40;

        /** \brief Define: DEST_REGION shift bits */
        public const ushort DEST_REGION = 32;

        /** \brief Define: DEST_POSITION shift bits */
        public const ushort DEST_POSITION = 24;

        /** \brief Define: mask of shift bits */
        public const byte MASK = 0xFF;

        /** \brief Configure: AGV Configure */
        public rtAGV_CFG tAGV_Cfg;

        /** \brief Input Data: AGV Command */
        public ulong ullAGV_Cmd = 0;

        /** \brief InOutput Data: AGV data */
        public rtAGV_Data tAGV_Data;

        /** \brief Output Data: AGV Status Buffer for Pause */
        byte ucAGV_StatusBuf;

        public rtPath_Info[] atPathInfoForkForth;

        public bool bCheckWheelAngle;

        /** \brief 初始化用 建構函式 */
        public rtAGV_Control()
        {
            // 載入設定擋做初始化，設定擋名稱為至先hard code
            Reset(this);
        }

        public void ExecuteCmd(ulong a_ullAGV_Cmd)
        {   // 有新命令且被接受才會 call
            uint ulAction = 0;

            ulAction = (uint)((a_ullAGV_Cmd >> CMD) & MASK);
            switch (ulAction)
            {
                // 運送貨物
                case (uint)rtAGVCmd.CMD_DELIVER:
                    ullAGV_Cmd = a_ullAGV_Cmd;
                    tAGV_Data.ucAGV_Status = (tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.PAUSE) ? ucAGV_StatusBuf : (byte)rtAGVStatus.MOVE_TO_SRC;
                    Deliver();
                    break;
                // 取貨物
                case (uint)rtAGVCmd.CMD_LOAD:
                    ullAGV_Cmd = a_ullAGV_Cmd;
                    tAGV_Data.ucAGV_Status = (tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.PAUSE) ? ucAGV_StatusBuf : (byte)rtAGVStatus.MOVE_TO_SRC;
                    LoadGoods();
                    break;
                // 放貨物
                case (uint)rtAGVCmd.CMD_UNLOAD:
                    ullAGV_Cmd = a_ullAGV_Cmd;
                    tAGV_Data.ucAGV_Status = (tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.PAUSE) ? ucAGV_StatusBuf : (byte)rtAGVStatus.MOVE_TO_DEST;
                    UnLoadGoods();
                    break;
                // 停止
                case (uint)rtAGVCmd.CMD_STOP:
                    ullAGV_Cmd = a_ullAGV_Cmd;
                    tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.STOP;
                    EmergencyStop();
                    break;
                // 從剛剛停止的地方繼續執行命令
                case (uint)rtAGVCmd.CMD_CONTINUE:
                    Continue();
                    break;
                // 暫停: 暫時停止，資料&動作先暫留
                case (uint)rtAGVCmd.CMD_PAUSE:
                    ucAGV_StatusBuf = tAGV_Data.ucAGV_Status;
                    tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.PAUSE;
                    Pause();
                    break;
                // 重新初始化
                case (uint)rtAGVCmd.CMD_RESET:
                    ullAGV_Cmd = a_ullAGV_Cmd;
                    tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.STANDBY;
                    Reset(this);
                    break;
                default:
                    // show error
                    break;
            }
        }

        public void rtAGV_Navigation(rtWarehousingInfo a_tLocatData, ROI[] a_atObstacle)
        {
#if HT_Lee_DEBUG
            if (tAGV_Data.atPathInfo.Length <= 0 || a_atObstacle.Length != 0)
            {
#endif
                // 沒路徑才計算，之後系統執行完導航都要清掉 path data避免之後動作載道上一次的路徑資料
                // path planning
                Console.WriteLine("NowPosition: " + tAGV_Data.tCarInfo.tPosition.eX + "," + tAGV_Data.tCarInfo.tPosition.eY);
                rtPathPlanning.rtAGV_PathPlanning(
                    tAGV_Cfg.tMapCfg, tAGV_Cfg.atWarehousingCfg, tAGV_Cfg.atRegionCfg,
                    ref tAGV_Data.atPathInfo, ref tAGV_Data.tCarInfo, a_tLocatData, a_atObstacle);
                PathModifyForStorage(ref tAGV_Data.atPathInfo, a_tLocatData.eDirection);
 #if HT_Lee_DEBUG
            }
#endif

#if rtAGV_DEBUG_PRINT
                for (int i = 0; i < tAGV_Data.atPathInfo.Length; i++)
                {
                    Console.WriteLine(i+"::" + tAGV_Data.atPathInfo[i].tSrc.eX + "," + tAGV_Data.atPathInfo[i].tSrc.eY + "-->" + tAGV_Data.atPathInfo[i].tDest.eX + "," + tAGV_Data.atPathInfo[i].tDest.eY + "--ucTurnType:" + tAGV_Data.atPathInfo[i].ucTurnType);
                   // Console.WriteLine("1::" + tAGV_Data.atPathInfo[1].tSrc.eX + "," + tAGV_Data.atPathInfo[1].tSrc.eY + "-->" + tAGV_Data.atPathInfo[1].tDest.eX + "," + tAGV_Data.atPathInfo[1].tDest.eY + "--ucTurnType:" + tAGV_Data.atPathInfo[1].ucTurnType);
                }
#endif
        }

        public void rtAGV_MotorCtrl(ref rtPath_Info[] a_atPathInfo, double a_eDirection, bool bBackwardEnable)
        {   // 一定要傳path 因為 path 不一定是agv裡面送貨用的 有可能是 取放貨時前進後退用的

            double eErrorPower = 0;
            double eTargetAngle = 0;
            int lPathIndex = 0;
            rtVector tV_S2D = new rtVector();
            bool bAlignment = false;

            // set control Cfg >>　沒設定會用預設值

            if(bBackwardEnable)
            {   // 不用對正路徑
                tAGV_Data.CMotor.tMotorData.bPathAngleMatch = true;
            }
            else
            {   // 須要對正路徑
                if (tAGV_Data.CMotor.tMotorData.bPathAngleMatch == false)
                {   // 
                    lPathIndex = tAGV_Data.CMotor.tMotorData.lPathNodeIndex;
                    tV_S2D.eX = a_atPathInfo[lPathIndex].tDest.eX - a_atPathInfo[lPathIndex].tSrc.eX;
                    tV_S2D.eY = a_atPathInfo[lPathIndex].tDest.eY - a_atPathInfo[lPathIndex].tSrc.eY;
                    eTargetAngle = rtVectorOP_2D.Vector2Angle(tV_S2D);
                    bAlignment = tAGV_Data.CMotor.CarAngleAlignment(eTargetAngle, tAGV_Data.tCarInfo);

                    if(bAlignment)
                    {   // 已對準 開始執行路徑 並且把對準旗標拉起來
                        tAGV_Data.CMotor.tMotorData.bPathAngleMatch = true;
                    }
                    else
                    {   // 沒對準 不做動作就離開
                        return;
                    }
                }
            }
            
            // 正常控制
            // decide Motor Power
            eErrorPower = tAGV_Data.CMotor.MotorPower_CtrlNavigate(a_atPathInfo, tAGV_Data.tCarInfo);

            // decide Path offset
            // tAGV_Data.CMotor.PathOffsetCal(a_atPathInfo, a_eDirection);

            // decide Motor Angle
            tAGV_Data.CMotor.MotorAngle_CtrlNavigate(a_atPathInfo, tAGV_Data.tCarInfo);
        }

        //static void ExtendPointAlongVector(ref rtVector a_tPoint, rtVector a_tDirection, int a_lExtendSize)
        //{
        //    double eT = 0, eSizeVetor = 0;

        //    eSizeVetor = rtVectorOP_2D.GetLength(a_tDirection);
        //    eT = a_lExtendSize / eSizeVetor;
        //    a_tPoint.eX = a_tPoint.eX + a_tDirection.eX * eT;
        //    a_tPoint.eY = a_tPoint.eY + a_tDirection.eY * eT;
        //}

        static void ExtendPathSize(ref rtPath_Info a_tPathInfo, int a_lExtendSize)
        {
            rtVector tDirection = new rtVector();
            tDirection = rtVectorOP_2D.GetVector(a_tPathInfo.tSrc, a_tPathInfo.tDest);
            a_tPathInfo.tDest = rtVectorOP_2D.ExtendPointAlongVector(a_tPathInfo.tDest, tDirection, a_lExtendSize);
        }

        public static void PathModifyForStorage(ref rtPath_Info[] a_atPathInfo, double a_eDestDirection)
        {
            int lCnt = 0, lFinalPathIndex = 0, lCntFix = 0, lLastPathIndex;
            double eCross = 0, eDeltaTmp = 0, eDeltaTmpLast = 0;
            rtVector tDestVector = new rtVector();
            rtVector tPathVectorFinal = new rtVector();
            rtVector tPathVectorLast = new rtVector();
            rtVector tVlaw = new rtVector();

            for (lCnt = 0; lCnt < a_atPathInfo.Length; lCnt++)
            {
                if (rtMotorCtrl.Link2DestCheck(a_atPathInfo, lCnt))
                {   // current path link to goods
                    lFinalPathIndex = a_atPathInfo.Length - 1;
                    tPathVectorFinal = rtVectorOP_2D.GetVector(a_atPathInfo[lFinalPathIndex].tSrc, a_atPathInfo[lFinalPathIndex].tDest);
                    tDestVector = rtVectorOP_2D.Angle2Vector(a_eDestDirection);
                    eDeltaTmp = rtVectorOP_2D.GetTheta(tPathVectorFinal, tDestVector);
                    if (eDeltaTmp > rtMotorCtrl.DELTA_ANGLE_TH)
                    {   // need path offset

                        // extend path before turn
                        lLastPathIndex = lCnt - 1;
                        if (lLastPathIndex >= 0)
                        {   // 如果有前一段就延長
                            tPathVectorLast = rtVectorOP_2D.GetVector(a_atPathInfo[lLastPathIndex].tSrc, a_atPathInfo[lLastPathIndex].tDest);
                            eDeltaTmpLast = rtVectorOP_2D.GetTheta(tPathVectorLast, tDestVector);
                            if (eDeltaTmpLast > rtMotorCtrl.DELTA_ANGLE_TH)
                            {   // 縮短
                                ExtendPathSize(ref a_atPathInfo[lLastPathIndex], -(rtMotorCtrl.DEFAULT_PATH_OFFSET));
                            }
                            else
                            {   // 延長
                                ExtendPathSize(ref a_atPathInfo[lLastPathIndex], rtMotorCtrl.DEFAULT_PATH_OFFSET);
                            }
                        }

                        eCross = rtVectorOP_2D.Cross(tPathVectorFinal, tDestVector);
                        if(eCross < 0)
                        {   // 物品在路徑向量右側 >> 取右側的法向量
                            tVlaw.eX = tPathVectorFinal.eY;
                            tVlaw.eY = -tPathVectorFinal.eX;
                        }
                        else
                        {   // 物品在路徑向量左側 >> 取左側的法向量
                            tVlaw.eX = -tPathVectorFinal.eY;
                            tVlaw.eY = tPathVectorFinal.eX;
                        }

                        // modify all path linked to dest of goods
                        for (lCntFix = lCnt; lCntFix < a_atPathInfo.Length; lCntFix++)
                        {
                            a_atPathInfo[lCntFix].tSrc = rtVectorOP_2D.ExtendPointAlongVector(a_atPathInfo[lCntFix].tSrc, tVlaw, rtMotorCtrl.DEFAULT_PATH_OFFSET);
                            a_atPathInfo[lCntFix].tDest = rtVectorOP_2D.ExtendPointAlongVector(a_atPathInfo[lCntFix].tDest, tVlaw, rtMotorCtrl.DEFAULT_PATH_OFFSET);
                        }
                        break;

                    }
                }
            }
        }

        public static void Reset(rtAGV_Control a_tAGV)
        {
            a_tAGV.ullAGV_Cmd = 0x00;
            a_tAGV.tAGV_Cfg.LoadDefault();
            a_tAGV.tAGV_Data.Init();
        }

        public void Continue()
        {
            ExecuteCmd(ullAGV_Cmd);
        }

        public void Pause()
        {
            // 停止車上任何會導致車子動作的訊號
            tAGV_Data.CFork.tForkData.bEnable = false;
            tAGV_Data.CMotor.tMotorData.lMotorPower = 0;
            tAGV_Data.CMotor.tMotorData.lMotorTorsion = 0;
        }


        public void MoveToAssignedPosition(WarehousPos a_tWarehousPos)
        {
            // 初始化 motor control Class
            tAGV_Data.CMotor = new rtMotorCtrl();

            while (tAGV_Data.CMotor.tMotorData.bFinishFlag == false && tAGV_Data.bEmergency == false)
            {
                AutoNavigate(a_tWarehousPos);
#if ThreadDelay
                Thread.Sleep(15);
#endif
            }
        }

#if rtAGV_TEST_0    // hard code 設定路徑
    public static void Test_0(ref rtAGV_Control a_CAGV_Sys)
    {
        // set cmd
        a_CAGV_Sys.ullAGV_Cmd = 0;

        // set cfg
        a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg = new rtWarehousingInfo[1][];  //  1個區域
        a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0] = new rtWarehousingInfo[2]; //  2個櫃位

        // 設定櫃位連到哪個節點
        a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][0].tNodeId.lRegion = 0;
        a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][0].tNodeId.lIndex = 0;
        a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][1].tNodeId.lRegion = 0;
        a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][1].tNodeId.lIndex = 1;

        a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeGlobal = new rtAGV_MAP_node[1];  //  1個區域
        a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal = new rtAGV_MAP_node[1][];  //  1個區域
        a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal[0] = new rtAGV_MAP_node[2];  //  2個節點

        // 設定結點 & 櫃位位置
        a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal[0][0].tCoordinate.eX = 0;
        a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal[0][0].tCoordinate.eY = 0;
        a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal[0][1].tCoordinate.eX = 0;
        a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal[0][1].tCoordinate.eY = 0;

        a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][0].tCoordinate.eX = 0;
        a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][0].tCoordinate.eY = 0;
        a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][1].tCoordinate.eX = 0;
        a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][1].tCoordinate.eY = 0;

        a_CAGV_Sys.Deliver();
    }
#endif

#if rtAGV_TEST_1    // hard code 設定路徑
        public static void Test_1(ref rtAGV_Control a_CAGV_Sys)
        {
            // set cmd
            a_CAGV_Sys.ullAGV_Cmd = 0;

            // set cfg
            a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg = new rtWarehousingInfo[1][];  //  1個區域
            a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0] = new rtWarehousingInfo[2]; //  2個櫃位

            // 設定櫃位連到哪個節點
            a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][0].tNodeId.lRegion = 0;
            a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][0].tNodeId.lIndex = 0;
            a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][1].tNodeId.lRegion = 0;
            a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][1].tNodeId.lIndex = 1;

            a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeGlobal = new rtAGV_MAP_node[1];  //  1個區域
            a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal = new rtAGV_MAP_node[1][];  //  1個區域
            a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal[0] = new rtAGV_MAP_node[2];  //  2個節點

            // 設定結點 & 櫃位位置
            a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal[0][0].tCoordinate.eX = 0;
            a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal[0][0].tCoordinate.eY = 0;
            a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal[0][1].tCoordinate.eX = 0;
            a_CAGV_Sys.tAGV_Cfg.tMapCfg.atNodeLocal[0][1].tCoordinate.eY = 0;

            a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][0].tCoordinate.eX = 0;
            a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][0].tCoordinate.eY = 0;
            a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][1].tCoordinate.eX = 0;
            a_CAGV_Sys.tAGV_Cfg.atWarehousingCfg[0][1].tCoordinate.eY = 0;

            // hard code 設定路徑

            // hard code 設定路徑

            a_CAGV_Sys.MoveToWareroomForGoods(true);
            
            if (a_CAGV_Sys.tAGV_Data.bEmergency == true)
            {
                a_CAGV_Sys.tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                // 初始化 motor & fork control Class
                a_CAGV_Sys.tAGV_Data.CFork = new rtForkCtrl();
                a_CAGV_Sys.tAGV_Data.CMotor = new rtMotorCtrl();
                return;
            }
        }
#endif

        static void Swap<T>(ref T a_tVar_1, ref T a_tVar_2)
        {
            T tVarTmp = a_tVar_1;
            a_tVar_1 = a_tVar_2;
            a_tVar_2 = tVarTmp;
        }

        public WarehousPos MoveToWareroomForGoods(bool a_bMode)
        {
            WarehousPos tWarehousPos = new WarehousPos();

            // 得到倉儲位置 (櫃位)
            if (a_bMode)
            {   // for load
                tWarehousPos.lRegion = (int)((ullAGV_Cmd >> SRC_REGION) & MASK);
                tWarehousPos.lIndex = (int)((ullAGV_Cmd >> SRC_POSITION) & MASK);
            }
            else
            {   // for unload
                tWarehousPos.lRegion = (int)((ullAGV_Cmd >> DEST_REGION) & MASK);
                tWarehousPos.lIndex = (int)((ullAGV_Cmd >> DEST_POSITION) & MASK);
            }

#if rtAGV_TEST_0    // hard code 設定路徑

#endif

            tWarehousPos.eDirection = tAGV_Cfg.atWarehousingCfg[tWarehousPos.lRegion][tWarehousPos.lIndex].eDirection;

            // 自動導航到該櫃位
            MoveToAssignedPosition(tWarehousPos);

            // 清空路徑資料避免被誤用
            tAGV_Data.atPathInfo = new rtPath_Info[0];

            return tWarehousPos;
        }

        public void LoadGoods()
        {
            bool bBreak = false;
            // NodeId tPosition = new NodeId();
            WarehousPos tWarehousPos = new WarehousPos();

            while (bBreak == false)
            {
                switch (tAGV_Data.ucAGV_Status)
                {
                    // 導航到取貨處
                    case (byte)rtAGVStatus.MOVE_TO_SRC:
                        tWarehousPos = MoveToWareroomForGoods(true);
                        if (tAGV_Data.bEmergency == false)
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.LOAD;
                        }
                        else
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                        }
                        break;

                    // 取貨
                    case (byte)rtAGVStatus.LOAD:
                        Storage(tWarehousPos);
                        if (tAGV_Data.bEmergency == false)
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.STANDBY;
                        }
                        else
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                        }
                        break;

                    default:
                        EmergencyStop();
                        bBreak = true;
                        break;
                }
            }
        }

        public void UnLoadGoods()
        {
            bool bBreak = false;
            // NodeId tPosition = new NodeId();
            WarehousPos tWarehousPos = new WarehousPos();

            while (bBreak == false)
            {
                switch (tAGV_Data.ucAGV_Status)
                {
                    // 導航到卸貨處
                    case (byte)rtAGVStatus.MOVE_TO_DEST:
                        tWarehousPos = MoveToWareroomForGoods(false);
                        if (tAGV_Data.bEmergency == false)
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.UNLOAD;
                        }
                        else
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                        }
                        break;

                    // 卸貨
                    case (byte)rtAGVStatus.UNLOAD:
                        Storage(tWarehousPos);
                        if (tAGV_Data.bEmergency == false)
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.STANDBY;
                        }
                        else
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                        }
                        break;

                    default:
                        EmergencyStop();
                        bBreak = true;
                        break;
                }
            }
        }

        public void Deliver()
        {
            bool bBreak = false;
            // NodeId tPosition = new NodeId();
            WarehousPos tWarehousPos = new WarehousPos();

            while (bBreak == false)
            {
                switch (tAGV_Data.ucAGV_Status)
                {
                    // 導航到取貨處
                    case (byte)rtAGVStatus.MOVE_TO_SRC:
                        tWarehousPos = MoveToWareroomForGoods(true);
                        if (tAGV_Data.bEmergency == false)
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.LOAD;
                        }
                        else
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                        }
                        break;

                    // 取貨
                    case (byte)rtAGVStatus.LOAD:
                        Storage(tWarehousPos);
                        if (tAGV_Data.bEmergency == false)
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.MOVE_TO_DEST;
                        }
                        else
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                        }
                        break;

                    // 導航到卸貨處
                    case (byte)rtAGVStatus.MOVE_TO_DEST:
                        tWarehousPos = MoveToWareroomForGoods(false);
                        if (tAGV_Data.bEmergency == false)
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.UNLOAD;
                        }
                        else
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                        }
                        break;

                    // 卸貨
                    case (byte)rtAGVStatus.UNLOAD:
                        Storage(tWarehousPos);
                        if (tAGV_Data.bEmergency == false)
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.STANDBY;
                        }
                        else
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                        }
                        break;

                    default:
                        EmergencyStop();
                        bBreak = true;
                        break;
                }
            }
        }

        public static rtAGV_SensorData ReadSensorData()
        {
            rtAGV_SensorData tSensorData = new rtAGV_SensorData();   // 感應器資料

            return tSensorData;
        }


        public bool ObstacleAvoidance(ref ROI[] a_atObstacle)
        {
            bool bEmergencyFlag = false;         

            return bEmergencyFlag;
        }
        

        public void AutoNavigate(WarehousPos a_tWarehousPos)
        {
            rtWarehousingInfo LocatData;    // 目的地
            ROI[] atObstacle = new ROI[0];

            // 從cfg中找出 目的地在哪個櫃位，不論是取貨點還是放貨點
            LocatData = tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex];

            // Obstacle Avoidance 檢查當下路徑或方向有沒有障礙物的威脅，並且回傳障礙物資訊和緊急訊號
            tAGV_Data.bEmergency = ObstacleAvoidance(ref atObstacle);

            if (tAGV_Data.bEmergency == false)
            {
                // 檢測或算出路徑
                rtAGV_Navigation(LocatData, atObstacle);

                // 修正路徑 >> 在走道中要靠某一邊來騰出旋轉空間
                //PathModifyForStorage(ref tAGV_Data.atPathInfo, LocatData.eDirection);

                // 控制馬達
                rtAGV_MotorCtrl(ref tAGV_Data.atPathInfo, a_tWarehousPos.eDirection, false);                
            }
        }

        bool ForkActionFinishCheck()
        {
            double eDiffHeight = 0, eDiffDepth = 0;
            eDiffHeight = Math.Abs(tAGV_Data.CFork.tForkData.height - tAGV_Data.tSensorData.tForkInputData.height);
            eDiffDepth = Math.Abs(tAGV_Data.CFork.tForkData.distanceDepth - tAGV_Data.tSensorData.tForkInputData.distanceDepth);

            if (eDiffHeight < rtForkCtrl.FORK_MATCH_TH && eDiffDepth < rtForkCtrl.FORK_MATCH_TH)
            {
#if rtAGV_DEBUG_PRINT
                //Console.WriteLine("ForkActionFinishCheck-OK");
#endif
                return true;
            }
            else
            {
#if rtAGV_DEBUG_PRINT
                //Console.WriteLine("ForkData.height: " + tAGV_Data.CFork.tForkData.height + ",ForkData.distanceDepth :" + tAGV_Data.CFork.tForkData.distanceDepth+ "--" + eDiffHeight + "," + eDiffDepth);
#endif
                return false;
            }
        }

        public void Storage(WarehousPos a_tWarehousPos)
        {
            bool bDone;
           // rtPath_Info[] atPathInfo;
            while (tAGV_Data.CFork.tForkData.ucStatus != (byte)rtForkCtrl.ForkStatus.FINISH && tAGV_Data.ucAGV_Status != (byte)rtAGVStatus.EMERGENCY_STOP)
            {
                switch (tAGV_Data.CFork.tForkData.ucStatus)
                {
                    // 初始狀態
                    case (byte)rtForkCtrl.ForkStatus.NULL:
                        // 初始化 motor & fork control Class
                        tAGV_Data.CFork = new rtForkCtrl();
                        tAGV_Data.CMotor = new rtMotorCtrl();
                        atPathInfoForkForth = new rtPath_Info[1];
                        bDone = false;
                        bCheckWheelAngle = false;    
                        tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.ALIMENT;
                        break;

                    // ALIMENT
                    case (byte)rtForkCtrl.ForkStatus.ALIMENT:
                        bDone = false;
                        bDone = tAGV_Data.CMotor.CarAngleAlignment(
                            tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].eDirection,
                            tAGV_Data.tCarInfo);

                        tAGV_Data.CFork.tForkData.distanceDepth = 0;

                        
                        if(tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.UNLOAD)
                        {   // UNLOAD要高一點
                            tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].eHeight + rtForkCtrl.FORK_PICKUP_HEIGHT;
                        }
                        else if(tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.LOAD)
                        {
                            tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].eHeight;
                        }
                        else
                        {   // error
                            tAGV_Data.CFork.tForkData.height = 0;
                            tAGV_Data.CFork.tForkData.bEnable = false;
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.ERROR;
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                        }

                        tAGV_Data.CFork.tForkData.bEnable = true;

                        if (ForkActionFinishCheck() && bDone)
                        {   // 達到要的深度跟高度 >> 進入FORTH動作
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.FORTH;
                            atPathInfoForkForth[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            atPathInfoForkForth[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            atPathInfoForkForth[0].tSrc.eX = tAGV_Data.tCarInfo.tPosition.eX;
                            atPathInfoForkForth[0].tSrc.eY = tAGV_Data.tCarInfo.tPosition.eY;
                            atPathInfoForkForth[0].tDest.eX = tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].tCoordinate.eX;
                            atPathInfoForkForth[0].tDest.eY = tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].tCoordinate.eY;
                        }
                        break;
                    // SET_HEIGHT
                    case (byte)rtForkCtrl.ForkStatus.SET_HEIGHT:
                        tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].eHeight;
                        tAGV_Data.CFork.tForkData.distanceDepth = rtForkCtrl.FORK_MAX_DEPTH;
                        
                        // UNLOAD要高一點
                        tAGV_Data.CFork.tForkData.height += (tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.UNLOAD) ? rtForkCtrl.FORK_PICKUP_HEIGHT : 0;
                        
                        tAGV_Data.CFork.tForkData.bEnable = true;

                        if (ForkActionFinishCheck())
                        {   // 達到要的深度跟高度 >> 進入FORTH動作
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.FORTH;
                            atPathInfoForkForth[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            atPathInfoForkForth[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            atPathInfoForkForth[0].tSrc.eX = tAGV_Data.tCarInfo.tPosition.eX;
                            atPathInfoForkForth[0].tSrc.eY = tAGV_Data.tCarInfo.tPosition.eY;
                            atPathInfoForkForth[0].tDest.eX = tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].tCoordinate.eX;
                            atPathInfoForkForth[0].tDest.eY = tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].tCoordinate.eY;
                        }
                        break;
                    // FORTH
                    case (byte)rtForkCtrl.ForkStatus.FORTH:
                        //先判斷輪胎是否回正
                        if (!bCheckWheelAngle)
                        {
                            bCheckWheelAngle = (Math.Abs(tAGV_Data.tCarInfo.eWheelAngle) < rtMotorCtrl.ANGLE_MATCH_TH) ? true : false;
                        }
                        else
                        {
                            tAGV_Data.CFork.tForkData.bEnable = false;
#if rtAGV_DEBUG_PRINT
                            tAGV_Data.atPathInfo = atPathInfoForkForth;
                            //Console.WriteLine("Path: " + atPathInfo[0].tDest.eX + "," + atPathInfo[0].tDest.eY);
#endif
                            rtAGV_MotorCtrl(ref atPathInfoForkForth, a_tWarehousPos.eDirection, true);  // 前面已轉正過 >>所以設為true
                            
                            if (tAGV_Data.CMotor.tMotorData.bFinishFlag == true)
                            {
                                // reset
                                tAGV_Data.atPathInfo = new rtPath_Info[0];
                                tAGV_Data.CMotor = new rtMotorCtrl();
                                tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.SET_DEPTH;
                                //if (tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.LOAD)
                                //{   // LOAD
                                //    tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.PICKUP;
                                //}
                                //else
                                //{   // UNLOAD
                                //    tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.PICKDOWN;
                                //}
                            }
                        }
                        break;
                    // SET_DEPTH
                    case (byte)rtForkCtrl.ForkStatus.SET_DEPTH:
                        tAGV_Data.CFork.tForkData.bEnable = true;
                        tAGV_Data.CFork.tForkData.distanceDepth = rtForkCtrl.FORK_MAX_DEPTH;
                        if (ForkActionFinishCheck())
                        {   // 達到要的深度跟高度 >> 進入PICKUP or PICKDOWN動作
                            
                            if (tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.LOAD)
                            {   // LOAD
                                tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.PICKUP;
                            }
                            else
                            {   // UNLOAD
                                tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.PICKDOWN;
                            }
                        }
                        break;
                    // PICKUP
                    case (byte)rtForkCtrl.ForkStatus.PICKUP:
                        tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].eHeight + rtForkCtrl.FORK_PICKUP_HEIGHT;
                        tAGV_Data.CFork.tForkData.bEnable = true;
                        //Console.WriteLine("PICKUP:" + tAGV_Data.CFork.tForkData.height + " , " + a_tWarehousPos.lRegion + " , " + a_tWarehousPos.lIndex);
                        if (ForkActionFinishCheck())
                        {   // 起點終點交換 & 進入 BACKWARD
                            atPathInfoForkForth[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            atPathInfoForkForth[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            Swap(ref atPathInfoForkForth[0].tSrc, ref atPathInfoForkForth[0].tDest);
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.RESET_DEPTH;
                        }
                        break;
                    // PICKDOWN
                    case (byte)rtForkCtrl.ForkStatus.PICKDOWN:
                        tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].eHeight;
                        tAGV_Data.CFork.tForkData.bEnable = true;

                        if (ForkActionFinishCheck())
                        {   // 起點終點交換 & 進入 BACKWARD
                            atPathInfoForkForth[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            atPathInfoForkForth[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            Swap(ref atPathInfoForkForth[0].tSrc, ref atPathInfoForkForth[0].tDest);
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.RESET_DEPTH;
                        }
                        break;
                    // RESET_DEPTH
                    case (byte)rtForkCtrl.ForkStatus.RESET_DEPTH:
                        tAGV_Data.CFork.tForkData.distanceDepth = 0;
                        tAGV_Data.CFork.tForkData.bEnable = true;
                        if (ForkActionFinishCheck())
                        {
                            tAGV_Data.CMotor = new rtMotorCtrl();   // 先清空車輪的資料 確保沒事
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.BACKWARD;
                        }
                        break;
                    // BACKWARD
                    case (byte)rtForkCtrl.ForkStatus.BACKWARD:
                        tAGV_Data.CFork.tForkData.bEnable = false;
#if rtAGV_DEBUG_PRINT
                        tAGV_Data.atPathInfo = atPathInfoForkForth;
                        //Console.WriteLine(atPathInfoForkForth[0].tSrc.eX + "," + atPathInfoForkForth[0].tSrc.eY + "---->" + atPathInfoForkForth[0].tDest.eX + "," + atPathInfoForkForth[0].tDest.eY);
#endif
                        rtAGV_MotorCtrl(ref atPathInfoForkForth, a_tWarehousPos.eDirection, true);

                        if (tAGV_Data.CMotor.tMotorData.bFinishFlag == true)
                        {
                            // reset
                            tAGV_Data.atPathInfo = new rtPath_Info[0];
                            tAGV_Data.CMotor = new rtMotorCtrl();

                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.RESET_HEIGHT;
                        }
                        break;
                    // RESET_HEIGHT
                    case (byte)rtForkCtrl.ForkStatus.RESET_HEIGHT:
                        tAGV_Data.CFork.tForkData.height = 70;
                        tAGV_Data.CFork.tForkData.distanceDepth = 0;
                        tAGV_Data.CFork.tForkData.bEnable = true;
                        if (ForkActionFinishCheck())
                        {
                            tAGV_Data.CFork.tForkData.bEnable = false;
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.FINISH;
                        }
                        break;
                    default:
                        // show error
                        break;
                }
            }
        }

        public void EmergencyStop()
        {
            // 初始化 motor & fork control Class
            tAGV_Data.CFork = new rtForkCtrl();
            tAGV_Data.CMotor = new rtMotorCtrl();
            return;
        }

        public static void StandBy()
        {

        }
    }
	
	public class rtAGV_communicate
	{
        /** \brief 連線到Server的IP */
        public string ServerIP;

        /** \brief 連線到Server的Port */
        public int Port;

        /** \brief Sock連線參數 */
        public Socket sender_TCP;

        /** \brief 接收到的Data buffer */
        public byte[] Receivebytes = new byte[1024];

        /** \brief 傳送的Data buffer */
        public byte[] Sendbytes;

        /** \brief 解析到的Command */
        public ulong ReceiveCommand;

        public bool ConnectToServerFunc()
        {
            try
            {
                if (ServerIP != "")
                {
                    string SendIP = ServerIP;
                    int port = Port;
                    sender_TCP = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                    sender_TCP.Connect(SendIP, port);
                    if (sender_TCP.Connected)
                    {
                        return true;
                    }
                    else return false;
                }
                else return false;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.ToString());
                return false;
            }  
        }

        public bool CRC_CheckReceiveData(byte[] ReceiveData)
        {
            //檢查CRC
            int DataLength = ReceiveData.Length;
            if (DataLength > 8) return false;
            byte[] TempArray = new byte[DataLength - 2];
            Array.Copy(ReceiveData, 0, TempArray, 0, DataLength - 2);
            byte[] ResultCRC = BitConverter.GetBytes(ModRTU_CRC(TempArray, 6));
            if (ResultCRC[0] == ReceiveData[6] && ResultCRC[1] == ReceiveData[7]) return true;
            else return false;
        }

        UInt16 ModRTU_CRC(byte[] buf, int len)
        {
            //計算CRC
            UInt16 crc = 0xFFFF;
            for (int pos = 0; pos < len; pos++)
            {
                crc ^= (UInt16)buf[pos];          // XOR byte into least sig. byte of crc
                for (int i = 8; i != 0; i--)
                {    // Loop over each bit
                    if ((crc & 0x0001) != 0)
                    {      // If the LSB is set
                        crc >>= 1;                    // Shift right and XOR 0xA001
                        crc ^= 0xA001;
                    }
                    else                            // Else LSB is not set
                        crc >>= 1;                    // Just shift right
                }
            }
            // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
            return crc;
        }

        public bool SendData(byte[] SendDataByte)
        {
            //傳送給server資料
            if (sender_TCP.Connected && SendDataByte != null)
            {
                try
                {
                    int bytesSent = sender_TCP.Send(SendDataByte);
                    return true;
                }
                catch
                {
                    return false;
                }
            }
            else return false;
        }

        public void ReceiveData()
        {

        }

        public void SentData(rtAGV_CFG a_tAGV_Cfg, byte a_ucAGV_Status)
        {

        }
    }
}