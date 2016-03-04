#define rtAGV_TEST_0

#define rtAGV_TEST_1


using System;

using rtAGV_Common;
using PLC_Control;
using rtAGV_Navigate;



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

        /** \brief Region Cfg 區域範圍設定*/
        public ROI[] atRegionCfg;

        /** \brief Map 地圖設定 */
        public rtAGV_MAP tMapCfg;

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

        /** \brief 初始化用 建構函式 */
        public rtAGV_Control()
        {
            // 載入設定擋做初始化，設定擋名稱為至先hard code
            Reset(this);
        }

        public void ExecuteCmd(ulong a_ullAGV_Cmd)
        {   // 有新命令且被接受才會 call
            uint ulAction = 0;

            ulAction = (uint)((ullAGV_Cmd >> CMD) & MASK);
            switch (ulAction)
            {
                // 運送貨物
                case (uint)rtAGVCmd.CMD_DELIVER:
                    ullAGV_Cmd = a_ullAGV_Cmd;
                    tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.MOVE_TO_SRC;
                    Deliver();
                    break;
                // 取貨物
                case (uint)rtAGVCmd.CMD_LOAD:
                    tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.MOVE_TO_SRC;
                    ullAGV_Cmd = a_ullAGV_Cmd;
                    LoadGoods();
                    break;
                // 放貨物
                case (uint)rtAGVCmd.CMD_UNLOAD:
                    tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.MOVE_TO_DEST;
                    ullAGV_Cmd = a_ullAGV_Cmd;
                    UnLoadGoods();
                    break;
                // 停止
                case (uint)rtAGVCmd.CMD_STOP:
                    tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.STOP;
                    ullAGV_Cmd = a_ullAGV_Cmd;
                    EmergencyStop();
                    break;
                // 從剛剛停止的地方繼續執行命令
                case (uint)rtAGVCmd.CMD_CONTINUE:
                    Continue();
                    break;
                // 暫停: 暫時停止，資料&動作先暫留
                case (uint)rtAGVCmd.CMD_PAUSE:
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

        public static void rtAGV_Navigation(rtWarehousingInfo a_tLocatData, rtAGV_CFG a_tAGV_Cfg, ref rtAGV_Data a_tAGV_Data, ROI[] a_atObstacle)
        {
            if (a_tAGV_Data.atPathInfo.Length <= 0)
            {   // 沒路徑才計算，之後系統執行完導航都要清掉 path data避免之後動作載道上一次的路徑資料
                // path planning
                rtPathPlanning.rtAGV_PathPlanning(
                    a_tAGV_Cfg.tMapCfg, a_tAGV_Cfg.atWarehousingCfg, a_tAGV_Cfg.atRegionCfg,
                    ref a_tAGV_Data.atPathInfo, ref a_tAGV_Data.tCarInfo, a_tLocatData, a_atObstacle);
            }
        }

        public static void rtAGV_MotorCtrl(ref rtPath_Info[] a_atPathInfo, ref rtMotor_Cfg a_tMotorCtrl_Cfg, rtCarData a_tCarInfo, ref rtMotorCtrl a_CMotor)
        {

            double eErrorPower = 0;
            double eErrorAngle = 0;
            double eDeltaCarAngle = 0;
            double eTargetAngle = 0;
            int lPathIndex = 0;
            rtVector tV_S2D = new rtVector();
            bool bAlignment = false;

            // set control Cfg >>　沒設定會用預設值

            // 判斷是否要車身校正
            lPathIndex = a_CMotor.tMotorData.lPathNodeIndex;
            if (lPathIndex == 0)
            {
                tV_S2D.eX = a_atPathInfo[lPathIndex].tDest.eX - a_atPathInfo[lPathIndex].tSrc.eX;
                tV_S2D.eY = a_atPathInfo[lPathIndex].tDest.eY - a_atPathInfo[lPathIndex].tSrc.eY;
                eDeltaCarAngle = rtMotorCtrl.AngleDifferenceCal(tV_S2D, a_tCarInfo.eAngle);
            }

            if (Math.Abs(eDeltaCarAngle) > rtMotorCtrl.ANGLE_TH_NEED_ALIGNMENT)
            {   // 需要原地旋轉 >> (在第一段小路徑)
                eTargetAngle = rtVectorOP.Vector2Angle(tV_S2D);
                bAlignment = rtMotorCtrl.CarAngleAlignment(eTargetAngle, a_tCarInfo, a_CMotor);
            }
            else
            {   // 正常控制
                // decide Motor Power
                eErrorPower = rtMotorCtrl.MotorPower_Ctrl(a_atPathInfo, a_tCarInfo, ref a_CMotor);

                // decide Motor Angle
                eErrorAngle = rtMotorCtrl.MotorAngle_CtrlNavigate(a_atPathInfo, a_tCarInfo, ref a_CMotor);
            }
        }

        public static void Reset(rtAGV_Control a_tAGV)
        {
            a_tAGV.ullAGV_Cmd = 0x00;
            a_tAGV.tAGV_Cfg.LoadDefault();
            a_tAGV.tAGV_Data.Init();
        }

        public static void Continue()
        {

        }

        public static void Pause()
        {

        }


        public void MoveToAssignedPosition(NodeId a_tAssignedNode)
        {
            // 初始化 motor control Class
            tAGV_Data.CMotor = new rtMotorCtrl();

            while (tAGV_Data.CMotor.tMotorData.bFinishFlag == false && tAGV_Data.bEmergency == false)
            {
                AutoNavigate(a_tAssignedNode, tAGV_Cfg, tAGV_Data.tSensorData, ref tAGV_Data);
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

        public NodeId MoveToWareroomForGoods(bool a_bMode)
        {
            NodeId tPosition;

            // 得到倉儲位置 (櫃位)
            if (a_bMode)
            {   // for load
                tPosition.lRegion = (int)((ullAGV_Cmd >> SRC_REGION) & MASK);
                tPosition.lIndex = (int)((ullAGV_Cmd >> SRC_POSITION) & MASK);
            }
            else
            {   // for unload
                tPosition.lRegion = (int)((ullAGV_Cmd >> DEST_REGION) & MASK);
                tPosition.lIndex = (int)((ullAGV_Cmd >> DEST_POSITION) & MASK);
            }

#if rtAGV_TEST_0    // hard code 設定路徑

#endif
            // 自動導航到該櫃位
            MoveToAssignedPosition(tPosition);

            // 清空路徑資料避免被誤用
            tAGV_Data.atPathInfo = new rtPath_Info[0];

            return tPosition;
        }

        public void LoadGoods()
        {
            bool bBreak = false;
            NodeId tPosition = new NodeId();

            while (bBreak == false)
            {
                switch (tAGV_Data.ucAGV_Status)
                {
                    // 導航到取貨處
                    case (byte)rtAGVStatus.MOVE_TO_SRC:
                        tPosition = MoveToWareroomForGoods(true);
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
                        Storage(tPosition);
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
            NodeId tPosition = new NodeId();

            while (bBreak == false)
            {
                switch (tAGV_Data.ucAGV_Status)
                {
                    // 導航到卸貨處
                    case (byte)rtAGVStatus.MOVE_TO_DEST:
                        tPosition = MoveToWareroomForGoods(false);
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
                        Storage(tPosition);
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
            NodeId tPosition = new NodeId();

            while (bBreak == false)
            {
                switch (tAGV_Data.ucAGV_Status)
                {
                    // 導航到取貨處
                    case (byte)rtAGVStatus.MOVE_TO_SRC:
                        tPosition = MoveToWareroomForGoods(true);
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
                        Storage(tPosition);
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
                        tPosition = MoveToWareroomForGoods(false);
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
                        Storage(tPosition);
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


        public static bool ObstacleAvoidance(rtCarCFG a_tCarCfg, rtAGV_SensorData a_tSensorData, ref rtCarData a_tCarInfo, ROI[] a_atObstacle)
        {
            bool bEmergencyFlag = false;         

            return bEmergencyFlag;
        }
        

        public static void AutoNavigate(NodeId a_tDestination, rtAGV_CFG a_tAGV_Cfg, rtAGV_SensorData a_tSensorData, ref rtAGV_Data a_tAGV_Data)
        {
            rtWarehousingInfo LocatData;    // 目的地
            ROI[] atObstacle = new ROI[0];

            // 從cfg中找出 目的地在哪個櫃位，不論是取貨點還是放貨點
            LocatData = a_tAGV_Cfg.atWarehousingCfg[a_tDestination.lRegion][a_tDestination.lIndex];

            // Obstacle Avoidance 檢查當下路徑或方向有沒有障礙物的威脅，並且回傳障礙物資訊和緊急訊號
            a_tAGV_Data.bEmergency = ObstacleAvoidance(a_tAGV_Cfg.tCarCfg, a_tSensorData, ref a_tAGV_Data.tCarInfo, atObstacle);

            if (a_tAGV_Data.bEmergency == false)
            {
                // 檢測或算出路徑
                rtAGV_Navigation(LocatData, a_tAGV_Cfg, ref a_tAGV_Data, atObstacle);

                // 控制馬達
                rtAGV_MotorCtrl(ref a_tAGV_Data.atPathInfo, ref a_tAGV_Cfg.tMotorCtrlCfg, a_tAGV_Data.tCarInfo, ref a_tAGV_Data.CMotor);                
            }
        }

        bool ForkActionFinishCheck()
        {
            double eDiffHeight = 0, eDiffDepth = 0;
            eDiffHeight = Math.Abs(tAGV_Data.CFork.tForkData.height - tAGV_Data.tSensorData.tForkInputData.height);
            eDiffDepth = Math.Abs(tAGV_Data.CFork.tForkData.distanceDepth - tAGV_Data.tSensorData.tForkInputData.distanceDepth);

            if (eDiffHeight < rtForkCtrl.FORK_MATCH_TH && eDiffDepth < rtForkCtrl.FORK_MATCH_TH)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        public void Storage(NodeId a_tStoragePos)
        {
            bool bDone = false;
            rtPath_Info[] atPathInfo = new rtPath_Info[1];

            // 初始化 motor & fork control Class
            tAGV_Data.CFork = new rtForkCtrl();
            tAGV_Data.CMotor = new rtMotorCtrl();

            while (tAGV_Data.CFork.tForkData.ucStatus != (byte)rtForkCtrl.ForkStatus.FINISH && tAGV_Data.ucAGV_Status != (byte)rtAGVStatus.EMERGENCY_STOP)
            {
                switch (tAGV_Data.CFork.tForkData.ucStatus)
                {
                    // 初始狀態
                    case (byte)rtForkCtrl.ForkStatus.NULL:
                        tAGV_Data.CFork.tForkData.bEnable = false;
                        tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.ALIMENT;
                        break;

                    // ALIMENT
                    case (byte)rtForkCtrl.ForkStatus.ALIMENT:
                        bDone = false;
                        bDone = rtMotorCtrl.CarAngleAlignment(
                            tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].eDirection,
                            tAGV_Data.tCarInfo, tAGV_Data.CMotor);

                        tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].eHeight;
                        tAGV_Data.CFork.tForkData.distanceDepth = 0;

                        // UNLOAD要高一點
                        tAGV_Data.CFork.tForkData.height += (tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.UNLOAD) ? rtForkCtrl.FORK_PICKUP_HEIGHT : 0;

                        tAGV_Data.CFork.tForkData.bEnable = true;

                        if (ForkActionFinishCheck() && bDone)
                        {   // 達到要的深度跟高度 >> 進入FORTH動作
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.FORTH;
                            atPathInfo[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            atPathInfo[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            atPathInfo[0].tSrc.eX = tAGV_Data.tSensorData.tPosition.eX;
                            atPathInfo[0].tSrc.eY = tAGV_Data.tSensorData.tPosition.eY;
                            atPathInfo[0].tDest.eX = tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].tCoordinate.eX;
                            atPathInfo[0].tDest.eY = tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].tCoordinate.eY;
                        }
                        break;
                    // SET_HEIGHT
                    case (byte)rtForkCtrl.ForkStatus.SET_HEIGHT:
                        tAGV_Data.CFork.tForkData.bEnable = true;
                        tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].eHeight;
                        tAGV_Data.CFork.tForkData.distanceDepth = rtForkCtrl.FORK_MAX_DEPTH;
                        
                        // UNLOAD要高一點
                        tAGV_Data.CFork.tForkData.height += (tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.UNLOAD) ? rtForkCtrl.FORK_PICKUP_HEIGHT : 0;
                        
                        if (ForkActionFinishCheck())
                        {   // 達到要的深度跟高度 >> 進入FORTH動作
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.FORTH;
                            atPathInfo[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            atPathInfo[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            atPathInfo[0].tSrc.eX = tAGV_Data.tSensorData.tPosition.eX;
                            atPathInfo[0].tSrc.eY = tAGV_Data.tSensorData.tPosition.eY;
                            atPathInfo[0].tDest.eX = tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].tCoordinate.eX;
                            atPathInfo[0].tDest.eY = tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].tCoordinate.eY;
                        }
                        break;
                    // FORTH
                    case (byte)rtForkCtrl.ForkStatus.FORTH:
                        rtAGV_MotorCtrl(ref atPathInfo, ref tAGV_Cfg.tMotorCtrlCfg, tAGV_Data.tCarInfo, ref tAGV_Data.CMotor);

                        tAGV_Data.CFork.tForkData.bEnable = false;

                        if (tAGV_Data.CMotor.tMotorData.bFinishFlag == true)
                        {
                            // reset
                            tAGV_Data.CMotor = new rtMotorCtrl();

                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.SET_DEPTH;
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
                        tAGV_Data.CFork.tForkData.bEnable = true;
                        tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].eHeight + rtForkCtrl.FORK_PICKUP_HEIGHT;
                        tAGV_Data.CFork.tForkData.distanceDepth = rtForkCtrl.FORK_MAX_DEPTH;

                        if (ForkActionFinishCheck())
                        {   // 起點終點交換 & 進入 BACKWARD
                            atPathInfo[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            atPathInfo[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            Swap(ref atPathInfo[0].tSrc, ref atPathInfo[0].tDest);
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.BACKWARD;
                        }
                        break;
                    // PICKDOWN
                    case (byte)rtForkCtrl.ForkStatus.PICKDOWN:
                        tAGV_Data.CFork.tForkData.bEnable = true;
                        tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].eHeight;
                        tAGV_Data.CFork.tForkData.distanceDepth = rtForkCtrl.FORK_MAX_DEPTH;

                        if (ForkActionFinishCheck())
                        {   // 起點終點交換 & 進入 BACKWARD
                            atPathInfo[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            atPathInfo[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            Swap(ref atPathInfo[0].tSrc, ref atPathInfo[0].tDest);
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.BACKWARD;
                        }
                        break;
                    // BACKWARD
                    case (byte)rtForkCtrl.ForkStatus.BACKWARD:
                        tAGV_Data.CFork.tForkData.bEnable = false;
                        rtAGV_MotorCtrl(ref atPathInfo, ref tAGV_Cfg.tMotorCtrlCfg, tAGV_Data.tCarInfo, ref tAGV_Data.CMotor);

                        if (tAGV_Data.CMotor.tMotorData.bFinishFlag == true)
                        {
                            // reset
                            tAGV_Data.CMotor = new rtMotorCtrl();

                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.RESET_HEIGHT;
                        }
                        break;
                    // RESET_HEIGHT
                    case (byte)rtForkCtrl.ForkStatus.RESET_HEIGHT:
                        tAGV_Data.CFork.tForkData.bEnable = true;
                        tAGV_Data.CFork.tForkData.height = 0;
                        tAGV_Data.CFork.tForkData.distanceDepth = 0;

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
        public void ReceiveData()
        {

        }

        public void SentData(rtAGV_CFG a_tAGV_Cfg, byte a_ucAGV_Status)
        {

        }
    }
}