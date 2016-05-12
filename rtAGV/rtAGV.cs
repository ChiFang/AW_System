#define rtAGV_TEST_0

#define rtAGV_TEST_1

#define rtAGV_DEBUG_PRINT

#define HT_Lee_DEBUG

#define ThreadDelay

// #define NAVIGATE_EXTEND_PATH

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

        /** \brief 叉貨的路徑 */
        public rtPath_Info[] atPathInfoForkForth;

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
            atPathInfoForkForth = new rtPath_Info[0];
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
        public enum rtAGVCmd { CMD_STOP = 0x00, CMD_DELIVER = 0x01, CMD_CONTINUE = 0x02, CMD_PAUSE = 0x03, CMD_RESET = 0x04, CMD_LOAD = 0x05, CMD_UNLOAD = 0x06, CMD_PARK = 0x07 };

        public enum rtAGVStatus { STANDBY, PAUSE, STOP, EMERGENCY_STOP, MOVE_TO_SRC, LOAD, MOVE_TO_DEST, UNLOAD, MOVE_TO_PARK, ERROR_NO_CFG, PARKING, ALIMENT};

        /** \brief Define: 取貨後後退距離 */
        public const int STORAGE_BACK_DISTANCE = 900;

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

        /** \brief Define: Aligment Safe Angle */
        public const ushort ALIGMENT_SAFE_ANGLE = 10;

        /** \brief Define: mask of shift bits */
        public const byte MASK = 0xFF;

        /** \brief Input Data: AGV Command */
        public ulong ullAGV_Cmd = 0;

        /** \brief Configure: AGV Configure */
        public rtAGV_CFG tAGV_Cfg;

        /** \brief InOutput Data: AGV data */
        public rtAGV_Data tAGV_Data;

        /** \brief Output Data: AGV Status Buffer for Pause */
        byte ucAGV_StatusBuf;

        // public rtPath_Info[] atPathInfoForkForth;

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
                // 停車
                case (uint)rtAGVCmd.CMD_PARK:
                    ullAGV_Cmd = a_ullAGV_Cmd;
                    tAGV_Data.ucAGV_Status = (tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.PAUSE) ? ucAGV_StatusBuf : (byte)rtAGVStatus.MOVE_TO_PARK;
                    Park();
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

        public int rtAGV_Navigation(rtWarehousingInfo a_tLocatData, ROI[] a_atObstacle, bool a_bPark)
        {   // 沒路徑 離終點太近無法校正等都會回傳-1 表示錯誤
            int lCheckResult = 1;
#if HT_Lee_DEBUG
            if (tAGV_Data.atPathInfo.Length <= 0 || a_atObstacle.Length != 0)
            {
#endif   
#if rtAGV_DEBUG_PRINT
                //Console.WriteLine("NowPosition: " + tAGV_Data.tCarInfo.tPosition.eX + "," + tAGV_Data.tCarInfo.tPosition.eY);
#endif
                // 沒路徑才計算，之後系統執行完導航都要清掉 path data避免之後動作載道上一次的路徑資料
                // path planning
                rtPathPlanning.rtAGV_PathPlanning(
                    tAGV_Cfg.tMapCfg, tAGV_Cfg.atWarehousingCfg, tAGV_Cfg.atRegionCfg,
                    ref tAGV_Data.atPathInfo, ref tAGV_Data.tCarInfo, a_tLocatData, a_atObstacle);

                if(a_bPark)
                {   // 如果是要停車 終點的轉彎模式要特別改
                    tAGV_Data.atPathInfo[tAGV_Data.atPathInfo.Length - 1].ucTurnType = (byte)rtMotorCtrl.rtTurnType.PARK;
                }

                if (tAGV_Data.atPathInfo.Length > 0)
                {   // 如果沒路徑就不需修正
                    // 修正路徑 >> 在走道中要靠某一邊來騰出旋轉空間
                    PathModifyForStorage(ref tAGV_Data.atPathInfo, tAGV_Data.tCarInfo, a_tLocatData.eDirection, a_bPark);
                }

                // 判斷是否已到達終點
                lCheckResult = ArriveCheck(tAGV_Data.atPathInfo, a_tLocatData);
#if HT_Lee_DEBUG
            }
#endif

#if rtAGV_DEBUG_PRINT
            for (int i = 0; i < tAGV_Data.atPathInfo.Length; i++)
                {
                    Console.WriteLine(i+"::" + tAGV_Data.atPathInfo[i].tSrc.eX + "," + tAGV_Data.atPathInfo[i].tSrc.eY + "-->" + tAGV_Data.atPathInfo[i].tDest.eX + "," + tAGV_Data.atPathInfo[i].tDest.eY + "--ucTurnType:" + tAGV_Data.atPathInfo[i].ucTurnType);
                }
#endif
            return lCheckResult;
        }

        public void rtAGV_MotorCtrl(ref rtPath_Info[] a_atPathInfo, double a_eDirection, bool a_bAligmentFree)
        {   // 一定要傳path 因為 path 不一定是agv裡面送貨用的 有可能是 取放貨時前進後退用的
            double eTargetAngle = 0, eTargetError = 0, eWheelTheta = 0;
            int lPathIndex = 0;
            rtVector tV_S2D = new rtVector();
            rtVector tV_Aligment = new rtVector();
            bool bAlignment = false;
            bool bBackMode = false;

            // set control Cfg >>　沒設定會用預設值


            lPathIndex = tAGV_Data.CMotor.tMotorData.lPathNodeIndex;
            tV_S2D = rtVectorOP_2D.GetVector(a_atPathInfo[lPathIndex].tSrc, a_atPathInfo[lPathIndex].tDest);

            if (a_bAligmentFree)
            {   // 不用對正路徑 除非差距過大
                tAGV_Data.CMotor.tMotorData.bPathAngleMatch = true;
            }
            else
            {   // 須要對正路徑 (會自動判斷要正走還是反走)
                if (tAGV_Data.CMotor.tMotorData.bPathAngleMatch == false)
                {   // 這段路徑還沒對正過
                    eTargetError = Math.Abs(rtAngleDiff.GetAngleDiff(tV_S2D, tAGV_Data.tCarInfo.eAngle));   // 車身角度跟路線的角度差

                    // Fang fix on 2016 0505
                    if(a_atPathInfo[lPathIndex].ucTurnType == (byte)rtMotorCtrl.rtTurnType.ARRIVE)
                    {   
                        bBackMode = false;    //  這段就得取貨 >> 強迫正走
                    }
                    else if(a_atPathInfo[lPathIndex].ucTurnType == (byte)rtMotorCtrl.rtTurnType.PARK)
                    {
                        bBackMode = true;    //  這段就得停車 >> 強迫反走
                    }
                    else
                    {
                        bBackMode = (eTargetError >= 90) ? true : false;    // 自行判斷正走還是反走方便
                    }
                    
                    eWheelTheta = Math.Abs(tAGV_Data.tCarInfo.eWheelAngle); // 當下車輪角度

                    // 初步檢測 >> 角度&輪胎偏差別太大就執行路徑
                    if (bBackMode)
                    {
                        tAGV_Data.CMotor.tMotorData.bPathAngleMatch = ((180 - eTargetError) < ALIGMENT_SAFE_ANGLE) ? true : false;
                    }
                    else
                    {
                        tAGV_Data.CMotor.tMotorData.bPathAngleMatch = (eTargetError < ALIGMENT_SAFE_ANGLE) ? true : false;
                    }
                    tAGV_Data.CMotor.tMotorData.bPathAngleMatch = (eWheelTheta < rtMotorCtrl.THETA_ERROR_TURN) ? tAGV_Data.CMotor.tMotorData.bPathAngleMatch : false;

                    if (tAGV_Data.CMotor.tMotorData.bPathAngleMatch == false)
                    {   // 初步檢測 fail >> do aligment
                        tV_Aligment = (bBackMode) ? rtVectorOP_2D.VectorMultiple(tV_S2D, -1) : tV_S2D;
                        eTargetAngle = rtVectorOP_2D.Vector2Angle(tV_Aligment);

                        bAlignment = tAGV_Data.CMotor.CarAngleAlignment(
                            eTargetAngle, tAGV_Data.tCarInfo);

                        if (bAlignment)
                        {   // 已對準 開始執行路徑 並且把對準旗標拉起來
                            tAGV_Data.CMotor.tMotorData.bPathAngleMatch = true;
                        }
                        else
                        {   // 沒對準 不做動作就離開
                            return;
                        }
                    }
                    else
                    {   // 初步檢測 success >> reset aligment flag "bAlignmentCarAngleMatch"
                        tAGV_Data.CMotor.ResetCarAlignmentFlag();
                    }
                }
            }

            // 正常控制

            if (a_atPathInfo[tAGV_Data.CMotor.tMotorData.lPathNodeIndex].ucStatus == (byte)rtMotorCtrl.rtStatus.STRAIGHT)
            {   //  走直線
                // decide Motor Power
                tAGV_Data.CMotor.MotorPower_CtrlNavigateStraight(a_atPathInfo, tAGV_Data.tCarInfo);

                // decide Motor Angle
                tAGV_Data.CMotor.MotorAngle_CtrlNavigateStraight(a_atPathInfo, tAGV_Data.tCarInfo);
                return;
            }

            if (a_atPathInfo[tAGV_Data.CMotor.tMotorData.lPathNodeIndex].ucStatus == (byte)rtMotorCtrl.rtStatus.TURN)
            {   //  轉彎
                switch (a_atPathInfo[tAGV_Data.CMotor.tMotorData.lPathNodeIndex].ucTurnType)
                {
                    case (byte)rtMotorCtrl.rtTurnType.SIMPLE:   // 用Aligment 機制
                        tAGV_Data.CMotor.Motor_CtrlNavigateAligment(a_atPathInfo, tAGV_Data.tCarInfo);
                        break;
                    case (byte)rtMotorCtrl.rtTurnType.SMOOTH:
                        // decide Motor Power
                        tAGV_Data.CMotor.MotorPower_CtrlNavigateSmoothTurn(a_atPathInfo, tAGV_Data.tCarInfo);

                        // decide Motor Angle
                        tAGV_Data.CMotor.MotorAngle_CtrlNavigateSmoothTurn(a_atPathInfo, tAGV_Data.tCarInfo);
                        break;
                    default:
                        // show error msg
                        break;
                }
                return;
            }

        }

        static void ExtendPathSize(ref rtPath_Info a_tPathInfo, int a_lExtendSize)
        {
            rtVector tDirection = new rtVector();
            tDirection = rtVectorOP_2D.GetVector(a_tPathInfo.tSrc, a_tPathInfo.tDest);
            a_tPathInfo.tDest = rtVectorOP_2D.ExtendPointAlongVector(a_tPathInfo.tDest, tDirection, a_lExtendSize);
        }

        public static void PathModifyForStorage(ref rtPath_Info[] a_atPathInfo, rtCarData a_tCarData, double a_eDestDirection, bool a_bPark)
        {
            int lCnt = 0, lFinalPathIndex = 0, lCntFix = 0, lLastPathIndex, lExtendLength = 0, lCarLength = 0;
            double eCross = 0, eDeltaTmp = 0, eTargetError = 0;
            bool bBackMode = false;
            rtVector tDestVector = new rtVector();
            rtVector tPathVectorFinal = new rtVector();
            rtVector tPathVectorLast = new rtVector();
            rtVector tVlaw = new rtVector();

            lFinalPathIndex = a_atPathInfo.Length - 1;
            tPathVectorFinal = rtVectorOP_2D.GetVector(a_atPathInfo[lFinalPathIndex].tSrc, a_atPathInfo[lFinalPathIndex].tDest);

            for (lCnt = 0; lCnt < a_atPathInfo.Length; lCnt++)
            {
                if (rtMotorCtrl.Link2DestCheck(a_atPathInfo, lCnt))
                {   // current path link to goods
                    tDestVector = rtVectorOP_2D.Angle2Vector(a_eDestDirection);
                    if (a_bPark)
                    {   // 停車要取反向
                        tDestVector = rtVectorOP_2D.VectorMultiple(tDestVector, -1);
                    }
                    eDeltaTmp = rtVectorOP_2D.GetTheta(tPathVectorFinal, tDestVector);

                    if (eDeltaTmp > rtMotorCtrl.DELTA_ANGLE_TH)
                    {   // need path offset
                        

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

                        if (a_bPark)
                        {   // 停車要取反向
                            tVlaw = rtVectorOP_2D.VectorMultiple(tVlaw, -1);
                        }

                        // modify all path linked to dest of goods
                        for (lCntFix = lCnt; lCntFix < a_atPathInfo.Length; lCntFix++)
                        {
                            a_atPathInfo[lCntFix].tSrc = rtVectorOP_2D.ExtendPointAlongVector(a_atPathInfo[lCntFix].tSrc, tVlaw, rtMotorCtrl.DEFAULT_PATH_OFFSET);
                            a_atPathInfo[lCntFix].tDest = rtVectorOP_2D.ExtendPointAlongVector(a_atPathInfo[lCntFix].tDest, tVlaw, rtMotorCtrl.DEFAULT_PATH_OFFSET);
                        }

                        lLastPathIndex = lCnt - 1;
                        if (lLastPathIndex >= 0)
                        {   // 如果有前一段就改變其終點 >> 改成下一段的起點 (連到最終點的那一整串的第一段)
                            a_atPathInfo[lLastPathIndex].tDest.Set(a_atPathInfo[lCnt].tSrc.eX, a_atPathInfo[lCnt].tSrc.eY);
                        }
                        break;
                    }
                    else
                    {   // 不需要偏移offset
#if NAVIGATE_EXTEND_PATH
                        rtVector tV_S2D = new rtVector();
                        rtVector tCar = new rtVector();
                        tV_S2D = rtVectorOP_2D.GetVector(a_atPathInfo[0].tSrc, a_atPathInfo[0].tDest);
                        tCar = tCar = rtVectorOP_2D.Angle2Vector(a_tCarData.eAngle);
                        eTargetError = rtVectorOP_2D.GetTheta(tV_S2D, tCar);
                        bBackMode = (eTargetError >= 90) ? true : false;
                        if (a_bPark && !bBackMode)
                        {   // 如果是停車又要反向(正著走)  就要預留空間給它轉彎(縮短)
                            lCarLength = (int)(rtVectorOP_2D.GetDistance(a_tCarData.tMotorPosition, a_tCarData.tPosition));
                            ExtendPathSize(ref a_atPathInfo[lFinalPathIndex], -lCarLength);
                        }
#endif
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

        public static void ResetForStandby(rtAGV_Control a_tAGV)
        {
            //a_tAGV.ullAGV_Cmd = 0x00;
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


        public void MoveToAssignedPosition(WarehousPos a_tWarehousPos, bool a_bPark)
        {
            while (tAGV_Data.CMotor.tMotorData.bFinishFlag == false && tAGV_Data.bEmergency == false)
            {
                AutoNavigate(a_tWarehousPos, a_bPark);
#if ThreadDelay
                Thread.Sleep(15);
#endif
            }
            // 初始化 motor control Class
            tAGV_Data.CMotor = new rtMotorCtrl();
        }

        static void Swap<T>(ref T a_tVar_1, ref T a_tVar_2)
        {
            T tVarTmp = a_tVar_1;
            a_tVar_1 = a_tVar_2;
            a_tVar_2 = tVarTmp;
        }

        public WarehousPos ObtainWarehousPosition(bool a_bMode)
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

            // 取得倉儲方向
            tWarehousPos.eDirection = tAGV_Cfg.atWarehousingCfg[tWarehousPos.lRegion][tWarehousPos.lIndex].eDirection;

            return tWarehousPos;
        }

        public void Park()
        {
            bool bBreak = false, bDone = false;
            WarehousPos tWarehousPos = new WarehousPos();
            tWarehousPos = ObtainWarehousPosition(true);

            while (bBreak == false)
            {
                switch (tAGV_Data.ucAGV_Status)
                {
                    // 導航到停車處
                    case (byte)rtAGVStatus.MOVE_TO_PARK:
                        // 自動導航到該停車位
                        MoveToAssignedPosition(tWarehousPos, true);

                        // 清空路徑資料避免被誤用
                        tAGV_Data.atPathInfo = new rtPath_Info[0];
                        if (tAGV_Data.bEmergency == false)
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.ALIMENT;
                        }
                        else
                        {
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                        }
                        break;

                    // 對準停車方向
                    case (byte)rtAGVStatus.ALIMENT:
                        bDone = tAGV_Data.CMotor.CarAngleAlignment(
                            tAGV_Cfg.atWarehousingCfg[tWarehousPos.lRegion][tWarehousPos.lIndex].eDirection,
                            tAGV_Data.tCarInfo);

                        if (bDone)
                        {   // 達到要的角度 >>  進入 PARKING 動作
                            rtVector tDirectionLocal = new rtVector();
                            tDirectionLocal = rtVectorOP_2D.Angle2Vector(tAGV_Cfg.atWarehousingCfg[tWarehousPos.lRegion][tWarehousPos.lIndex].eDirection);
                            tDirectionLocal = rtVectorOP_2D.VectorMultiple(tDirectionLocal, -1);
                            tAGV_Data.atPathInfoForkForth = new rtPath_Info[1];
                            tAGV_Data.atPathInfoForkForth[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            tAGV_Data.atPathInfoForkForth[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;

                            tAGV_Data.atPathInfoForkForth[0].tDest.eX = tAGV_Cfg.atWarehousingCfg[tWarehousPos.lRegion][tWarehousPos.lIndex].tCoordinate.eX;
                            tAGV_Data.atPathInfoForkForth[0].tDest.eY = tAGV_Cfg.atWarehousingCfg[tWarehousPos.lRegion][tWarehousPos.lIndex].tCoordinate.eY;

                            // 暫時先退 1 公尺
                            tAGV_Data.atPathInfoForkForth[0].tSrc = rtVectorOP_2D.ExtendPointAlongVector(
                                tAGV_Data.atPathInfoForkForth[0].tDest, tDirectionLocal, STORAGE_BACK_DISTANCE);

                            bCheckWheelAngle = false;
                            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.PARKING;
                        }
                        break;

                    // 停車
                    case (byte)rtAGVStatus.PARKING:
                        //先判斷輪胎是否回正
                        if (!bCheckWheelAngle)
                        {
                            bCheckWheelAngle = (Math.Abs(tAGV_Data.tCarInfo.eWheelAngle) < rtMotorCtrl.ANGLE_MATCH_TH) ? true : false;
                        }
                        else
                        {
                            rtAGV_MotorCtrl(ref tAGV_Data.atPathInfoForkForth, tWarehousPos.eDirection, true);  // 前面已對正過 >>所以設為true

                            if (tAGV_Data.CMotor.tMotorData.bFinishFlag == true)
                            {
                                // reset
                                tAGV_Data.CMotor = new rtMotorCtrl();
                                tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.STANDBY;
                                tAGV_Data.atPathInfoForkForth = new rtPath_Info[0];
                            }
                        }
                        break;
                    // 結束
                    case (byte)rtAGVStatus.STANDBY:
                        ResetForStandby(this);
                        bBreak = true;
                        break;

                    default:
                        EmergencyStop();
                        bBreak = true;
                        break;
                }
            }
        }

        public void LoadGoods()
        {
            bool bBreak = false;
            WarehousPos tWarehousPos = new WarehousPos();
            tWarehousPos = ObtainWarehousPosition(true);

            while (bBreak == false)
            {
                switch (tAGV_Data.ucAGV_Status)
                {
                    // 導航到取貨處
                    case (byte)rtAGVStatus.MOVE_TO_SRC:
                        // 自動導航到該櫃位
                        MoveToAssignedPosition(tWarehousPos, false);

                        // 清空路徑資料避免被誤用
                        tAGV_Data.atPathInfo = new rtPath_Info[0];
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
                    // 結束
                    case (byte)rtAGVStatus.STANDBY:
                        ResetForStandby(this);
                        bBreak = true;
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
            WarehousPos tWarehousPos = new WarehousPos();

            tWarehousPos = ObtainWarehousPosition(false);

            while (bBreak == false)
            {
                switch (tAGV_Data.ucAGV_Status)
                {
                    // 導航到卸貨處
                    case (byte)rtAGVStatus.MOVE_TO_DEST:
                        // 自動導航到該櫃位
                        MoveToAssignedPosition(tWarehousPos, false);

                        // 清空路徑資料避免被誤用
                        tAGV_Data.atPathInfo = new rtPath_Info[0];
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
                        Console.WriteLine("Input UnLoad");
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
                    // 結束
                    case (byte)rtAGVStatus.STANDBY:
                        ResetForStandby(this);
                        bBreak = true;
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
            WarehousPos tWarehousPos = new WarehousPos();

            while (bBreak == false)
            {
                switch (tAGV_Data.ucAGV_Status)
                {
                    // 導航到取貨處
                    case (byte)rtAGVStatus.MOVE_TO_SRC:
                        tWarehousPos = ObtainWarehousPosition(true);
                        // 自動導航到該櫃位
                        MoveToAssignedPosition(tWarehousPos, false);

                        // 清空路徑資料避免被誤用
                        tAGV_Data.atPathInfo = new rtPath_Info[0];
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
                        tWarehousPos = ObtainWarehousPosition(true);
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
                        tWarehousPos = ObtainWarehousPosition(false);
                        // 自動導航到該櫃位
                        MoveToAssignedPosition(tWarehousPos, false);

                        // 清空路徑資料避免被誤用
                        tAGV_Data.atPathInfo = new rtPath_Info[0];
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
                        tWarehousPos = ObtainWarehousPosition(false);
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
                    // 結束
                    case (byte)rtAGVStatus.STANDBY:
                        ResetForStandby(this);
                        bBreak = true;
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
        
        private int ArriveCheck(rtPath_Info[] a_atPathInfo, rtWarehousingInfo a_LocatData)
        {
            double eDistance = 0;

            if (a_atPathInfo.Length <= 0)
            {
                return -1;
            }

            if (a_atPathInfo.Length > 1)
            {
                return 1;
            }
            else
            {   // == 1
                eDistance = rtVectorOP_2D.GetDistance(tAGV_Data.tCarInfo.tPosition, a_atPathInfo[a_atPathInfo.Length - 1].tDest);

                if(eDistance < 50)
                {   // 夠近不用走了
                    return 0;
                }

                if (eDistance > 2000)
                {   // 夠遠 >> 可走看看
                    return 1;
                }
                return -1;  // 可能不方便移動 >> 先給error
            }
        }

        public void AutoNavigate(WarehousPos a_tWarehousPos, bool a_bPark)
        {
            rtWarehousingInfo LocatData;    // 目的地
            ROI[] atObstacle = new ROI[0];
            int lCheckResult = 0;

            // 從cfg中找出 目的地在哪個櫃位，不論是取貨點還是放貨點
            LocatData = tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex];

            // Obstacle Avoidance 檢查當下路徑或方向有沒有障礙物的威脅，並且回傳障礙物資訊和緊急訊號
            tAGV_Data.bEmergency = ObstacleAvoidance(ref atObstacle);

            if (tAGV_Data.bEmergency == false)
            {
                // 檢測或算出路徑
                lCheckResult = rtAGV_Navigation(LocatData, atObstacle, a_bPark);

                if (lCheckResult > 0)
                {   //  行走控制
                    // 控制馬達
                    rtAGV_MotorCtrl(ref tAGV_Data.atPathInfo, a_tWarehousPos.eDirection, false);
                    return;
                }
                if (lCheckResult == 0)
                {   //  已到達
                    tAGV_Data.CMotor.tMotorData.bFinishFlag = true;
                    return;
                }
                if (lCheckResult < 0)
                {   //  不能行走
                    tAGV_Data.CMotor.tMotorData.bFinishFlag = true;
                    tAGV_Data.bEmergency = true;
                    return;
                }
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
                //Console.WriteLine("ForkData.height: " + tAGV_Data.CFork.tForkData.height + ",ForkData.distanceDepth :" + tAGV_Data.tSensorData.tForkInputData.distanceDepth + "--" + eDiffHeight + "," + eDiffDepth);
#endif
                return false;
            }
        }

        public void Storage(WarehousPos a_tWarehousPos)
        {
            bool bDone;

            while (tAGV_Data.CFork.tForkData.ucStatus != (byte)rtForkCtrl.ForkStatus.FINISH && tAGV_Data.ucAGV_Status != (byte)rtAGVStatus.EMERGENCY_STOP)
            {
                switch (tAGV_Data.CFork.tForkData.ucStatus)
                {
                    // 初始狀態
                    case (byte)rtForkCtrl.ForkStatus.NULL:
                        // 初始化 motor & fork control Class
                        tAGV_Data.CFork = new rtForkCtrl();
                        tAGV_Data.CMotor = new rtMotorCtrl();
                        tAGV_Data.atPathInfoForkForth = new rtPath_Info[1];
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
                            rtVector tDirectionLocal = new rtVector();
                            tDirectionLocal = rtVectorOP_2D.Angle2Vector(tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].eDirection);
                            tDirectionLocal = rtVectorOP_2D.VectorMultiple(tDirectionLocal, -1);

                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.FORTH;
                            tAGV_Data.atPathInfoForkForth[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            tAGV_Data.atPathInfoForkForth[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            //tAGV_Data.atPathInfoForkForth[0].tSrc.eX = tAGV_Data.tCarInfo.tPosition.eX;
                            //tAGV_Data.atPathInfoForkForth[0].tSrc.eY = tAGV_Data.tCarInfo.tPosition.eY;
                            
                            tAGV_Data.atPathInfoForkForth[0].tDest.eX = tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].tCoordinate.eX;
                            tAGV_Data.atPathInfoForkForth[0].tDest.eY = tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].tCoordinate.eY;

                            // 暫時先退 1 公尺
                            tAGV_Data.atPathInfoForkForth[0].tSrc = rtVectorOP_2D.ExtendPointAlongVector(
                                tAGV_Data.atPathInfoForkForth[0].tDest, tDirectionLocal, STORAGE_BACK_DISTANCE);
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
                            tAGV_Data.atPathInfo = tAGV_Data.atPathInfoForkForth;
                            //Console.WriteLine("Path: " + atPathInfo[0].tDest.eX + "," + atPathInfo[0].tDest.eY);
#endif
                            rtAGV_MotorCtrl(ref tAGV_Data.atPathInfoForkForth, a_tWarehousPos.eDirection, true);  // 前面已轉正過 >>所以設為true
                            
                            if (tAGV_Data.CMotor.tMotorData.bFinishFlag == true)
                            {
                                // reset
                                tAGV_Data.atPathInfo = new rtPath_Info[0];
                                tAGV_Data.CMotor = new rtMotorCtrl();
                                // tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.SET_DEPTH;
                                tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.ALIMENT_FORTH;
                                
                            }
                        }
                        break;
                    // ALIMENT_FORTH
                    case (byte)rtForkCtrl.ForkStatus.ALIMENT_FORTH:
                        bDone = false;
                        bDone = tAGV_Data.CMotor.CarAngleAlignment(
                            tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].eDirection,
                            tAGV_Data.tCarInfo);

                       /* tAGV_Data.CFork.tForkData.distanceDepth = 0;
                        tAGV_Data.CFork.tForkData.height = 0;*/
                        tAGV_Data.CFork.tForkData.bEnable = false;

                        if (bDone)
                        {   // 達到要的角度 >> 進入 SET_DEPTH 動作
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
                        tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].eHeight + rtForkCtrl.FORK_PICKUP_HEIGHT;
                        tAGV_Data.CFork.tForkData.bEnable = true;
                        //Console.WriteLine("PICKUP:" + tAGV_Data.CFork.tForkData.height + " , " + a_tWarehousPos.lRegion + " , " + a_tWarehousPos.lIndex);
                        if (ForkActionFinishCheck())
                        {   // 起點終點交換 & 進入 BACKWARD
                            tAGV_Data.atPathInfoForkForth[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            tAGV_Data.atPathInfoForkForth[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            Swap(ref tAGV_Data.atPathInfoForkForth[0].tSrc, ref tAGV_Data.atPathInfoForkForth[0].tDest);
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.RESET_DEPTH;
                        }
                        break;
                    // PICKDOWN
                    case (byte)rtForkCtrl.ForkStatus.PICKDOWN:
                        tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tWarehousPos.lRegion][a_tWarehousPos.lIndex].eHeight;
                        tAGV_Data.CFork.tForkData.bEnable = true;

                        if (ForkActionFinishCheck())
                        {   // 起點終點交換 & 進入 BACKWARD
                            tAGV_Data.atPathInfoForkForth[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            tAGV_Data.atPathInfoForkForth[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            Swap(ref tAGV_Data.atPathInfoForkForth[0].tSrc, ref tAGV_Data.atPathInfoForkForth[0].tDest);
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
                        tAGV_Data.atPathInfo = tAGV_Data.atPathInfoForkForth;
                        //Console.WriteLine(atPathInfoForkForth[0].tSrc.eX + "," + atPathInfoForkForth[0].tSrc.eY + "---->" + atPathInfoForkForth[0].tDest.eX + "," + atPathInfoForkForth[0].tDest.eY);
#endif
                        rtAGV_MotorCtrl(ref tAGV_Data.atPathInfoForkForth, a_tWarehousPos.eDirection, true);

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
                        tAGV_Data.CFork.tForkData.height = 90;
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
                        Console.WriteLine("Error");
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

        /** \brief InOutput Data: AGV data */
        public rtAGV_Data tAGV_Data;

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

        public void Send_AGV_InfoToServer()
        {
            if (sender_TCP != null && sender_TCP.Connected)
            {
                byte[] SendDataByte;
                string SendDataStr = string.Empty;
                //SendDataStr = "ForkLiftResponse" + "," + GlobalVar.This_AGV_ID + "," + tAGV_Data.tCarInfo.tPosition.eX + "," + tAGV_Data.tCarInfo.tPosition.eY + "," + tAGV_Data.tCarInfo.eAngle + "," +
                //    tAGV_Data.tSensorData.tForkInputData.height + "," + tAGV_Data.ucAGV_Status + "," + tAGV_Data.CFork.tForkData.ucStatus;
                SendDataByte = Encoding.UTF8.GetBytes(SendDataStr);
                SendData(SendDataByte);
            }
        }
    }
}