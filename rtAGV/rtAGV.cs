﻿
using System;

using rtAGV_Common;
using PLC_Control;
using rtAGV_Navigate;


namespace rtAGV_Sys
{
	public struct rtCarCFG
    {
        /** \brief 兩輪中心到後輪距離 */
        public int lLength;

        /** \brief 兩輪距離 */
        public int lWidth;
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
    }

    public struct rtAGV_Data
    {
        /** \brief Emergency Flag */
        public bool bEmergencyFlag;

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

        /** \brief Finish Flag 用於檢查每個小步驟是否完成 (TBD) */
        public bool bFinishFlag;
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
        public enum rtAGVCmd {STOP = 0x00, DELIVER = 0x01, CONTINUE = 0x02, PAUSE = 0x03, RESET = 0x04};

        public enum rtAGVStatus { NON_INITAILIZE = 0, BUSY = 1, PAUSE = 2, STANDBY = 3, STOP = 4, EMERGENCY_STOP = 5, ERROR_NO_CFG = 6};

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
        public uint ulAGV_Cmd = 0;

        /** \brief InOutput Data: AGV data */
        public rtAGV_Data tAGV_Data;

        /** \brief InOutput Data: Sensor Data */
        public rtAGV_SensorData tSensorData;

        /** \brief 初始化用 建構函式 */
        public rtAGV_Control()
        {
            // 載入設定擋做初始化，設定擋名稱為至先hard code
            Reset(this);
        }

        public void ExecuteCmd(uint a_ulAGV_Cmd)
        {
            uint ulAction = 0;
            if (a_ulAGV_Cmd != ulAGV_Cmd)
            { // 不一樣 >> 新命令
                ulAGV_Cmd = a_ulAGV_Cmd;
                ulAction = (ulAGV_Cmd >> CMD) & MASK;
                switch (ulAction)
                {
                    // 運送貨物
                    case (uint)rtAGVCmd.DELIVER:
                        Deliver();
                        break;
                    // 停止
                    case (uint)rtAGVCmd.STOP:
                        EmergencyStop();
                        break;
                    // 從剛剛停止的地方繼續執行命令
                    case (uint)rtAGVCmd.CONTINUE:
                        Continue();
                        break;
                    // 暫停: 暫時停止，資料&動作先暫留
                    case (uint)rtAGVCmd.PAUSE:
                        Pause();
                        break;
                    // 重新初始化
                    case (uint)rtAGVCmd.RESET:
                        Reset(this);
                        break;
                    default:
                        // show error
                        break;
                }
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

        public static void rtAGV_MotorCtrl(ref rtMotor_Cfg a_tMotorCtrl_Cfg, ref rtAGV_Data a_tAGV_Data)
        {

            double eErrorPower = 0;
            double eErrorAngle = 0;

            // set control Cfg >>　沒設定會用預設值

            // decide Motor Power
            eErrorPower = rtMotorCtrl.MotorPower_Ctrl(a_tAGV_Data.atPathInfo, a_tAGV_Data.tCarInfo, ref a_tAGV_Data.CMotor);

            // decide Motor Angle
            eErrorAngle = rtMotorCtrl.MotorAngle_CtrlNavigate(a_tAGV_Data.atPathInfo, a_tAGV_Data.tCarInfo, ref a_tAGV_Data.CMotor);
        }

        public static void Reset(rtAGV_Control tAGV)
        {

        }

        public static void Continue()
        {

        }

        public static void Pause()
        {

        }

        public void Deliver()
        {
            NodeId tDest;
            NodeId tSrc;

            // step 0: extract element from command
            tSrc.lRegion = (int)((ulAGV_Cmd >> SRC_REGION) & MASK);
            tSrc.lIndex = (int)((ulAGV_Cmd >> SRC_POSITION) & MASK);
            tDest.lRegion = (int)((ulAGV_Cmd >> DEST_REGION) & MASK);
            tDest.lIndex = (int)((ulAGV_Cmd >> DEST_POSITION) & MASK);

            // step 1: move to goods position
            // 初始化 Class
            tAGV_Data.CMotor = new rtMotorCtrl();

            while (tAGV_Data.CMotor.tMotorData.bFinishFlag == false || tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.EMERGENCY_STOP)
            {
                AutoNavigate(tSrc, tAGV_Cfg, tSensorData, ref tAGV_Data);
            }

            // step 2:Load goods
            // 初始化 Class
            tAGV_Data.CFork = new rtForkCtrl();

            while (tAGV_Data.CFork.tForkData.ucStatus == (byte)rtForkCtrl.ForkStatus.FINISH)
            {
                LOAD(tAGV_Cfg.atWarehousingCfg[tSrc.lRegion][tSrc.lIndex], tSensorData, ref tAGV_Data);
            }

            // step 3:move to destination

            // 初始化 Class
            tAGV_Data.CMotor = new rtMotorCtrl();

            while (tAGV_Data.CMotor.tMotorData.bFinishFlag == false || tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.EMERGENCY_STOP)
            {
                AutoNavigate(tDest, tAGV_Cfg, tSensorData, ref tAGV_Data);
            }

            // step 4:Unload goods
            // 初始化 Class
            tAGV_Data.CFork = new rtForkCtrl();

            while (tAGV_Data.CFork.tForkData.ucStatus == (byte)rtForkCtrl.ForkStatus.FINISH)
            {
                UNLOAD(tAGV_Cfg.atWarehousingCfg[tDest.lRegion][tDest.lIndex], tSensorData, ref tAGV_Data);
            }

            // step 5:stand by at assign position (TBD)
            StandBy();
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
            a_tAGV_Data.bEmergencyFlag = ObstacleAvoidance(a_tAGV_Cfg.tCarCfg, a_tSensorData, ref a_tAGV_Data.tCarInfo, atObstacle);

            if (a_tAGV_Data.bEmergencyFlag == true)
            {
                a_tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
            }
            else
            {
                // 檢測或算出路徑
                rtAGV_Navigation(LocatData, a_tAGV_Cfg, ref a_tAGV_Data, atObstacle);

                // 控制馬達
                rtAGV_MotorCtrl(ref a_tAGV_Cfg.tMotorCtrlCfg, ref a_tAGV_Data);
            }
        }

        public static void LOAD(rtWarehousingInfo a_tLocatData, rtAGV_SensorData a_tSensorData, ref rtAGV_Data a_tAGV_Data)
        {
            bool bDone = false;

            switch (a_tAGV_Data.CFork.tForkData.ucStatus)
            {
                // 初始狀態
                case (byte)rtForkCtrl.ForkStatus.NULL:
                    a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.ALIMENT;
                    break;

                // 對齊狀態
                case (byte)rtForkCtrl.ForkStatus.ALIMENT:
                    bDone = rtMotorCtrl.CarAngleAlignment(a_tLocatData.eDirection, a_tAGV_Data.tCarInfo, a_tAGV_Data.CMotor);
                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.SET_HEIGHT;
                    }
                    break;
                // step 0: 升到貨物的高度
                case (byte)rtForkCtrl.ForkStatus.SET_HEIGHT:
                  
                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.FORTH;
                    }
                    break;
                // step 1: 伸貨叉
                case (byte)rtForkCtrl.ForkStatus.FORTH:

                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.PICKUP;
                    }
                    break;
                // step 2: 舉貨叉
                case (byte)rtForkCtrl.ForkStatus.PICKUP:

                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.BACKWARD;
                    }
                    break;
                // step 3: 收貨叉
                case (byte)rtForkCtrl.ForkStatus.BACKWARD:

                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.RESET_HEIGHT;
                    }
                    break;
                // step 4: 降回最低點
                case (byte)rtForkCtrl.ForkStatus.RESET_HEIGHT:

                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.FINISH;
                    }
                    break;
                default:
                    // show error
                    break;
            }
        }

        public static void UNLOAD(rtWarehousingInfo a_tLocatData, rtAGV_SensorData a_tSensorData, ref rtAGV_Data a_tAGV_Data)
        {
            bool bDone = false;

            switch (a_tAGV_Data.CFork.tForkData.ucStatus)
            {
                // 初始狀態
                case (byte)rtForkCtrl.ForkStatus.NULL:
                    a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.ALIMENT;
                    break;

                // 對齊狀態
                case (byte)rtForkCtrl.ForkStatus.ALIMENT:
                    bDone = rtMotorCtrl.CarAngleAlignment(a_tLocatData.eDirection, a_tAGV_Data.tCarInfo, a_tAGV_Data.CMotor);
                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.SET_HEIGHT;
                    }
                    break;
                // step 0: 升到貨物的高度
                case (byte)rtForkCtrl.ForkStatus.SET_HEIGHT:

                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.FORTH;
                    }
                    break;
                // step 1: 伸貨叉
                case (byte)rtForkCtrl.ForkStatus.FORTH:

                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.PICKDOWN;
                    }
                    break;
                // step 2: 舉貨叉
                case (byte)rtForkCtrl.ForkStatus.PICKDOWN:

                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.BACKWARD;
                    }
                    break;
                // step 3: 收貨叉
                case (byte)rtForkCtrl.ForkStatus.BACKWARD:

                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.RESET_HEIGHT;
                    }
                    break;
                // step 4: 降回最低點
                case (byte)rtForkCtrl.ForkStatus.RESET_HEIGHT:

                    if (bDone)
                    {
                        a_tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.FINISH;
                    }
                    break;
                default:
                    // show error
                    break;
            }
        }

        public static void EmergencyStop()
        {

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