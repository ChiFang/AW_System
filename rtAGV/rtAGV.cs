﻿#define rtAGV_TEST_0


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

        /** \brief Fork Input Data */
        public rtForkCtrl_Data tForkInputData;
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

        /** \brief InOutput Data: Sensor Data */
        public rtAGV_SensorData tSensorData;
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
        public enum rtAGVCmd { STOP = 0x00, DELIVER = 0x01, CONTINUE = 0x02, PAUSE = 0x03, RESET = 0x04 };

        public enum rtAGVStatus { NON_INITAILIZE = 0, BUSY = 1, PAUSE = 2, STANDBY = 3, STOP = 4, EMERGENCY_STOP = 5, ERROR_NO_CFG = 6, MOVE_TO_SRC = 7, LOAD = 8, MOVE_TO_DEST = 9, UNLOAD = 10 };

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
        // public rtAGV_SensorData tSensorData;

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

        public static void rtAGV_MotorCtrl(ref rtPath_Info[] a_atPathInfo, ref rtMotor_Cfg a_tMotorCtrl_Cfg, rtCarData a_tCarInfo, ref rtMotorCtrl a_CMotor)
        {

            double eErrorPower = 0;
            double eErrorAngle = 0;

            // set control Cfg >>　沒設定會用預設值

            // decide Motor Power
            eErrorPower = rtMotorCtrl.MotorPower_Ctrl(a_atPathInfo, a_tCarInfo, ref a_CMotor);

            // decide Motor Angle
            eErrorAngle = rtMotorCtrl.MotorAngle_CtrlNavigate(a_atPathInfo, a_tCarInfo, ref a_CMotor);
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


        public void MoveToAssignedPosition(NodeId a_tAssignedNode)
        {
            // 初始化 motor control Class
            tAGV_Data.CMotor = new rtMotorCtrl();

            while (tAGV_Data.CMotor.tMotorData.bFinishFlag == false && tAGV_Data.ucAGV_Status != (byte)rtAGVStatus.EMERGENCY_STOP)
            {
                AutoNavigate(a_tAssignedNode, tAGV_Cfg, tAGV_Data.tSensorData, ref tAGV_Data);
            }
        }

#if rtAGV_TEST_0    // hard code 設定路徑
    public static void Test_0(ref rtAGV_Control a_CAGV_Sys)
    {

        // set cmd
        a_CAGV_Sys.ulAGV_Cmd = 0;

        // set cfg
        a_CAGV_Sys.tAGV_Cfg.tCarCfg.lLength = 1500;
        a_CAGV_Sys.tAGV_Cfg.tCarCfg.lWidth = 1140;
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
#if rtAGV_TEST_0    // hard code 設定路徑

#endif
            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.MOVE_TO_SRC;
            MoveToAssignedPosition(tSrc);
            tAGV_Data.atPathInfo = new rtPath_Info[0]; // 清空路靖資料

            // step 2:Load goods
            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.LOAD;
            Storage(tSrc);

            // step 3:move to destination
#if rtAGV_TEST_0    // hard code 設定路徑

#endif
            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.MOVE_TO_DEST;
            MoveToAssignedPosition(tDest);
            tAGV_Data.atPathInfo = new rtPath_Info[0]; // 清空路靖資料

            // step 4:Unload goods
            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.UNLOAD;
            Storage(tDest);

            // step 5:stand by at assign position (TBD)
            tAGV_Data.ucAGV_Status = (byte)rtAGVStatus.STANDBY;
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
                rtAGV_MotorCtrl(ref a_tAGV_Data.atPathInfo, ref a_tAGV_Cfg.tMotorCtrlCfg, a_tAGV_Data.tCarInfo, ref a_tAGV_Data.CMotor);
            }
        }

        public void Storage(NodeId a_tStoragePos)
        {
            bool bDone = false;
            double eDiffHeight = 0;
            double eDiffDepth = 0;
            rtPath_Info[] atPathInfo = new rtPath_Info[1];
            rtVector tTmpPos = new rtVector(0,0);

            // 初始化 motor & fork control Class
            tAGV_Data.CFork = new rtForkCtrl();
            tAGV_Data.CMotor = new rtMotorCtrl();

            while (tAGV_Data.CFork.tForkData.ucStatus == (byte)rtForkCtrl.ForkStatus.FINISH && tAGV_Data.ucAGV_Status != (byte)rtAGVStatus.EMERGENCY_STOP)
            {
                switch (tAGV_Data.CFork.tForkData.ucStatus)
                {
                    // 初始狀態
                    case (byte)rtForkCtrl.ForkStatus.NULL:
                        tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.ALIMENT;
                        break;

                    // ALIMENT
                    case (byte)rtForkCtrl.ForkStatus.ALIMENT:
                        bDone = false;
                        tAGV_Data.CFork.tForkData.bEnable = false;
                        bDone = rtMotorCtrl.CarAngleAlignment(
                        tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].eDirection,
                        tAGV_Data.tCarInfo, tAGV_Data.CMotor);
                        if(bDone)
                        {
                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.SET_HEIGHT;
                        }
                        break;
                    // SET_HEIGHT
                    case (byte)rtForkCtrl.ForkStatus.SET_HEIGHT:

                        if (tAGV_Data.ucAGV_Status == (byte)rtAGVStatus.LOAD)
                        {   // LOAD
                            tAGV_Data.CFork.tForkData.bEnable = true;
                            tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].eHeight;
                            tAGV_Data.CFork.tForkData.distanceDepth = rtForkCtrl.FORK_MAX_DEPTH;
                        }
                        else
                        {   // UNLOAD
                            tAGV_Data.CFork.tForkData.bEnable = true;
                            tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].eHeight;
                            tAGV_Data.CFork.tForkData.height += rtForkCtrl.FORK_PICKUP_HEIGHT;
                            tAGV_Data.CFork.tForkData.distanceDepth = rtForkCtrl.FORK_MAX_DEPTH;
                        }
                        eDiffHeight = 0;
                        eDiffDepth = 0;

                        eDiffHeight = Math.Abs(tAGV_Data.CFork.tForkData.height - tAGV_Data.tSensorData.tForkInputData.height);
                        eDiffDepth = Math.Abs(tAGV_Data.CFork.tForkData.distanceDepth - tAGV_Data.tSensorData.tForkInputData.distanceDepth);

                        if (eDiffHeight < rtForkCtrl.FORK_MATCH_TH && eDiffDepth < rtForkCtrl.FORK_MATCH_TH)
                        {   // 達到要的深度跟高度
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
                        tAGV_Data.CFork.tForkData.bEnable = false;
                        rtAGV_MotorCtrl(ref atPathInfo, ref tAGV_Cfg.tMotorCtrlCfg, tAGV_Data.tCarInfo, ref tAGV_Data.CMotor);

                        if (tAGV_Data.CMotor.tMotorData.bFinishFlag == true)
                        {
                            // reset
                            tAGV_Data.CMotor = new rtMotorCtrl();

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
                        tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].eHeight;
                        tAGV_Data.CFork.tForkData.height += rtForkCtrl.FORK_PICKUP_HEIGHT;
                        tAGV_Data.CFork.tForkData.distanceDepth = rtForkCtrl.FORK_MAX_DEPTH;
                        eDiffHeight = Math.Abs(tAGV_Data.CFork.tForkData.height - tAGV_Data.tSensorData.tForkInputData.height);
                        eDiffDepth = Math.Abs(tAGV_Data.CFork.tForkData.distanceDepth - tAGV_Data.tSensorData.tForkInputData.distanceDepth);

                        if (eDiffHeight < rtForkCtrl.FORK_MATCH_TH && eDiffDepth < rtForkCtrl.FORK_MATCH_TH)
                        {
                            // 起點終點交換
                            atPathInfo[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            atPathInfo[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            tTmpPos.eX = atPathInfo[0].tSrc.eX;
                            tTmpPos.eY = atPathInfo[0].tSrc.eY;
                            atPathInfo[0].tSrc.eX = atPathInfo[0].tDest.eX;
                            atPathInfo[0].tSrc.eY = atPathInfo[0].tDest.eY;
                            atPathInfo[0].tDest.eX = tTmpPos.eX;
                            atPathInfo[0].tDest.eY = tTmpPos.eY;

                            tAGV_Data.CFork.tForkData.ucStatus = (byte)rtForkCtrl.ForkStatus.BACKWARD;
                        }
                        break;
                    // PICKDOWN
                    case (byte)rtForkCtrl.ForkStatus.PICKDOWN:
                        tAGV_Data.CFork.tForkData.bEnable = true;
                        tAGV_Data.CFork.tForkData.height = (int)tAGV_Cfg.atWarehousingCfg[a_tStoragePos.lRegion][a_tStoragePos.lIndex].eHeight;
                        tAGV_Data.CFork.tForkData.distanceDepth = rtForkCtrl.FORK_MAX_DEPTH;
                        eDiffHeight = Math.Abs(tAGV_Data.CFork.tForkData.height - tAGV_Data.tSensorData.tForkInputData.height);
                        eDiffDepth = Math.Abs(tAGV_Data.CFork.tForkData.distanceDepth - tAGV_Data.tSensorData.tForkInputData.distanceDepth);

                        if (eDiffHeight < rtForkCtrl.FORK_MATCH_TH && eDiffDepth < rtForkCtrl.FORK_MATCH_TH)
                        {
                            // 起點終點交換
                            atPathInfo[0].ucStatus = (byte)rtMotorCtrl.rtStatus.STRAIGHT;
                            atPathInfo[0].ucTurnType = (byte)rtMotorCtrl.rtTurnType.ARRIVE;
                            tTmpPos.eX = atPathInfo[0].tSrc.eX;
                            tTmpPos.eY = atPathInfo[0].tSrc.eY;
                            atPathInfo[0].tSrc.eX = atPathInfo[0].tDest.eX;
                            atPathInfo[0].tSrc.eY = atPathInfo[0].tDest.eY;
                            atPathInfo[0].tDest.eX = tTmpPos.eX;
                            atPathInfo[0].tDest.eY = tTmpPos.eY;

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
                    // step 4: 降回最低點
                    case (byte)rtForkCtrl.ForkStatus.RESET_HEIGHT:
                        tAGV_Data.CFork.tForkData.bEnable = true;
                        tAGV_Data.CFork.tForkData.height = 0;
                        tAGV_Data.CFork.tForkData.distanceDepth = 0;

                        eDiffHeight = Math.Abs(tAGV_Data.CFork.tForkData.height - tAGV_Data.tSensorData.tForkInputData.height);
                        eDiffDepth = Math.Abs(tAGV_Data.CFork.tForkData.distanceDepth - tAGV_Data.tSensorData.tForkInputData.distanceDepth);

                        if (eDiffHeight < rtForkCtrl.FORK_MATCH_TH && eDiffDepth < rtForkCtrl.FORK_MATCH_TH)
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