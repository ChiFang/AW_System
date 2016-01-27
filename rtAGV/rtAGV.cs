
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
        public rtWarehousingInfo[][] atWarehousingInfo;

        /** \brief Region Cfg 區域範圍設定*/
        public ROI[] atRegionCfg;

        /** \brief Map 地圖設定 */
        public rtAGV_MAP tMap;

        /** \brief Spec of AGV car 車身規格設定 */
        public rtCarCFG tCarData;
    }

    public struct rtAGV_SensorData
    {
        /** \brief 光達資料 */
        public double[] aeLiDarData;

        /** \brief 定位座標 */
        public rtVector tPosition;

        /** \brief 定位方向 */
        public double eDirection;
    }

    public struct rtAGV_Data
    {
        /** \brief Emergency Flag */
        public bool bEmergencyFlag;

        /** \brief 導航算出來的路徑 */
        public rtPath_Info[] atPathInfo;

        /** \brief Motor data 馬達相關數值 */
        public rtMotorCtrl tMotor;

        /** \brief Fork data 貨叉相關數值 */
        public rtForkCtrl tFork;

        /** \brief Finish Flag 用於檢查每個小步驟是否完成 */
        public bool bFinishFlag;

        /** \brief 當下車子資訊 */
        public rtCarData tCarInfo;
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

        /** \brief Output Data: AGV Status */
        public byte ucAGV_Status = 0;

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
                        Deliver(ulAGV_Cmd);
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

        public static void rtAGV_Navigation(rtWarehousingInfo a_tLocatData, rtAGV_CFG a_tAGV_Cfg, ref rtAGV_Data a_tAGV_Data, ref byte a_ucAGV_Status, ROI[] a_atObstacle)
        {
            if (a_tAGV_Data.atPathInfo.Length <= 0)
            {   // 沒路徑才計算，之後系統執行完導航都要清掉 path data避免之後動作載道上一次的路徑資料
                // path planning
                rtPathPlanning.rtAGV_PathPlanning(
                    a_tAGV_Cfg.tMap, a_tAGV_Cfg.atWarehousingInfo, a_tAGV_Cfg.atRegionCfg,
                    ref a_tAGV_Data.atPathInfo, ref a_tAGV_Data.tCarInfo, a_tLocatData, a_atObstacle);
            }
        }

        public static void rtAGV_MotorCtrl(ref rtAGV_Data a_tAGV_Data, ref byte a_ucAGV_Status)
        {

            double eErrorPower = 0;
            double eErrorAngle = 0;

            eErrorPower = rtMotorCtrl.MotorPower_Ctrl(a_tAGV_Data.atPathInfo, a_tAGV_Data.tCarInfo, ref a_tAGV_Data.tMotor);

            eErrorAngle = rtMotorCtrl.MotorAngle_CtrlNavigate(a_tAGV_Data.atPathInfo, a_tAGV_Data.tCarInfo, ref a_tAGV_Data.tMotor);
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

        public void Deliver(uint a_ulAGV_Cmd)
        {
            NodeId tDest;
            NodeId tSrc;

            // step 0: extract element from command
            tSrc.lRegion = (int)((a_ulAGV_Cmd >> SRC_REGION) & MASK);
            tSrc.lIndex = (int)((a_ulAGV_Cmd >> SRC_POSITION) & MASK);
            tDest.lRegion = (int)((a_ulAGV_Cmd >> DEST_REGION) & MASK);
            tDest.lIndex = (int)((a_ulAGV_Cmd >> DEST_POSITION) & MASK);

            // step 1: move to goods position
            tAGV_Data.bFinishFlag = false;
            while(tAGV_Data.bFinishFlag == false)
            {
                AutoNavigate(tSrc, tAGV_Cfg, tSensorData, ref tAGV_Data, ref ucAGV_Status);
            }

            // step 2:Load goods
            tAGV_Data.bFinishFlag = false;
            while (tAGV_Data.bFinishFlag == false)
            {
                LOAD(tAGV_Cfg.atWarehousingInfo[tSrc.lRegion][tSrc.lIndex], tSensorData, ref tAGV_Data);
            }

            // step 3:move to destination
            tAGV_Data.bFinishFlag = false;
            while (tAGV_Data.bFinishFlag == false)
            {
                AutoNavigate(tDest, tAGV_Cfg, tSensorData, ref tAGV_Data, ref ucAGV_Status);
            }

            // step 4:Unload goods
            tAGV_Data.bFinishFlag = false;
            while (tAGV_Data.bFinishFlag == false)
            {
                UNLOAD(tAGV_Cfg.atWarehousingInfo[tDest.lRegion][tDest.lIndex], tSensorData, ref tAGV_Data);
            }

            // step 5:stand by at assign position (TBD)
            StandBy();
        }

        public static rtAGV_SensorData ReadSensorData()
        {
            rtAGV_SensorData tSensorData = new rtAGV_SensorData();   // 感應器資料

            return tSensorData;
        }


        public static ROI[] ObstacleAvoidance(rtCarCFG a_tCarCfg, ref bool a_bEmergencyFlag)
        {
            ROI[] atObstacle = new ROI[0];
            

            return atObstacle;
        }
        

        public static void AutoNavigate(NodeId a_tDestination, rtAGV_CFG a_tAGV_Cfg, rtAGV_SensorData a_tSensorData, ref rtAGV_Data a_tAGV_Data, ref byte a_ucAGV_Status)
        {
            rtWarehousingInfo LocatData;    // 目的地
            ROI[] atObstacle;

            // 從cfg中找出 目的地在哪個櫃位，不論是取貨點還是放貨點
            LocatData = a_tAGV_Cfg.atWarehousingInfo[a_tDestination.lRegion][a_tDestination.lIndex];

            // Obstacle Avoidance 檢查 當下路徑 或方向有沒有障礙物的威脅，並且回傳障礙物資訊
            atObstacle = ObstacleAvoidance(a_tAGV_Cfg.tCarData, ref a_tAGV_Data.bEmergencyFlag);

            if (a_tAGV_Data.bEmergencyFlag == true)
            {
                a_ucAGV_Status = (byte)rtAGVStatus.EMERGENCY_STOP;
                a_tAGV_Data.bFinishFlag = true;
            }
            else
            {
                // 檢測或算出路徑
                rtAGV_Navigation(LocatData, a_tAGV_Cfg, ref a_tAGV_Data, ref a_ucAGV_Status, atObstacle);

                // 控制馬達
                rtAGV_MotorCtrl(ref a_tAGV_Data, ref a_ucAGV_Status);
            }
        }

        public static void LOAD(rtWarehousingInfo a_tLocatData, rtAGV_SensorData a_tSensorData, ref rtAGV_Data a_tAGV_Data)
        {
            bool bMatched = false;

            while(bMatched)
            {   // 一直執行到 對齊為止 >> 另有執行緒 隨時更新 a_tAGV_Data.tCarInfo
                bMatched = rtMotorCtrl.CarAngleAlignment(a_tLocatData.eDirection, a_tAGV_Data.tCarInfo, a_tAGV_Data.tMotor);
            }
            
            if (bMatched)
            {   // 已對齊 開始取貨
                // step 0: 升到貨物的高度

                // step 1: 伸貨叉

                // step 2: 舉貨叉 (舉起 x mm ??)

                // step 3: 收貨叉

                // step 4: 降回最低點
            }
        }

        public static void UNLOAD(rtWarehousingInfo a_tLocatData, rtAGV_SensorData a_tSensorData, ref rtAGV_Data a_tAGV_Data)
        {
            bool bMatched = false;

            while (bMatched)
            {   // 一直執行到 對齊為止 >> 另有執行緒 隨時更新 a_tAGV_Data.tCarInfo
                bMatched = rtMotorCtrl.CarAngleAlignment(a_tLocatData.eDirection, a_tAGV_Data.tCarInfo, a_tAGV_Data.tMotor);
            }

            if (bMatched)
            {   // 已對齊 開始卸貨
                // step 0: 升到貨物的高度+ x mm ?? 

                // step 1: 伸貨叉

                // step 2: 降貨叉 (下降 x mm ??)

                // step 3: 收貨叉

                // step 4: 降回最低點
            }
        }

#if CHUNK
        //2016/01/25 Lee Add
        public bool LOADbMatched = false;//是否完成接近貨物
        public int LOADCount = 0;//取貨進度計數
        public bool UnLOADbMatched = false;//是否完成接近貨物
        public int UnLOADCount = 0;//放貨進度計數
        public enum ForkLODAStatus { SetHeight, Forth, Backward, Pickup, Finished };   //堆高機貨叉狀態宣告
        public enum ForkUnLODAStatus { SetHeight, Forth, Backward, PutDown, Finished };   //堆高機貨叉狀態宣告
        public ForkLODAStatus ForkLoStatus = new ForkLODAStatus(); //堆高機取貨貨叉運形狀態
        public ForkUnLODAStatus ForkUnLoStatus = new ForkUnLODAStatus(); //堆高機放貨貨叉運形狀態

        public static void LOAD_FotTest(rtWarehousingInfo a_tLocatData, rtMotorCtrl a_tMotorData, rtAGV_Control a_tAGV_Class, ref rtAGV_Data a_tAGV_Data, ObjectPLC_KV obj_PLC)
        {
            if (!a_tAGV_Class.LOADbMatched)
            {
                a_tAGV_Class.LOADbMatched = rtMotorCtrl.CarAngleAlignment(a_tLocatData.eDirection, a_tAGV_Data.tCarInfo, a_tMotorData);
            }
            else
            {
                if (a_tAGV_Class.LOADCount == 0)
                {
                    // step 0: 升到貨物的高度+ x mm ?? 
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "20", Convert.ToInt16(a_tLocatData.eHeight));
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "007", 1);
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "010", 0);
                    a_tAGV_Class.ForkLoStatus = ForkLODAStatus.SetHeight;
                }
                if (a_tAGV_Class.ForkLoStatus == ForkLODAStatus.SetHeight)
                {
                    int ForkHeightStatus = obj_PLC.doReadDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "010");
                    if (ForkHeightStatus != 0)
                    {
                        // step 1: 伸貨叉
                        a_tAGV_Class.ForkLoStatus = ForkLODAStatus.Forth;
                        a_tAGV_Class.LOADCount = 100000;
                        obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "1", 3250);//貨插-往前
                    }
                }
                if (a_tAGV_Class.LOADCount == 104500)
                {
                    // step 2: 抬起貨物
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "1", 2000);//貨插-停
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "20", Convert.ToInt16(a_tLocatData.eHeight) + 10);
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "007", 1);
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "010", 0);
                    a_tAGV_Class.ForkLoStatus = ForkLODAStatus.Pickup;
                }
                if (a_tAGV_Class.ForkLoStatus == ForkLODAStatus.Pickup)
                {
                    int ForkHeightStatus = obj_PLC.doReadDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "010");
                    if (ForkHeightStatus != 0)
                    {
                        // step 3: 收貨叉
                        a_tAGV_Class.ForkLoStatus = ForkLODAStatus.Backward;
                        a_tAGV_Class.LOADCount = 200000;
                        obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "1", 750);//貨插-往後
                    }
                }
                if (a_tAGV_Class.LOADCount == 204500)
                {
                    // step 4:結束
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "1", 2000);//貨插-停
                    a_tAGV_Class.ForkLoStatus = ForkLODAStatus.Finished;
                }
                a_tAGV_Class.LOADCount += 50;
                //Console.WriteLine(a_tAGV_Class.LOADCount);
            }
        }

        public static void UNLOAD_ForTest(rtWarehousingInfo a_tLocatData, rtMotorCtrl a_tMotorData, rtAGV_Control a_tAGV_Class, ref rtAGV_Data a_tAGV_Data, ObjectPLC_KV obj_PLC)
        {
            if (!a_tAGV_Class.UnLOADbMatched)
            {   // 一直執行到 對齊為止 >> 另有執行緒 隨時更新 a_tAGV_Data.tCurrentInfo
                a_tAGV_Class.UnLOADbMatched = rtMotorCtrl.CarAngleAlignment(a_tLocatData.eDirection, a_tAGV_Data.tCarInfo, a_tMotorData);
            }
            else
            {
                if (a_tAGV_Class.UnLOADCount == 0)
                {
                    // step 0: 升到貨物的高度+ x mm ?? 
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "20", Convert.ToInt16(a_tLocatData.eHeight) + 10);
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "007", 1);
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "010", 0);
                    a_tAGV_Class.ForkUnLoStatus = ForkUnLODAStatus.SetHeight;
                }
                if (a_tAGV_Class.ForkUnLoStatus == ForkUnLODAStatus.SetHeight)
                {
                    int ForkHeightStatus = obj_PLC.doReadDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "010");
                    if (ForkHeightStatus != 0)
                    {
                        // step 1: 伸貨叉
                        a_tAGV_Class.ForkUnLoStatus = ForkUnLODAStatus.Forth;
                        a_tAGV_Class.UnLOADCount = 100000;
                        obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "1", 3250);//貨插-往前
                    }
                }
                if (a_tAGV_Class.UnLOADCount == 104500)
                {
                    // step 2: 降貨叉 (下降 x mm ??)
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "1", 2000);//貨插-停
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "20", Convert.ToInt16(a_tLocatData.eHeight));
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "007", 1);
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "010", 0);
                    a_tAGV_Class.ForkUnLoStatus = ForkUnLODAStatus.PutDown;
                }
                if (a_tAGV_Class.ForkUnLoStatus == ForkUnLODAStatus.PutDown)
                {
                    int ForkHeightStatus = obj_PLC.doReadDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "010");
                    if (ForkHeightStatus != 0)
                    {
                        // step 3: 收貨叉
                        a_tAGV_Class.ForkUnLoStatus = ForkUnLODAStatus.Backward;
                        a_tAGV_Class.UnLOADCount = 200000;
                        obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "1", 750);//貨插-往後
                    }
                }
                if (a_tAGV_Class.UnLOADCount == 204500)
                {
                    // step 4:結束
                    obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "1", 2000);//貨插-停
                    a_tAGV_Class.ForkUnLoStatus = ForkUnLODAStatus.Finished;
                }
                a_tAGV_Class.UnLOADCount += 50;
            }
        }
#endif
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