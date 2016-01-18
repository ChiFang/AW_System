
using System;

using PLC_Control;

namespace rtAGV_Sys
{
	public struct rtCarCFG
    {
        public int lLength;
    }

    public struct rtWarehousingInfo
    {
        /** \brief Warehousing position */
        public rtVector tCoordinate;

        /** \brief Warehousing Height */
        public double eHeight;

        /** \brief Warehousing Direction: Radius*/
        public double eDirection;
    }

    public struct rtAGV_CFG
    {
        /** \brief Map between region */
        public double[] aePathRegion;

        /** \brief Map between position in local */
        public double[] aePathFactory;

        /** \brief Goods Infomation of each one */
        public rtWarehousingInfo[][] atWarehousingInfo;

        /** \brief Spec of AGV car */
        public rtCarCFG tCarData;
    }
	
	public struct rtAGV_Data
    {
        public rtCarData tCurrentInfo;

        public bool bEmergencyFlag;
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

        /** \brief InOut Data: AGV Status */
        public byte ucAGV_Status = 0;

        /** \brief Output Data: AGV data */
        public rtAGV_Data tAGV_Data;

        /** \brief Output Data: Motor data */
        public rtMotorCtrl tMotor;
		
        public rtAGV_Control()
        {
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
                        Deliver(ulAGV_Cmd, this);
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

        public static void Reset(rtAGV_Control tAGV)
        {

        }

        public static void Continue()
        {

        }

        public static void Pause()
        {

        }

        public static void Deliver(uint a_ulAGV_Cmd, rtAGV_Control a_tAGV_Class)
        {
            short wSrcRegion = 0, wSrcPosition = 0;
            short wDestRegion = 0, wDestPosition = 0;

           
            // step 0: extract element from command
            wSrcRegion = (short)((a_ulAGV_Cmd >> SRC_REGION) & MASK);
            wSrcPosition = (short)((a_ulAGV_Cmd >> SRC_POSITION) & MASK);
            wDestRegion = (short)((a_ulAGV_Cmd >> DEST_REGION) & MASK);
            wDestPosition = (short)((a_ulAGV_Cmd >> DEST_POSITION) & MASK);

            // step 1: move to goods position
            Navigate();

            // step 2:Load goods
            LOAD(a_tAGV_Class.tAGV_Cfg.atWarehousingInfo[wSrcRegion][wSrcPosition], a_tAGV_Class.tMotor, ref a_tAGV_Class.tAGV_Data);

            // step 3:move to destination
            Navigate();

            // step 4:Unload goods
            UNLOAD(a_tAGV_Class.tAGV_Cfg.atWarehousingInfo[wDestRegion][wDestPosition], a_tAGV_Class.tMotor, ref a_tAGV_Class.tAGV_Data);

            // step 5:stand by at assign position (TBD)
            StandBy();
        }

        public static void Navigate()
        {
            
        }

        public static void LOAD(rtWarehousingInfo a_tLocatData, rtMotorCtrl a_tMotorData, ref rtAGV_Data a_tAGV_Data)
        {
            bool bMatched = false;

            while(bMatched)
            {   // 一直執行到 對齊為止 >> 另有執行緒 隨時更新 a_tAGV_Data.tCurrentInfo
                bMatched = rtMotorCtrl.CarAngleAlignment(a_tLocatData.eDirection, a_tAGV_Data.tCurrentInfo, a_tMotorData);
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

        public static void UNLOAD(rtWarehousingInfo a_tLocatData, rtMotorCtrl a_tMotorData, ref rtAGV_Data a_tAGV_Data)
        {
            bool bMatched = false;

            while (bMatched)
            {   // 一直執行到 對齊為止 >> 另有執行緒 隨時更新 a_tAGV_Data.tCurrentInfo
                bMatched = rtMotorCtrl.CarAngleAlignment(a_tLocatData.eDirection, a_tAGV_Data.tCurrentInfo, a_tMotorData);
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

        public void SentData()
        {

        }
    }
}