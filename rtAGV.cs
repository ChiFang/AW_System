
using System;

using PLC_Control;

namespace rtAGV_Sys
{
	public struct rtCarCFG
    {
        public int eLength;
    }
	
	public struct rtAGV_CFG
    {
        public double[] aePathRegion;
		public double[] aePathFactory;
		public rtCarCFG tCarData;
    }
	
	public struct rtAGV_Data
    {
        public rtCarData tCurrentInfo;

        public bool bEmergencyFlag;
    }
	
    public class rtAGV_Control
    {
        public enum rtAGVCmd {STOP = 0, DELIVER = 1, CONTINUE = 2, PAUSE = 3, RESET = 4};

        public enum rtAGVStatus { NON_INITAILIZE = 0, BUSY = 1, PAUSE = 2, STANDBY = 3, STOP = 4, EMERGENCY_STOP = 5, ERROR_NO_CFG = 6};

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

        public static void Reset(rtAGV_Control tAGV)
        {

        }

        public static void Continue()
        {

        }

        public static void Pause()
        {

        }

        public static void Deliver()
        {
            // step 0: extract element from command

            // step 1: move to goods position
            Navigate();

            // step 2:Load goods
            LOAD();

            // step 3:move to destination
            Navigate();

            // step 4:Unload goods
            UNLOAD();

            // step 5:stand by at assign position (TBD)
            StandBy();
        }

        public static void Navigate()
        {
            
        }

        public static void LOAD()
        {

        }

        public static void UNLOAD()
        {

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