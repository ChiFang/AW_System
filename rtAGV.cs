
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
        public enum rtAGVCmd {STOP = 0, DELIVER = 1};

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
           
        }

        public void Initailize()
        {

        }

        public void ExecuteCmd(uint a_ulAGV_Cmd, rtAGV_Control a_tAGVInfo)
		{
            uint ulAction = 0;

            if (a_ulAGV_Cmd != a_tAGVInfo.ulAGV_Cmd)
            { // 不一樣 >> 新命令
                switch (ulAction)
                {
                    case (uint)rtAGVCmd.DELIVER:
                        Deliver();
                        break;
                    case (uint)rtAGVCmd.STOP:
                        EmergencyStop();
                        break;
                    default:
                        // show error
                        break;
                }
            }
		}

        public static void Deliver()
        {
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