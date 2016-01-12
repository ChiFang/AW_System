
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
    }
	
    public class rtAGV_Control
    {
        public enum rtAGVCmd {STOP = 0, DELIVER = 1};

        /** \brief AGV Configure */
        public rtAGV_CFG tAGV_Cfg;
		
		/** \brief AGV Status */
		public byte ucAGV_Status = 0;
		
		/** \brief AGV Command */
		public uint ulAGV_Cmd = 0;
		
		/** \brief AGV data */
		public rtAGV_Data tAGV_Data;
		
		/** \brief Motor data */
		public rtMotorCtrl tMotor;
		
        public rtAGV_Control()
        {
           
        }
		
		public void ExecuteCmd()
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