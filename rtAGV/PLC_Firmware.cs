using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using PLC_Control;
using rtAGV_Sys;
using rtAGV_Common;

// #define FW_CTRL

#if FW_CTRL
namespace PLC_Firmware
{
    public class PLC_FW
    {
        /** \brief Object PLC*/
        public static ObjectPLC_KV obj_PLC;

        /** \brief 取貨進度計數 */
        public static int LOADCount = 0;

        /** \brief 放貨進度計數 */
        public static int UnLOADCount = 0;

        /** \brief 堆高機取貨貨叉運形狀態 */
        public static rtForkCtrl.ForkLODAStatus ForkLoStatus = new rtForkCtrl.ForkLODAStatus();

        /** \brief 堆高機放貨貨叉運形狀態 */
        public static rtForkCtrl.ForkUnLODAStatus ForkUnLoStatus = new rtForkCtrl.ForkUnLODAStatus(); 

        public static void SetForkHeight(int height)
        {
            //貨插高度設定
            obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "20", height);
            obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "007", 1);
            obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "010", 0);
        }

        public static int CheckDoneForkHeight()
        {
            //確認貨叉是否到達高度
            return obj_PLC.doReadDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_MR, "010");
        }

        public static void SetForkForth()
        {
            //貨插-往前
            obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "1", 3250);
        }

        public static void SetForkBack()
        {
            //貨插-往後
            obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "1", 750);
        }

        public static void SetForkStop()
        {
            //貨插-停
            obj_PLC.doWriteDevice(DATABUILDERAXLibLB.DBPlcDevice.DKV3000_DM, "1", 2000);
        }

        public static void InitParameter()
        {
            LOADCount = 0;
            UnLOADCount = 0;
            ForkLoStatus = new rtForkCtrl.ForkLODAStatus();
            ForkUnLoStatus = new rtForkCtrl.ForkUnLODAStatus();
        }

        public static bool LoadFWFunc(double height , double distance)
        {
            if (LOADCount == 0)
            {
                // step 0: 升到貨物的高度+ x mm ?? 
                PLC_FW.SetForkHeight((int)height);
                ForkLoStatus = rtForkCtrl.ForkLODAStatus.SetHeight;
            }
            if (ForkLoStatus == rtForkCtrl.ForkLODAStatus.SetHeight)
            {
                int ForkHeightStatus = PLC_FW.CheckDoneForkHeight();
                if (ForkHeightStatus != 0)
                {
                    // step 1: 伸貨叉
                    ForkLoStatus = rtForkCtrl.ForkLODAStatus.Forth;
                    LOADCount = 100000;
                    PLC_FW.SetForkForth();
                }
            }
            if (LOADCount == 104500)
            {
                // step 2: 抬起貨物
                PLC_FW.SetForkStop();
                PLC_FW.SetForkHeight((int)height + 10);
                ForkLoStatus = rtForkCtrl.ForkLODAStatus.Pickup;
            }
            if (ForkLoStatus == rtForkCtrl.ForkLODAStatus.Pickup)
            {
                int ForkHeightStatus = PLC_FW.CheckDoneForkHeight();
                if (ForkHeightStatus != 0)
                {
                    // step 3: 收貨叉
                    ForkLoStatus = rtForkCtrl.ForkLODAStatus.Backward;
                    LOADCount = 200000;
                    PLC_FW.SetForkBack();
                }
            }
            if (LOADCount == 204500)
            {
                // step 4:結束
                PLC_FW.SetForkStop();
                ForkLoStatus = rtForkCtrl.ForkLODAStatus.Finished;
                return true;
            }
            LOADCount += 50;
            return false;
            //Console.WriteLine(a_tAGV_Class.LOADCount);
        }

        public static bool UnLoadFWFunc( double height, double distance)
        {
            if (UnLOADCount == 0)
            {
                // step 0: 升到貨物的高度+ x mm ?? 
                PLC_FW.SetForkHeight((int)height + 10);
                ForkUnLoStatus = rtForkCtrl.ForkUnLODAStatus.SetHeight;
            }
            if (ForkUnLoStatus == rtForkCtrl.ForkUnLODAStatus.SetHeight)
            {
                int ForkHeightStatus = PLC_FW.CheckDoneForkHeight();
                if (ForkHeightStatus != 0)
                {
                    // step 1: 伸貨叉
                    ForkUnLoStatus = rtForkCtrl.ForkUnLODAStatus.Forth;
                    UnLOADCount = 100000;
                    PLC_FW.SetForkForth();
                }
            }
            if (UnLOADCount == 104500)
            {
                // step 2: 降貨叉 (下降 x mm ??)
                PLC_FW.SetForkStop();
                PLC_FW.SetForkHeight((int)height);
                ForkUnLoStatus = rtForkCtrl.ForkUnLODAStatus.PutDown;
            }
            if (ForkUnLoStatus == rtForkCtrl.ForkUnLODAStatus.PutDown)
            {
                int ForkHeightStatus = PLC_FW.CheckDoneForkHeight();
                if (ForkHeightStatus != 0)
                {
                    // step 3: 收貨叉
                    ForkUnLoStatus = rtForkCtrl.ForkUnLODAStatus.Backward;
                    UnLOADCount = 200000;
                    PLC_FW.SetForkBack();
                }
            }
            if (UnLOADCount == 204500)
            {
                // step 4:結束
                PLC_FW.SetForkStop();
                ForkUnLoStatus = rtForkCtrl.ForkUnLODAStatus.Finished;
                return true;
            }
            UnLOADCount += 50;
            return false;
        }
        
    }
}

#endif