
using System;

using rtAGV_Common;

namespace rtAGV_Navigate
{
    public class rtPathPlanning
    {
        public static void rtAGV_PathPlanning(
            rtAGV_MAP a_tMap, rtWarehousingInfo[][] a_atWarehousingInfo, ROI[] a_atRegionCfg, 
            ref rtPath_Info[] a_atPathInfo, ref rtCarData a_tCarInfo, rtWarehousingInfo a_tDestData, ROI[] a_atObstacle)
        {
            NodeId tNodeId;
            int lRegionIndex = 0;

            int[] alPathofRegion = new int[0];

            tNodeId = rtAGV_FindCurrentNode();

            if (a_tDestData.tNodeId.lRegion == tNodeId.lRegion)
            {   // different region
                FindPathofRegion();

                if(alPathofRegion.Length == 0)
                {
                    // show error
                }

                while (lRegionIndex < alPathofRegion.Length)
                {
                    FindPathNode2NextRegion();
                    lRegionIndex++;
                }
            }

            // already in same region
            rtAGV_FindPathNode2Dest();
        }

        public static NodeId rtAGV_FindCurrentNode()
        {
            NodeId tNodeId = new NodeId();

            return tNodeId;
        }

        public static void rtAGV_FindPathNode2Dest()
        {

        }

        public static void FindPathNode2NextRegion()
        {

        }

        public static void FindPathofRegion()
        {

        }
    }
}