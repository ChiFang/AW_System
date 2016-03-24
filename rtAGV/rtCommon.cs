
using System;

namespace rtAGV_Common
{
    public struct rtVector
    {
        public double eX;
        public double eY;

        public void Init()
        {
            eX = 0;
            eY = 0;
        }

        public rtVector(double a_eX, double a_eY)
        {
            eX = a_eX;
            eY = a_eY;
        }

        public void Set(double a_eX, double a_eY)
        {
            eX = a_eX;
            eY = a_eY;
        }

    }

    public struct rtPath_Info
    {
        /** \brief source position */
        public rtVector tSrc;

        /** \brief destination position */
        public rtVector tDest;

        /** \brief status of this segment */
        public byte ucStatus;

        /** \brief turn ytpe of this segment */
        public byte ucTurnType;

        public static rtPath_Info[] InitSet(int a_lPathLength)
        {
            int lCnt = 0;
            rtPath_Info[] atPathInfo = new rtPath_Info[a_lPathLength];

            for(lCnt = 0; lCnt< a_lPathLength - 1; lCnt++)
            {
                atPathInfo[lCnt].tSrc.Init();
                atPathInfo[lCnt].tDest.Init();
                atPathInfo[lCnt].ucStatus = 1;
                atPathInfo[lCnt].ucTurnType = 0;
            }
            atPathInfo[lCnt].tSrc.Init();
            atPathInfo[lCnt].tDest.Init();
            atPathInfo[lCnt].ucStatus = 1;
            atPathInfo[lCnt].ucTurnType = 2;

            return atPathInfo;
        }
    }

    public struct rtCarData
    {
        /** \brief car position */
        public rtVector tPosition;

        /** \brief Motor position */
        public rtVector tMotorPosition;

        /** \brief car left Tire position */
        public rtVector tCarTirepositionL;

        /** \brief car right Tire position */
        public rtVector tCarTirepositionR;

        /** \brief car angle (direction) */
        public double eAngle;

        /** \brief car left Tire speed */
        public double eCarTireSpeedLeft;

        /** \brief car right Tire speed */
        public double eCarTireSpeedRight;

        /** \brief Motor(or wheel) angle (direction) */
        public double eWheelAngle;

        public void Init()
        {
            tPosition.Init();
            tMotorPosition.Init();
            tCarTirepositionL.Init();
            tCarTirepositionR.Init();
            eAngle = 0;
            eCarTireSpeedLeft = 0;
            eCarTireSpeedRight = 0;
            eWheelAngle = 0;
        }
    }

    public struct ROI
    {
        /** \brief Start X */
        public int lStartX;

        /** \brief Start X */
        public int lStartY;

        /** \brief width */
        public int lwidth;

        /** \brief height */
        public int lheight;
    }

    public struct NodeId
    {
        /** \brief region index */
        public int lRegion;

        /** \brief node index */
        public int lIndex;
    }

    public struct WarehousPos
    {
        /** \brief region index */
        public int lRegion;

        /** \brief node index */
        public int lIndex;

        /** \brief Warehousing Direction: Radius*/
        public double eDirection;
    }

    public struct rtWarehousingInfo
    {
        /** \brief Node Id: 代表他連結到MAP中哪一個結點 */
        public NodeId tNodeId;

        /** \brief Warehousing position */
        public rtVector tCoordinate;

        /** \brief Warehousing Height */
        public double eHeight;

        /** \brief Warehousing Direction: Radius*/
        public double eDirection;

        public double DistanceDepth;
    }

    public struct rtAGV_MAP_node
    {
        /** \brief Node Id */
        public NodeId tNodeId;

        /** \brief node position */
        public rtVector tCoordinate;

        /** \brief Link of region: length > 0 表示此點可以通往其它 region */
        public int[] alLinkofRegion;

        /** \brief index of linked region  */
        public int[] alIndexfRegion;
    }

    public struct rtAGV_MAP
    {
        /** \brief Region number: 區域個數 */
        public int alRegionNum;

        /** \brief node number in local: 每一區節點個數，array長度代表區域個數 */
        public int[] alNodeNumLocal;

        /** \brief node in global 區域節點*/
        public rtAGV_MAP_node[] atNodeGlobal;

        /** \brief node in local 每一區的節點 */
        public rtAGV_MAP_node[][] atNodeLocal;

        /** \brief Map table in global : 區域間的路徑權重表 算最短路徑用*/
        public int[] alPathTableGlobal;

        /** \brief Map table in local : 每區的路徑權重表 算最短路徑用*/
        public int[][] alPathTableLocal;

        public void Init()
        {
            alRegionNum = 0;
            alNodeNumLocal = new int[alRegionNum];
            atNodeGlobal = new rtAGV_MAP_node[alRegionNum];
            atNodeLocal = new rtAGV_MAP_node[alRegionNum][];
            alPathTableGlobal = new int[alRegionNum];
            alPathTableLocal = new int[alRegionNum][];
        }
    }
}