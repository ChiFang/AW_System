
using System;

namespace rtAGV_Common
{
    public struct rtVector
    {
        public double eX;
        public double eY;

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

    public struct rtWarehousingInfo
    {
        /** \brief Node Id */
        public NodeId tNodeId;

        /** \brief Warehousing position */
        public rtVector tCoordinate;

        /** \brief Warehousing Height */
        public double eHeight;

        /** \brief Warehousing Direction: Radius*/
        public double eDirection;
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
        /** \brief node number in local: 每一區節點個數，array長度代表區域個數 */
        public int[] alNodeNumLocal;

        /** \brief node in global 區域節點*/
        public rtAGV_MAP_node[] atNodeGlobal;

        /** \brief node in local 每一區的節點 */
        public rtAGV_MAP_node[][] atNodeLocal;

        /** \brief Map table in global : 區域間的路徑權重表 算最短路徑用*/
        public double[] aePathTableGlobal;

        /** \brief Map table in local : 每區的路徑權重表 算最短路徑用*/
        public double[][] aePathTableLocal;
    }
}