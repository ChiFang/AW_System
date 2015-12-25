using System;

namespace PLC_Control
{

    public struct rtVector
    {
        public double eX;
        public double eY;
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

    public struct rtPID_Coefficient
    {
        /** \brief Kp */
        public double eKp;

        /** \brief Ki */
        public double eKi;

        /** \brief Kd */
        public double eKd;
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

    public class rtVectorOP
    {
        public static double GetLength(rtVector a_tIn)
        {
            double eOut = 0;
            eOut = Math.Sqrt(a_tIn.eX* a_tIn.eX + a_tIn.eY* a_tIn.eY);
            return eOut;
        }

        public static double Dot(rtVector a_tV1, rtVector a_tV2)
        {
            double eOut = 0;

            eOut = a_tV1.eX* a_tV2.eX + a_tV1.eY* a_tV2.eY;
            return eOut;
        }

        public static double GetTheta(rtVector a_tV1, rtVector a_tV2)
        {
            double eTheta = 0;

            eTheta = Dot(a_tV1, a_tV2);
            eTheta /= GetLength(a_tV1) * GetLength(a_tV2);
            eTheta = Math.Acos(eTheta) * 180.0 / Math.PI;
            return eTheta;
        }

        public static rtVector Rotate(rtVector a_tPoint, rtVector a_tCenter, double a_eTheta)
        { // 角度單位是徑度 甭轉換
            rtVector tResult = new rtVector();
            rtVector tTmp, tTmp1;

            tTmp.eX = a_tPoint.eX - a_tCenter.eX;
            tTmp.eY = a_tPoint.eY - a_tCenter.eY;

            tTmp1.eX = Math.Cos(a_eTheta) * tTmp.eX - Math.Sin(a_eTheta) * tTmp.eY;
            tTmp1.eY = Math.Sin(a_eTheta) * tTmp.eX - Math.Cos(a_eTheta) * tTmp.eY;

            tResult.eX = tTmp1.eX + a_tCenter.eX;
            tResult.eY = tTmp1.eY + a_tCenter.eY;

            return tResult;
        }

        /**
        \brief Get Distance of two points
        \param a_tP1 [IN] vector 1
        \param a_tP2 [IN] vector 2
        \return Distance
        */
        public static double GetDistance(rtVector a_tP1, rtVector a_tP2)
        {
            double eDistance = 0;
            double eGapX = 0, eGapY = 0;

            eGapX = a_tP2.eX - a_tP1.eX;
            eGapY = a_tP2.eY - a_tP1.eY;
            eDistance = Math.Sqrt(eGapX* eGapX + eGapY* eGapY);

            return eDistance;
        }

    } 


    public class rtMotorCtrl
    {
        public enum rtNavigateStatus { UNDO = 0, DONE = 1 };

        public enum rtTurnType_Simple { ERROR = 0, TURN_RIGHT = 1, TURN_LEFT = -1 };

        public enum rtStatus { STRAIGHT = 1, TURN = 2, DONE = 0 };

        public enum rtTurnType { SIMPLE = 0, SMOOTH = 1, ARRIVE = 2 };

        /** \brief PID Power Coeffient */
        public rtPID_Coefficient tPID_PowerCoe;

        /** \brief PID angle Coeffient */
        public rtPID_Coefficient tPID_AngleCoe;

        /** \brief Finish Flag */
        public byte ucFinishFlag = 0;

        /** \brief path segment index */
        public int lPathNodeIndex = 0;

        /** \brief Rotation Radius of turn in smooth mode */
        public int lRotationRadius = 0;

        /** \brief Rotation distance of turn in smooth mode */
        public int lRotationDistance = 0;

        /** \brief rotation center of smooth turn */
        public rtVector tRotateCenter;

        /** \brief 旋轉方向: 1: 中心在左邊>> 向右轉  -1:中心在右邊>> 向左轉  0: 出錯*/
        public int lTurnDirection;

        /** \brief motor power */
        public int lMotorPower;

        /** \brief motor torsion */
        public int lMotorTorsion;

        /** \brief motor angle */
        public int lMotorAngle;

        /** \brief Car Angle Weighting */
        public double eCarAngleWeighting;

        /** \brief false: 沒車速資訊  true: 有車速資訊 */
        public const bool CAR_SPEED = false;

        /** \brief 系統頻率 8Hz = 0.125s 1次 */
        public const double FREQUENCY = 8;

        /** \brief angle threshold: 判斷是否走過頭用的 */
        public const double ANGLE_TH = 90;

        /** \brief 可以在原地打轉的角度 */
        public const int ANGLE_ROTATION = 90;

        /** \brief 馬達(驅動輪胎)在這角度以內以直行計算 */
        public const int ANGLE_TH_MOTION_PREDICT = 5;

        /** \brief distance threshold of simple mode: 判斷是否到達定點 開始準備轉向動作或停止 */
        public const double DISTANCE_ERROR_SIMPLE = 40;

        /** \brief distance threshold of smooth mode: 判斷是否到達定點 開始準備轉向動作 */
        public const int DISTANCE_ERROR_SMOOTH = 1500;

        /** \brief 判斷是否做完轉彎的動作 */
        public const double THETA_ERROR_TURN = 3;

        /** \brief 轉向時馬達的power */
        public const int TURN_POWER = 50;

        /** \brief max power of motor */
        public const int MAX_POWER = 255;

        /** \brief min power of motor */
        public const int MIN_POWER = 18;

        /** \brief max amgle value of motor */
        public const int MAX_ANGLE_OFFSET_MOTOR = 70;

        /** \brief Error Sum for motor angle modify */
        public double eAngleErrorSum = 0;

        /** \brief last Error for motor angle modify */
        public double eAngleErrorLast = 0;


        public static int test123(int a, int b)
        {
            rtVector[] tttt = new rtVector[2];

            tttt[1].eX = 1;

            return (int)(a * b* ANGLE_TH);
        }

        public static int test456(int a, rtVector test)
        {
            test.eX = 1;
            test.eY = 4;
            rtVectorOP.GetLength(test);
            return 1;
        }


        public rtMotorCtrl()
        {
            eAngleErrorSum = 0;
            eAngleErrorLast = 0;
            lPathNodeIndex = 0;
            lMotorPower = 0;
            lMotorTorsion = 0;
            lMotorAngle = 0;

            tRotateCenter.eX = 0;
            tRotateCenter.eY = 0;

            ucFinishFlag = (byte)rtNavigateStatus.UNDO;
            lRotationRadius = 0;
            lRotationDistance = DISTANCE_ERROR_SMOOTH;
            eCarAngleWeighting = 1;

            tPID_PowerCoe.eKp = 0;
            tPID_PowerCoe.eKi = 0;
            tPID_PowerCoe.eKd = 0;

            tPID_AngleCoe.eKp = 0;
            tPID_AngleCoe.eKi = 0;
            tPID_AngleCoe.eKd = 0;
    }

        /**
        \brief initail path infomation attay
        \param a_atPathInfo [IN] path infomation attay
        \return void
        */
        public static void Init_rtPath_Info(rtPath_Info[] a_atPathInfo)
        {
            int lPathIndex = 0;
            for (lPathIndex = 0; lPathIndex < a_atPathInfo.Length-2; lPathIndex++)
            {
                a_atPathInfo[lPathIndex].ucStatus = (byte)rtStatus.STRAIGHT;
                a_atPathInfo[lPathIndex].ucTurnType = (byte)rtTurnType.SMOOTH;
                a_atPathInfo[lPathIndex].tSrc.eX = 0;
                a_atPathInfo[lPathIndex].tDest.eX = 0;
            }

            a_atPathInfo[lPathIndex].ucStatus = (byte)rtStatus.STRAIGHT;
            a_atPathInfo[lPathIndex].ucTurnType = (byte)rtTurnType.ARRIVE;
            a_atPathInfo[lPathIndex].tSrc.eX = 0;
            a_atPathInfo[lPathIndex].tDest.eX = 0;
        }

        public static void MotorPower_SetParamsLinear(int a_lErrorBoundL, int a_lErrorBoundH, double[] a_aeMotor_Params)
        {
            a_aeMotor_Params[0] = (double)(MAX_POWER- MIN_POWER) / (a_lErrorBoundH - a_lErrorBoundL);
        }


        /**
        \brief calculate error in straight mode of power control
        \param a_atPathInfo [IN] path infomation attay
        \param a_tCurrentInfo [IN] Current Info of car
        \param a_tMotorData [IN] motor data
        \return error
        */
        public static double MotorPower_StraightErrorCal(
            rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo,
            ref rtMotorCtrl a_tMotorData
        )
        {
            double eErrorCurrent = 0, eAngle = 0, eTheta = 0;
            rtVector tV_C2D; // current point to destination
            rtVector tV_S2D; // source point to destination

            eAngle = a_tCurrentInfo.eAngle;
            tV_S2D.eX = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eX - a_atPathInfo[a_tMotorData.lPathNodeIndex].tSrc.eX;
            tV_S2D.eY = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eY - a_atPathInfo[a_tMotorData.lPathNodeIndex].tSrc.eY;
            tV_C2D.eX = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eX - a_tCurrentInfo.tPosition.eX;
            tV_C2D.eY = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eY - a_tCurrentInfo.tPosition.eY;
            eErrorCurrent = rtVectorOP.GetLength(tV_C2D);

            eTheta = rtVectorOP.GetTheta(tV_S2D, tV_C2D);

            // Motor power = function(Error)
            a_tMotorData.lMotorPower = (int)(a_tMotorData.tPID_PowerCoe.eKp * eErrorCurrent) + MIN_POWER;

            // 判斷是否已超終點
            if (eTheta >= ANGLE_TH)
            { // 超過終點 >>　必須反向行走
                a_tMotorData.lMotorPower = -a_tMotorData.lMotorPower;
            }

            return eErrorCurrent;
        }

        public static bool CarDirectionVerify(rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo, rtMotorCtrl a_tMotorData)
        {
            double eErrorCurrent = 0, eAngle = 0, eTheta = 0;

            rtVector tV_C2D; // current point to destination
            rtVector tV_Car; // car current direction

            eAngle = a_tCurrentInfo.eAngle;
            tV_C2D.eX = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eX - a_tCurrentInfo.tPosition.eX;
            tV_C2D.eY = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eY - a_tCurrentInfo.tPosition.eY;
            eErrorCurrent = rtVectorOP.GetLength(tV_C2D);

            tV_Car.eX = Math.Cos(eAngle * Math.PI / 180);
            tV_Car.eY = Math.Sin(eAngle * Math.PI / 180);

            eTheta = rtVectorOP.GetTheta(tV_Car, tV_C2D);

            if (eTheta >= ANGLE_TH)
            { // 車子方向跟行走方向差太大
                return true;
            }
            else
            { // 車子方向跟行走方向沒有問題
                return false;
            }  
        }

        public static double MotorPower_TurnErrorCal(
            byte a_ucTurnType, rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo, ref rtMotorCtrl a_tMotorData)
        {
            double eErrorCurrent = 0, eAngle = 0, eTheta = 0;
            int lNextPathID = 0;
            rtVector tV_Car; // car current direction
            rtVector tV_NextS2D; // next src point to destination

            a_tMotorData.lMotorPower = TURN_POWER;

            lNextPathID = a_tMotorData.lPathNodeIndex + 1;
            eAngle = a_tCurrentInfo.eAngle;
            tV_Car.eX = Math.Cos(eAngle * Math.PI / 180);
            tV_Car.eY = Math.Sin(eAngle * Math.PI / 180);

            tV_NextS2D.eX = a_atPathInfo[lNextPathID].tDest.eX - a_atPathInfo[lNextPathID].tSrc.eX;
            tV_NextS2D.eY = a_atPathInfo[lNextPathID].tDest.eY - a_atPathInfo[lNextPathID].tSrc.eY;

            switch (a_ucTurnType)
            {
                case (byte)rtTurnType.SIMPLE:
                    eTheta = rtVectorOP.GetTheta(tV_Car, tV_NextS2D);
                    eErrorCurrent = eTheta;
                    break;
                case (byte)rtTurnType.SMOOTH:
                    eTheta = rtVectorOP.GetTheta(tV_Car, tV_NextS2D);
                    eErrorCurrent = eTheta;
                    break;
                default:
                    // show error msg
                    break;
            }

            return eErrorCurrent;
        }

        public static double MotorPower_Ctrl(
            rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo,
            ref rtMotorCtrl a_tMotorData)
        {
            double eErrorCurrent = 0;
            bool bOutFlag = false;

            switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus)
            {
                // 直走狀態
                case (byte)rtStatus.STRAIGHT:
                    eErrorCurrent = MotorPower_StraightErrorCal(a_atPathInfo, a_tCurrentInfo, ref a_tMotorData);
                    switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType)
                    {
                        case (byte)rtTurnType.SIMPLE:
                            if(eErrorCurrent < DISTANCE_ERROR_SIMPLE)
                            {
                                a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.TURN;
                            }
                            break;
                        case (byte)rtTurnType.SMOOTH:
                            if (eErrorCurrent < DISTANCE_ERROR_SMOOTH)
                            {
                                a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.TURN;
                            }
                            break;
                        case (byte)rtTurnType.ARRIVE:
                            if (eErrorCurrent < DISTANCE_ERROR_SIMPLE)
                            { // 到達最終目的地
                                a_tMotorData.lMotorPower = 0;
                                a_tMotorData.lMotorAngle = 0;
                                a_tMotorData.lMotorTorsion = 0;
                                a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.DONE;
                                a_tMotorData.ucFinishFlag = (byte)rtNavigateStatus.DONE;
                            }
                            break;
                        default:
                            // show error msg
                            break;
                    }
                    break;
                // 轉彎狀態
                case (byte)rtStatus.TURN:
                    eErrorCurrent = MotorPower_TurnErrorCal(
                                        a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType,
                                        a_atPathInfo, a_tCurrentInfo, ref a_tMotorData);

                    // 算出旋轉半徑中心座標
                    bOutFlag = MotorAngle_RotationCenterCal(a_atPathInfo, a_tCurrentInfo, ref a_tMotorData);

                    switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType)
                    {
                        case (byte)rtTurnType.SIMPLE:
                            if (eErrorCurrent < THETA_ERROR_TURN)
                            {
                                a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.DONE;
                                a_tMotorData.lPathNodeIndex++;
                            }
                            break;
                        case (byte)rtTurnType.SMOOTH:
                            if (eErrorCurrent < THETA_ERROR_TURN && bOutFlag == true)
                            {
                                a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.DONE;
                                a_tMotorData.lPathNodeIndex++;
                            }
                            break;
                        default:
                            // show error msg
                            break;
                    }
                    break;
                // 完成狀態: 似乎不會遇到這個 因為設定為 DONE 通常就 index ++了 >> TBD
                case (byte)rtStatus.DONE:
                    // show error msg
                    break;
                default:
                    // show error
                    break;
            }

            return eErrorCurrent;
        }

        

        public static double MotorAngle_StraightErrorCal(
            rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo,
            rtMotorCtrl a_tMotorData
            )
        {
            double eErrorCurrent = 0, eAngle = 0, eT = 0;
            double eSrcX, eSrcY, eDestX, eDestY;
            double eCurrentX, eCurrentY;
            rtVector tVS2D, tVlaw;


            eCurrentX = a_tCurrentInfo.tPosition.eX;
            eCurrentY = a_tCurrentInfo.tPosition.eY;
            eAngle = a_tCurrentInfo.eAngle;

            eSrcX = a_atPathInfo[a_tMotorData.lPathNodeIndex].tSrc.eX;
            eSrcY = a_atPathInfo[a_tMotorData.lPathNodeIndex].tSrc.eY;

            eDestX = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eX;
            eDestY = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eY;

            tVS2D.eX = eDestX - eSrcX;
            tVS2D.eY = eDestY - eSrcY;

            // 取右側的法向量
            tVlaw.eX = tVS2D.eY;
            tVlaw.eY = -tVS2D.eX;

            // 求交點  (current point 沿著法向量與路徑的交點)
            eT = eSrcX * tVS2D.eY - eSrcY * tVS2D.eX - eCurrentX * tVS2D.eY + eCurrentY * tVS2D.eX;
            eT /= tVlaw.eX * tVS2D.eY - tVlaw.eY * tVS2D.eX;

            // 算出目前座標到切線的距離當誤差 distance = (Vx*T)^2 + (Vy*T)^2
            eErrorCurrent = Math.Sqrt((tVlaw.eX * eT) * (tVlaw.eX * eT) + (tVlaw.eY * eT) * (tVlaw.eY * eT));

            if (eT > 0)
            { // left side >> tire need to turn left
                eErrorCurrent = -eErrorCurrent;
            }

            return eErrorCurrent;
        }

        public static bool MotorAngle_RotationCenterCal(
            rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo,
            ref rtMotorCtrl a_tMotorData
            )
        {
            int lPathIndex = 0;
            double eT = 0; // 比例
            double eLength = 0;
            double eThetaBoundaty = 0;
            double eThetaCurrent = 0;
            rtVector tSrc;
            rtVector tDest;
            rtVector tVd2sCurrent;
            rtVector tVs2dNext;
            rtVector tVd2sCurrentLaw;
            rtVector tVs2dNextLaw;
            rtVector tCurrent;
            rtVector tNext;

            rtVector tCenter2SrcTurn;
            rtVector tCenter2DestTurn;
            rtVector tCenter2Current;

            lPathIndex = a_tMotorData.lPathNodeIndex;

            // set vector & point current
            tSrc.eX = a_atPathInfo[lPathIndex].tSrc.eX;
            tSrc.eY = a_atPathInfo[lPathIndex].tSrc.eY;
            tDest.eX = a_atPathInfo[lPathIndex].tDest.eX;
            tDest.eY = a_atPathInfo[lPathIndex].tDest.eY;
            tVd2sCurrent.eX = tSrc.eX - tDest.eX;
            tVd2sCurrent.eY = tSrc.eY - tDest.eY;

            // 取右側法向量
            tVd2sCurrentLaw.eX = tVd2sCurrent.eY;
            tVd2sCurrentLaw.eY = -tVd2sCurrent.eX;

            // 取轉彎起始點
            eLength = rtVectorOP.GetLength(tVd2sCurrent);
            eT = a_tMotorData.lRotationDistance / eLength;
            tCurrent.eX = tDest.eX + eT * tVd2sCurrent.eX;
            tCurrent.eY = tDest.eY + eT * tVd2sCurrent.eY;

            // set vector & point next
            tSrc.eX = a_atPathInfo[lPathIndex+1].tSrc.eX;
            tSrc.eY = a_atPathInfo[lPathIndex + 1].tSrc.eY;
            tDest.eX = a_atPathInfo[lPathIndex + 1].tDest.eX;
            tDest.eY = a_atPathInfo[lPathIndex + 1].tDest.eY;
            tVs2dNext.eX = tDest.eX - tSrc.eX;
            tVs2dNext.eY = tDest.eY - tSrc.eY;

            // 取右側法向量
            tVs2dNextLaw.eX = tVs2dNext.eY;
            tVs2dNextLaw.eY = -tVs2dNext.eX;

            // 取轉彎結束點
            eLength = rtVectorOP.GetLength(tVs2dNext);
            eT = a_tMotorData.lRotationDistance / eLength;
            tNext.eX = tSrc.eX + eT * tVs2dNext.eX;
            tNext.eY = tSrc.eY + eT * tVs2dNext.eY;


            // 取兩條線交點當旋轉中心座標
            eT = tNext.eX * tVs2dNextLaw.eY - tNext.eY * tVs2dNextLaw.eX - tCurrent.eX * tVs2dNextLaw.eY + tCurrent.eY * tVs2dNextLaw.eX;
            eT /= tVd2sCurrentLaw.eX * tVs2dNextLaw.eY - tVd2sCurrentLaw.eY * tVs2dNextLaw.eX;
            a_tMotorData.tRotateCenter.eX = tCurrent.eX + eT * tVd2sCurrentLaw.eX;
            a_tMotorData.tRotateCenter.eY = tCurrent.eY + eT * tVd2sCurrentLaw.eY;

            // 以下計算是否超出扇形區域
            tCenter2SrcTurn.eX = tCurrent.eX - a_tMotorData.tRotateCenter.eX;
            tCenter2SrcTurn.eY = tCurrent.eY - a_tMotorData.tRotateCenter.eY;
            tCenter2DestTurn.eX = tNext.eX - a_tMotorData.tRotateCenter.eX;
            tCenter2DestTurn.eY = tNext.eY - a_tMotorData.tRotateCenter.eY;
            tCenter2Current.eX = a_tCurrentInfo.tPosition.eX - a_tMotorData.tRotateCenter.eX;
            tCenter2Current.eY = a_tCurrentInfo.tPosition.eY - a_tMotorData.tRotateCenter.eY;

            eThetaBoundaty = rtVectorOP.GetTheta(tCenter2DestTurn, tCenter2SrcTurn);
            eThetaCurrent = rtVectorOP.GetTheta(tCenter2Current, tCenter2SrcTurn);

            // 計算旋轉半徑
            a_tMotorData.lRotationRadius = (int)Math.Round(rtVectorOP.GetDistance(tCurrent, a_tMotorData.tRotateCenter));
           

            // 判斷旋轉圓心在路徑向量的左邊還是右邊
            if (tVd2sCurrentLaw.eX != 0)
            {
                eT = (a_tMotorData.tRotateCenter.eX - tCurrent.eX) / tVd2sCurrentLaw.eX;
                
            }
            else if (tVd2sCurrentLaw.eY != 0)
            {
                eT = (a_tMotorData.tRotateCenter.eY - tCurrent.eY) / tVd2sCurrentLaw.eY;
            }
            else
            {
                // 不可能為0向量 show error
                eT = 0;
            }

            if(eT > 0)
            { // 點在左邊 >> 馬達向右轉
                a_tMotorData.lTurnDirection = (int)rtTurnType_Simple.TURN_RIGHT;
            }
            else if (eT < 0)
            { // 點在右邊 >> 馬達向左轉
                a_tMotorData.lTurnDirection = (int)rtTurnType_Simple.TURN_LEFT;
            }
            else
            {
                // 不可能為0向量 show error
                a_tMotorData.lTurnDirection = (int)rtTurnType_Simple.ERROR;
            }
                 

            if (eThetaCurrent > eThetaBoundaty)
            {
                return true;
            }
            else
            {
                return false;
            }

        }

        public static double MotorAngle_TurnErrorCal(
            rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo,
            ref rtMotorCtrl a_tMotorData
            )
        {
            double eErrorCurrent = 0;
            double eDistance = 0;
            byte ucTurnType = 0;
            bool bOutFlag = false;

            // 算出旋轉半徑中心座標
            bOutFlag = MotorAngle_RotationCenterCal(a_atPathInfo, a_tCurrentInfo, ref a_tMotorData);

            ucTurnType = a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType;

            switch (ucTurnType)
            {
                case (byte)rtTurnType.SIMPLE:
                    // Do nothing >> 原本不需要做事情 但需要知道是左90度還是右90度
                    break;
                case (byte)rtTurnType.SMOOTH:
                    eDistance = rtVectorOP.GetDistance(a_tCurrentInfo.tPosition, a_tMotorData.tRotateCenter);
                    eErrorCurrent = eDistance - a_tMotorData.lRotationRadius; // 可能會有錯誤 如果在內側非扇型區域 >> TBD
                    break;
                case (byte)rtTurnType.ARRIVE:
                    // show error msg
                    break;
                default:
                    // show error msg
                    break;
            }



            switch (a_tMotorData.lTurnDirection)
            {
                case (int)rtTurnType_Simple.TURN_RIGHT:
                    // Do nothing
                    break;
                case (int)rtTurnType_Simple.TURN_LEFT:
                    // inverse
                    eErrorCurrent = -eErrorCurrent;
                    break;
                case (int)rtTurnType_Simple.ERROR:
                    // show error msg
                    break;
                default:
                    // show error msg
                    break;
            }

            return eErrorCurrent;
        }

        public static double GetTheta_Path2Car(rtVector a_tV_Car, rtVector a_tV_S2D)
        {
            double Theta = 0;

            double ThetaTmp = 0;

            rtVector tVectorRotae; // car current direction after rotate

            Theta = rtVectorOP.GetTheta(a_tV_Car, a_tV_S2D);

            tVectorRotae.eX = a_tV_Car.eX * Math.Cos(Theta * Math.PI / 180) - Math.Sin(Theta * Math.PI / 180) * a_tV_Car.eY;
            tVectorRotae.eY = a_tV_Car.eX * Math.Sin(Theta * Math.PI / 180) + Math.Cos(Theta * Math.PI / 180) * a_tV_Car.eY;

            ThetaTmp = rtVectorOP.GetTheta(tVectorRotae, a_tV_S2D);

            if(ThetaTmp < 1)
            {
                return Theta;
            }
            else
            {
                return -Theta;
            }
        }

        public static rtVector Motion_Predict(rtCarData a_tCurrentInfo, rtMotorCtrl a_tMotorData)
        {
            double eDistance = 0, eAngle = 0, eTheta = 0, eSpeed = 0, eT = 0, ePhi = 0, ePhiTest = 0;
            double eLength_C2M = 0; // 兩輪中心到後馬達的距離 
            double eLength_C2O = 0; // 兩輪中心到旋轉中心的距離 = 旋轉半徑
            double eLength_R2O = 0; // 右輪中心到旋轉中心的距離 
            rtVector tNextPosition = new rtVector();
            rtVector tV_Car, tVlaw, tRotateCenter;


            eAngle = a_tCurrentInfo.eAngle;
            tV_Car.eX = Math.Cos(eAngle * Math.PI / 180);
            tV_Car.eY = Math.Sin(eAngle * Math.PI / 180);

            // 取右側的法向量
            tVlaw.eX = tV_Car.eY;
            tVlaw.eY = -tV_Car.eX;

            eTheta = Math.Abs(a_tMotorData.lMotorAngle);

            eLength_C2M = rtVectorOP.GetDistance(a_tCurrentInfo.tPosition, a_tCurrentInfo.tMotorPosition);
            eLength_C2O = Math.Tan((90-eTheta) * Math.PI / 180) * eLength_C2M;

            if (eTheta > ANGLE_TH_MOTION_PREDICT)
            { // 用車模型預測 (對圓心旋轉)
                eT = Math.Sqrt(eLength_C2O*eLength_C2O / (tVlaw.eX*tVlaw.eX + tVlaw.eY*tVlaw.eY));
                if(a_tMotorData.lMotorAngle >= 0)
                {
                    eT = -eT;
                }

                tRotateCenter.eX = a_tCurrentInfo.tPosition.eX + tVlaw.eX * eT;
                tRotateCenter.eY = a_tCurrentInfo.tPosition.eY + tVlaw.eY * eT;

                eLength_R2O = rtVectorOP.GetDistance(a_tCurrentInfo.tCarTirepositionR, tRotateCenter);

                eSpeed = Math.Abs(a_tCurrentInfo.eCarTireSpeedRight) * eLength_C2O / eLength_R2O;

                eDistance = eSpeed * (1 / FREQUENCY); // distance = V x T = 所旋轉的弧長

                ePhi = eDistance / eLength_C2O; // 這裡單位是徑度 >> 旋轉角度 = 弧長 / 旋轉半徑

                if(eT >= 0)
                { // 旋轉中心在右邊 >> 旋轉角度要取負值
                    ePhi = -ePhi;
                }

                if (a_tMotorData.lMotorPower < 0)
                { // 馬達反轉 角度也要取負號
                    ePhi = -ePhi;
                }
                ePhiTest = ePhi * 180 / Math.PI;

                tNextPosition = rtVectorOP.Rotate(a_tCurrentInfo.tPosition, tRotateCenter, ePhi);
            }
            else
            { // 直行模式
                eSpeed = (a_tCurrentInfo.eCarTireSpeedLeft + a_tCurrentInfo.eCarTireSpeedRight) /2;
                eDistance = eSpeed * (1 / FREQUENCY); // distance = V x T
                if (a_tMotorData.lMotorPower >= 0)
                {
                    tNextPosition.eX = a_tCurrentInfo.tPosition.eX + eDistance * tV_Car.eX;
                    tNextPosition.eY = a_tCurrentInfo.tPosition.eY + eDistance * tV_Car.eY;
                }
                else
                {
                    tNextPosition.eX = a_tCurrentInfo.tPosition.eX - eDistance * tV_Car.eX;
                    tNextPosition.eY = a_tCurrentInfo.tPosition.eY - eDistance * tV_Car.eY;
                }
            }
                
            return tNextPosition;
        }

        public static double MotorAngle_Ctrl(
            rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo, 
            ref rtMotorCtrl a_tMotorData)
        {
            double eErrorCurrent = 0, eErrorNext = 0;
            double eCarAngle = 0, ePhi = 0, eTheta = 0;
            double eSpeedWeighting = 1;
            double eMototAngleTmp = 0;
            byte ucSinpleModeFlag = 0; // 0: OFF 1: ON
            int lPathIndex = 0;
            rtVector tV_Car; // car current direction
            rtVector tV_S2D; // src point to destination

            lPathIndex = a_tMotorData.lPathNodeIndex;

            eCarAngle = a_tCurrentInfo.eAngle;
            tV_Car.eX = Math.Cos(eCarAngle * Math.PI / 180);
            tV_Car.eY = Math.Sin(eCarAngle * Math.PI / 180);

            tV_S2D.eX = a_atPathInfo[lPathIndex].tDest.eX - a_atPathInfo[lPathIndex].tSrc.eX;
            tV_S2D.eY = a_atPathInfo[lPathIndex].tDest.eY - a_atPathInfo[lPathIndex].tSrc.eY;

            ePhi = rtVectorOP.GetTheta(tV_Car, tV_S2D);

            switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus)
            {
                // 直走狀態
                case (byte)rtStatus.STRAIGHT:
                    eErrorCurrent = MotorAngle_StraightErrorCal(a_atPathInfo, a_tCurrentInfo, a_tMotorData);
                    break;
                // 轉彎狀態
                case (byte)rtStatus.TURN:
                    eErrorCurrent = MotorAngle_TurnErrorCal(a_atPathInfo, a_tCurrentInfo, ref a_tMotorData);
                    if(a_atPathInfo[lPathIndex].ucTurnType == (byte)rtTurnType.SIMPLE)
                    {
                        ucSinpleModeFlag = 1;
                    }
                    
                    break;
                // 完成狀態: 似乎不會遇到這個 因為設定為 DONE 通常就 index ++了 >> TBD
                case (byte)rtStatus.DONE:
                    // show error
                    break;
                default:
                    // show error
                    break;
            }

            a_tMotorData.eAngleErrorSum += eErrorCurrent;

            if (CAR_SPEED == false)
            { // 用差值預測
                eErrorNext = eErrorCurrent + (eErrorCurrent - a_tMotorData.eAngleErrorLast);
            }
            else
            { // 用運動模型預測

            }
            a_tMotorData.eAngleErrorLast = eErrorCurrent;

            // angle = function(Error) >> Kp*eErrorCurrent + Ki*eAngleErrorSum + Kd*eErrorNext
            eMototAngleTmp = a_tMotorData.tPID_AngleCoe.eKp * eErrorCurrent + a_tMotorData.tPID_AngleCoe.eKi * a_tMotorData.eAngleErrorSum + a_tMotorData.tPID_AngleCoe.eKd * eErrorNext;

            if(CAR_SPEED == false)
            { // 沒車速資訊
                // 算出車身跟路徑向量夾角 並乘上權重
                eTheta = GetTheta_Path2Car(tV_Car, tV_S2D);
                eTheta = eTheta * a_tMotorData.eCarAngleWeighting;

                eMototAngleTmp = eMototAngleTmp + eTheta;

                // 這邊需要有weighting的table 暫時設為1 >> TBD
                eMototAngleTmp = eMototAngleTmp * eSpeedWeighting;
            }
            a_tMotorData.lMotorAngle = (int)(Math.Round(eMototAngleTmp));

            // boundary
            if (a_tMotorData.lMotorAngle > MAX_ANGLE_OFFSET_MOTOR)
            {
                a_tMotorData.lMotorAngle = MAX_ANGLE_OFFSET_MOTOR;
            }
            if (a_tMotorData.lMotorAngle < -MAX_ANGLE_OFFSET_MOTOR)
            {
                a_tMotorData.lMotorAngle = -MAX_ANGLE_OFFSET_MOTOR;
            }

            if(ucSinpleModeFlag == 1)
            { // 直接打正90度或負90度
                a_tMotorData.lMotorAngle = ANGLE_ROTATION * a_tMotorData.lTurnDirection;
            }

            return eErrorCurrent;
        }
    }
}