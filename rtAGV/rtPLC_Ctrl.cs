
#define rtAGV_DEBUG_PREDICT


using System;

using rtAGV_Common;

namespace PLC_Control
{
    

    public struct rtPID_Coefficient
    {
        /** \brief Kp */
        public double eKp;

        /** \brief Ki */
        public double eKi;

        /** \brief Kd */
        public double eKd;
    }

    public struct rtAngle_CtrlParams
    {
        /** \brief Kp for PID */
        public double eKp;

        /** \brief Ki for PID */
        public double eKi;

        /** \brief Kd for PID */
        public double eKd;

        /** \brief Coefficient for angle difference */
        public double eAlpha;

        /** \brief Coefficient for Theta Offset */
        public double eThetaOffsetCoe;

        /** \brief Coefficient for Car Angle to motor angle */
        public double eCarAngleCoe;
    }

    

    public class rtVectorOP
    {
        public static double GetLength(rtVector a_tIn)
        {
            double eOut = 0;
            eOut = Math.Sqrt(a_tIn.eX * a_tIn.eX + a_tIn.eY * a_tIn.eY);
            return eOut;
        }

        public static double Dot(rtVector a_tV1, rtVector a_tV2)
        {
            double eOut = 0;

            eOut = a_tV1.eX * a_tV2.eX + a_tV1.eY * a_tV2.eY;
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
        {   // 角度單位是徑度 甭轉換
            rtVector tResult = new rtVector();
            rtVector tTmp, tTmp1;

            tTmp.eX = a_tPoint.eX - a_tCenter.eX;
            tTmp.eY = a_tPoint.eY - a_tCenter.eY;

            tTmp1.eX = Math.Cos(a_eTheta) * tTmp.eX - Math.Sin(a_eTheta) * tTmp.eY;
            tTmp1.eY = Math.Sin(a_eTheta) * tTmp.eX + Math.Cos(a_eTheta) * tTmp.eY;

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
            eDistance = Math.Sqrt(eGapX * eGapX + eGapY * eGapY);

            return eDistance;
        }

        public static double Vector2Angle(rtVector a_tVector)
        {
            double eAngle = 0;

            if (a_tVector.eX == 0)
            {
                if (a_tVector.eY > 0)
                {
                    eAngle = 90;
                }
                else if (a_tVector.eY < 0)
                {
                    eAngle = -90;
                }
                else
                {
                    // show error msg
                    eAngle = 999;
                }
            }
            else
            {
                eAngle = Math.Atan(a_tVector.eY / a_tVector.eX) * 180 / Math.PI;

                if (eAngle > 0)
                {
                    if (a_tVector.eX < 0)
                    {
                        eAngle += 180;
                    }
                }
                else
                {
                    if (a_tVector.eX < 0)
                    {
                        eAngle += 180;
                    }
                    else
                    {
                        eAngle += 360;
                    }
                }
            }

            return eAngle;
        }

    }


    public class rtForkCtrl
    {
        public rtForkCtrl()
        {

        }
    }

    public class rtMotorCtrl
    {
        public enum rtNavigateStatus { UNDO = 0, DONE = 1 };

        public enum rtTurnType_Simple { ERROR = 0, TURN_RIGHT = 1, TURN_LEFT = -1 };

        public enum rtStatus { STRAIGHT = 1, TURN = 2, DONE = 0 };

        public enum rtTurnType { SIMPLE = 0, SMOOTH = 1, ARRIVE = 2 };

        /** \brief Define: 角度對齊的閥值 */
        public const double ANGLE_MATCH_TH = 0.5;

        /** \brief Define: 系統頻率 8Hz = 0.125s 1次 */
        public const double FREQUENCY = 8;

        /** \brief Define: angle threshold: 判斷是否走過頭用的 */
        public const double ANGLE_TH = 90;

        /** \brief Define: 可以在原地打轉的角度 */
        public const int ANGLE_ROTATION = 90;

        /** \brief Define: 馬達(驅動輪胎)在這角度以內以直行計算 */
        public const int ANGLE_TH_MOTION_PREDICT = 5;

        /** \brief Define: distance threshold of simple mode: 判斷是否到達定點 開始準備轉向動作或停止 */
        public const double DISTANCE_ERROR_SIMPLE = 60;

        /** \brief Define: distance threshold of smooth mode: 判斷是否到達定點 開始準備轉向動作 */
        public const int DISTANCE_ERROR_SMOOTH = 1000;

        /** \brief Define: 判斷是否做完轉彎的動作 */
        public const double THETA_ERROR_TURN = 5;

        /** \brief Define: 轉向時馬達的power */
        public const int TURN_POWER = 50;

        /** \brief Define: max power of motor */
        public const int MAX_POWER = 255;

        /** \brief Define: min power of motor */
        public const int MIN_POWER = 18;

        /** \brief Define: max angle value of motor */
        public const int MAX_ANGLE_OFFSET_MOTOR = 70;

        /** \brief Define: max angle value of path */
        public const int MAX_ANGLE_OFFSET_PATH = 70;

        /** \brief Configure: PID Power Coeffient */
        public rtPID_Coefficient tPID_PowerCoe;

        /** \brief Configure: Ki coefficient in angle control */
        public double eKiCoeAngle = 0.66666667;

        /** \brief Configure: 跟隨路徑時角度控制的參數 */
        public rtAngle_CtrlParams tAngleCtrlParams;

        /** \brief Configure: Rotation distance of turn in smooth mode */
        public int lRotationDistance = 0;

        /** \brief Input Data: motor angle read*/
        public double eMotorAngleIn;

        /** \brief InOut Data: path segment index */
        public int lPathNodeIndex = 0;

        /** \brief InOut Data: navigate offset: 決定要靠路線左側(負) 還是右側(正) 走 */
        public int lNavigateOffset = 0;

        /** \brief Output Data: Finish Flag */
        public byte ucFinishFlag = 0;

        /** \brief Output Data: Rotation Radius of turn in smooth mode */
        public int lRotationRadius = 0;

        /** \brief Output Data: rotation center of smooth turn */
        public rtVector tRotateCenter;

        /** \brief Output Data: 旋轉方向: 1: 中心在左邊>> 向右轉  -1:中心在右邊>> 向左轉  0: 出錯*/
        public int lTurnDirection;

        /** \brief Output Data: motor power */
        public int lMotorPower;

        /** \brief Output Data: motor torsion */
        public int lMotorTorsion;

        /** \brief Output Data: motor angle */
        public int lMotorAngle;

        /** \brief Output Data: Error Sum for motor angle modify */
        public double eAngleErrorSum = 0;

        /** \brief Output Data: 轉彎角度的Offset*/
        public double lTargetAngle;

        /** \brief Output Data: 預測下次的轉彎Error*/
        public double eAngleErroNext;

        /** \brief Output Data: 車身改變的角度 預測模型用*/
        public double eDeltaAngle;

        /** \brief Output Data: 預測下次的位置資訊*/
        public rtVector tNextPosition;

        /** \brief Output Data: 預測實際圓心*/
        public rtVector PredRotationCenter;

        public double Debug_eWightingDistance;

        public double Debug_eDistance;

        public double Debug_eThetaError;

#if rtAGV_DEBUG_PREDICT
        /** \brief Output Data: 預測下次的位置資訊*/
        public rtVector tNextPositionTest;

        /** \brief Output Data: 預測counter*/
        public int lCntTest = 0;

        /** \brief Output Data: 預測error*/
        public double ePredictErrorTest = 0;

        public static void Test_Predict(rtCarData a_tCurrentInfo, ref rtMotorCtrl a_tMotorData)
        {
            if (a_tMotorData.lCntTest > 0)
            {
                a_tMotorData.ePredictErrorTest = rtVectorOP.GetDistance(a_tCurrentInfo.tPosition, a_tMotorData.tNextPositionTest);
            }
            else
            {
                a_tMotorData.ePredictErrorTest = 0;
            }
            a_tMotorData.tNextPositionTest = Motion_Predict(a_tCurrentInfo, a_tMotorData);

            a_tMotorData.lCntTest++;
        }
#endif

        public rtMotorCtrl()
        {
            // init configure
            tPID_PowerCoe.eKp = 0;
            tPID_PowerCoe.eKi = 0;
            tPID_PowerCoe.eKd = 0;

            eKiCoeAngle = 0;
            tAngleCtrlParams.eAlpha = 0;
            tAngleCtrlParams.eKp = 0;
            tAngleCtrlParams.eKi = 0;
            tAngleCtrlParams.eKd = 0;

            tAngleCtrlParams.eThetaOffsetCoe = 0.035;
            tAngleCtrlParams.eCarAngleCoe = 0.75;

            lRotationDistance = DISTANCE_ERROR_SMOOTH;

            // init output data
            eAngleErroNext = 0;
            eAngleErrorSum = 0;

            lPathNodeIndex = 0;

            lMotorPower = 0;
            lMotorTorsion = 0;
            lMotorAngle = 0;

            lTargetAngle = 0;
            tRotateCenter.eX = 0;
            tRotateCenter.eY = 0;
            lRotationRadius = 0;
            ucFinishFlag = (byte)rtNavigateStatus.UNDO;
        }

        /**
        \brief initail path infomation attay
        \param a_atPathInfo [IN] path infomation attay
        \return void
        */
        public static void Init_rtPath_Info(rtPath_Info[] a_atPathInfo)
        {
            int lPathIndex = 0;
            for (lPathIndex = 0; lPathIndex < a_atPathInfo.Length - 1; lPathIndex++)
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

        public static double MotorPower_StraightErrorCal(rtPath_Info[] a_atPathInfo, rtVector a_tPosition, int a_lPathNodeIndex)
        {
            double eErrorCurrent = 0;
            rtVector tV_C2D; // current point to destination

            tV_C2D.eX = a_atPathInfo[a_lPathNodeIndex].tDest.eX - a_tPosition.eX;
            tV_C2D.eY = a_atPathInfo[a_lPathNodeIndex].tDest.eY - a_tPosition.eY;
            eErrorCurrent = rtVectorOP.GetLength(tV_C2D);

            return eErrorCurrent;
        }

        public static bool OverDestination(rtPath_Info[] a_atPathInfo, rtVector a_tPosition, int a_lPathNodeIndex)
        {
            double eTheta = 0;
            rtVector tV_C2D; // current point to destination
            rtVector tV_S2D; // source point to destination

            tV_S2D.eX = a_atPathInfo[a_lPathNodeIndex].tDest.eX - a_atPathInfo[a_lPathNodeIndex].tSrc.eX;
            tV_S2D.eY = a_atPathInfo[a_lPathNodeIndex].tDest.eY - a_atPathInfo[a_lPathNodeIndex].tSrc.eY;
            tV_C2D.eX = a_atPathInfo[a_lPathNodeIndex].tDest.eX - a_tPosition.eX;
            tV_C2D.eY = a_atPathInfo[a_lPathNodeIndex].tDest.eY - a_tPosition.eY;

            eTheta = rtVectorOP.GetTheta(tV_S2D, tV_C2D);

            // 判斷是否已超終點
            if (eTheta >= ANGLE_TH)
            { // 超過終點 >>　必須反向行走
                return true;
            }
            return false;
        }

        public static double MotorPower_TurnErrorCal(rtPath_Info[] a_atPathInfo, rtVector a_tPosition, double a_eCarAngle, int a_lPathNodeIndex)
        {
            double eErrorCurrent = 0, eTheta = 0;
            int lNextPathID = 0;
            rtVector tV_Car;        // car current direction
            rtVector tV_NextS2D;    // next src point to destination

            lNextPathID = a_lPathNodeIndex + 1;
            tV_Car.eX = Math.Cos(a_eCarAngle * Math.PI / 180);
            tV_Car.eY = Math.Sin(a_eCarAngle * Math.PI / 180);

            tV_NextS2D.eX = a_atPathInfo[lNextPathID].tDest.eX - a_atPathInfo[lNextPathID].tSrc.eX;
            tV_NextS2D.eY = a_atPathInfo[lNextPathID].tDest.eY - a_atPathInfo[lNextPathID].tSrc.eY;

            eTheta = rtVectorOP.GetTheta(tV_Car, tV_NextS2D);
            eErrorCurrent = eTheta;

            return eErrorCurrent;
        }

        public static double MotorPower_Ctrl(
            rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo,
            ref rtMotorCtrl a_tMotorData)
        {
            double eErrorCurrent = 0;
            bool bOutFlag = false;
            bool bOverDestFlag = false;

            switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus)
            {
                // 直走狀態
                case (byte)rtStatus.STRAIGHT:
                    eErrorCurrent = MotorPower_StraightErrorCal(a_atPathInfo, a_tCurrentInfo.tPosition, a_tMotorData.lPathNodeIndex);

                    // Motor power = function(Error)
                    a_tMotorData.lMotorPower = (int)(a_tMotorData.tPID_PowerCoe.eKp * eErrorCurrent) + MIN_POWER;

                    bOverDestFlag = OverDestination(a_atPathInfo, a_tCurrentInfo.tPosition, a_tMotorData.lPathNodeIndex);

                    if (bOverDestFlag == true)
                    { // 超過終點 >>　TBD
                        // a_tMotorData.lMotorPower = -a_tMotorData.lMotorPower;
                        eErrorCurrent = -eErrorCurrent;
                        if (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType == (byte)rtTurnType.ARRIVE)
                        {   // 到達最終目的地
                            a_tMotorData.lMotorPower = 0;
                            a_tMotorData.lMotorAngle = 0;
                            a_tMotorData.lMotorTorsion = 0;
                            a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.DONE;
                            a_tMotorData.ucFinishFlag = (byte)rtNavigateStatus.DONE;
                        }
                        else
                        { // 趕快進入下一段 (要不要先轉正再說)
                            a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.DONE;
                            a_tMotorData.lPathNodeIndex++;

                            // 將旋轉半徑、中心等資料清空
                            a_tMotorData.tRotateCenter.eX = 0;
                            a_tMotorData.tRotateCenter.eY = 0;
                            a_tMotorData.lRotationRadius = 0;
                        }
                    }
                    else
                    {
                        switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType)
                        {
                            case (byte)rtTurnType.SIMPLE:
                                if (eErrorCurrent < DISTANCE_ERROR_SIMPLE)
                                {
                                    a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.TURN;
                                }
                                break;
                            case (byte)rtTurnType.SMOOTH:
                                if (eErrorCurrent < DISTANCE_ERROR_SMOOTH)
                                {
                                    // 算出旋轉半徑中心座標
                                    MotorAngle_RotationCenterCal(a_atPathInfo, a_tCurrentInfo, ref a_tMotorData);

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
                    }
                    break;
                // 轉彎狀態
                case (byte)rtStatus.TURN:
                    // 判斷是否已經走完轉彎的扇形區
                    bOutFlag = FinishTurnCheck(
                        a_tMotorData.lPathNodeIndex, a_tMotorData.lRotationDistance,
                        a_atPathInfo, a_tCurrentInfo, a_tMotorData.tRotateCenter);

                    eErrorCurrent = MotorPower_TurnErrorCal(a_atPathInfo, a_tCurrentInfo.tPosition, a_tCurrentInfo.eAngle, a_tMotorData.lPathNodeIndex);
                    a_tMotorData.lMotorPower = TURN_POWER;

                    switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType)
                    {
                        case (byte)rtTurnType.SIMPLE:
                            if (eErrorCurrent < THETA_ERROR_TURN)
                            {
                                a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.DONE;
                                a_tMotorData.lPathNodeIndex++;
                            }
                            else
                            {
                                if (Math.Abs(Math.Abs(a_tMotorData.eMotorAngleIn) - ANGLE_ROTATION) < ANGLE_MATCH_TH)
                                {   // 還沒轉到原地旋轉的角度時先不行走
                                    a_tMotorData.lMotorPower = 0;
                                }
                            }
                            break;
                        case (byte)rtTurnType.SMOOTH:
                            if (eErrorCurrent < THETA_ERROR_TURN || bOutFlag == true) // 判斷旋轉角度會來不及 >> 只看車身跟路線夾角
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
            rtPath_Info[] a_atPathInfo, rtVector a_tPosition,
            rtMotorCtrl a_tMotorData
            )
        {
            double eErrorCurrent = 0, eT = 0;
            double eSrcX, eSrcY, eDestX, eDestY;
            double eCurrentX, eCurrentY;
            rtVector tVS2D, tVlaw;

            eCurrentX = a_tPosition.eX;
            eCurrentY = a_tPosition.eY;

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

        public static int TurnDirectionCal(rtPath_Info[] a_atPathInfo, int a_lPathNodeIndex)
        {
            int lPathIndex = 0, lTurnDirection = 0;
            double eAngle = 0, eAngleFix = 0;
            rtVector tSrc;
            rtVector tDest;
            rtVector tVd2sCurrent;
            rtVector tVs2dNext;
            rtVector tVd2sCurrentFix;
            rtVector tCnter = new rtVector();

            lPathIndex = a_lPathNodeIndex;

            // set vector & point current
            tSrc.eX = a_atPathInfo[lPathIndex].tSrc.eX;
            tSrc.eY = a_atPathInfo[lPathIndex].tSrc.eY;
            tDest.eX = a_atPathInfo[lPathIndex].tDest.eX;
            tDest.eY = a_atPathInfo[lPathIndex].tDest.eY;
            tVd2sCurrent.eX = tSrc.eX - tDest.eX;
            tVd2sCurrent.eY = tSrc.eY - tDest.eY;

            // set vector & point next
            tSrc.eX = a_atPathInfo[lPathIndex + 1].tSrc.eX;
            tSrc.eY = a_atPathInfo[lPathIndex + 1].tSrc.eY;
            tDest.eX = a_atPathInfo[lPathIndex + 1].tDest.eX;
            tDest.eY = a_atPathInfo[lPathIndex + 1].tDest.eY;
            tVs2dNext.eX = tDest.eX - tSrc.eX;
            tVs2dNext.eY = tDest.eY - tSrc.eY;

            eAngle = rtVectorOP.GetTheta(tVd2sCurrent, tVs2dNext);
            tVd2sCurrentFix = rtVectorOP.Rotate(tVd2sCurrent, tCnter, eAngle * Math.PI / 180);
            eAngleFix = rtVectorOP.GetTheta(tVd2sCurrentFix, tVs2dNext);


            if (eAngle > 10)
            {
                if (eAngle < 1)
                {   // 點在右邊 >> 馬達向左轉
                    lTurnDirection = (int)rtTurnType_Simple.TURN_LEFT;
                }
                else
                {   // 點在左邊 >> 馬達向右轉
                    lTurnDirection = (int)rtTurnType_Simple.TURN_RIGHT;
                }
            }
            else
            {   // 不可能轉這麼大
                lTurnDirection = (int)rtTurnType_Simple.ERROR;
            }

            return lTurnDirection;
        }

        public static void MotorAngle_RotationCenterCal(
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
            tSrc.eX = a_atPathInfo[lPathIndex + 1].tSrc.eX;
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
        }

        public static bool FinishTurnCheck(int a_lPathIndex, int a_lRotationDistance,
            rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo, rtVector a_tRotateCenter
            )
        {
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

            // set vector & point current
            tSrc.eX = a_atPathInfo[a_lPathIndex].tSrc.eX;
            tSrc.eY = a_atPathInfo[a_lPathIndex].tSrc.eY;
            tDest.eX = a_atPathInfo[a_lPathIndex].tDest.eX;
            tDest.eY = a_atPathInfo[a_lPathIndex].tDest.eY;
            tVd2sCurrent.eX = tSrc.eX - tDest.eX;
            tVd2sCurrent.eY = tSrc.eY - tDest.eY;

            // 取右側法向量
            tVd2sCurrentLaw.eX = tVd2sCurrent.eY;
            tVd2sCurrentLaw.eY = -tVd2sCurrent.eX;

            // 取轉彎起始點
            eLength = rtVectorOP.GetLength(tVd2sCurrent);
            eT = a_lRotationDistance / eLength;
            tCurrent.eX = tDest.eX + eT * tVd2sCurrent.eX;
            tCurrent.eY = tDest.eY + eT * tVd2sCurrent.eY;

            // set vector & point next
            tSrc.eX = a_atPathInfo[a_lPathIndex + 1].tSrc.eX;
            tSrc.eY = a_atPathInfo[a_lPathIndex + 1].tSrc.eY;
            tDest.eX = a_atPathInfo[a_lPathIndex + 1].tDest.eX;
            tDest.eY = a_atPathInfo[a_lPathIndex + 1].tDest.eY;
            tVs2dNext.eX = tDest.eX - tSrc.eX;
            tVs2dNext.eY = tDest.eY - tSrc.eY;

            // 取右側法向量
            tVs2dNextLaw.eX = tVs2dNext.eY;
            tVs2dNextLaw.eY = -tVs2dNext.eX;

            // 取轉彎結束點
            eLength = rtVectorOP.GetLength(tVs2dNext);
            eT = a_lRotationDistance / eLength;
            tNext.eX = tSrc.eX + eT * tVs2dNext.eX;
            tNext.eY = tSrc.eY + eT * tVs2dNext.eY;

            // 以下計算是否超出扇形區域
            tCenter2SrcTurn.eX = tCurrent.eX - a_tRotateCenter.eX;
            tCenter2SrcTurn.eY = tCurrent.eY - a_tRotateCenter.eY;
            tCenter2DestTurn.eX = tNext.eX - a_tRotateCenter.eX;
            tCenter2DestTurn.eY = tNext.eY - a_tRotateCenter.eY;
            tCenter2Current.eX = a_tCurrentInfo.tPosition.eX - a_tRotateCenter.eX;
            tCenter2Current.eY = a_tCurrentInfo.tPosition.eY - a_tRotateCenter.eY;

            eThetaBoundaty = rtVectorOP.GetTheta(tCenter2DestTurn, tCenter2SrcTurn);
            eThetaCurrent = rtVectorOP.GetTheta(tCenter2Current, tCenter2SrcTurn);

            if (eThetaCurrent > eThetaBoundaty)
            {
                return true;
            }
            else
            {
                return false;
            }

        }

        public static double MotorAngle_TurnErrorCal(rtPath_Info[] a_atPathInfo, rtVector a_tPosition, rtMotorCtrl a_tMotorData)
        {
            double eErrorCurrent = 0;
            double eDistance = 0;
            byte ucTurnType = 0;

            ucTurnType = a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType;

            switch (ucTurnType)
            {
                case (byte)rtTurnType.SIMPLE:
                    // Do nothing >> 原本不需要做事情 但需要知道是左90度還是右90度
                    break;
                case (byte)rtTurnType.SMOOTH:
                    eDistance = rtVectorOP.GetDistance(a_tPosition, a_tMotorData.tRotateCenter);
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

            if (ThetaTmp < 1)
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
            double eDistance = 0, eAngle = 0, eTheta = 0, eSpeed = 0, eT = 0, ePhi = 0, ePhiRad = 0;
            double eLength_C2M = 0; // 兩輪中心到後馬達的距離 
            double eLength_C2O = 0; // 兩輪中心到旋轉中心的距離 = 旋轉半徑
            double eLength_R2O = 0; // 右輪中心到旋轉中心的距離 
            double eLength_L2O = 0; // 右輪中心到旋轉中心的距離
            rtVector tNextPosition = new rtVector();
            rtVector tV_Car, tVlaw, tRotateCenter;


            eAngle = a_tCurrentInfo.eAngle;
            tV_Car.eX = Math.Cos(eAngle * Math.PI / 180);
            tV_Car.eY = Math.Sin(eAngle * Math.PI / 180);

            // 取右側的法向量
            tVlaw.eX = tV_Car.eY;
            tVlaw.eY = -tV_Car.eX;

            eTheta = Math.Abs(a_tMotorData.eMotorAngleIn);

            eLength_C2M = rtVectorOP.GetDistance(a_tCurrentInfo.tPosition, a_tCurrentInfo.tMotorPosition);
            eLength_C2O = Math.Tan((90 - eTheta) * Math.PI / 180) * eLength_C2M;

            if (eTheta > ANGLE_TH_MOTION_PREDICT)
            { // 用車模型預測 (對圓心旋轉)
                eT = Math.Sqrt(eLength_C2O * eLength_C2O / (tVlaw.eX * tVlaw.eX + tVlaw.eY * tVlaw.eY));
                if (a_tMotorData.eMotorAngleIn >= 0)
                {
                    eT = -eT;
                }

                tRotateCenter.eX = a_tCurrentInfo.tPosition.eX + tVlaw.eX * eT;
                tRotateCenter.eY = a_tCurrentInfo.tPosition.eY + tVlaw.eY * eT;

                a_tMotorData.PredRotationCenter.eX = tRotateCenter.eX;
                a_tMotorData.PredRotationCenter.eY = tRotateCenter.eY;

                eLength_R2O = rtVectorOP.GetDistance(a_tCurrentInfo.tCarTirepositionR, tRotateCenter);
                eLength_L2O = rtVectorOP.GetDistance(a_tCurrentInfo.tCarTirepositionL, tRotateCenter);

                if (a_tCurrentInfo.eCarTireSpeedLeft > a_tCurrentInfo.eCarTireSpeedRight || a_tMotorData.eMotorAngleIn < 0)
                { // 往右轉
                    eSpeed = Math.Abs(a_tCurrentInfo.eCarTireSpeedLeft) * eLength_C2O / eLength_L2O;
                }
                else
                { // 往左轉
                    eSpeed = Math.Abs(a_tCurrentInfo.eCarTireSpeedRight) * eLength_C2O / eLength_R2O;
                }


                eDistance = eSpeed * (1 / FREQUENCY); // distance = V x T = 所旋轉的弧長

                ePhi = eDistance / eLength_C2O; // 這裡單位是徑度 >> 旋轉角度 = 弧長 / 旋轉半徑

                if (eT >= 0)
                { // 旋轉中心在右邊 >> 旋轉角度要取負值
                    ePhi = -ePhi;
                }

                if (a_tMotorData.lMotorPower < 0)
                { // 馬達反轉 角度也要取負號
                    ePhi = -ePhi;
                }
                ePhiRad = ePhi * 180 / Math.PI;

                tNextPosition = rtVectorOP.Rotate(a_tCurrentInfo.tPosition, tRotateCenter, ePhi);
            }
            else
            { // 直行模式
                ePhi = 0;
                ePhiRad = ePhi * 180 / Math.PI;

                eSpeed = (a_tCurrentInfo.eCarTireSpeedLeft + a_tCurrentInfo.eCarTireSpeedRight) / 2;
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

            a_tMotorData.eDeltaAngle = ePhiRad;

            return tNextPosition;
        }

        public static double TargetAngle_Cal(rtCarData a_tCurrentInfo, rtMotorCtrl a_tMotorData)
        {
            double eTargetAngle = 0, eLengthM2C = 0, eTanTheta = 0;

            eLengthM2C = rtVectorOP.GetDistance(a_tCurrentInfo.tMotorPosition, a_tCurrentInfo.tPosition);

            eTanTheta = a_tMotorData.lRotationRadius / eLengthM2C;

            eTargetAngle = Math.Atan(eTanTheta);

            eTargetAngle = 90 - (eTargetAngle * 180 / Math.PI);

            return eTargetAngle;
        }

        public static double DeltaAngleCal(double a_eInputAngle, double a_eTargetAngle)
        {
            double eDeltaAngle = 0;

            eDeltaAngle = a_eTargetAngle - a_eInputAngle;

            if (eDeltaAngle > 180)
            {
                eDeltaAngle -= 360;
            }

            if (eDeltaAngle < -180)
            {
                eDeltaAngle += 360;
            }

            return eDeltaAngle;
        }

        public static double AngleDifferenceCal(rtVector a_tTargetVector, double a_eCarAngle)
        {
            double eAngleDiff = 0;
            double eTargetAngle = 0;

            eTargetAngle = rtVectorOP.Vector2Angle(a_tTargetVector);
            eAngleDiff = DeltaAngleCal(a_eCarAngle, eTargetAngle);

            return eAngleDiff;
        }


        public static double DecideDistanceWighting_old(double a_eDistance)
        {
            // 暫時 hard code 之後得加入 configure 設定

            double eWightingDistance = 0;
            double eDistanceH = 1000;
            double eDistanceL = 50;
            double eWightingH = 1;
            double eWightingL = 0.25;

            if (a_eDistance > eDistanceH)
            { // 全看距離
                eWightingDistance = eWightingH;
            }
            else if (a_eDistance < eDistanceL)
            { // 最低權限
                eWightingDistance = eWightingL;
            }
            else
            { // 線性計算
                eWightingDistance = eWightingL + (a_eDistance - eDistanceL) * (eWightingH - eWightingL) / (eDistanceH - eDistanceL);
            }

            return eWightingDistance;
        }

        public static double DecideDistanceWighting(double a_eDistance, double a_eTheta)
        {
            // 暫時 hard code 之後得加入 configure 設定

            double eWightingDistance = 0;
            double eDistanceLimitH = 500;
            double eDistanceLimitL = 30;
            double eDistanceH = 1000;
            double eThetaH = 20;
            double ePartDistance = 0, ePartTheta = 0;
            double eWightingH = 1;
            double eWightingL = 0.15;

            if (a_eDistance > eDistanceLimitH)
            { // 全看距離
                eWightingDistance = eWightingH;
            }
            else if (a_eDistance < eDistanceLimitL)
            { // 最低權限
                eWightingDistance = eWightingL;
            }
            else
            { // 權重計算
                ePartDistance = (a_eDistance > eDistanceH)? 100 : 100 * a_eDistance / eDistanceH;
                ePartTheta = (a_eTheta > eThetaH) ? 100 : 100 * a_eTheta / eThetaH;
                eWightingDistance = ePartDistance / (ePartDistance+ ePartTheta);
            }

            return eWightingDistance;
        }

        public static double MotorAngle_CtrlNavigate(
            rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo, ref rtMotorCtrl a_tMotorData)
        {
            double eError = 0, eErrorNext = 0;
            double eDistance = 0, eDistanceNext = 0;
            double eThetaError = 0, eThetaErrorNext = 0;
            double eCarAngle = 0, eCarAngleNext = 0, eTargetAngle = 0;
            double eMototAngleTmp = 0;
            double eWightingDistance = 1;
            double eWightingDistanceNext = 1;
            byte ucSinpleModeFlag = 0; // 0: OFF 1: ON
            int lPathIndex = 0;
            rtVector tNextPosition = new rtVector();
            rtVector tPathVector = new rtVector();
            rtVector tPathVectorNext = new rtVector();
            rtVector tVector = new rtVector();
            rtVector tVectorNext = new rtVector();

            lPathIndex = a_tMotorData.lPathNodeIndex;

            eCarAngle = a_tCurrentInfo.eAngle;

            tPathVector.eX = a_atPathInfo[lPathIndex].tDest.eX - a_atPathInfo[lPathIndex].tSrc.eX;
            tPathVector.eY = a_atPathInfo[lPathIndex].tDest.eY - a_atPathInfo[lPathIndex].tSrc.eY;


            // 用運動模型預測下一個座標
            tNextPosition = Motion_Predict(a_tCurrentInfo, a_tMotorData);

            eCarAngleNext = eCarAngle + a_tMotorData.eDeltaAngle;


            switch (a_atPathInfo[lPathIndex].ucStatus)
            {
                // 直走狀態
                case (byte)rtStatus.STRAIGHT:
                    eThetaError = AngleDifferenceCal(tPathVector, eCarAngle);
                    eThetaErrorNext = AngleDifferenceCal(tPathVector, eCarAngleNext);
                    eTargetAngle = 0;
                    a_tMotorData.lTargetAngle = eTargetAngle;
                    eDistance = MotorAngle_StraightErrorCal(a_atPathInfo, a_tCurrentInfo.tPosition, a_tMotorData);

                    // 用運動模型預測下一次直行誤差
                    eDistanceNext = MotorAngle_StraightErrorCal(a_atPathInfo, tNextPosition, a_tMotorData);
                    break;
                // 轉彎狀態
                case (byte)rtStatus.TURN:
                    a_tMotorData.lTurnDirection = TurnDirectionCal(a_atPathInfo, a_tMotorData.lPathNodeIndex);
                    if (a_atPathInfo[lPathIndex].ucTurnType == (byte)rtTurnType.SIMPLE)
                    {
                        ucSinpleModeFlag = 1;
                        eDistance = 0;
                        eDistanceNext = 0;
                        eThetaError = 0;
                        eThetaErrorNext = 0;
                    }
                    else
                    {
                        eDistance = MotorAngle_TurnErrorCal(a_atPathInfo, a_tCurrentInfo.tPosition, a_tMotorData);

                        ///
                        if (a_tMotorData.lTurnDirection == (int)rtTurnType_Simple.TURN_RIGHT)
                        {   // 馬達向右轉
                            tVector.eX = a_tMotorData.tRotateCenter.eX - a_tCurrentInfo.tPosition.eX;
                            tVector.eY = a_tMotorData.tRotateCenter.eY - a_tCurrentInfo.tPosition.eY;
                            tVectorNext.eX = a_tMotorData.tRotateCenter.eX - tNextPosition.eX;
                            tVectorNext.eY = a_tMotorData.tRotateCenter.eY - tNextPosition.eY;
                        }
                        else if (a_tMotorData.lTurnDirection == (int)rtTurnType_Simple.TURN_LEFT)
                        {   // 馬達向左轉
                            tVector.eX = a_tCurrentInfo.tPosition.eX - a_tMotorData.tRotateCenter.eX;
                            tVector.eY = a_tCurrentInfo.tPosition.eY - a_tMotorData.tRotateCenter.eY;
                            tVectorNext.eX = tNextPosition.eX - a_tMotorData.tRotateCenter.eX;
                            tVectorNext.eY = tNextPosition.eY - a_tMotorData.tRotateCenter.eY;
                        }
                        else
                        {
                            tVector.eX = 0;
                            tVector.eY = 0;
                            tVectorNext.eX = 0;
                            tVectorNext.eY = 0;
                        }

                        // 取右側的法向量
                        tPathVector.eX = tVector.eY;
                        tPathVector.eY = -tVector.eX;
                        tPathVectorNext.eX = tVectorNext.eY;
                        tPathVectorNext.eY = -tVectorNext.eX;

                        eThetaError = AngleDifferenceCal(tPathVector, eCarAngle);
                        eThetaErrorNext = AngleDifferenceCal(tPathVectorNext, eCarAngleNext);
                        ///

                        eTargetAngle = TargetAngle_Cal(a_tCurrentInfo, a_tMotorData);
                        eTargetAngle = eTargetAngle * a_tMotorData.lTurnDirection;
                        a_tMotorData.lTargetAngle = eTargetAngle;

                        // 用運動模型預測下一次轉彎誤差
                        eDistanceNext = MotorAngle_TurnErrorCal(a_atPathInfo, tNextPosition, a_tMotorData);
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

            if (ucSinpleModeFlag == 1)
            { // 直接打正90度或負90度
                a_tMotorData.lMotorAngle = ANGLE_ROTATION * a_tMotorData.lTurnDirection;
            }
            else
            {
                // 考慮靠左 or 靠右的 offset
                eDistance += a_tMotorData.lNavigateOffset;
                eDistanceNext += a_tMotorData.lNavigateOffset;

                // decide eWightingDistance and eWightingDistanceNext
                eWightingDistance = DecideDistanceWighting(Math.Abs(eDistance), Math.Abs(eThetaError));
                eWightingDistanceNext = DecideDistanceWighting(Math.Abs(eDistanceNext), Math.Abs(eThetaErrorNext));
                
                a_tMotorData.Debug_eWightingDistance = eWightingDistance;

                a_tMotorData.Debug_eDistance = eDistance;

                a_tMotorData.Debug_eThetaError = eThetaError;

                eError = eWightingDistance * eDistance + (1 - eWightingDistance) * a_tMotorData.tAngleCtrlParams.eAlpha * eThetaError;
                eErrorNext = eWightingDistanceNext * eDistanceNext + (1 - eWightingDistanceNext) * a_tMotorData.tAngleCtrlParams.eAlpha * eThetaErrorNext;

                a_tMotorData.eAngleErroNext = eErrorNext;
                a_tMotorData.eAngleErrorSum = a_tMotorData.eAngleErrorSum * a_tMotorData.eKiCoeAngle + eError;

                // angle = function(Error) >> Kp*eError + Ki*eAngleErrorSum + Kd*eErrorNext
                eMototAngleTmp = a_tMotorData.tAngleCtrlParams.eKp * eError + a_tMotorData.tAngleCtrlParams.eKi * a_tMotorData.eAngleErrorSum + a_tMotorData.tAngleCtrlParams.eKd * eErrorNext;

                eMototAngleTmp = eMototAngleTmp + eTargetAngle;

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
            }
            
            return eError;
        }

        public static bool CarAngleAlignment(double a_eTargetAngle, rtCarData a_tCurrentInfo, rtMotorCtrl a_tMotorData)
        {
            bool bMatched = false;
            double eAngleError = 0;
            double eAngleDelay = 0;

            eAngleError = DeltaAngleCal(a_tCurrentInfo.eAngle, a_eTargetAngle);

            if (Math.Abs(eAngleError) < ANGLE_MATCH_TH)
            {
                a_tMotorData.lMotorPower = 0;
                a_tMotorData.lMotorAngle = 0;
                bMatched = true;
            }
            else
            {
                if(eAngleError > 0)
                {
                    a_tMotorData.lMotorAngle = ANGLE_ROTATION;
                }
                else
                {
                    a_tMotorData.lMotorAngle = -ANGLE_ROTATION;
                }
                eAngleDelay = a_tMotorData.lMotorAngle - a_tMotorData.eMotorAngleIn;
                if(Math.Abs(eAngleDelay) < ANGLE_MATCH_TH)
                {
                    a_tMotorData.lMotorPower = TURN_POWER;
                }
                else
                {
                    a_tMotorData.lMotorPower = 0;
                }
                bMatched = false;
            }
            return bMatched;
        }

        public static double PathAngleOffsetCal(double a_eDistance, rtAngle_CtrlParams a_tAngleParams)
        {
            double eDistanceLimtH = 2000;   // 之後弄成define

            double eThetaOffset = 0;

            if (a_eDistance > eDistanceLimtH)
            {   // 超過距離限制 最多跟路徑差70度
                eThetaOffset = (a_eDistance > 0) ? MAX_ANGLE_OFFSET_PATH : -MAX_ANGLE_OFFSET_PATH;
            }
            else
            {   // 按系數計算
                eThetaOffset = a_eDistance * a_tAngleParams.eThetaOffsetCoe;
            }
            return eThetaOffset;
        }

        public static double MotorAngleCal(double a_eDeltaCarAngle, double a_eCarSpeed, rtAngle_CtrlParams a_tAngleParams)
        {   // 目前不考慮 車速的因素
            double eDeltaCarAngleLimtH = 90;   // 之後弄成define

            double MotorAngle = 0;

            if (a_eDeltaCarAngle > eDeltaCarAngleLimtH)
            {   // 超過角度限制 最多轉70度
                MotorAngle = (a_eDeltaCarAngle > 0) ? MAX_ANGLE_OFFSET_MOTOR : -MAX_ANGLE_OFFSET_MOTOR;
            }
            else
            {   // 按系數計算
                MotorAngle = a_eDeltaCarAngle * a_tAngleParams.eCarAngleCoe;
            }
            return MotorAngle;
        }

        public static double MotorAngle_CtrlNavigate_New(
            rtPath_Info[] a_atPathInfo, rtCarData a_tCurrentInfo, ref rtMotorCtrl a_tMotorData)
        {
            double eError = 0;
            double eDistance = 0;
            double eCarAngle = 0, eMotorAngleOffset = 0, eTargetCarAngle = 0, eDeltaCarAngle = 0;
            double eMototAngleTmp = 0;
            byte ucSinpleModeFlag = 0; // 0: OFF 1: ON
            int lPathIndex = 0;
            rtVector tPathVector = new rtVector();
            rtVector tTargetCarVector = new rtVector();
            rtVector tVector = new rtVector();


            lPathIndex = a_tMotorData.lPathNodeIndex;

            eCarAngle = a_tCurrentInfo.eAngle;

            tPathVector.eX = a_atPathInfo[lPathIndex].tDest.eX - a_atPathInfo[lPathIndex].tSrc.eX;
            tPathVector.eY = a_atPathInfo[lPathIndex].tDest.eY - a_atPathInfo[lPathIndex].tSrc.eY;

            switch (a_atPathInfo[lPathIndex].ucStatus)
            {
                // 直走狀態
                case (byte)rtStatus.STRAIGHT:
                    eMotorAngleOffset = 0;
                    eDistance = MotorAngle_StraightErrorCal(a_atPathInfo, a_tCurrentInfo.tPosition, a_tMotorData);

                    break;
                // 轉彎狀態
                case (byte)rtStatus.TURN:
                    a_tMotorData.lTurnDirection = TurnDirectionCal(a_atPathInfo, a_tMotorData.lPathNodeIndex);
                    if (a_atPathInfo[lPathIndex].ucTurnType == (byte)rtTurnType.SIMPLE)
                    {
                        ucSinpleModeFlag = 1;
                        eMotorAngleOffset = 0;
                        eDistance = 0;
                    }
                    else
                    {
                        //  算出要往左轉還是右轉
                        eDistance = MotorAngle_TurnErrorCal(a_atPathInfo, a_tCurrentInfo.tPosition, a_tMotorData);

                        //

                        tVector.eX = (a_tMotorData.tRotateCenter.eX - a_tCurrentInfo.tPosition.eX) * a_tMotorData.lTurnDirection;
                        tVector.eY = (a_tMotorData.tRotateCenter.eY - a_tCurrentInfo.tPosition.eY) * a_tMotorData.lTurnDirection;

                        // 取右側的法向量 為路徑切線向量
                        tPathVector.eX = tVector.eY;
                        tPathVector.eY = -tVector.eX;
                        //

                        // 用算出的旋轉半徑得知車輪至少要轉幾度
                        eMotorAngleOffset = TargetAngle_Cal(a_tCurrentInfo, a_tMotorData);
                        eMotorAngleOffset = eMotorAngleOffset * a_tMotorData.lTurnDirection;
                    }

                    break;
                default:    // 不應該出現這種case
                    // show error
                    break;
            }

            a_tMotorData.lTargetAngle = eMotorAngleOffset;

            if (ucSinpleModeFlag == 1)
            { // 直接打正90度或負90度
                a_tMotorData.lMotorAngle = ANGLE_ROTATION * a_tMotorData.lTurnDirection;
            }
            else
            {
                // 考慮靠左 or 靠右的 offset
                eDistance += a_tMotorData.lNavigateOffset;

                // 算出要跟路徑的夾角
                eTargetCarAngle = PathAngleOffsetCal(eDistance, a_tMotorData.tAngleCtrlParams);

                // 算出目標車身角度
                rtVector tZeroVector = new rtVector(0,0);
                tTargetCarVector = rtVectorOP.Rotate(tPathVector, tZeroVector, eTargetCarAngle* Math.PI / 180);

                // 算出目標車身角度與當下車身角度的差距
                eDeltaCarAngle = AngleDifferenceCal(tTargetCarVector, eCarAngle);

                // 用角度差距算出 適當的車輪馬達轉角
                eMototAngleTmp = MotorAngleCal(eDeltaCarAngle, 0, a_tMotorData.tAngleCtrlParams);

                // 加上之前的角度 offset
                eMototAngleTmp = eMototAngleTmp + eMotorAngleOffset;

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
            }

            return eDistance;
        }
    }
}