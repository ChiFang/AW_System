
#define rtAGV_DEBUG_PREDICT

// #define rtAGV_DEBUG_PRINT

#define rtAGV_DEBUG_OFFSET_MODIFY
// #define FW_CTRL

#define rtAGV_POWER_SMOOTH

#define rtAGV_BACK_MODE

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

        public void Init()
        {
            eKp = 0;
            eKi = 0;
            eKd = 0;
        }
    }

    public struct rtMotor_Cfg
    {
        /** \brief Define: PID Power Coeffient Kp */
        public const double PID_POWER_COE_KP = 0.0256;

        /** \brief Define: PID Coefficient for Theta Offset Kp */
        public const double PID_THETA_OFFSET_COE_KP = 0.34;

        /** \brief Define: PID Coefficient for Car Angle to motor angle Kp */
        public const double PID_ANGLE_CAR_MOTOR_COE_KP = 0.75;

        /** \brief Define: Radius of smooth mode: 旋轉半徑 (判斷是否到達定點 開始準備轉向動作) */
        public const int RADIUS_SMOOTH = 1000;

        /** \brief Configure: PID Power Coeffient */
        public rtPID_Coefficient tPID_PowerCoe;

        /** \brief Configure: PID Coefficient for Theta Offset */
        public rtPID_Coefficient tPID_ThetaOffsetCoe;

        /** \brief Configure: PID Coefficient for Car Angle to motor angle */
        public rtPID_Coefficient tPID_MotorAngleCoe;

        /** \brief Configure: Rotation distance of turn in smooth mode */
        public int lRotationDistance;

        public void Init()
        {
            tPID_PowerCoe.Init();
            tPID_ThetaOffsetCoe.Init();
            tPID_MotorAngleCoe.Init();
            lRotationDistance = 0;
        }

        public void LoadDefault()
        {
            tPID_PowerCoe.Init();
            tPID_ThetaOffsetCoe.Init();
            tPID_MotorAngleCoe.Init();

            tPID_PowerCoe.eKp = PID_POWER_COE_KP;
            tPID_ThetaOffsetCoe.eKp = PID_THETA_OFFSET_COE_KP;
            tPID_MotorAngleCoe.eKp = PID_ANGLE_CAR_MOTOR_COE_KP;
            lRotationDistance = RADIUS_SMOOTH;
        }

        
    }

    public struct rtMotor_Data
    {
        /** \brief InOut Data: path segment index */
        public int lPathNodeIndex;

        /** \brief InOut Data: navigate offset: 決定要靠路線左側(負) 還是右側(正) 走 */
        public int lNavigateOffset;

        /** \brief Output Data: Finish Flag */
        public bool bFinishFlag;

        /** \brief Output Data: Rotation Radius of turn in smooth mode */
        public int lRotationRadius;

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

        /** \brief Output Data: 轉彎角度的Offset*/
        public double lTargetAngle;

        /** \brief Output Data: 車身改變的角度 預測模型用*/
        public double eDeltaAngle;

        /** \brief Output Data: 預測實際圓心*/
        public rtVector PredRotationCenter;

        public double Debug_eDistance;

        public double Debug_eThetaError;

        public double Debug_TargetAngleOffset1;

        public double Debug_TargetAngleOffset2;

        public double Debug_CenterSpeed;

        public double Debug_eDeltaCarAngle;

        public bool Debug_bOverDestFlag;

        public bool Debug_bOverDestFlag2;

        public void Init()
        {
            lPathNodeIndex = 0;
            lNavigateOffset = 0;
            bFinishFlag = false;
            lRotationRadius = 0;
            tRotateCenter.Init();
            lTurnDirection = 0;
            lMotorPower = 0;
            lMotorTorsion = 0;
            lMotorAngle = 0;
            lTargetAngle = 0;
            eDeltaAngle = 0;
            PredRotationCenter.Init();
            Debug_eDistance = 0;
            Debug_eThetaError = 0;
        }
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
                    eAngle = 0;
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

        public static double GetTheta_Difference(rtVector a_tV_Src, rtVector a_tV_Target)
        {
            double Theta = 0;
            double ThetaTmp = 0;
            rtVector tVectorRotae; // vector after rotate

            Theta = GetTheta(a_tV_Src, a_tV_Target);

            tVectorRotae.eX = a_tV_Src.eX * Math.Cos(Theta * Math.PI / 180) - Math.Sin(Theta * Math.PI / 180) * a_tV_Src.eY;
            tVectorRotae.eY = a_tV_Src.eX * Math.Sin(Theta * Math.PI / 180) + Math.Cos(Theta * Math.PI / 180) * a_tV_Src.eY;

            ThetaTmp = GetTheta(tVectorRotae, a_tV_Target);

            if (ThetaTmp < 1)
            {
                return Theta;
            }
            else
            {
                return -Theta;
            }
        }

    }

    public class rtMotorCtrl
    {
        public enum rtNavigateStatus { UNDO = 0, DONE = 1 };

        public enum rtTurnType_Simple { ERROR = 0, TURN_RIGHT = 1, TURN_LEFT = -1 };

        public enum rtStatus { STRAIGHT = 1, TURN = 2, DONE = 0 };

        public enum rtTurnType { SIMPLE = 0, SMOOTH = 1, ARRIVE = 2 };

        /** \brief Define: 角度對齊的閥值 */
        public const double ANGLE_MATCH_TH = 1.0;

        /** \brief Define: 角度對齊時決定馬達力道的角度閥值 */
        public const double ALIGNMENT_SPEED_ANGLE_TH = 15.0;

        /** \brief Define: 系統頻率 8Hz = 0.125s 1次 */
        public const double FREQUENCY = 8;

        /** \brief Define: angle threshold: 判斷是否走過頭用的 */
        public const double ANGLE_TH = 90;

        /** \brief Define: 可以在原地打轉的角度 */
        public static int ANGLE_ROTATION = 85;

        /** \brief Define: 馬達(驅動輪胎)在這角度以內以直行計算 */
        public const int ANGLE_TH_MOTION_PREDICT = 5;

        /** \brief Define: distance threshold of simple mode: 判斷是否到達定點 開始準備轉向動作或停止 */
        public const double DISTANCE_ERROR_SIMPLE = 60;

        /** \brief Define: 判斷是否做完轉彎的動作 */
        public const double THETA_ERROR_TURN = 5;

        /** \brief Define: 基礎速度(mm/s) : 判斷車身跟路徑所需夾角的基礎速度 */
        public const double BASE_SPEED = 125;

        /** \brief Define: 轉向時馬達的power */
        public const int TURN_POWER = 50;

        /** \brief Define: max power of motor */
        public const int MAX_POWER = 255;

        /** \brief Define: min power of motor */
        public const int MIN_POWER = 30;

        /** \brief Define: max angle value of motor */
        public const int MAX_ANGLE_OFFSET_MOTOR = 70;

        /** \brief Define: max angle value of path */
        public const int MAX_ANGLE_OFFSET_PATH = 70;

        /** \brief 判斷想要的角度跟感測器回傳的角度是否差距過大的閥值 */
        public const int DELTA_ANGLE_TH = 40;

        /** \brief Motor Control configure */
        public rtMotor_Cfg tMotorCfg;

        /** \brief Motor Control Data */
        public rtMotor_Data tMotorData;

#if rtAGV_DEBUG_PREDICT
        /** \brief Output Data: 預測下次的位置資訊*/
        public rtVector tNextPositionTest;

        /** \brief Output Data: 預測counter*/
        public int lCntTest = 0;

        /** \brief Output Data: 預測error*/
        public double ePredictErrorTest = 0;

        public static void Test_Predict(rtCarData a_tCarData, ref rtMotorCtrl a_CMotorInfo)
        {
            if (a_CMotorInfo.lCntTest > 0)
            {
                a_CMotorInfo.ePredictErrorTest = rtVectorOP.GetDistance(a_tCarData.tPosition, a_CMotorInfo.tNextPositionTest);
            }
            else
            {
                a_CMotorInfo.ePredictErrorTest = 0;
            }
            a_CMotorInfo.tNextPositionTest = Coordinate_Predict(a_tCarData, a_CMotorInfo, (1 / FREQUENCY));

            a_CMotorInfo.lCntTest++;
        }
#endif

        public rtMotorCtrl()
        {
            // Load default configure
            tMotorCfg.LoadDefault();

            // init output data
            tMotorData.Init();
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

#if rtAGV_POWER_SMOOTH
            rtVector tV_C2D_Last; // current point to destination
            int lPathLength = 0;
            int lNodeIndex = 0;
            double ePathTheta = 0;
#endif
            tV_C2D.eX = a_atPathInfo[a_lPathNodeIndex].tDest.eX - a_tPosition.eX;
            tV_C2D.eY = a_atPathInfo[a_lPathNodeIndex].tDest.eY - a_tPosition.eY;
            eErrorCurrent = rtVectorOP.GetLength(tV_C2D);

#if rtAGV_POWER_SMOOTH
            lPathLength = a_atPathInfo.Length;
            for (lNodeIndex = a_lPathNodeIndex+1; lNodeIndex < lPathLength; lPathLength++)
            {
                tV_C2D.eX = a_atPathInfo[lNodeIndex].tDest.eX - a_atPathInfo[lNodeIndex].tSrc.eX;
                tV_C2D.eY = a_atPathInfo[lNodeIndex].tDest.eY - a_atPathInfo[lNodeIndex].tSrc.eY;

                tV_C2D_Last.eX = a_atPathInfo[lNodeIndex - 1].tDest.eX - a_atPathInfo[lNodeIndex - 1].tSrc.eX;
                tV_C2D_Last.eY = a_atPathInfo[lNodeIndex - 1].tDest.eY - a_atPathInfo[lNodeIndex - 1].tSrc.eY;
                ePathTheta = rtVectorOP.GetTheta(tV_C2D_Last, tV_C2D);

                if (ePathTheta< THETA_ERROR_TURN)
                {
                    eErrorCurrent += rtVectorOP.GetLength(tV_C2D);
                }
                else
                {
                    break;
                }

                if (lNodeIndex + 1 >= lPathLength)
                {
                    break;
                }
            }            
#endif
            return eErrorCurrent;
        }

        public static bool CarVectorVerify(rtPath_Info[] a_atPathInfo, double a_eCarAngle, int a_lPathNodeIndex)
        {
            double eDeltaAngle = 0;
            rtVector tV_S2D; // source point to destination

            tV_S2D.eX = a_atPathInfo[a_lPathNodeIndex].tDest.eX - a_atPathInfo[a_lPathNodeIndex].tSrc.eX;
            tV_S2D.eY = a_atPathInfo[a_lPathNodeIndex].tDest.eY - a_atPathInfo[a_lPathNodeIndex].tSrc.eY;

            eDeltaAngle = AngleDifferenceCal(tV_S2D, a_eCarAngle);

            // 判斷是否該倒退走比較合適
            if (Math.Abs(eDeltaAngle) >= ANGLE_TH)
            { // 必須反向行走
                return true;
            }
            return false;
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
            rtPath_Info[] a_atPathInfo, rtCarData a_tCarData,
            ref rtMotorCtrl a_CMotorInfo)
        {
            double eErrorCurrent = 0;
            bool bOutFlag = false;
            bool bOverDestFlag = false;

            a_CMotorInfo.tMotorData.bFinishFlag = false;
            a_CMotorInfo.tMotorData.Debug_bOverDestFlag2 = false;
            a_CMotorInfo.tMotorData.Debug_bOverDestFlag = false;
            switch (a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucStatus)
            {
                // 直走狀態
                case (byte)rtStatus.STRAIGHT:
                    eErrorCurrent = MotorPower_StraightErrorCal(a_atPathInfo, a_tCarData.tPosition, a_CMotorInfo.tMotorData.lPathNodeIndex);

                    // Motor power = function(Error)
                    a_CMotorInfo.tMotorData.lMotorPower = (int)(a_CMotorInfo.tMotorCfg.tPID_PowerCoe.eKp * eErrorCurrent) + MIN_POWER;


                    // 判斷是否是原地旋轉完 但角度還沒恢復的狀況 >> 是的話先停止動力值到角度可以接受
                    if(Math.Abs(a_CMotorInfo.tMotorData.lMotorAngle - a_tCarData.eWheelAngle) > DELTA_ANGLE_TH)
                    {   // 角度差距過大
                        if (a_CMotorInfo.tMotorData.lPathNodeIndex > 0)
                        {   // 至少轉過一次灣
                            if (a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex -1].ucTurnType == (byte)rtTurnType.SIMPLE)
                            {   // 上一段轉彎方式為原地旋轉
                                a_CMotorInfo.tMotorData.lMotorPower = 0;
                            }   
                        }
                    }
                    
                    bOverDestFlag = OverDestination(a_atPathInfo, a_tCarData.tPosition, a_CMotorInfo.tMotorData.lPathNodeIndex);
                    a_CMotorInfo.tMotorData.Debug_bOverDestFlag = bOverDestFlag;
                    if (bOverDestFlag == true)
                    { // 判斷超過終點 >>　TBD
                        if (a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucTurnType == (byte)rtTurnType.ARRIVE)
                        {   // 到達最終目的地
                            a_CMotorInfo.tMotorData.lMotorPower = 0;
                            a_CMotorInfo.tMotorData.lMotorAngle = 0;
                            a_CMotorInfo.tMotorData.lMotorTorsion = 0;
                            a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.DONE;
                            a_CMotorInfo.tMotorData.bFinishFlag = true;
                        }
                        else
                        { // 趕快進入下一段 (要不要先轉正再說)
                            a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.DONE;
                            a_CMotorInfo.tMotorData.lPathNodeIndex++;

                            // 將旋轉半徑、中心等資料清空
                            a_CMotorInfo.tMotorData.tRotateCenter.eX = 0;
                            a_CMotorInfo.tMotorData.tRotateCenter.eY = 0;
                            a_CMotorInfo.tMotorData.lRotationRadius = 0;
                        }
                    }
                    else
                    {
                        switch (a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucTurnType)
                        {
                            case (byte)rtTurnType.SIMPLE:
                                if (eErrorCurrent < DISTANCE_ERROR_SIMPLE)
                                {
                                    a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.TURN;
                                }
                                break;
                            case (byte)rtTurnType.SMOOTH:
                                if (eErrorCurrent < rtMotor_Cfg.RADIUS_SMOOTH)
                                {
                                    // 算出旋轉半徑中心座標
                                    MotorAngle_RotationCenterCal(a_atPathInfo, a_tCarData, ref a_CMotorInfo);

                                    a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.TURN;
                                }
                                break;
                            case (byte)rtTurnType.ARRIVE:
                                if (eErrorCurrent < DISTANCE_ERROR_SIMPLE)
                                { // 到達最終目的地
                                    a_CMotorInfo.tMotorData.lMotorPower = 0;
                                    a_CMotorInfo.tMotorData.lMotorAngle = 0;
                                    a_CMotorInfo.tMotorData.lMotorTorsion = 0;
                                    a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.DONE;
                                    a_CMotorInfo.tMotorData.bFinishFlag = true;
                                }
                                break;
                            default:
                                // show error msg
                                break;
                        }

                        if(a_CMotorInfo.tMotorData.bFinishFlag == false)
                        {
                            bOverDestFlag = CarVectorVerify(a_atPathInfo, a_tCarData.eAngle, a_CMotorInfo.tMotorData.lPathNodeIndex);
                            a_CMotorInfo.tMotorData.Debug_bOverDestFlag2 = bOverDestFlag;
                            if (bOverDestFlag == true)
                            {   // 往後走
                                a_CMotorInfo.tMotorData.lMotorPower = -a_CMotorInfo.tMotorData.lMotorPower;
                                eErrorCurrent = -eErrorCurrent;
                            }
                        }
                        

                    }
                    break;
                // 轉彎狀態
                case (byte)rtStatus.TURN:
                    // 判斷是否已經走完轉彎的扇形區
                    bOutFlag = FinishTurnCheck(
                        a_CMotorInfo.tMotorData.lPathNodeIndex, a_CMotorInfo.tMotorCfg.lRotationDistance,
                        a_atPathInfo, a_tCarData, a_CMotorInfo.tMotorData.tRotateCenter);

                    eErrorCurrent = MotorPower_TurnErrorCal(a_atPathInfo, a_tCarData.tPosition, a_tCarData.eAngle, a_CMotorInfo.tMotorData.lPathNodeIndex);
                    a_CMotorInfo.tMotorData.lMotorPower = TURN_POWER;

                    switch (a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucTurnType)
                    {
                        case (byte)rtTurnType.SIMPLE:
                            if (eErrorCurrent < THETA_ERROR_TURN)
                            {
                                a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.DONE;
                                a_CMotorInfo.tMotorData.lPathNodeIndex++;
                            }
                            else
                            {
                                if (Math.Abs(Math.Abs(a_tCarData.eWheelAngle) - ANGLE_ROTATION) > ANGLE_MATCH_TH)
                                {   // 還沒轉到原地旋轉的角度時先不行走
                                    a_CMotorInfo.tMotorData.lMotorPower = 0;
                                }
                            }
                            break;
                        case (byte)rtTurnType.SMOOTH:
                            if (eErrorCurrent < THETA_ERROR_TURN || bOutFlag == true) // 判斷旋轉角度會來不及 >> 只看車身跟路線夾角
                            {
                                a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucStatus = (byte)rtStatus.DONE;
                                a_CMotorInfo.tMotorData.lPathNodeIndex++;
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
            rtMotorCtrl a_CMotorInfo
            )
        {
            double eErrorCurrent = 0, eT = 0;
            double eSrcX, eSrcY, eDestX, eDestY;
            double eCurrentX, eCurrentY;
            rtVector tVS2D, tVlaw;

            eCurrentX = a_tPosition.eX;
            eCurrentY = a_tPosition.eY;

            eSrcX = a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].tSrc.eX;
            eSrcY = a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].tSrc.eY;

            eDestX = a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].tDest.eX;
            eDestY = a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].tDest.eY;

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
            rtPath_Info[] a_atPathInfo, rtCarData a_tCarData,
            ref rtMotorCtrl a_CMotorInfo
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

            lPathIndex = a_CMotorInfo.tMotorData.lPathNodeIndex;

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
            eT = a_CMotorInfo.tMotorCfg.lRotationDistance / eLength;
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
            eT = a_CMotorInfo.tMotorCfg.lRotationDistance / eLength;
            tNext.eX = tSrc.eX + eT * tVs2dNext.eX;
            tNext.eY = tSrc.eY + eT * tVs2dNext.eY;


            // 取兩條線交點當旋轉中心座標
            eT = tNext.eX * tVs2dNextLaw.eY - tNext.eY * tVs2dNextLaw.eX - tCurrent.eX * tVs2dNextLaw.eY + tCurrent.eY * tVs2dNextLaw.eX;
            eT /= tVd2sCurrentLaw.eX * tVs2dNextLaw.eY - tVd2sCurrentLaw.eY * tVs2dNextLaw.eX;
            a_CMotorInfo.tMotorData.tRotateCenter.eX = tCurrent.eX + eT * tVd2sCurrentLaw.eX;
            a_CMotorInfo.tMotorData.tRotateCenter.eY = tCurrent.eY + eT * tVd2sCurrentLaw.eY;

            // 以下計算是否超出扇形區域
            tCenter2SrcTurn.eX = tCurrent.eX - a_CMotorInfo.tMotorData.tRotateCenter.eX;
            tCenter2SrcTurn.eY = tCurrent.eY - a_CMotorInfo.tMotorData.tRotateCenter.eY;
            tCenter2DestTurn.eX = tNext.eX - a_CMotorInfo.tMotorData.tRotateCenter.eX;
            tCenter2DestTurn.eY = tNext.eY - a_CMotorInfo.tMotorData.tRotateCenter.eY;
            tCenter2Current.eX = a_tCarData.tPosition.eX - a_CMotorInfo.tMotorData.tRotateCenter.eX;
            tCenter2Current.eY = a_tCarData.tPosition.eY - a_CMotorInfo.tMotorData.tRotateCenter.eY;

            eThetaBoundaty = rtVectorOP.GetTheta(tCenter2DestTurn, tCenter2SrcTurn);
            eThetaCurrent = rtVectorOP.GetTheta(tCenter2Current, tCenter2SrcTurn);

            // 計算旋轉半徑
            a_CMotorInfo.tMotorData.lRotationRadius = (int)Math.Round(rtVectorOP.GetDistance(tCurrent, a_CMotorInfo.tMotorData.tRotateCenter));
        }

        public static bool FinishTurnCheck(int a_lPathIndex, int a_lRotationDistance,
            rtPath_Info[] a_atPathInfo, rtCarData a_tCarData, rtVector a_tRotateCenter
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
            tCenter2Current.eX = a_tCarData.tPosition.eX - a_tRotateCenter.eX;
            tCenter2Current.eY = a_tCarData.tPosition.eY - a_tRotateCenter.eY;

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

        public static double MotorAngle_TurnErrorCal(rtPath_Info[] a_atPathInfo, rtVector a_tPosition, rtMotorCtrl a_CMotorInfo)
        {
            double eErrorCurrent = 0;
            double eDistance = 0;
            byte ucTurnType = 0;

            ucTurnType = a_atPathInfo[a_CMotorInfo.tMotorData.lPathNodeIndex].ucTurnType;

            switch (ucTurnType)
            {
                case (byte)rtTurnType.SIMPLE:
                    // Do nothing >> 原本不需要做事情 但需要知道是左90度還是右90度
                    break;
                case (byte)rtTurnType.SMOOTH:
                    eDistance = rtVectorOP.GetDistance(a_tPosition, a_CMotorInfo.tMotorData.tRotateCenter);
                    eErrorCurrent = eDistance - a_CMotorInfo.tMotorData.lRotationRadius; // 可能會有錯誤 如果在內側非扇型區域 >> TBD
                    break;
                case (byte)rtTurnType.ARRIVE:
                    // show error msg
                    break;
                default:
                    // show error msg
                    break;
            }

            switch (a_CMotorInfo.tMotorData.lTurnDirection)
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

        public static double CarCenterSpeedCal(rtCarData a_tCarData, double a_eMotorAngle)
        {
            double eSpeed = 0;
            double eTheta = 0, eT = 0;
            double eLength_C2M = 0; // 兩輪中心到後馬達的距離 
            double eLength_C2O = 0; // 兩輪中心到旋轉中心的距離 = 旋轉半徑
            double eLength_R2O = 0; // 右輪中心到旋轉中心的距離 
            double eLength_L2O = 0; // 右輪中心到旋轉中心的距離
            rtVector tV_Car, tVlaw, tRotateCenter;

            eTheta = Math.Abs(a_eMotorAngle);
            eLength_C2M = rtVectorOP.GetDistance(a_tCarData.tPosition, a_tCarData.tMotorPosition);
            eLength_C2O = Math.Tan((90 - eTheta) * Math.PI / 180) * eLength_C2M;

            tV_Car.eX = Math.Cos(a_tCarData.eAngle * Math.PI / 180);
            tV_Car.eY = Math.Sin(a_tCarData.eAngle * Math.PI / 180);

            // 取右側的法向量
            tVlaw.eX = tV_Car.eY;
            tVlaw.eY = -tV_Car.eX;

            eT = Math.Sqrt(eLength_C2O * eLength_C2O / (tVlaw.eX * tVlaw.eX + tVlaw.eY * tVlaw.eY));
            if (a_eMotorAngle >= 0)
            {
                eT = -eT;
            }

            tRotateCenter.eX = a_tCarData.tPosition.eX + tVlaw.eX * eT;
            tRotateCenter.eY = a_tCarData.tPosition.eY + tVlaw.eY * eT;

            eLength_R2O = rtVectorOP.GetDistance(a_tCarData.tCarTirepositionR, tRotateCenter);
            eLength_L2O = rtVectorOP.GetDistance(a_tCarData.tCarTirepositionL, tRotateCenter);

            if (eTheta > ANGLE_TH_MOTION_PREDICT)
            {
                if (a_eMotorAngle < 0)
                { // 車子往右轉 輪子角度為負
                    eSpeed = Math.Abs(a_tCarData.eCarTireSpeedLeft) * eLength_C2O / eLength_L2O;
                }
                else
                { // 車子往左轉 輪子角度為正
                    eSpeed = Math.Abs(a_tCarData.eCarTireSpeedRight) * eLength_C2O / eLength_R2O;
                }
            }
            else
            {   // 當作直行
                eSpeed = (a_tCarData.eCarTireSpeedLeft + a_tCarData.eCarTireSpeedRight) / 2;
            }

            return eSpeed;
        }

        public static void Motion_Predict(rtCarData a_tCarData, rtMotorCtrl a_CMotorInfo, double a_eDeltaTime, out rtVector a_tCarCoordinate, out double a_eCarAngle)
        {
            a_eCarAngle = 0;
            a_tCarCoordinate = new rtVector();

            a_tCarCoordinate = Coordinate_Predict(a_tCarData, a_CMotorInfo, a_eDeltaTime);
            a_eCarAngle = a_tCarData.eAngle + a_CMotorInfo.tMotorData.eDeltaAngle;

        }

        public static rtVector Coordinate_Predict(rtCarData a_tCarData, rtMotorCtrl a_CMotorInfo, double a_eDeltaTime)
        {
            double eDistance = 0, eAngle = 0, eTheta = 0, eSpeed = 0, eT = 0, ePhi = 0, ePhiRad = 0;
            double eLength_C2M = 0; // 兩輪中心到後馬達的距離 
            double eLength_C2O = 0; // 兩輪中心到旋轉中心的距離 = 旋轉半徑
            double eLength_R2O = 0; // 右輪中心到旋轉中心的距離 
            double eLength_L2O = 0; // 右輪中心到旋轉中心的距離
            rtVector tNextPosition = new rtVector();
            rtVector tV_Car, tVlaw, tRotateCenter;


            eAngle = a_tCarData.eAngle;
            tV_Car.eX = Math.Cos(eAngle * Math.PI / 180);
            tV_Car.eY = Math.Sin(eAngle * Math.PI / 180);

            // 取右側的法向量
            tVlaw.eX = tV_Car.eY;
            tVlaw.eY = -tV_Car.eX;

            eTheta = Math.Abs(a_tCarData.eWheelAngle);

            eLength_C2M = rtVectorOP.GetDistance(a_tCarData.tPosition, a_tCarData.tMotorPosition);
            eLength_C2O = Math.Tan((90 - eTheta) * Math.PI / 180) * eLength_C2M;

            if (eTheta > ANGLE_TH_MOTION_PREDICT)
            { // 用車模型預測 (對圓心旋轉)
                eT = Math.Sqrt(eLength_C2O * eLength_C2O / (tVlaw.eX * tVlaw.eX + tVlaw.eY * tVlaw.eY));
                if (a_tCarData.eWheelAngle >= 0)
                {
                    eT = -eT;
                }

                tRotateCenter.eX = a_tCarData.tPosition.eX + tVlaw.eX * eT;
                tRotateCenter.eY = a_tCarData.tPosition.eY + tVlaw.eY * eT;

                a_CMotorInfo.tMotorData.PredRotationCenter.eX = tRotateCenter.eX;
                a_CMotorInfo.tMotorData.PredRotationCenter.eY = tRotateCenter.eY;

                eLength_R2O = rtVectorOP.GetDistance(a_tCarData.tCarTirepositionR, tRotateCenter);
                eLength_L2O = rtVectorOP.GetDistance(a_tCarData.tCarTirepositionL, tRotateCenter);

                if (a_tCarData.eCarTireSpeedLeft > a_tCarData.eCarTireSpeedRight || a_tCarData.eWheelAngle < 0)
                { // 往右轉
                    eSpeed = Math.Abs(a_tCarData.eCarTireSpeedLeft) * eLength_C2O / eLength_L2O;
                }
                else
                { // 往左轉
                    eSpeed = Math.Abs(a_tCarData.eCarTireSpeedRight) * eLength_C2O / eLength_R2O;
                }


                eDistance = eSpeed * a_eDeltaTime; // distance = V x T = 所旋轉的弧長

                ePhi = eDistance / eLength_C2O; // 這裡單位是徑度 >> 旋轉角度 = 弧長 / 旋轉半徑

                if (eT >= 0)
                { // 旋轉中心在右邊 >> 旋轉角度要取負值
                    ePhi = -ePhi;
                }

                if (a_CMotorInfo.tMotorData.lMotorPower < 0)
                { // 馬達反轉 角度也要取負號
                    ePhi = -ePhi;
                }
                ePhiRad = ePhi * 180 / Math.PI;

                tNextPosition = rtVectorOP.Rotate(a_tCarData.tPosition, tRotateCenter, ePhi);
            }
            else
            { // 直行模式
                ePhi = 0;
                ePhiRad = ePhi * 180 / Math.PI;

                eSpeed = (a_tCarData.eCarTireSpeedLeft + a_tCarData.eCarTireSpeedRight) / 2;
                eDistance = eSpeed * (1 / FREQUENCY); // distance = V x T
                if (a_CMotorInfo.tMotorData.lMotorPower >= 0)
                {
                    tNextPosition.eX = a_tCarData.tPosition.eX + eDistance * tV_Car.eX;
                    tNextPosition.eY = a_tCarData.tPosition.eY + eDistance * tV_Car.eY;
                }
                else
                {
                    tNextPosition.eX = a_tCarData.tPosition.eX - eDistance * tV_Car.eX;
                    tNextPosition.eY = a_tCarData.tPosition.eY - eDistance * tV_Car.eY;
                }
            }

            a_CMotorInfo.tMotorData.eDeltaAngle = ePhiRad;

            return tNextPosition;
        }

        public static double TargetAngle_Cal(rtCarData a_tCarData, rtMotorCtrl a_CMotorInfo)
        {
            double eTargetAngle = 0, eLengthM2C = 0, eTanTheta = 0;

            eLengthM2C = rtVectorOP.GetDistance(a_tCarData.tMotorPosition, a_tCarData.tPosition);

            eTanTheta = a_CMotorInfo.tMotorData.lRotationRadius / eLengthM2C;

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


        public static bool CarAngleAlignment(double a_eTargetAngle, rtCarData a_tCarData, rtMotorCtrl a_CMotorInfo)
        {
            bool bMatched = false;
            double eAngleError = 0;
            double eAngleDelay = 0; // 確認輪胎有轉到90度

            eAngleError = DeltaAngleCal(a_tCarData.eAngle, a_eTargetAngle);

            if (Math.Abs(eAngleError) < ANGLE_MATCH_TH)
            {
                a_CMotorInfo.tMotorData.lMotorPower = 0;
                a_CMotorInfo.tMotorData.lMotorAngle = 0;
                bMatched = true;
            }
            else
            {
                if(eAngleError > 0)
                {
                    a_CMotorInfo.tMotorData.lMotorAngle = ANGLE_ROTATION;
                }
                else
                {
                    a_CMotorInfo.tMotorData.lMotorAngle = -ANGLE_ROTATION;
                }
                eAngleDelay = a_CMotorInfo.tMotorData.lMotorAngle - a_tCarData.eWheelAngle;
                if(Math.Abs(eAngleDelay) < ANGLE_MATCH_TH)
                {
                    if (Math.Abs(eAngleError) > ALIGNMENT_SPEED_ANGLE_TH)
                    {
                        a_CMotorInfo.tMotorData.lMotorPower = TURN_POWER;
                    }
                    else
                    {
                        a_CMotorInfo.tMotorData.lMotorPower = MIN_POWER;
                    }
                        
                }
                else
                {
                    a_CMotorInfo.tMotorData.lMotorPower = 0;
                }
                bMatched = false;
            }
            return bMatched;
        }

        public static double PathAngleOffsetCal(double a_eDistance, rtPID_Coefficient a_tPID_ThetaOffsetCoe)
        {
            double eDistanceLimtH = 2000;   // 之後弄成define

            double eThetaOffset = 0;

            if (a_eDistance > eDistanceLimtH)
            {   // 超過距離限制 最多跟路徑差70度
                eThetaOffset = (a_eDistance > 0) ? MAX_ANGLE_OFFSET_PATH : -MAX_ANGLE_OFFSET_PATH;
            }
            else
            {   // 按系數計算
                eThetaOffset = a_eDistance * a_tPID_ThetaOffsetCoe.eKp;
            }
            return eThetaOffset;
        }

        public static double MotorAngleCal(double a_eDeltaCarAngle, double a_eCarSpeed, rtPID_Coefficient a_tPID_MotorAngleCoe)
        {   // 目前不考慮 車速的因素
            double eDeltaCarAngleLimtH = 90;   // 之後弄成define

            double MotorAngle = 0;

            if (a_eDeltaCarAngle > eDeltaCarAngleLimtH)
            {   // 超過角度限制 最多轉70度
                MotorAngle = (a_eDeltaCarAngle > 0) ? MAX_ANGLE_OFFSET_MOTOR : -MAX_ANGLE_OFFSET_MOTOR;
            }
            else
            {   // 按系數計算
                MotorAngle = a_eDeltaCarAngle * a_tPID_MotorAngleCoe.eKp;
            }
            return MotorAngle;
        }

        public static double TargetAngleOffsetModify(double a_eDistanceEroor, double a_eCenterSpeed, double a_eTargetAngleOffset)
        {
            double eAbsDistanceEroor = 0;
            double eLength = 0;
            double eLengthModify = 0;
            double eAbsTargetAngleOffset = 0;
            double eModifiedAngleOffset = 0;

            eAbsDistanceEroor = Math.Abs(a_eDistanceEroor);
            eAbsTargetAngleOffset = Math.Abs(a_eTargetAngleOffset);

            if (eAbsTargetAngleOffset < ANGLE_MATCH_TH)
            {   // 角度太小就不修正 避免三角函數算錯
                eModifiedAngleOffset = a_eTargetAngleOffset;
                return eModifiedAngleOffset;
            }
            else
            {
                eLength = eAbsDistanceEroor / (Math.Sin(eAbsTargetAngleOffset * Math.PI / 180));
            }
            
            eLengthModify = Math.Abs(a_eCenterSpeed) / BASE_SPEED * eLength;

#if rtAGV_DEBUG_PRINT
            Console.WriteLine("eLength:" + eLength.ToString());
            Console.WriteLine("eLengthModify:" + eLengthModify.ToString());
#endif
            eModifiedAngleOffset = (eLengthModify != 0) ? Math.Asin(eAbsDistanceEroor / eLengthModify) : 0;

            if (eLengthModify == 0)
            {   // 速度 = 0 就不更改
                eModifiedAngleOffset = a_eTargetAngleOffset;
            }
            else
            {
                eModifiedAngleOffset = eModifiedAngleOffset * 180 / Math.PI;
                eModifiedAngleOffset = (a_eTargetAngleOffset < 0) ? -eModifiedAngleOffset : eModifiedAngleOffset;
            }

            return eModifiedAngleOffset;
        }

        public static double MotorAngle_CtrlNavigate(
            rtPath_Info[] a_atPathInfo, rtCarData a_tCarData, ref rtMotorCtrl a_CMotorInfo)
        {
            double eDistance = 0, eThetaError;

            double eCarCenterSpeed = 0;
            double eCarAngle = 0, eMotorAngleOffset = 0, eTargetCarAngleOffset = 0, eDeltaCarAngle = 0;
            double eMototAngleTmp = 0;
            byte ucSinpleModeFlag = 0; // 0: OFF 1: ON
            int lPathIndex = 0;
            rtVector tPathVector = new rtVector();
            rtVector tTargetCarVector = new rtVector();
            rtVector tVector = new rtVector();

            lPathIndex = a_CMotorInfo.tMotorData.lPathNodeIndex;

            eCarAngle = a_tCarData.eAngle;

            tPathVector.eX = a_atPathInfo[lPathIndex].tDest.eX - a_atPathInfo[lPathIndex].tSrc.eX;
            tPathVector.eY = a_atPathInfo[lPathIndex].tDest.eY - a_atPathInfo[lPathIndex].tSrc.eY;

            switch (a_atPathInfo[lPathIndex].ucStatus)
            {
                // 直走狀態
                case (byte)rtStatus.STRAIGHT:
                    eMotorAngleOffset = 0;
                    eDistance = MotorAngle_StraightErrorCal(a_atPathInfo, a_tCarData.tPosition, a_CMotorInfo);
                    break;
                // 轉彎狀態
                case (byte)rtStatus.TURN:
                    //  算出要往左轉還是右轉
                    a_CMotorInfo.tMotorData.lTurnDirection = TurnDirectionCal(a_atPathInfo, a_CMotorInfo.tMotorData.lPathNodeIndex);

                    if (a_atPathInfo[lPathIndex].ucTurnType == (byte)rtTurnType.SIMPLE)
                    {
                        ucSinpleModeFlag = 1;
                        eMotorAngleOffset = 0;
                        eDistance = 0;
                    }
                    else
                    {
                        eDistance = MotorAngle_TurnErrorCal(a_atPathInfo, a_tCarData.tPosition, a_CMotorInfo);

                        tVector.eX = (a_CMotorInfo.tMotorData.tRotateCenter.eX - a_tCarData.tPosition.eX) * a_CMotorInfo.tMotorData.lTurnDirection;
                        tVector.eY = (a_CMotorInfo.tMotorData.tRotateCenter.eY - a_tCarData.tPosition.eY) * a_CMotorInfo.tMotorData.lTurnDirection;

                        // 取右側的法向量 為路徑切線向量
                        tPathVector.eX = tVector.eY;
                        tPathVector.eY = -tVector.eX;

                        // 用算出的旋轉半徑得知車輪至少要轉幾度
                        eMotorAngleOffset = TargetAngle_Cal(a_tCarData, a_CMotorInfo);
                        eMotorAngleOffset = eMotorAngleOffset * a_CMotorInfo.tMotorData.lTurnDirection;
                    }

                    break;
                default:    // 不應該出現這種case
                    // show error
                    break;
            }

            a_CMotorInfo.tMotorData.Debug_eDistance = eDistance;   // for test

            a_CMotorInfo.tMotorData.lTargetAngle = eMotorAngleOffset;

            // 算出車身角度根路徑角度偏差多少 (這裡目前僅為了 LOG)
            eThetaError = AngleDifferenceCal(tPathVector, eCarAngle);
            a_CMotorInfo.tMotorData.Debug_eThetaError = eThetaError;

            if (ucSinpleModeFlag == 1)
            { // 直接打正90度或負90度
                a_CMotorInfo.tMotorData.lMotorAngle = ANGLE_ROTATION * a_CMotorInfo.tMotorData.lTurnDirection;
            }
            else
            {
                // 考慮靠左 or 靠右的 offset
                eDistance += a_CMotorInfo.tMotorData.lNavigateOffset;

                // 算出要跟路徑的夾角
                eTargetCarAngleOffset = PathAngleOffsetCal(eDistance, a_CMotorInfo.tMotorCfg.tPID_ThetaOffsetCoe);
#if rtAGV_BACK_MODE
                if(a_CMotorInfo.tMotorData.lMotorPower<0)
                {   // 倒退走
                    eTargetCarAngleOffset = -eTargetCarAngleOffset;
                }
#endif
                a_CMotorInfo.tMotorData.Debug_TargetAngleOffset1 = eTargetCarAngleOffset;
#if rtAGV_DEBUG_PRINT
                Console.WriteLine("eTargetCarAngleOffset1:" + eTargetCarAngleOffset.ToString());
#endif
                // 算出兩輪中心的速度
                eCarCenterSpeed = CarCenterSpeedCal(a_tCarData, a_tCarData.eWheelAngle);
                a_CMotorInfo.tMotorData.Debug_CenterSpeed = eCarCenterSpeed;
#if rtAGV_DEBUG_PRINT
                Console.WriteLine("eCarCenterSpeed:" + eCarCenterSpeed.ToString());
#endif

#if rtAGV_DEBUG_OFFSET_MODIFY
                // 根據車速調整跟路徑的夾角
                eTargetCarAngleOffset = TargetAngleOffsetModify(eDistance, eCarCenterSpeed, eTargetCarAngleOffset);
#endif
                a_CMotorInfo.tMotorData.Debug_TargetAngleOffset2 = eTargetCarAngleOffset;
#if rtAGV_DEBUG_PRINT
                Console.WriteLine("eTargetCarAngleOffset2:" + eTargetCarAngleOffset.ToString());
#endif


                // 算出目標車身角度 (vector format)
                rtVector tZeroVector = new rtVector(0,0);
                tTargetCarVector = rtVectorOP.Rotate(tPathVector, tZeroVector, eTargetCarAngleOffset * Math.PI / 180);
#if rtAGV_BACK_MODE
                if (a_CMotorInfo.tMotorData.lMotorPower < 0)
                {   // 倒退走
                    tTargetCarVector = rtVectorOP.Rotate(tTargetCarVector, tZeroVector, 180 * Math.PI / 180);
                }
#endif

                // 算出目標車身角度與當下車身角度的差距
                eDeltaCarAngle = AngleDifferenceCal(tTargetCarVector, eCarAngle);
                a_CMotorInfo.tMotorData.Debug_eDeltaCarAngle = eDeltaCarAngle;
                // 用角度差距算出 適當的車輪馬達轉角
                eMototAngleTmp = MotorAngleCal(eDeltaCarAngle, 0, a_CMotorInfo.tMotorCfg.tPID_MotorAngleCoe);

                // 加上之前的角度 offset
                eMototAngleTmp = eMototAngleTmp + eMotorAngleOffset;

                a_CMotorInfo.tMotorData.lMotorAngle = (int)(Math.Round(eMototAngleTmp));

                // boundary
                if (a_CMotorInfo.tMotorData.lMotorAngle > MAX_ANGLE_OFFSET_MOTOR)
                {
                    a_CMotorInfo.tMotorData.lMotorAngle = MAX_ANGLE_OFFSET_MOTOR;
                }
                if (a_CMotorInfo.tMotorData.lMotorAngle < -MAX_ANGLE_OFFSET_MOTOR)
                {
                    a_CMotorInfo.tMotorData.lMotorAngle = -MAX_ANGLE_OFFSET_MOTOR;
                }
            }

            return eDistance;
        }
    }

    public struct rtForkCtrl_Data
    {
        public byte ucStatus;

        public int height;

        public int distanceDepth;

        public bool bEnable;

        public void Init()
        {
            ucStatus = (byte)rtForkCtrl.ForkStatus.NULL;
            height = 0;
            distanceDepth = 0;
            bEnable = false;
        }

    }

    public class rtForkCtrl
    {
        /** \brief Fork Control Data */
        public rtForkCtrl_Data tForkData;

        /** \brief 堆高機貨叉狀態宣告 */
        public enum ForkLODAStatus { SetHeight, Forth, Backward, Pickup, Finished };

        /** \brief 堆高機貨叉狀態宣告 */
        public enum ForkUnLODAStatus { SetHeight, Forth, Backward, PutDown, Finished };

        /** \brief 堆高機貨叉狀態宣告 */
        public enum ForkStatus { NULL, ALIMENT, SET_HEIGHT, FORTH, BACKWARD, PICKUP, PICKDOWN, RESET_HEIGHT, FINISH , ERROR};

        /** \brief 堆高機貨叉動作模式 */
        public enum ForkActionMode { LOAD = 0,UNLOAD = 1};

        /** \brief Define: 判斷是否到達要的高度跟深度 */
        public const double FORK_MATCH_TH = 25;

        /** \brief Define: 貨叉最大深度 */
        public const int FORK_MAX_DEPTH = 4500;

        /** \brief Define: 貨叉舉起高度 */
        public const int FORK_PICKUP_HEIGHT = 100;

        public rtForkCtrl()
        {
            tForkData.Init();
        }

        public static void LOADTest(ref rtForkCtrl_Data tForkData)
        {
            tForkData.height = 125;
            tForkData.distanceDepth = FORK_MAX_DEPTH;
            tForkData.bEnable = true;
        }

        public static void UNLOADTest(ref rtForkCtrl_Data tForkData)
        {
            tForkData.height = 125;
            tForkData.distanceDepth = FORK_MAX_DEPTH;
            tForkData.bEnable = true;
        }

     }
}