using System;

namespace Motor
{
    public class rtLocateData
    {
        public rtVector tPosition;
        public double eAngle;
    }

    public class rtVector
    {
        public double eX = 0;
        public double eY = 0;
		

        public static double GetLength(rtVector tIn)
        {
            double eOut = 0;

            eOut = Math.Sqrt(tIn.eX*tIn.eX + tIn.eY*tIn.eY);
            return eOut;
        }

        public static double Dot(rtVector tV1, rtVector tV2)
        {
            double eOut = 0;

            eOut = tV1.eX*tV2.eX + tV1.eY*tV2.eY;
            return eOut;
        }

        public static double GetTheta(rtVector tV1, rtVector tV2)
        {
            double eTheta = 0;

            eTheta = Dot(tV1, tV2);
            eTheta /= GetLength(tV1) + GetLength(tV2);
            eTheta = Math.Acos(eTheta) * 180.0 / Math.PI;
            return eTheta;
        }

        public static double GetDistance(rtVector tP1, rtVector tP2)
        {
            double eDistance = 0;
            double eGapX = 0, eGapY = 0;

            eGapX = tP2.eX - tP1.eX;
            eGapY = tP2.eY - tP1.eY;
            eDistance = Math.Sqrt(eGapX* eGapX + eGapY* eGapY);

            return eDistance;
        }

    }

    public class rtPath_Info
    {
        public enum rtStatus {STRAIGHT = 1, TURN = 2, DONE = 0};
        public enum rtTurnType {SIMPLE = 0, SMOOTH = 1, ARRIVE = 2};

        public rtVector tSrc;
        public rtVector tDest;

        public byte ucStatus;
        public byte ucTurnType;
    }

    public class rtMotorCtrl
    {
        public enum rtNavigateStatus { UNDO = 0, DONE = 1 };

        public enum rtTurnType_Simple { ERROR = 0, TURN_RIGHT = 1, TURN_LEFT = -1 };

        /** \brief Finish Flag */
        public byte ucFinishFlag = 0;

        /** \brief path segment index */
        public int lPathNodeIndex = 0;

        /** \brief Rotation Radius of turn in smooth mode */
        public int lRotationRadius = 0;

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

        /** \brief angle threshold: 判斷是否走過頭用的 */
        public const double ANGLE_TH = 90;

        /** \brief 可以在原地打轉的角度 */
        public const int ANGLE_ROTATION = 90;

        /** \brief distance threshold of simple mode: 判斷是否到達定點 開始準備轉向動作或停止 */
        public const double DISTANCE_ERROR_SIMPLE = 40;

        /** \brief distance threshold of smooth mode: 判斷是否到達定點 開始準備轉向動作 */
        public const double DISTANCE_ERROR_SMOOTH = 1500;

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

        //public static double eAngleErrorSum = 0;
        //public static double eAngleErrorLast = 0;

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
            rtVector.GetLength(test);

            // int YYY = (int)rtPath_Info.rtStatus.DONE;
            return 1;
        }

        public static void MotorPower_SetParamsLinear(int a_lErrorBoundL, int a_lErrorBoundH, double[] a_aeMotor_Params)
        {
            a_aeMotor_Params[0] = (double)(MAX_POWER- MIN_POWER) / (a_lErrorBoundH - a_lErrorBoundL);
        }

        public static double MotorPower_StraightErrorCal(
            rtPath_Info[] a_atPathInfo, rtLocateData a_tCurrentInfo,
            double[] a_aeMotor_Params, ref rtMotorCtrl a_tMotorData
        )
        {
            double eErrorCurrent = 0, eAngle = 0, eTheta = 0;

            rtVector tV_C2D = new rtVector(); // current point to destination
            rtVector tV_Car = new rtVector(); // car current direction

            eAngle = a_tCurrentInfo.eAngle;
            tV_C2D.eX = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eX - a_tCurrentInfo.tPosition.eX;
            tV_C2D.eY = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eY - a_tCurrentInfo.tPosition.eY;
            eErrorCurrent = rtVector.GetLength(tV_C2D);

            tV_Car.eX = Math.Cos(eAngle * Math.PI / 180);
            tV_Car.eY = Math.Sin(eAngle * Math.PI / 180);

            eTheta = rtVector.GetTheta(tV_Car, tV_C2D);

            // Motor power = function(Error)
            a_tMotorData.lMotorPower = (int)(a_aeMotor_Params[0] * eErrorCurrent);
            if (eTheta >= ANGLE_TH)
            {
                a_tMotorData.lMotorPower = -a_tMotorData.lMotorPower;
            }

            return eErrorCurrent;
        }

        public static double MotorPower_TurnErrorCal(
            byte a_ucTurnType, rtPath_Info[] a_atPathInfo, rtLocateData a_tCurrentInfo, ref rtMotorCtrl a_tMotorData)
        {
            double eErrorCurrent = 0, eAngle = 0, eTheta = 0;
            int lNextPathID = 0;
            rtVector tV_Car = new rtVector(); // car current direction
            rtVector tV_NextS2D = new rtVector(); // next src point to destination

            a_tMotorData.lMotorPower = TURN_POWER;

            lNextPathID = a_tMotorData.lPathNodeIndex + 1;
            eAngle = a_tCurrentInfo.eAngle;
            tV_Car.eX = Math.Cos(eAngle * Math.PI / 180);
            tV_Car.eY = Math.Sin(eAngle * Math.PI / 180);

            tV_NextS2D.eX = a_atPathInfo[lNextPathID].tDest.eX - a_atPathInfo[lNextPathID].tSrc.eX;
            tV_NextS2D.eY = a_atPathInfo[lNextPathID].tDest.eY - a_atPathInfo[lNextPathID].tSrc.eY;

            switch (a_ucTurnType)
            {
                case (byte)rtPath_Info.rtTurnType.SIMPLE:
                    eTheta = rtVector.GetTheta(tV_Car, tV_NextS2D);
                    eErrorCurrent = eTheta;
                    break;
                case (byte)rtPath_Info.rtTurnType.SMOOTH:
                    eTheta = rtVector.GetTheta(tV_Car, tV_NextS2D);
                    eErrorCurrent = eTheta;
                    break;
                default:
                    // show error msg
                    break;
            }

            return eErrorCurrent;
        }

        public static void MotorPower_Ctrl(
            rtPath_Info[] a_atPathInfo, int a_lAGVSpeed, rtLocateData a_tCurrentInfo,
            double[] a_aeMotor_Params, ref rtMotorCtrl a_tMotorData)
        {
            double eErrorCurrent = 0;
            bool bOutFlag = false;

            switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus)
            {
                // 直走狀態
                case (byte)rtPath_Info.rtStatus.STRAIGHT:
                    eErrorCurrent = MotorPower_StraightErrorCal(
                                        a_atPathInfo, a_tCurrentInfo,
                                        a_aeMotor_Params, ref a_tMotorData);
                    switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType)
                    {
                        case (byte)rtPath_Info.rtTurnType.SIMPLE:
                            if(eErrorCurrent < DISTANCE_ERROR_SIMPLE)
                            {
                                a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtPath_Info.rtStatus.TURN;
                            }
                            break;
                        case (byte)rtPath_Info.rtTurnType.SMOOTH:
                            if (eErrorCurrent < DISTANCE_ERROR_SMOOTH)
                            {
                                a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtPath_Info.rtStatus.TURN;
                            }
                            break;
                        case (byte)rtPath_Info.rtTurnType.ARRIVE:
                            if (eErrorCurrent < DISTANCE_ERROR_SIMPLE)
                            { // 到達最終目的地
                                a_tMotorData.lMotorPower = 0;
                                a_tMotorData.lMotorAngle = 0;
                                a_tMotorData.lMotorTorsion = 0;
                                a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtPath_Info.rtStatus.DONE;
                                a_tMotorData.ucFinishFlag = (byte)rtNavigateStatus.DONE;
                            }
                            break;
                        default:
                            // show error msg
                            break;
                    }
                    break;
                // 轉彎狀態
                case (byte)rtPath_Info.rtStatus.TURN:
                    eErrorCurrent = MotorPower_TurnErrorCal(
                                        a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType,
                                        a_atPathInfo, a_tCurrentInfo, ref a_tMotorData);

                    // 算出旋轉半徑中心座標
                    bOutFlag = MotorAngle_RotationCenterCal(a_atPathInfo, a_tCurrentInfo, ref a_tMotorData);

                    switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType)
                    {
                        case (byte)rtPath_Info.rtTurnType.SIMPLE:
                            if (eErrorCurrent < THETA_ERROR_TURN)
                            {
                                a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtPath_Info.rtStatus.DONE;
                                a_tMotorData.lPathNodeIndex++;
                            }
                            break;
                        case (byte)rtPath_Info.rtTurnType.SMOOTH:
                            if (eErrorCurrent < THETA_ERROR_TURN && bOutFlag == true)
                            {
                                a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus = (byte)rtPath_Info.rtStatus.DONE;
                                a_tMotorData.lPathNodeIndex++;
                            }
                            break;
                        default:
                            // show error msg
                            break;
                    }
                    break;
                // 完成狀態: 似乎不會遇到這個 因為設定為 DONE 通常就 index ++了 >> TBD
                case (byte)rtPath_Info.rtStatus.DONE:
                    switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType)
                    {
                        case (byte)rtPath_Info.rtTurnType.SIMPLE:
                            a_tMotorData.lPathNodeIndex++;
                            a_tMotorData.ucFinishFlag = (byte)rtNavigateStatus.UNDO;
                            break;
                        case (byte)rtPath_Info.rtTurnType.SMOOTH:
                            a_tMotorData.lPathNodeIndex++;
                            a_tMotorData.ucFinishFlag = (byte)rtNavigateStatus.UNDO;
                            break;
                        case (byte)rtPath_Info.rtTurnType.ARRIVE:
                            a_tMotorData.ucFinishFlag = (byte)rtNavigateStatus.DONE;
                            a_tMotorData.lMotorPower = 0;
                            a_tMotorData.lMotorAngle = 0;
                            a_tMotorData.lMotorTorsion = 0;
                            break;
                        default:
                            // show error msg
                            break;
                    }
                    break;
                default:
                    // show error
                    break;
            }
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
            lRotationRadius = (int)DISTANCE_ERROR_SMOOTH;
        }

        public static double MotorAngle_StraightErrorCal(
            rtPath_Info[] a_atPathInfo, rtLocateData a_tCurrentInfo,
            double[] a_aeMotor_Params, rtMotorCtrl a_tMotorData
            )
        {
            double eErrorCurrent = 0, eAngle = 0, eT = 0;
            double eSrcX, eSrcY, eDestX, eDestY;
            double eCurrentX, eCurrentY;
            rtVector tVS2D = new rtVector(), tVlaw = new rtVector();


            eCurrentX = a_tCurrentInfo.tPosition.eX;
            eCurrentY = a_tCurrentInfo.tPosition.eY;
            eAngle = a_tCurrentInfo.eAngle;

            eSrcX = a_atPathInfo[a_tMotorData.lPathNodeIndex].tSrc.eX;
            eSrcY = a_atPathInfo[a_tMotorData.lPathNodeIndex].tSrc.eY;

            eDestX = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eX;
            eDestY = a_atPathInfo[a_tMotorData.lPathNodeIndex].tDest.eY;

            tVS2D.eX = eDestX - eSrcX;
            tVS2D.eY = eDestY - a_atPathInfo[a_tMotorData.lPathNodeIndex].tSrc.eY;

            // 取右側的法向量
            tVlaw.eX = tVS2D.eY;
            tVlaw.eY = -tVS2D.eX;

            eT = eSrcX * tVS2D.eY - eSrcY * tVS2D.eX - eCurrentX * tVS2D.eY + eCurrentY * tVS2D.eX;
            eT /= tVlaw.eX * tVS2D.eY - tVlaw.eY * tVS2D.eX;

            // 算出目前座標到切線的距離當誤差
            eErrorCurrent = Math.Sqrt((tVlaw.eX * eT) * (tVlaw.eX * eT) + (tVlaw.eY * eT) * (tVlaw.eY * eT));

            if (eT > 0)
            { // left side >> tire need to turn left
                eErrorCurrent = -eErrorCurrent;
            }

            return eErrorCurrent;
        }

        public static bool MotorAngle_RotationCenterCal(
            rtPath_Info[] a_atPathInfo, rtLocateData a_tCurrentInfo,
            ref rtMotorCtrl a_tMotorData
            )
        {
            int lPathIndex = 0;
            double eT = 0; // 比例
            double eLength = 0;
            double eThetaBoundaty = 0;
            double eThetaCurrent = 0;
            rtVector tSrc = new rtVector();
            rtVector tDest = new rtVector();
            rtVector tVd2sCurrent = new rtVector();
            rtVector tVs2dNext = new rtVector();
            rtVector tVd2sCurrentLaw = new rtVector();
            rtVector tVs2dNextLaw = new rtVector();
            rtVector tCurrent = new rtVector();
            rtVector tNext = new rtVector();

            rtVector tCenter2SrcTurn = new rtVector();
            rtVector tCenter2DestTurn = new rtVector();
            rtVector tCenter2Current = new rtVector();

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
            eLength = rtVector.GetLength(tVd2sCurrent);
            eT = a_tMotorData.lRotationRadius / eLength;
            tCurrent.eX = tDest.eX + eT * tVd2sCurrent.eX;
            tCurrent.eY = tDest.eX + eT * tVd2sCurrent.eY;

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
            eLength = rtVector.GetLength(tVs2dNext);
            eT = a_tMotorData.lRotationRadius / eLength;
            tNext.eX = tSrc.eX + eT * tVs2dNext.eX;
            tNext.eY = tSrc.eX + eT * tVs2dNext.eY;


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

            eThetaBoundaty = rtVector.GetTheta(tCenter2DestTurn, tCenter2SrcTurn);
            eThetaCurrent = rtVector.GetTheta(tCenter2Current, tCenter2SrcTurn);

            // 判斷旋轉圓心在路徑向量的左邊還是右邊
            if(tVd2sCurrentLaw.eX != 0)
            {
                eT = (a_tMotorData.tRotateCenter.eX - a_tMotorData.tRotateCenter.eX) / tVd2sCurrentLaw.eX;
                
            }
            else if (tVd2sCurrentLaw.eY != 0)
            {
                eT = (a_tMotorData.tRotateCenter.eY - a_tMotorData.tRotateCenter.eY) / tVd2sCurrentLaw.eY;
            }
            else
            {
                // 不可能為0向量 show error
                eT = 0;
            }

            if(eT > 0)
            { // 點在左邊 >> 向右轉
                a_tMotorData.lTurnDirection = (int)rtTurnType_Simple.TURN_RIGHT;
            }
            else if (eT < 0)
            {
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
            rtPath_Info[] a_atPathInfo, rtLocateData a_tCurrentInfo,
            double[] a_aeMotor_Params, ref rtMotorCtrl a_tMotorData
            )
        {
            double eErrorCurrent = 0;
            double eDistance = 0;
            byte ucTurnType = 0;
            bool bOutFlag = false;
            rtVector tVS2D = new rtVector(), tVlaw = new rtVector();

            // 算出旋轉半徑中心座標
            bOutFlag = MotorAngle_RotationCenterCal(a_atPathInfo, a_tCurrentInfo, ref a_tMotorData);

            ucTurnType = a_atPathInfo[a_tMotorData.lPathNodeIndex].ucTurnType;

            switch (ucTurnType)
            {
                case (byte)rtPath_Info.rtTurnType.SIMPLE:
                    // Do nothing >> 原本不需要做事情 但需要知道是左90度還是右90度
                    break;
                case (byte)rtPath_Info.rtTurnType.SMOOTH:
                    eDistance = rtVector.GetDistance(a_tCurrentInfo.tPosition, a_tMotorData.tRotateCenter);
                    eErrorCurrent = a_tMotorData.lRotationRadius - eDistance; // 可能會有錯誤 如果在內側非扇型區域 >> TBD
                    break;
                case (byte)rtPath_Info.rtTurnType.ARRIVE:
                    // show error msg
                    break;
                default:
                    // show error msg
                    break;
            }

            return eErrorCurrent;
        }

        public static void MotorAngle_Ctrl(
            rtPath_Info[] a_atPathInfo, rtLocateData a_tCurrentInfo, int a_lAGVSpeed, 
            double[] a_aeMotor_Params, ref rtMotorCtrl a_tMotorData)
        {
            double eErrorCurrent = 0, eErrorNext = 0;
            double eCarAngle = 0, ePhi = 0; 
            byte ucSinpleModeFlag = 0; // 0: OFF 1: ON
            int lPathIndex = 0;
            rtVector tV_Car = new rtVector(); // car current direction
            rtVector tV_S2D = new rtVector(); // src point to destination

            lPathIndex = a_tMotorData.lPathNodeIndex;

            eCarAngle = a_tCurrentInfo.eAngle;
            tV_Car.eX = Math.Cos(eCarAngle * Math.PI / 180);
            tV_Car.eY = Math.Sin(eCarAngle * Math.PI / 180);

            tV_S2D.eX = a_atPathInfo[lPathIndex].tDest.eX - a_atPathInfo[lPathIndex].tSrc.eX;
            tV_S2D.eY = a_atPathInfo[lPathIndex].tDest.eY - a_atPathInfo[lPathIndex].tSrc.eY;

            ePhi = rtVector.GetTheta(tV_Car, tV_S2D);

            switch (a_atPathInfo[a_tMotorData.lPathNodeIndex].ucStatus)
            {
                // 直走狀態
                case (byte)rtPath_Info.rtStatus.STRAIGHT:
                    eErrorCurrent = MotorAngle_StraightErrorCal(a_atPathInfo, a_tCurrentInfo, a_aeMotor_Params, a_tMotorData);
                    break;
                // 轉彎狀態
                case (byte)rtPath_Info.rtStatus.TURN:
                    eErrorCurrent = MotorAngle_TurnErrorCal(a_atPathInfo, a_tCurrentInfo, a_aeMotor_Params, ref a_tMotorData);
                    if(a_atPathInfo[lPathIndex].ucTurnType == (byte)rtPath_Info.rtTurnType.SIMPLE)
                    {
                        ucSinpleModeFlag = 1;
                    }
                    
                    break;
                // 完成狀態: 似乎不會遇到這個 因為設定為 DONE 通常就 index ++了 >> TBD
                case (byte)rtPath_Info.rtStatus.DONE:
                    // show error
                    break;
                default:
                    // show error
                    break;
            }

            eErrorCurrent = -eErrorCurrent; // error 左邊是正 右邊是負 >> 但轉向角 左邊得向左轉(-) 右邊得向右轉(+) 所以取負號

            a_tMotorData.eAngleErrorSum += eErrorCurrent;

            eErrorNext = eErrorCurrent + (eErrorCurrent - a_tMotorData.eAngleErrorLast);

            a_tMotorData.eAngleErrorLast = eErrorCurrent;

            // angle = function(Error) >> Kp*eErrorCurrent + Ki*eAngleErrorSum + Kd*eErrorNext
            a_tMotorData.lMotorAngle = (int)(a_aeMotor_Params[0] * eErrorCurrent + a_aeMotor_Params[1] * a_tMotorData.eAngleErrorSum + a_aeMotor_Params[2] * eErrorNext);



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

        }
    }
}