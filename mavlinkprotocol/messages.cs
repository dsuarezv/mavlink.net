using System;
using System.IO;

namespace MavLinkNet
{

    public enum MavAutopilot { Generic, Pixhawk, Slugs, Ardupilotmega, Openpilot, GenericWaypointsOnly, GenericWaypointsAndSimpleNavigationOnly, GenericMissionFull, Invalid, Ppz, Udb, Fp, Px4, Smaccmpilot, Autoquad, Armazila, Aerob };

    public enum MavType { Generic, FixedWing, Quadrotor, Coaxial, Helicopter, AntennaTracker, Gcs, Airship, FreeBalloon, Rocket, GroundRover, SurfaceBoat, Submarine, Hexarotor, Octorotor, Tricopter, FlappingWing, Kite };

    public enum MavModeFlag { SafetyArmed, ManualInputEnabled, HilEnabled, StabilizeEnabled, GuidedEnabled, AutoEnabled, TestEnabled, CustomModeEnabled };

    public enum MavModeFlagDecodePosition { Safety, Manual, Hil, Stabilize, Guided, Auto, Test, CustomMode };

    public enum MavGoto { DoHold, DoContinue, HoldAtCurrentPosition, HoldAtSpecifiedPosition };

    public enum MavMode { Preflight, StabilizeDisarmed, StabilizeArmed, ManualDisarmed, ManualArmed, GuidedDisarmed, GuidedArmed, AutoDisarmed, AutoArmed, TestDisarmed, TestArmed };

    public enum MavState { Uninit, Boot, Calibrating, Standby, Active, Critical, Emergency, Poweroff };

    public enum MavComponent { MavCompIdAll, MavCompIdGps, MavCompIdMissionplanner, MavCompIdPathplanner, MavCompIdMapper, MavCompIdCamera, MavCompIdImu, MavCompIdImu2, MavCompIdImu3, MavCompIdUdpBridge, MavCompIdUartBridge, MavCompIdSystemControl, MavCompIdServo1, MavCompIdServo2, MavCompIdServo3, MavCompIdServo4, MavCompIdServo5, MavCompIdServo6, MavCompIdServo7, MavCompIdServo8, MavCompIdServo9, MavCompIdServo10, MavCompIdServo11, MavCompIdServo12, MavCompIdServo13, MavCompIdServo14 };

    public enum MavSysStatusSensor { _3dGyro, _3dAccel, _3dMag, AbsolutePressure, DifferentialPressure, Gps, OpticalFlow, VisionPosition, LaserPosition, ExternalGroundTruth, AngularRateControl, AttitudeStabilization, YawPosition, ZAltitudeControl, XyPositionControl, MotorOutputs, RcReceiver };

    public enum MavFrame { Global, LocalNed, Mission, GlobalRelativeAlt, LocalEnu };

    public enum MavlinkDataStreamType { MavlinkDataStreamImgJpeg, MavlinkDataStreamImgBmp, MavlinkDataStreamImgRaw8u, MavlinkDataStreamImgRaw32u, MavlinkDataStreamImgPgm, MavlinkDataStreamImgPng };

    public enum MavCmd { NavWaypoint, NavLoiterUnlim, NavLoiterTurns, NavLoiterTime, NavReturnToLaunch, NavLand, NavTakeoff, NavRoi, NavPathplanning, NavLast, ConditionDelay, ConditionChangeAlt, ConditionDistance, ConditionYaw, ConditionLast, DoSetMode, DoJump, DoChangeSpeed, DoSetHome, DoSetParameter, DoSetRelay, DoRepeatRelay, DoSetServo, DoRepeatServo, DoControlVideo, DoSetRoi, DoLast, PreflightCalibration, PreflightSetSensorOffsets, PreflightStorage, PreflightRebootShutdown, OverrideGoto, MissionStart, ComponentArmDisarm, StartRxPair };

    public enum MavDataStream { All, RawSensors, ExtendedStatus, RcChannels, RawController, Position, Extra1, Extra2, Extra3 };

    public enum MavRoi { None, Wpnext, Wpindex, Location, Target };

    public enum MavCmdAck { Ok, ErrFail, ErrAccessDenied, ErrNotSupported, ErrCoordinateFrameNotSupported, ErrCoordinatesOutOfRange, ErrXLatOutOfRange, ErrYLonOutOfRange, ErrZAltOutOfRange };

    public enum MavParamType { Uint8, Int8, Uint16, Int16, Uint32, Int32, Uint64, Int64, Real32, Real64 };

    public enum MavResult { Accepted, TemporarilyRejected, Denied, Unsupported, Failed };

    public enum MavMissionResult { MavMissionAccepted, MavMissionError, MavMissionUnsupportedFrame, MavMissionUnsupported, MavMissionNoSpace, MavMissionInvalid, MavMissionInvalidParam1, MavMissionInvalidParam2, MavMissionInvalidParam3, MavMissionInvalidParam4, MavMissionInvalidParam5X, MavMissionInvalidParam6Y, MavMissionInvalidParam7, MavMissionInvalidSequence, MavMissionDenied };

    public enum MavSeverity { Emergency, Alert, Critical, Error, Warning, Notice, Info, Debug };


    // ___________________________________________________________________________________


    public class UasHeartbeat: UasMessage
    {
        public UInt32 CustomMode {
            get { return mCustomMode; }
            set { mCustomMode = value; NotifyUpdated(); }
        }

        public byte Type {
            get { return mType; }
            set { mType = value; NotifyUpdated(); }
        }

        public byte Autopilot {
            get { return mAutopilot; }
            set { mAutopilot = value; NotifyUpdated(); }
        }

        public byte BaseMode {
            get { return mBaseMode; }
            set { mBaseMode = value; NotifyUpdated(); }
        }

        public byte SystemStatus {
            get { return mSystemStatus; }
            set { mSystemStatus = value; NotifyUpdated(); }
        }

        public byte MavlinkVersion {
            get { return mMavlinkVersion; }
            set { mMavlinkVersion = value; NotifyUpdated(); }
        }

        private UInt32 mCustomMode;
        private byte mType;
        private byte mAutopilot;
        private byte mBaseMode;
        private byte mSystemStatus;
        private byte mMavlinkVersion;
    }


    // ___________________________________________________________________________________


    public class UasSysStatus: UasMessage
    {
        public UInt32 OnboardControlSensorsPresent {
            get { return mOnboardControlSensorsPresent; }
            set { mOnboardControlSensorsPresent = value; NotifyUpdated(); }
        }

        public UInt32 OnboardControlSensorsEnabled {
            get { return mOnboardControlSensorsEnabled; }
            set { mOnboardControlSensorsEnabled = value; NotifyUpdated(); }
        }

        public UInt32 OnboardControlSensorsHealth {
            get { return mOnboardControlSensorsHealth; }
            set { mOnboardControlSensorsHealth = value; NotifyUpdated(); }
        }

        public UInt16 Load {
            get { return mLoad; }
            set { mLoad = value; NotifyUpdated(); }
        }

        public UInt16 VoltageBattery {
            get { return mVoltageBattery; }
            set { mVoltageBattery = value; NotifyUpdated(); }
        }

        public Int16 CurrentBattery {
            get { return mCurrentBattery; }
            set { mCurrentBattery = value; NotifyUpdated(); }
        }

        public UInt16 DropRateComm {
            get { return mDropRateComm; }
            set { mDropRateComm = value; NotifyUpdated(); }
        }

        public UInt16 ErrorsComm {
            get { return mErrorsComm; }
            set { mErrorsComm = value; NotifyUpdated(); }
        }

        public UInt16 ErrorsCount1 {
            get { return mErrorsCount1; }
            set { mErrorsCount1 = value; NotifyUpdated(); }
        }

        public UInt16 ErrorsCount2 {
            get { return mErrorsCount2; }
            set { mErrorsCount2 = value; NotifyUpdated(); }
        }

        public UInt16 ErrorsCount3 {
            get { return mErrorsCount3; }
            set { mErrorsCount3 = value; NotifyUpdated(); }
        }

        public UInt16 ErrorsCount4 {
            get { return mErrorsCount4; }
            set { mErrorsCount4 = value; NotifyUpdated(); }
        }

        public SByte BatteryRemaining {
            get { return mBatteryRemaining; }
            set { mBatteryRemaining = value; NotifyUpdated(); }
        }

        private UInt32 mOnboardControlSensorsPresent;
        private UInt32 mOnboardControlSensorsEnabled;
        private UInt32 mOnboardControlSensorsHealth;
        private UInt16 mLoad;
        private UInt16 mVoltageBattery;
        private Int16 mCurrentBattery;
        private UInt16 mDropRateComm;
        private UInt16 mErrorsComm;
        private UInt16 mErrorsCount1;
        private UInt16 mErrorsCount2;
        private UInt16 mErrorsCount3;
        private UInt16 mErrorsCount4;
        private SByte mBatteryRemaining;
    }


    // ___________________________________________________________________________________


    public class UasSystemTime: UasMessage
    {
        public UInt64 TimeUnixUsec {
            get { return mTimeUnixUsec; }
            set { mTimeUnixUsec = value; NotifyUpdated(); }
        }

        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUnixUsec;
        private UInt32 mTimeBootMs;
    }


    // ___________________________________________________________________________________


    public class UasPing: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public UInt32 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private UInt32 mSeq;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasChangeOperatorControl: UasMessage
    {
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte ControlRequest {
            get { return mControlRequest; }
            set { mControlRequest = value; NotifyUpdated(); }
        }

        public byte Version {
            get { return mVersion; }
            set { mVersion = value; NotifyUpdated(); }
        }

        public char[] Passkey {
            get { return mPasskey; }
            set { mPasskey = value; NotifyUpdated(); }
        }

        private byte mTargetSystem;
        private byte mControlRequest;
        private byte mVersion;
        private char[] mPasskey = new char[25];
    }


    // ___________________________________________________________________________________


    public class UasChangeOperatorControlAck: UasMessage
    {
        public byte GcsSystemId {
            get { return mGcsSystemId; }
            set { mGcsSystemId = value; NotifyUpdated(); }
        }

        public byte ControlRequest {
            get { return mControlRequest; }
            set { mControlRequest = value; NotifyUpdated(); }
        }

        public byte Ack {
            get { return mAck; }
            set { mAck = value; NotifyUpdated(); }
        }

        private byte mGcsSystemId;
        private byte mControlRequest;
        private byte mAck;
    }


    // ___________________________________________________________________________________


    public class UasAuthKey: UasMessage
    {
        public char[] Key {
            get { return mKey; }
            set { mKey = value; NotifyUpdated(); }
        }

        private char[] mKey = new char[32];
    }


    // ___________________________________________________________________________________


    public class UasSetMode: UasMessage
    {
        public UInt32 CustomMode {
            get { return mCustomMode; }
            set { mCustomMode = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte BaseMode {
            get { return mBaseMode; }
            set { mBaseMode = value; NotifyUpdated(); }
        }

        private UInt32 mCustomMode;
        private byte mTargetSystem;
        private byte mBaseMode;
    }


    // ___________________________________________________________________________________


    public class UasParamRequestRead: UasMessage
    {
        public Int16 ParamIndex {
            get { return mParamIndex; }
            set { mParamIndex = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public char[] ParamId {
            get { return mParamId; }
            set { mParamId = value; NotifyUpdated(); }
        }

        private Int16 mParamIndex;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private char[] mParamId = new char[16];
    }


    // ___________________________________________________________________________________


    public class UasParamRequestList: UasMessage
    {
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasParamValue: UasMessage
    {
        public float ParamValue {
            get { return mParamValue; }
            set { mParamValue = value; NotifyUpdated(); }
        }

        public UInt16 ParamCount {
            get { return mParamCount; }
            set { mParamCount = value; NotifyUpdated(); }
        }

        public UInt16 ParamIndex {
            get { return mParamIndex; }
            set { mParamIndex = value; NotifyUpdated(); }
        }

        public char[] ParamId {
            get { return mParamId; }
            set { mParamId = value; NotifyUpdated(); }
        }

        public byte ParamType {
            get { return mParamType; }
            set { mParamType = value; NotifyUpdated(); }
        }

        private float mParamValue;
        private UInt16 mParamCount;
        private UInt16 mParamIndex;
        private char[] mParamId = new char[16];
        private byte mParamType;
    }


    // ___________________________________________________________________________________


    public class UasParamSet: UasMessage
    {
        public float ParamValue {
            get { return mParamValue; }
            set { mParamValue = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public char[] ParamId {
            get { return mParamId; }
            set { mParamId = value; NotifyUpdated(); }
        }

        public byte ParamType {
            get { return mParamType; }
            set { mParamType = value; NotifyUpdated(); }
        }

        private float mParamValue;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private char[] mParamId = new char[16];
        private byte mParamType;
    }


    // ___________________________________________________________________________________


    public class UasGpsRawInt: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public Int32 Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        public Int32 Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        public Int32 Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        public UInt16 Eph {
            get { return mEph; }
            set { mEph = value; NotifyUpdated(); }
        }

        public UInt16 Epv {
            get { return mEpv; }
            set { mEpv = value; NotifyUpdated(); }
        }

        public UInt16 Vel {
            get { return mVel; }
            set { mVel = value; NotifyUpdated(); }
        }

        public UInt16 Cog {
            get { return mCog; }
            set { mCog = value; NotifyUpdated(); }
        }

        public byte FixType {
            get { return mFixType; }
            set { mFixType = value; NotifyUpdated(); }
        }

        public byte SatellitesVisible {
            get { return mSatellitesVisible; }
            set { mSatellitesVisible = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private Int32 mLat;
        private Int32 mLon;
        private Int32 mAlt;
        private UInt16 mEph;
        private UInt16 mEpv;
        private UInt16 mVel;
        private UInt16 mCog;
        private byte mFixType;
        private byte mSatellitesVisible;
    }


    // ___________________________________________________________________________________


    public class UasGpsStatus: UasMessage
    {
        public byte SatellitesVisible {
            get { return mSatellitesVisible; }
            set { mSatellitesVisible = value; NotifyUpdated(); }
        }

        public byte[] SatellitePrn {
            get { return mSatellitePrn; }
            set { mSatellitePrn = value; NotifyUpdated(); }
        }

        public byte[] SatelliteUsed {
            get { return mSatelliteUsed; }
            set { mSatelliteUsed = value; NotifyUpdated(); }
        }

        public byte[] SatelliteElevation {
            get { return mSatelliteElevation; }
            set { mSatelliteElevation = value; NotifyUpdated(); }
        }

        public byte[] SatelliteAzimuth {
            get { return mSatelliteAzimuth; }
            set { mSatelliteAzimuth = value; NotifyUpdated(); }
        }

        public byte[] SatelliteSnr {
            get { return mSatelliteSnr; }
            set { mSatelliteSnr = value; NotifyUpdated(); }
        }

        private byte mSatellitesVisible;
        private byte[] mSatellitePrn = new byte[20];
        private byte[] mSatelliteUsed = new byte[20];
        private byte[] mSatelliteElevation = new byte[20];
        private byte[] mSatelliteAzimuth = new byte[20];
        private byte[] mSatelliteSnr = new byte[20];
    }


    // ___________________________________________________________________________________


    public class UasScaledImu: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public Int16 Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        public Int16 Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        public Int16 Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        public Int16 Xgyro {
            get { return mXgyro; }
            set { mXgyro = value; NotifyUpdated(); }
        }

        public Int16 Ygyro {
            get { return mYgyro; }
            set { mYgyro = value; NotifyUpdated(); }
        }

        public Int16 Zgyro {
            get { return mZgyro; }
            set { mZgyro = value; NotifyUpdated(); }
        }

        public Int16 Xmag {
            get { return mXmag; }
            set { mXmag = value; NotifyUpdated(); }
        }

        public Int16 Ymag {
            get { return mYmag; }
            set { mYmag = value; NotifyUpdated(); }
        }

        public Int16 Zmag {
            get { return mZmag; }
            set { mZmag = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private Int16 mXacc;
        private Int16 mYacc;
        private Int16 mZacc;
        private Int16 mXgyro;
        private Int16 mYgyro;
        private Int16 mZgyro;
        private Int16 mXmag;
        private Int16 mYmag;
        private Int16 mZmag;
    }


    // ___________________________________________________________________________________


    public class UasRawImu: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public Int16 Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        public Int16 Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        public Int16 Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        public Int16 Xgyro {
            get { return mXgyro; }
            set { mXgyro = value; NotifyUpdated(); }
        }

        public Int16 Ygyro {
            get { return mYgyro; }
            set { mYgyro = value; NotifyUpdated(); }
        }

        public Int16 Zgyro {
            get { return mZgyro; }
            set { mZgyro = value; NotifyUpdated(); }
        }

        public Int16 Xmag {
            get { return mXmag; }
            set { mXmag = value; NotifyUpdated(); }
        }

        public Int16 Ymag {
            get { return mYmag; }
            set { mYmag = value; NotifyUpdated(); }
        }

        public Int16 Zmag {
            get { return mZmag; }
            set { mZmag = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private Int16 mXacc;
        private Int16 mYacc;
        private Int16 mZacc;
        private Int16 mXgyro;
        private Int16 mYgyro;
        private Int16 mZgyro;
        private Int16 mXmag;
        private Int16 mYmag;
        private Int16 mZmag;
    }


    // ___________________________________________________________________________________


    public class UasRawPressure: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public Int16 PressAbs {
            get { return mPressAbs; }
            set { mPressAbs = value; NotifyUpdated(); }
        }

        public Int16 PressDiff1 {
            get { return mPressDiff1; }
            set { mPressDiff1 = value; NotifyUpdated(); }
        }

        public Int16 PressDiff2 {
            get { return mPressDiff2; }
            set { mPressDiff2 = value; NotifyUpdated(); }
        }

        public Int16 Temperature {
            get { return mTemperature; }
            set { mTemperature = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private Int16 mPressAbs;
        private Int16 mPressDiff1;
        private Int16 mPressDiff2;
        private Int16 mTemperature;
    }


    // ___________________________________________________________________________________


    public class UasScaledPressure: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public float PressAbs {
            get { return mPressAbs; }
            set { mPressAbs = value; NotifyUpdated(); }
        }

        public float PressDiff {
            get { return mPressDiff; }
            set { mPressDiff = value; NotifyUpdated(); }
        }

        public Int16 Temperature {
            get { return mTemperature; }
            set { mTemperature = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private float mPressAbs;
        private float mPressDiff;
        private Int16 mTemperature;
    }


    // ___________________________________________________________________________________


    public class UasAttitude: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public float Rollspeed {
            get { return mRollspeed; }
            set { mRollspeed = value; NotifyUpdated(); }
        }

        public float Pitchspeed {
            get { return mPitchspeed; }
            set { mPitchspeed = value; NotifyUpdated(); }
        }

        public float Yawspeed {
            get { return mYawspeed; }
            set { mYawspeed = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private float mRoll;
        private float mPitch;
        private float mYaw;
        private float mRollspeed;
        private float mPitchspeed;
        private float mYawspeed;
    }


    // ___________________________________________________________________________________


    public class UasAttitudeQuaternion: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public float Q1 {
            get { return mQ1; }
            set { mQ1 = value; NotifyUpdated(); }
        }

        public float Q2 {
            get { return mQ2; }
            set { mQ2 = value; NotifyUpdated(); }
        }

        public float Q3 {
            get { return mQ3; }
            set { mQ3 = value; NotifyUpdated(); }
        }

        public float Q4 {
            get { return mQ4; }
            set { mQ4 = value; NotifyUpdated(); }
        }

        public float Rollspeed {
            get { return mRollspeed; }
            set { mRollspeed = value; NotifyUpdated(); }
        }

        public float Pitchspeed {
            get { return mPitchspeed; }
            set { mPitchspeed = value; NotifyUpdated(); }
        }

        public float Yawspeed {
            get { return mYawspeed; }
            set { mYawspeed = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private float mQ1;
        private float mQ2;
        private float mQ3;
        private float mQ4;
        private float mRollspeed;
        private float mPitchspeed;
        private float mYawspeed;
    }


    // ___________________________________________________________________________________


    public class UasLocalPositionNed: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        public float Vx {
            get { return mVx; }
            set { mVx = value; NotifyUpdated(); }
        }

        public float Vy {
            get { return mVy; }
            set { mVy = value; NotifyUpdated(); }
        }

        public float Vz {
            get { return mVz; }
            set { mVz = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private float mX;
        private float mY;
        private float mZ;
        private float mVx;
        private float mVy;
        private float mVz;
    }


    // ___________________________________________________________________________________


    public class UasGlobalPositionInt: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public Int32 Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        public Int32 Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        public Int32 Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        public Int32 RelativeAlt {
            get { return mRelativeAlt; }
            set { mRelativeAlt = value; NotifyUpdated(); }
        }

        public Int16 Vx {
            get { return mVx; }
            set { mVx = value; NotifyUpdated(); }
        }

        public Int16 Vy {
            get { return mVy; }
            set { mVy = value; NotifyUpdated(); }
        }

        public Int16 Vz {
            get { return mVz; }
            set { mVz = value; NotifyUpdated(); }
        }

        public UInt16 Hdg {
            get { return mHdg; }
            set { mHdg = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private Int32 mLat;
        private Int32 mLon;
        private Int32 mAlt;
        private Int32 mRelativeAlt;
        private Int16 mVx;
        private Int16 mVy;
        private Int16 mVz;
        private UInt16 mHdg;
    }


    // ___________________________________________________________________________________


    public class UasRcChannelsScaled: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public Int16 Chan1Scaled {
            get { return mChan1Scaled; }
            set { mChan1Scaled = value; NotifyUpdated(); }
        }

        public Int16 Chan2Scaled {
            get { return mChan2Scaled; }
            set { mChan2Scaled = value; NotifyUpdated(); }
        }

        public Int16 Chan3Scaled {
            get { return mChan3Scaled; }
            set { mChan3Scaled = value; NotifyUpdated(); }
        }

        public Int16 Chan4Scaled {
            get { return mChan4Scaled; }
            set { mChan4Scaled = value; NotifyUpdated(); }
        }

        public Int16 Chan5Scaled {
            get { return mChan5Scaled; }
            set { mChan5Scaled = value; NotifyUpdated(); }
        }

        public Int16 Chan6Scaled {
            get { return mChan6Scaled; }
            set { mChan6Scaled = value; NotifyUpdated(); }
        }

        public Int16 Chan7Scaled {
            get { return mChan7Scaled; }
            set { mChan7Scaled = value; NotifyUpdated(); }
        }

        public Int16 Chan8Scaled {
            get { return mChan8Scaled; }
            set { mChan8Scaled = value; NotifyUpdated(); }
        }

        public byte Port {
            get { return mPort; }
            set { mPort = value; NotifyUpdated(); }
        }

        public byte Rssi {
            get { return mRssi; }
            set { mRssi = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private Int16 mChan1Scaled;
        private Int16 mChan2Scaled;
        private Int16 mChan3Scaled;
        private Int16 mChan4Scaled;
        private Int16 mChan5Scaled;
        private Int16 mChan6Scaled;
        private Int16 mChan7Scaled;
        private Int16 mChan8Scaled;
        private byte mPort;
        private byte mRssi;
    }


    // ___________________________________________________________________________________


    public class UasRcChannelsRaw: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public UInt16 Chan1Raw {
            get { return mChan1Raw; }
            set { mChan1Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan2Raw {
            get { return mChan2Raw; }
            set { mChan2Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan3Raw {
            get { return mChan3Raw; }
            set { mChan3Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan4Raw {
            get { return mChan4Raw; }
            set { mChan4Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan5Raw {
            get { return mChan5Raw; }
            set { mChan5Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan6Raw {
            get { return mChan6Raw; }
            set { mChan6Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan7Raw {
            get { return mChan7Raw; }
            set { mChan7Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan8Raw {
            get { return mChan8Raw; }
            set { mChan8Raw = value; NotifyUpdated(); }
        }

        public byte Port {
            get { return mPort; }
            set { mPort = value; NotifyUpdated(); }
        }

        public byte Rssi {
            get { return mRssi; }
            set { mRssi = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private UInt16 mChan1Raw;
        private UInt16 mChan2Raw;
        private UInt16 mChan3Raw;
        private UInt16 mChan4Raw;
        private UInt16 mChan5Raw;
        private UInt16 mChan6Raw;
        private UInt16 mChan7Raw;
        private UInt16 mChan8Raw;
        private byte mPort;
        private byte mRssi;
    }


    // ___________________________________________________________________________________


    public class UasServoOutputRaw: UasMessage
    {
        public UInt32 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public UInt16 Servo1Raw {
            get { return mServo1Raw; }
            set { mServo1Raw = value; NotifyUpdated(); }
        }

        public UInt16 Servo2Raw {
            get { return mServo2Raw; }
            set { mServo2Raw = value; NotifyUpdated(); }
        }

        public UInt16 Servo3Raw {
            get { return mServo3Raw; }
            set { mServo3Raw = value; NotifyUpdated(); }
        }

        public UInt16 Servo4Raw {
            get { return mServo4Raw; }
            set { mServo4Raw = value; NotifyUpdated(); }
        }

        public UInt16 Servo5Raw {
            get { return mServo5Raw; }
            set { mServo5Raw = value; NotifyUpdated(); }
        }

        public UInt16 Servo6Raw {
            get { return mServo6Raw; }
            set { mServo6Raw = value; NotifyUpdated(); }
        }

        public UInt16 Servo7Raw {
            get { return mServo7Raw; }
            set { mServo7Raw = value; NotifyUpdated(); }
        }

        public UInt16 Servo8Raw {
            get { return mServo8Raw; }
            set { mServo8Raw = value; NotifyUpdated(); }
        }

        public byte Port {
            get { return mPort; }
            set { mPort = value; NotifyUpdated(); }
        }

        private UInt32 mTimeUsec;
        private UInt16 mServo1Raw;
        private UInt16 mServo2Raw;
        private UInt16 mServo3Raw;
        private UInt16 mServo4Raw;
        private UInt16 mServo5Raw;
        private UInt16 mServo6Raw;
        private UInt16 mServo7Raw;
        private UInt16 mServo8Raw;
        private byte mPort;
    }


    // ___________________________________________________________________________________


    public class UasMissionRequestPartialList: UasMessage
    {
        public Int16 StartIndex {
            get { return mStartIndex; }
            set { mStartIndex = value; NotifyUpdated(); }
        }

        public Int16 EndIndex {
            get { return mEndIndex; }
            set { mEndIndex = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private Int16 mStartIndex;
        private Int16 mEndIndex;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasMissionWritePartialList: UasMessage
    {
        public Int16 StartIndex {
            get { return mStartIndex; }
            set { mStartIndex = value; NotifyUpdated(); }
        }

        public Int16 EndIndex {
            get { return mEndIndex; }
            set { mEndIndex = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private Int16 mStartIndex;
        private Int16 mEndIndex;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasMissionItem: UasMessage
    {
        public float Param1 {
            get { return mParam1; }
            set { mParam1 = value; NotifyUpdated(); }
        }

        public float Param2 {
            get { return mParam2; }
            set { mParam2 = value; NotifyUpdated(); }
        }

        public float Param3 {
            get { return mParam3; }
            set { mParam3 = value; NotifyUpdated(); }
        }

        public float Param4 {
            get { return mParam4; }
            set { mParam4 = value; NotifyUpdated(); }
        }

        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        public UInt16 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        public UInt16 Command {
            get { return mCommand; }
            set { mCommand = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public byte Frame {
            get { return mFrame; }
            set { mFrame = value; NotifyUpdated(); }
        }

        public byte Current {
            get { return mCurrent; }
            set { mCurrent = value; NotifyUpdated(); }
        }

        public byte Autocontinue {
            get { return mAutocontinue; }
            set { mAutocontinue = value; NotifyUpdated(); }
        }

        private float mParam1;
        private float mParam2;
        private float mParam3;
        private float mParam4;
        private float mX;
        private float mY;
        private float mZ;
        private UInt16 mSeq;
        private UInt16 mCommand;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private byte mFrame;
        private byte mCurrent;
        private byte mAutocontinue;
    }


    // ___________________________________________________________________________________


    public class UasMissionRequest: UasMessage
    {
        public UInt16 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private UInt16 mSeq;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasMissionSetCurrent: UasMessage
    {
        public UInt16 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private UInt16 mSeq;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasMissionCurrent: UasMessage
    {
        public UInt16 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        private UInt16 mSeq;
    }


    // ___________________________________________________________________________________


    public class UasMissionRequestList: UasMessage
    {
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasMissionCount: UasMessage
    {
        public UInt16 Count {
            get { return mCount; }
            set { mCount = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private UInt16 mCount;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasMissionClearAll: UasMessage
    {
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasMissionItemReached: UasMessage
    {
        public UInt16 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        private UInt16 mSeq;
    }


    // ___________________________________________________________________________________


    public class UasMissionAck: UasMessage
    {
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public byte Type {
            get { return mType; }
            set { mType = value; NotifyUpdated(); }
        }

        private byte mTargetSystem;
        private byte mTargetComponent;
        private byte mType;
    }


    // ___________________________________________________________________________________


    public class UasSetGpsGlobalOrigin: UasMessage
    {
        public Int32 Latitude {
            get { return mLatitude; }
            set { mLatitude = value; NotifyUpdated(); }
        }

        public Int32 Longitude {
            get { return mLongitude; }
            set { mLongitude = value; NotifyUpdated(); }
        }

        public Int32 Altitude {
            get { return mAltitude; }
            set { mAltitude = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        private Int32 mLatitude;
        private Int32 mLongitude;
        private Int32 mAltitude;
        private byte mTargetSystem;
    }


    // ___________________________________________________________________________________


    public class UasGpsGlobalOrigin: UasMessage
    {
        public Int32 Latitude {
            get { return mLatitude; }
            set { mLatitude = value; NotifyUpdated(); }
        }

        public Int32 Longitude {
            get { return mLongitude; }
            set { mLongitude = value; NotifyUpdated(); }
        }

        public Int32 Altitude {
            get { return mAltitude; }
            set { mAltitude = value; NotifyUpdated(); }
        }

        private Int32 mLatitude;
        private Int32 mLongitude;
        private Int32 mAltitude;
    }


    // ___________________________________________________________________________________


    public class UasSetLocalPositionSetpoint: UasMessage
    {
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public byte CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        private float mX;
        private float mY;
        private float mZ;
        private float mYaw;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private byte mCoordinateFrame;
    }


    // ___________________________________________________________________________________


    public class UasLocalPositionSetpoint: UasMessage
    {
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public byte CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        private float mX;
        private float mY;
        private float mZ;
        private float mYaw;
        private byte mCoordinateFrame;
    }


    // ___________________________________________________________________________________


    public class UasGlobalPositionSetpointInt: UasMessage
    {
        public Int32 Latitude {
            get { return mLatitude; }
            set { mLatitude = value; NotifyUpdated(); }
        }

        public Int32 Longitude {
            get { return mLongitude; }
            set { mLongitude = value; NotifyUpdated(); }
        }

        public Int32 Altitude {
            get { return mAltitude; }
            set { mAltitude = value; NotifyUpdated(); }
        }

        public Int16 Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public byte CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        private Int32 mLatitude;
        private Int32 mLongitude;
        private Int32 mAltitude;
        private Int16 mYaw;
        private byte mCoordinateFrame;
    }


    // ___________________________________________________________________________________


    public class UasSetGlobalPositionSetpointInt: UasMessage
    {
        public Int32 Latitude {
            get { return mLatitude; }
            set { mLatitude = value; NotifyUpdated(); }
        }

        public Int32 Longitude {
            get { return mLongitude; }
            set { mLongitude = value; NotifyUpdated(); }
        }

        public Int32 Altitude {
            get { return mAltitude; }
            set { mAltitude = value; NotifyUpdated(); }
        }

        public Int16 Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public byte CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        private Int32 mLatitude;
        private Int32 mLongitude;
        private Int32 mAltitude;
        private Int16 mYaw;
        private byte mCoordinateFrame;
    }


    // ___________________________________________________________________________________


    public class UasSafetySetAllowedArea: UasMessage
    {
        public float P1x {
            get { return mP1x; }
            set { mP1x = value; NotifyUpdated(); }
        }

        public float P1y {
            get { return mP1y; }
            set { mP1y = value; NotifyUpdated(); }
        }

        public float P1z {
            get { return mP1z; }
            set { mP1z = value; NotifyUpdated(); }
        }

        public float P2x {
            get { return mP2x; }
            set { mP2x = value; NotifyUpdated(); }
        }

        public float P2y {
            get { return mP2y; }
            set { mP2y = value; NotifyUpdated(); }
        }

        public float P2z {
            get { return mP2z; }
            set { mP2z = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public byte Frame {
            get { return mFrame; }
            set { mFrame = value; NotifyUpdated(); }
        }

        private float mP1x;
        private float mP1y;
        private float mP1z;
        private float mP2x;
        private float mP2y;
        private float mP2z;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private byte mFrame;
    }


    // ___________________________________________________________________________________


    public class UasSafetyAllowedArea: UasMessage
    {
        public float P1x {
            get { return mP1x; }
            set { mP1x = value; NotifyUpdated(); }
        }

        public float P1y {
            get { return mP1y; }
            set { mP1y = value; NotifyUpdated(); }
        }

        public float P1z {
            get { return mP1z; }
            set { mP1z = value; NotifyUpdated(); }
        }

        public float P2x {
            get { return mP2x; }
            set { mP2x = value; NotifyUpdated(); }
        }

        public float P2y {
            get { return mP2y; }
            set { mP2y = value; NotifyUpdated(); }
        }

        public float P2z {
            get { return mP2z; }
            set { mP2z = value; NotifyUpdated(); }
        }

        public byte Frame {
            get { return mFrame; }
            set { mFrame = value; NotifyUpdated(); }
        }

        private float mP1x;
        private float mP1y;
        private float mP1z;
        private float mP2x;
        private float mP2y;
        private float mP2z;
        private byte mFrame;
    }


    // ___________________________________________________________________________________


    public class UasSetRollPitchYawThrust: UasMessage
    {
        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private float mRoll;
        private float mPitch;
        private float mYaw;
        private float mThrust;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasSetRollPitchYawSpeedThrust: UasMessage
    {
        public float RollSpeed {
            get { return mRollSpeed; }
            set { mRollSpeed = value; NotifyUpdated(); }
        }

        public float PitchSpeed {
            get { return mPitchSpeed; }
            set { mPitchSpeed = value; NotifyUpdated(); }
        }

        public float YawSpeed {
            get { return mYawSpeed; }
            set { mYawSpeed = value; NotifyUpdated(); }
        }

        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private float mRollSpeed;
        private float mPitchSpeed;
        private float mYawSpeed;
        private float mThrust;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasRollPitchYawThrustSetpoint: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private float mRoll;
        private float mPitch;
        private float mYaw;
        private float mThrust;
    }


    // ___________________________________________________________________________________


    public class UasRollPitchYawSpeedThrustSetpoint: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public float RollSpeed {
            get { return mRollSpeed; }
            set { mRollSpeed = value; NotifyUpdated(); }
        }

        public float PitchSpeed {
            get { return mPitchSpeed; }
            set { mPitchSpeed = value; NotifyUpdated(); }
        }

        public float YawSpeed {
            get { return mYawSpeed; }
            set { mYawSpeed = value; NotifyUpdated(); }
        }

        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private float mRollSpeed;
        private float mPitchSpeed;
        private float mYawSpeed;
        private float mThrust;
    }


    // ___________________________________________________________________________________


    public class UasSetQuadMotorsSetpoint: UasMessage
    {
        public UInt16 MotorFrontNw {
            get { return mMotorFrontNw; }
            set { mMotorFrontNw = value; NotifyUpdated(); }
        }

        public UInt16 MotorRightNe {
            get { return mMotorRightNe; }
            set { mMotorRightNe = value; NotifyUpdated(); }
        }

        public UInt16 MotorBackSe {
            get { return mMotorBackSe; }
            set { mMotorBackSe = value; NotifyUpdated(); }
        }

        public UInt16 MotorLeftSw {
            get { return mMotorLeftSw; }
            set { mMotorLeftSw = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        private UInt16 mMotorFrontNw;
        private UInt16 mMotorRightNe;
        private UInt16 mMotorBackSe;
        private UInt16 mMotorLeftSw;
        private byte mTargetSystem;
    }


    // ___________________________________________________________________________________


    public class UasSetQuadSwarmRollPitchYawThrust: UasMessage
    {
        public Int16[] Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public Int16[] Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public Int16[] Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public UInt16[] Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        public byte Group {
            get { return mGroup; }
            set { mGroup = value; NotifyUpdated(); }
        }

        public byte Mode {
            get { return mMode; }
            set { mMode = value; NotifyUpdated(); }
        }

        private Int16[] mRoll = new Int16[4];
        private Int16[] mPitch = new Int16[4];
        private Int16[] mYaw = new Int16[4];
        private UInt16[] mThrust = new UInt16[4];
        private byte mGroup;
        private byte mMode;
    }


    // ___________________________________________________________________________________


    public class UasNavControllerOutput: UasMessage
    {
        public float NavRoll {
            get { return mNavRoll; }
            set { mNavRoll = value; NotifyUpdated(); }
        }

        public float NavPitch {
            get { return mNavPitch; }
            set { mNavPitch = value; NotifyUpdated(); }
        }

        public float AltError {
            get { return mAltError; }
            set { mAltError = value; NotifyUpdated(); }
        }

        public float AspdError {
            get { return mAspdError; }
            set { mAspdError = value; NotifyUpdated(); }
        }

        public float XtrackError {
            get { return mXtrackError; }
            set { mXtrackError = value; NotifyUpdated(); }
        }

        public Int16 NavBearing {
            get { return mNavBearing; }
            set { mNavBearing = value; NotifyUpdated(); }
        }

        public Int16 TargetBearing {
            get { return mTargetBearing; }
            set { mTargetBearing = value; NotifyUpdated(); }
        }

        public UInt16 WpDist {
            get { return mWpDist; }
            set { mWpDist = value; NotifyUpdated(); }
        }

        private float mNavRoll;
        private float mNavPitch;
        private float mAltError;
        private float mAspdError;
        private float mXtrackError;
        private Int16 mNavBearing;
        private Int16 mTargetBearing;
        private UInt16 mWpDist;
    }


    // ___________________________________________________________________________________


    public class UasSetQuadSwarmLedRollPitchYawThrust: UasMessage
    {
        public Int16[] Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public Int16[] Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public Int16[] Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public UInt16[] Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        public byte Group {
            get { return mGroup; }
            set { mGroup = value; NotifyUpdated(); }
        }

        public byte Mode {
            get { return mMode; }
            set { mMode = value; NotifyUpdated(); }
        }

        public byte[] LedRed {
            get { return mLedRed; }
            set { mLedRed = value; NotifyUpdated(); }
        }

        public byte[] LedBlue {
            get { return mLedBlue; }
            set { mLedBlue = value; NotifyUpdated(); }
        }

        public byte[] LedGreen {
            get { return mLedGreen; }
            set { mLedGreen = value; NotifyUpdated(); }
        }

        private Int16[] mRoll = new Int16[4];
        private Int16[] mPitch = new Int16[4];
        private Int16[] mYaw = new Int16[4];
        private UInt16[] mThrust = new UInt16[4];
        private byte mGroup;
        private byte mMode;
        private byte[] mLedRed = new byte[4];
        private byte[] mLedBlue = new byte[4];
        private byte[] mLedGreen = new byte[4];
    }


    // ___________________________________________________________________________________


    public class UasStateCorrection: UasMessage
    {
        public float Xerr {
            get { return mXerr; }
            set { mXerr = value; NotifyUpdated(); }
        }

        public float Yerr {
            get { return mYerr; }
            set { mYerr = value; NotifyUpdated(); }
        }

        public float Zerr {
            get { return mZerr; }
            set { mZerr = value; NotifyUpdated(); }
        }

        public float Rollerr {
            get { return mRollerr; }
            set { mRollerr = value; NotifyUpdated(); }
        }

        public float Pitcherr {
            get { return mPitcherr; }
            set { mPitcherr = value; NotifyUpdated(); }
        }

        public float Yawerr {
            get { return mYawerr; }
            set { mYawerr = value; NotifyUpdated(); }
        }

        public float Vxerr {
            get { return mVxerr; }
            set { mVxerr = value; NotifyUpdated(); }
        }

        public float Vyerr {
            get { return mVyerr; }
            set { mVyerr = value; NotifyUpdated(); }
        }

        public float Vzerr {
            get { return mVzerr; }
            set { mVzerr = value; NotifyUpdated(); }
        }

        private float mXerr;
        private float mYerr;
        private float mZerr;
        private float mRollerr;
        private float mPitcherr;
        private float mYawerr;
        private float mVxerr;
        private float mVyerr;
        private float mVzerr;
    }


    // ___________________________________________________________________________________


    public class UasRequestDataStream: UasMessage
    {
        public UInt16 ReqMessageRate {
            get { return mReqMessageRate; }
            set { mReqMessageRate = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public byte ReqStreamId {
            get { return mReqStreamId; }
            set { mReqStreamId = value; NotifyUpdated(); }
        }

        public byte StartStop {
            get { return mStartStop; }
            set { mStartStop = value; NotifyUpdated(); }
        }

        private UInt16 mReqMessageRate;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private byte mReqStreamId;
        private byte mStartStop;
    }


    // ___________________________________________________________________________________


    public class UasDataStream: UasMessage
    {
        public UInt16 MessageRate {
            get { return mMessageRate; }
            set { mMessageRate = value; NotifyUpdated(); }
        }

        public byte StreamId {
            get { return mStreamId; }
            set { mStreamId = value; NotifyUpdated(); }
        }

        public byte OnOff {
            get { return mOnOff; }
            set { mOnOff = value; NotifyUpdated(); }
        }

        private UInt16 mMessageRate;
        private byte mStreamId;
        private byte mOnOff;
    }


    // ___________________________________________________________________________________


    public class UasManualControl: UasMessage
    {
        public Int16 X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        public Int16 Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        public Int16 Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        public Int16 R {
            get { return mR; }
            set { mR = value; NotifyUpdated(); }
        }

        public UInt16 Buttons {
            get { return mButtons; }
            set { mButtons = value; NotifyUpdated(); }
        }

        public byte Target {
            get { return mTarget; }
            set { mTarget = value; NotifyUpdated(); }
        }

        private Int16 mX;
        private Int16 mY;
        private Int16 mZ;
        private Int16 mR;
        private UInt16 mButtons;
        private byte mTarget;
    }


    // ___________________________________________________________________________________


    public class UasRcChannelsOverride: UasMessage
    {
        public UInt16 Chan1Raw {
            get { return mChan1Raw; }
            set { mChan1Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan2Raw {
            get { return mChan2Raw; }
            set { mChan2Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan3Raw {
            get { return mChan3Raw; }
            set { mChan3Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan4Raw {
            get { return mChan4Raw; }
            set { mChan4Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan5Raw {
            get { return mChan5Raw; }
            set { mChan5Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan6Raw {
            get { return mChan6Raw; }
            set { mChan6Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan7Raw {
            get { return mChan7Raw; }
            set { mChan7Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan8Raw {
            get { return mChan8Raw; }
            set { mChan8Raw = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        private UInt16 mChan1Raw;
        private UInt16 mChan2Raw;
        private UInt16 mChan3Raw;
        private UInt16 mChan4Raw;
        private UInt16 mChan5Raw;
        private UInt16 mChan6Raw;
        private UInt16 mChan7Raw;
        private UInt16 mChan8Raw;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    public class UasVfrHud: UasMessage
    {
        public float Airspeed {
            get { return mAirspeed; }
            set { mAirspeed = value; NotifyUpdated(); }
        }

        public float Groundspeed {
            get { return mGroundspeed; }
            set { mGroundspeed = value; NotifyUpdated(); }
        }

        public float Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        public float Climb {
            get { return mClimb; }
            set { mClimb = value; NotifyUpdated(); }
        }

        public Int16 Heading {
            get { return mHeading; }
            set { mHeading = value; NotifyUpdated(); }
        }

        public UInt16 Throttle {
            get { return mThrottle; }
            set { mThrottle = value; NotifyUpdated(); }
        }

        private float mAirspeed;
        private float mGroundspeed;
        private float mAlt;
        private float mClimb;
        private Int16 mHeading;
        private UInt16 mThrottle;
    }


    // ___________________________________________________________________________________


    public class UasCommandLong: UasMessage
    {
        public float Param1 {
            get { return mParam1; }
            set { mParam1 = value; NotifyUpdated(); }
        }

        public float Param2 {
            get { return mParam2; }
            set { mParam2 = value; NotifyUpdated(); }
        }

        public float Param3 {
            get { return mParam3; }
            set { mParam3 = value; NotifyUpdated(); }
        }

        public float Param4 {
            get { return mParam4; }
            set { mParam4 = value; NotifyUpdated(); }
        }

        public float Param5 {
            get { return mParam5; }
            set { mParam5 = value; NotifyUpdated(); }
        }

        public float Param6 {
            get { return mParam6; }
            set { mParam6 = value; NotifyUpdated(); }
        }

        public float Param7 {
            get { return mParam7; }
            set { mParam7 = value; NotifyUpdated(); }
        }

        public UInt16 Command {
            get { return mCommand; }
            set { mCommand = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public byte Confirmation {
            get { return mConfirmation; }
            set { mConfirmation = value; NotifyUpdated(); }
        }

        private float mParam1;
        private float mParam2;
        private float mParam3;
        private float mParam4;
        private float mParam5;
        private float mParam6;
        private float mParam7;
        private UInt16 mCommand;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private byte mConfirmation;
    }


    // ___________________________________________________________________________________


    public class UasCommandAck: UasMessage
    {
        public UInt16 Command {
            get { return mCommand; }
            set { mCommand = value; NotifyUpdated(); }
        }

        public byte Result {
            get { return mResult; }
            set { mResult = value; NotifyUpdated(); }
        }

        private UInt16 mCommand;
        private byte mResult;
    }


    // ___________________________________________________________________________________


    public class UasRollPitchYawRatesThrustSetpoint: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public float RollRate {
            get { return mRollRate; }
            set { mRollRate = value; NotifyUpdated(); }
        }

        public float PitchRate {
            get { return mPitchRate; }
            set { mPitchRate = value; NotifyUpdated(); }
        }

        public float YawRate {
            get { return mYawRate; }
            set { mYawRate = value; NotifyUpdated(); }
        }

        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private float mRollRate;
        private float mPitchRate;
        private float mYawRate;
        private float mThrust;
    }


    // ___________________________________________________________________________________


    public class UasManualSetpoint: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        public byte ModeSwitch {
            get { return mModeSwitch; }
            set { mModeSwitch = value; NotifyUpdated(); }
        }

        public byte ManualOverrideSwitch {
            get { return mManualOverrideSwitch; }
            set { mManualOverrideSwitch = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private float mRoll;
        private float mPitch;
        private float mYaw;
        private float mThrust;
        private byte mModeSwitch;
        private byte mManualOverrideSwitch;
    }


    // ___________________________________________________________________________________


    public class UasLocalPositionNedSystemGlobalOffset: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private float mX;
        private float mY;
        private float mZ;
        private float mRoll;
        private float mPitch;
        private float mYaw;
    }


    // ___________________________________________________________________________________


    public class UasHilState: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public float Rollspeed {
            get { return mRollspeed; }
            set { mRollspeed = value; NotifyUpdated(); }
        }

        public float Pitchspeed {
            get { return mPitchspeed; }
            set { mPitchspeed = value; NotifyUpdated(); }
        }

        public float Yawspeed {
            get { return mYawspeed; }
            set { mYawspeed = value; NotifyUpdated(); }
        }

        public Int32 Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        public Int32 Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        public Int32 Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        public Int16 Vx {
            get { return mVx; }
            set { mVx = value; NotifyUpdated(); }
        }

        public Int16 Vy {
            get { return mVy; }
            set { mVy = value; NotifyUpdated(); }
        }

        public Int16 Vz {
            get { return mVz; }
            set { mVz = value; NotifyUpdated(); }
        }

        public Int16 Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        public Int16 Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        public Int16 Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private float mRoll;
        private float mPitch;
        private float mYaw;
        private float mRollspeed;
        private float mPitchspeed;
        private float mYawspeed;
        private Int32 mLat;
        private Int32 mLon;
        private Int32 mAlt;
        private Int16 mVx;
        private Int16 mVy;
        private Int16 mVz;
        private Int16 mXacc;
        private Int16 mYacc;
        private Int16 mZacc;
    }


    // ___________________________________________________________________________________


    public class UasHilControls: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public float RollAilerons {
            get { return mRollAilerons; }
            set { mRollAilerons = value; NotifyUpdated(); }
        }

        public float PitchElevator {
            get { return mPitchElevator; }
            set { mPitchElevator = value; NotifyUpdated(); }
        }

        public float YawRudder {
            get { return mYawRudder; }
            set { mYawRudder = value; NotifyUpdated(); }
        }

        public float Throttle {
            get { return mThrottle; }
            set { mThrottle = value; NotifyUpdated(); }
        }

        public float Aux1 {
            get { return mAux1; }
            set { mAux1 = value; NotifyUpdated(); }
        }

        public float Aux2 {
            get { return mAux2; }
            set { mAux2 = value; NotifyUpdated(); }
        }

        public float Aux3 {
            get { return mAux3; }
            set { mAux3 = value; NotifyUpdated(); }
        }

        public float Aux4 {
            get { return mAux4; }
            set { mAux4 = value; NotifyUpdated(); }
        }

        public byte Mode {
            get { return mMode; }
            set { mMode = value; NotifyUpdated(); }
        }

        public byte NavMode {
            get { return mNavMode; }
            set { mNavMode = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private float mRollAilerons;
        private float mPitchElevator;
        private float mYawRudder;
        private float mThrottle;
        private float mAux1;
        private float mAux2;
        private float mAux3;
        private float mAux4;
        private byte mMode;
        private byte mNavMode;
    }


    // ___________________________________________________________________________________


    public class UasHilRcInputsRaw: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public UInt16 Chan1Raw {
            get { return mChan1Raw; }
            set { mChan1Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan2Raw {
            get { return mChan2Raw; }
            set { mChan2Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan3Raw {
            get { return mChan3Raw; }
            set { mChan3Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan4Raw {
            get { return mChan4Raw; }
            set { mChan4Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan5Raw {
            get { return mChan5Raw; }
            set { mChan5Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan6Raw {
            get { return mChan6Raw; }
            set { mChan6Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan7Raw {
            get { return mChan7Raw; }
            set { mChan7Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan8Raw {
            get { return mChan8Raw; }
            set { mChan8Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan9Raw {
            get { return mChan9Raw; }
            set { mChan9Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan10Raw {
            get { return mChan10Raw; }
            set { mChan10Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan11Raw {
            get { return mChan11Raw; }
            set { mChan11Raw = value; NotifyUpdated(); }
        }

        public UInt16 Chan12Raw {
            get { return mChan12Raw; }
            set { mChan12Raw = value; NotifyUpdated(); }
        }

        public byte Rssi {
            get { return mRssi; }
            set { mRssi = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private UInt16 mChan1Raw;
        private UInt16 mChan2Raw;
        private UInt16 mChan3Raw;
        private UInt16 mChan4Raw;
        private UInt16 mChan5Raw;
        private UInt16 mChan6Raw;
        private UInt16 mChan7Raw;
        private UInt16 mChan8Raw;
        private UInt16 mChan9Raw;
        private UInt16 mChan10Raw;
        private UInt16 mChan11Raw;
        private UInt16 mChan12Raw;
        private byte mRssi;
    }


    // ___________________________________________________________________________________


    public class UasOpticalFlow: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public float FlowCompMX {
            get { return mFlowCompMX; }
            set { mFlowCompMX = value; NotifyUpdated(); }
        }

        public float FlowCompMY {
            get { return mFlowCompMY; }
            set { mFlowCompMY = value; NotifyUpdated(); }
        }

        public float GroundDistance {
            get { return mGroundDistance; }
            set { mGroundDistance = value; NotifyUpdated(); }
        }

        public Int16 FlowX {
            get { return mFlowX; }
            set { mFlowX = value; NotifyUpdated(); }
        }

        public Int16 FlowY {
            get { return mFlowY; }
            set { mFlowY = value; NotifyUpdated(); }
        }

        public byte SensorId {
            get { return mSensorId; }
            set { mSensorId = value; NotifyUpdated(); }
        }

        public byte Quality {
            get { return mQuality; }
            set { mQuality = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private float mFlowCompMX;
        private float mFlowCompMY;
        private float mGroundDistance;
        private Int16 mFlowX;
        private Int16 mFlowY;
        private byte mSensorId;
        private byte mQuality;
    }


    // ___________________________________________________________________________________


    public class UasGlobalVisionPositionEstimate: UasMessage
    {
        public UInt64 Usec {
            get { return mUsec; }
            set { mUsec = value; NotifyUpdated(); }
        }

        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        private UInt64 mUsec;
        private float mX;
        private float mY;
        private float mZ;
        private float mRoll;
        private float mPitch;
        private float mYaw;
    }


    // ___________________________________________________________________________________


    public class UasVisionPositionEstimate: UasMessage
    {
        public UInt64 Usec {
            get { return mUsec; }
            set { mUsec = value; NotifyUpdated(); }
        }

        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        private UInt64 mUsec;
        private float mX;
        private float mY;
        private float mZ;
        private float mRoll;
        private float mPitch;
        private float mYaw;
    }


    // ___________________________________________________________________________________


    public class UasVisionSpeedEstimate: UasMessage
    {
        public UInt64 Usec {
            get { return mUsec; }
            set { mUsec = value; NotifyUpdated(); }
        }

        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        private UInt64 mUsec;
        private float mX;
        private float mY;
        private float mZ;
    }


    // ___________________________________________________________________________________


    public class UasViconPositionEstimate: UasMessage
    {
        public UInt64 Usec {
            get { return mUsec; }
            set { mUsec = value; NotifyUpdated(); }
        }

        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        private UInt64 mUsec;
        private float mX;
        private float mY;
        private float mZ;
        private float mRoll;
        private float mPitch;
        private float mYaw;
    }


    // ___________________________________________________________________________________


    public class UasHighresImu: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public float Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        public float Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        public float Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        public float Xgyro {
            get { return mXgyro; }
            set { mXgyro = value; NotifyUpdated(); }
        }

        public float Ygyro {
            get { return mYgyro; }
            set { mYgyro = value; NotifyUpdated(); }
        }

        public float Zgyro {
            get { return mZgyro; }
            set { mZgyro = value; NotifyUpdated(); }
        }

        public float Xmag {
            get { return mXmag; }
            set { mXmag = value; NotifyUpdated(); }
        }

        public float Ymag {
            get { return mYmag; }
            set { mYmag = value; NotifyUpdated(); }
        }

        public float Zmag {
            get { return mZmag; }
            set { mZmag = value; NotifyUpdated(); }
        }

        public float AbsPressure {
            get { return mAbsPressure; }
            set { mAbsPressure = value; NotifyUpdated(); }
        }

        public float DiffPressure {
            get { return mDiffPressure; }
            set { mDiffPressure = value; NotifyUpdated(); }
        }

        public float PressureAlt {
            get { return mPressureAlt; }
            set { mPressureAlt = value; NotifyUpdated(); }
        }

        public float Temperature {
            get { return mTemperature; }
            set { mTemperature = value; NotifyUpdated(); }
        }

        public UInt16 FieldsUpdated {
            get { return mFieldsUpdated; }
            set { mFieldsUpdated = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private float mXacc;
        private float mYacc;
        private float mZacc;
        private float mXgyro;
        private float mYgyro;
        private float mZgyro;
        private float mXmag;
        private float mYmag;
        private float mZmag;
        private float mAbsPressure;
        private float mDiffPressure;
        private float mPressureAlt;
        private float mTemperature;
        private UInt16 mFieldsUpdated;
    }


    // ___________________________________________________________________________________


    public class UasOmnidirectionalFlow: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public float FrontDistanceM {
            get { return mFrontDistanceM; }
            set { mFrontDistanceM = value; NotifyUpdated(); }
        }

        public Int16[] Left {
            get { return mLeft; }
            set { mLeft = value; NotifyUpdated(); }
        }

        public Int16[] Right {
            get { return mRight; }
            set { mRight = value; NotifyUpdated(); }
        }

        public byte SensorId {
            get { return mSensorId; }
            set { mSensorId = value; NotifyUpdated(); }
        }

        public byte Quality {
            get { return mQuality; }
            set { mQuality = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private float mFrontDistanceM;
        private Int16[] mLeft = new Int16[10];
        private Int16[] mRight = new Int16[10];
        private byte mSensorId;
        private byte mQuality;
    }


    // ___________________________________________________________________________________


    public class UasHilSensor: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public float Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        public float Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        public float Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        public float Xgyro {
            get { return mXgyro; }
            set { mXgyro = value; NotifyUpdated(); }
        }

        public float Ygyro {
            get { return mYgyro; }
            set { mYgyro = value; NotifyUpdated(); }
        }

        public float Zgyro {
            get { return mZgyro; }
            set { mZgyro = value; NotifyUpdated(); }
        }

        public float Xmag {
            get { return mXmag; }
            set { mXmag = value; NotifyUpdated(); }
        }

        public float Ymag {
            get { return mYmag; }
            set { mYmag = value; NotifyUpdated(); }
        }

        public float Zmag {
            get { return mZmag; }
            set { mZmag = value; NotifyUpdated(); }
        }

        public float AbsPressure {
            get { return mAbsPressure; }
            set { mAbsPressure = value; NotifyUpdated(); }
        }

        public float DiffPressure {
            get { return mDiffPressure; }
            set { mDiffPressure = value; NotifyUpdated(); }
        }

        public float PressureAlt {
            get { return mPressureAlt; }
            set { mPressureAlt = value; NotifyUpdated(); }
        }

        public float Temperature {
            get { return mTemperature; }
            set { mTemperature = value; NotifyUpdated(); }
        }

        public UInt32 FieldsUpdated {
            get { return mFieldsUpdated; }
            set { mFieldsUpdated = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private float mXacc;
        private float mYacc;
        private float mZacc;
        private float mXgyro;
        private float mYgyro;
        private float mZgyro;
        private float mXmag;
        private float mYmag;
        private float mZmag;
        private float mAbsPressure;
        private float mDiffPressure;
        private float mPressureAlt;
        private float mTemperature;
        private UInt32 mFieldsUpdated;
    }


    // ___________________________________________________________________________________


    public class UasSimState: UasMessage
    {
        public float Q1 {
            get { return mQ1; }
            set { mQ1 = value; NotifyUpdated(); }
        }

        public float Q2 {
            get { return mQ2; }
            set { mQ2 = value; NotifyUpdated(); }
        }

        public float Q3 {
            get { return mQ3; }
            set { mQ3 = value; NotifyUpdated(); }
        }

        public float Q4 {
            get { return mQ4; }
            set { mQ4 = value; NotifyUpdated(); }
        }

        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public float Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        public float Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        public float Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        public float Xgyro {
            get { return mXgyro; }
            set { mXgyro = value; NotifyUpdated(); }
        }

        public float Ygyro {
            get { return mYgyro; }
            set { mYgyro = value; NotifyUpdated(); }
        }

        public float Zgyro {
            get { return mZgyro; }
            set { mZgyro = value; NotifyUpdated(); }
        }

        public float Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        public float Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        public float Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        public float StdDevHorz {
            get { return mStdDevHorz; }
            set { mStdDevHorz = value; NotifyUpdated(); }
        }

        public float StdDevVert {
            get { return mStdDevVert; }
            set { mStdDevVert = value; NotifyUpdated(); }
        }

        public float Vn {
            get { return mVn; }
            set { mVn = value; NotifyUpdated(); }
        }

        public float Ve {
            get { return mVe; }
            set { mVe = value; NotifyUpdated(); }
        }

        public float Vd {
            get { return mVd; }
            set { mVd = value; NotifyUpdated(); }
        }

        private float mQ1;
        private float mQ2;
        private float mQ3;
        private float mQ4;
        private float mRoll;
        private float mPitch;
        private float mYaw;
        private float mXacc;
        private float mYacc;
        private float mZacc;
        private float mXgyro;
        private float mYgyro;
        private float mZgyro;
        private float mLat;
        private float mLon;
        private float mAlt;
        private float mStdDevHorz;
        private float mStdDevVert;
        private float mVn;
        private float mVe;
        private float mVd;
    }


    // ___________________________________________________________________________________


    public class UasRadioStatus: UasMessage
    {
        public UInt16 Rxerrors {
            get { return mRxerrors; }
            set { mRxerrors = value; NotifyUpdated(); }
        }

        public UInt16 Fixed {
            get { return mFixed; }
            set { mFixed = value; NotifyUpdated(); }
        }

        public byte Rssi {
            get { return mRssi; }
            set { mRssi = value; NotifyUpdated(); }
        }

        public byte Remrssi {
            get { return mRemrssi; }
            set { mRemrssi = value; NotifyUpdated(); }
        }

        public byte Txbuf {
            get { return mTxbuf; }
            set { mTxbuf = value; NotifyUpdated(); }
        }

        public byte Noise {
            get { return mNoise; }
            set { mNoise = value; NotifyUpdated(); }
        }

        public byte Remnoise {
            get { return mRemnoise; }
            set { mRemnoise = value; NotifyUpdated(); }
        }

        private UInt16 mRxerrors;
        private UInt16 mFixed;
        private byte mRssi;
        private byte mRemrssi;
        private byte mTxbuf;
        private byte mNoise;
        private byte mRemnoise;
    }


    // ___________________________________________________________________________________


    public class UasFileTransferStart: UasMessage
    {
        public UInt64 TransferUid {
            get { return mTransferUid; }
            set { mTransferUid = value; NotifyUpdated(); }
        }

        public UInt32 FileSize {
            get { return mFileSize; }
            set { mFileSize = value; NotifyUpdated(); }
        }

        public char[] DestPath {
            get { return mDestPath; }
            set { mDestPath = value; NotifyUpdated(); }
        }

        public byte Direction {
            get { return mDirection; }
            set { mDirection = value; NotifyUpdated(); }
        }

        public byte Flags {
            get { return mFlags; }
            set { mFlags = value; NotifyUpdated(); }
        }

        private UInt64 mTransferUid;
        private UInt32 mFileSize;
        private char[] mDestPath = new char[240];
        private byte mDirection;
        private byte mFlags;
    }


    // ___________________________________________________________________________________


    public class UasFileTransferDirList: UasMessage
    {
        public UInt64 TransferUid {
            get { return mTransferUid; }
            set { mTransferUid = value; NotifyUpdated(); }
        }

        public char[] DirPath {
            get { return mDirPath; }
            set { mDirPath = value; NotifyUpdated(); }
        }

        public byte Flags {
            get { return mFlags; }
            set { mFlags = value; NotifyUpdated(); }
        }

        private UInt64 mTransferUid;
        private char[] mDirPath = new char[240];
        private byte mFlags;
    }


    // ___________________________________________________________________________________


    public class UasFileTransferRes: UasMessage
    {
        public UInt64 TransferUid {
            get { return mTransferUid; }
            set { mTransferUid = value; NotifyUpdated(); }
        }

        public byte Result {
            get { return mResult; }
            set { mResult = value; NotifyUpdated(); }
        }

        private UInt64 mTransferUid;
        private byte mResult;
    }


    // ___________________________________________________________________________________


    public class UasHilGps: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public Int32 Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        public Int32 Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        public Int32 Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        public UInt16 Eph {
            get { return mEph; }
            set { mEph = value; NotifyUpdated(); }
        }

        public UInt16 Epv {
            get { return mEpv; }
            set { mEpv = value; NotifyUpdated(); }
        }

        public UInt16 Vel {
            get { return mVel; }
            set { mVel = value; NotifyUpdated(); }
        }

        public Int16 Vn {
            get { return mVn; }
            set { mVn = value; NotifyUpdated(); }
        }

        public Int16 Ve {
            get { return mVe; }
            set { mVe = value; NotifyUpdated(); }
        }

        public Int16 Vd {
            get { return mVd; }
            set { mVd = value; NotifyUpdated(); }
        }

        public UInt16 Cog {
            get { return mCog; }
            set { mCog = value; NotifyUpdated(); }
        }

        public byte FixType {
            get { return mFixType; }
            set { mFixType = value; NotifyUpdated(); }
        }

        public byte SatellitesVisible {
            get { return mSatellitesVisible; }
            set { mSatellitesVisible = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private Int32 mLat;
        private Int32 mLon;
        private Int32 mAlt;
        private UInt16 mEph;
        private UInt16 mEpv;
        private UInt16 mVel;
        private Int16 mVn;
        private Int16 mVe;
        private Int16 mVd;
        private UInt16 mCog;
        private byte mFixType;
        private byte mSatellitesVisible;
    }


    // ___________________________________________________________________________________


    public class UasHilOpticalFlow: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public float FlowCompMX {
            get { return mFlowCompMX; }
            set { mFlowCompMX = value; NotifyUpdated(); }
        }

        public float FlowCompMY {
            get { return mFlowCompMY; }
            set { mFlowCompMY = value; NotifyUpdated(); }
        }

        public float GroundDistance {
            get { return mGroundDistance; }
            set { mGroundDistance = value; NotifyUpdated(); }
        }

        public Int16 FlowX {
            get { return mFlowX; }
            set { mFlowX = value; NotifyUpdated(); }
        }

        public Int16 FlowY {
            get { return mFlowY; }
            set { mFlowY = value; NotifyUpdated(); }
        }

        public byte SensorId {
            get { return mSensorId; }
            set { mSensorId = value; NotifyUpdated(); }
        }

        public byte Quality {
            get { return mQuality; }
            set { mQuality = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private float mFlowCompMX;
        private float mFlowCompMY;
        private float mGroundDistance;
        private Int16 mFlowX;
        private Int16 mFlowY;
        private byte mSensorId;
        private byte mQuality;
    }


    // ___________________________________________________________________________________


    public class UasHilStateQuaternion: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public float[] AttitudeQuaternion {
            get { return mAttitudeQuaternion; }
            set { mAttitudeQuaternion = value; NotifyUpdated(); }
        }

        public float Rollspeed {
            get { return mRollspeed; }
            set { mRollspeed = value; NotifyUpdated(); }
        }

        public float Pitchspeed {
            get { return mPitchspeed; }
            set { mPitchspeed = value; NotifyUpdated(); }
        }

        public float Yawspeed {
            get { return mYawspeed; }
            set { mYawspeed = value; NotifyUpdated(); }
        }

        public Int32 Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        public Int32 Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        public Int32 Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        public Int16 Vx {
            get { return mVx; }
            set { mVx = value; NotifyUpdated(); }
        }

        public Int16 Vy {
            get { return mVy; }
            set { mVy = value; NotifyUpdated(); }
        }

        public Int16 Vz {
            get { return mVz; }
            set { mVz = value; NotifyUpdated(); }
        }

        public UInt16 IndAirspeed {
            get { return mIndAirspeed; }
            set { mIndAirspeed = value; NotifyUpdated(); }
        }

        public UInt16 TrueAirspeed {
            get { return mTrueAirspeed; }
            set { mTrueAirspeed = value; NotifyUpdated(); }
        }

        public Int16 Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        public Int16 Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        public Int16 Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private float[] mAttitudeQuaternion = new float[4];
        private float mRollspeed;
        private float mPitchspeed;
        private float mYawspeed;
        private Int32 mLat;
        private Int32 mLon;
        private Int32 mAlt;
        private Int16 mVx;
        private Int16 mVy;
        private Int16 mVz;
        private UInt16 mIndAirspeed;
        private UInt16 mTrueAirspeed;
        private Int16 mXacc;
        private Int16 mYacc;
        private Int16 mZacc;
    }


    // ___________________________________________________________________________________


    public class UasBatteryStatus: UasMessage
    {
        public Int32 CurrentConsumed {
            get { return mCurrentConsumed; }
            set { mCurrentConsumed = value; NotifyUpdated(); }
        }

        public Int32 EnergyConsumed {
            get { return mEnergyConsumed; }
            set { mEnergyConsumed = value; NotifyUpdated(); }
        }

        public UInt16 VoltageCell1 {
            get { return mVoltageCell1; }
            set { mVoltageCell1 = value; NotifyUpdated(); }
        }

        public UInt16 VoltageCell2 {
            get { return mVoltageCell2; }
            set { mVoltageCell2 = value; NotifyUpdated(); }
        }

        public UInt16 VoltageCell3 {
            get { return mVoltageCell3; }
            set { mVoltageCell3 = value; NotifyUpdated(); }
        }

        public UInt16 VoltageCell4 {
            get { return mVoltageCell4; }
            set { mVoltageCell4 = value; NotifyUpdated(); }
        }

        public UInt16 VoltageCell5 {
            get { return mVoltageCell5; }
            set { mVoltageCell5 = value; NotifyUpdated(); }
        }

        public UInt16 VoltageCell6 {
            get { return mVoltageCell6; }
            set { mVoltageCell6 = value; NotifyUpdated(); }
        }

        public Int16 CurrentBattery {
            get { return mCurrentBattery; }
            set { mCurrentBattery = value; NotifyUpdated(); }
        }

        public byte AccuId {
            get { return mAccuId; }
            set { mAccuId = value; NotifyUpdated(); }
        }

        public SByte BatteryRemaining {
            get { return mBatteryRemaining; }
            set { mBatteryRemaining = value; NotifyUpdated(); }
        }

        private Int32 mCurrentConsumed;
        private Int32 mEnergyConsumed;
        private UInt16 mVoltageCell1;
        private UInt16 mVoltageCell2;
        private UInt16 mVoltageCell3;
        private UInt16 mVoltageCell4;
        private UInt16 mVoltageCell5;
        private UInt16 mVoltageCell6;
        private Int16 mCurrentBattery;
        private byte mAccuId;
        private SByte mBatteryRemaining;
    }


    // ___________________________________________________________________________________


    public class UasSetpoint8dof: UasMessage
    {
        public float Val1 {
            get { return mVal1; }
            set { mVal1 = value; NotifyUpdated(); }
        }

        public float Val2 {
            get { return mVal2; }
            set { mVal2 = value; NotifyUpdated(); }
        }

        public float Val3 {
            get { return mVal3; }
            set { mVal3 = value; NotifyUpdated(); }
        }

        public float Val4 {
            get { return mVal4; }
            set { mVal4 = value; NotifyUpdated(); }
        }

        public float Val5 {
            get { return mVal5; }
            set { mVal5 = value; NotifyUpdated(); }
        }

        public float Val6 {
            get { return mVal6; }
            set { mVal6 = value; NotifyUpdated(); }
        }

        public float Val7 {
            get { return mVal7; }
            set { mVal7 = value; NotifyUpdated(); }
        }

        public float Val8 {
            get { return mVal8; }
            set { mVal8 = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        private float mVal1;
        private float mVal2;
        private float mVal3;
        private float mVal4;
        private float mVal5;
        private float mVal6;
        private float mVal7;
        private float mVal8;
        private byte mTargetSystem;
    }


    // ___________________________________________________________________________________


    public class UasSetpoint6dof: UasMessage
    {
        public float TransX {
            get { return mTransX; }
            set { mTransX = value; NotifyUpdated(); }
        }

        public float TransY {
            get { return mTransY; }
            set { mTransY = value; NotifyUpdated(); }
        }

        public float TransZ {
            get { return mTransZ; }
            set { mTransZ = value; NotifyUpdated(); }
        }

        public float RotX {
            get { return mRotX; }
            set { mRotX = value; NotifyUpdated(); }
        }

        public float RotY {
            get { return mRotY; }
            set { mRotY = value; NotifyUpdated(); }
        }

        public float RotZ {
            get { return mRotZ; }
            set { mRotZ = value; NotifyUpdated(); }
        }

        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        private float mTransX;
        private float mTransY;
        private float mTransZ;
        private float mRotX;
        private float mRotY;
        private float mRotZ;
        private byte mTargetSystem;
    }


    // ___________________________________________________________________________________


    public class UasMemoryVect: UasMessage
    {
        public UInt16 Address {
            get { return mAddress; }
            set { mAddress = value; NotifyUpdated(); }
        }

        public byte Ver {
            get { return mVer; }
            set { mVer = value; NotifyUpdated(); }
        }

        public byte Type {
            get { return mType; }
            set { mType = value; NotifyUpdated(); }
        }

        public SByte[] Value {
            get { return mValue; }
            set { mValue = value; NotifyUpdated(); }
        }

        private UInt16 mAddress;
        private byte mVer;
        private byte mType;
        private SByte[] mValue = new SByte[32];
    }


    // ___________________________________________________________________________________


    public class UasDebugVect: UasMessage
    {
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        public char[] Name {
            get { return mName; }
            set { mName = value; NotifyUpdated(); }
        }

        private UInt64 mTimeUsec;
        private float mX;
        private float mY;
        private float mZ;
        private char[] mName = new char[10];
    }


    // ___________________________________________________________________________________


    public class UasNamedValueFloat: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public float Value {
            get { return mValue; }
            set { mValue = value; NotifyUpdated(); }
        }

        public char[] Name {
            get { return mName; }
            set { mName = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private float mValue;
        private char[] mName = new char[10];
    }


    // ___________________________________________________________________________________


    public class UasNamedValueInt: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public Int32 Value {
            get { return mValue; }
            set { mValue = value; NotifyUpdated(); }
        }

        public char[] Name {
            get { return mName; }
            set { mName = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private Int32 mValue;
        private char[] mName = new char[10];
    }


    // ___________________________________________________________________________________


    public class UasStatustext: UasMessage
    {
        public byte Severity {
            get { return mSeverity; }
            set { mSeverity = value; NotifyUpdated(); }
        }

        public char[] Text {
            get { return mText; }
            set { mText = value; NotifyUpdated(); }
        }

        private byte mSeverity;
        private char[] mText = new char[50];
    }


    // ___________________________________________________________________________________


    public class UasDebug: UasMessage
    {
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public byte Ind {
            get { return mInd; }
            set { mInd = value; NotifyUpdated(); }
        }

        public float Value {
            get { return mValue; }
            set { mValue = value; NotifyUpdated(); }
        }

        private UInt32 mTimeBootMs;
        private byte mInd;
        private float mValue;
    }

}
