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

        public MavType Type {
            get { return mType; }
            set { mType = value; NotifyUpdated(); }
        }

        public MavAutopilot Autopilot {
            get { return mAutopilot; }
            set { mAutopilot = value; NotifyUpdated(); }
        }

        public MavModeFlag BaseMode {
            get { return mBaseMode; }
            set { mBaseMode = value; NotifyUpdated(); }
        }

        public MavState SystemStatus {
            get { return mSystemStatus; }
            set { mSystemStatus = value; NotifyUpdated(); }
        }

        public byte MavlinkVersion {
            get { return mMavlinkVersion; }
            set { mMavlinkVersion = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mCustomMode);
            s.Write((byte)mType);
            s.Write((byte)mAutopilot);
            s.Write((byte)mBaseMode);
            s.Write((byte)mSystemStatus);
            s.Write(mMavlinkVersion);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mCustomMode = s.ReadUInt32();
            this.mType = (MavType)s.ReadByte();
            this.mAutopilot = (MavAutopilot)s.ReadByte();
            this.mBaseMode = (MavModeFlag)s.ReadByte();
            this.mSystemStatus = (MavState)s.ReadByte();
            this.mMavlinkVersion = s.ReadByte();
        }

        private UInt32 mCustomMode;
        private MavType mType;
        private MavAutopilot mAutopilot;
        private MavModeFlag mBaseMode;
        private MavState mSystemStatus;
        private byte mMavlinkVersion;
    }


    // ___________________________________________________________________________________


    public class UasSysStatus: UasMessage
    {
        public MavSysStatusSensor OnboardControlSensorsPresent {
            get { return mOnboardControlSensorsPresent; }
            set { mOnboardControlSensorsPresent = value; NotifyUpdated(); }
        }

        public MavSysStatusSensor OnboardControlSensorsEnabled {
            get { return mOnboardControlSensorsEnabled; }
            set { mOnboardControlSensorsEnabled = value; NotifyUpdated(); }
        }

        public MavSysStatusSensor OnboardControlSensorsHealth {
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write((UInt32)mOnboardControlSensorsPresent);
            s.Write((UInt32)mOnboardControlSensorsEnabled);
            s.Write((UInt32)mOnboardControlSensorsHealth);
            s.Write(mLoad);
            s.Write(mVoltageBattery);
            s.Write(mCurrentBattery);
            s.Write(mDropRateComm);
            s.Write(mErrorsComm);
            s.Write(mErrorsCount1);
            s.Write(mErrorsCount2);
            s.Write(mErrorsCount3);
            s.Write(mErrorsCount4);
            s.Write(mBatteryRemaining);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mOnboardControlSensorsPresent = (MavSysStatusSensor)s.ReadUInt32();
            this.mOnboardControlSensorsEnabled = (MavSysStatusSensor)s.ReadUInt32();
            this.mOnboardControlSensorsHealth = (MavSysStatusSensor)s.ReadUInt32();
            this.mLoad = s.ReadUInt16();
            this.mVoltageBattery = s.ReadUInt16();
            this.mCurrentBattery = s.ReadInt16();
            this.mDropRateComm = s.ReadUInt16();
            this.mErrorsComm = s.ReadUInt16();
            this.mErrorsCount1 = s.ReadUInt16();
            this.mErrorsCount2 = s.ReadUInt16();
            this.mErrorsCount3 = s.ReadUInt16();
            this.mErrorsCount4 = s.ReadUInt16();
            this.mBatteryRemaining = s.ReadSByte();
        }

        private MavSysStatusSensor mOnboardControlSensorsPresent;
        private MavSysStatusSensor mOnboardControlSensorsEnabled;
        private MavSysStatusSensor mOnboardControlSensorsHealth;
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUnixUsec);
            s.Write(mTimeBootMs);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUnixUsec = s.ReadUInt64();
            this.mTimeBootMs = s.ReadUInt32();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mSeq);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mSeq = s.ReadUInt32();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTargetSystem);
            s.Write(mControlRequest);
            s.Write(mVersion);
            s.Write(mPasskey[0]); 
            s.Write(mPasskey[1]); 
            s.Write(mPasskey[2]); 
            s.Write(mPasskey[3]); 
            s.Write(mPasskey[4]); 
            s.Write(mPasskey[5]); 
            s.Write(mPasskey[6]); 
            s.Write(mPasskey[7]); 
            s.Write(mPasskey[8]); 
            s.Write(mPasskey[9]); 
            s.Write(mPasskey[10]); 
            s.Write(mPasskey[11]); 
            s.Write(mPasskey[12]); 
            s.Write(mPasskey[13]); 
            s.Write(mPasskey[14]); 
            s.Write(mPasskey[15]); 
            s.Write(mPasskey[16]); 
            s.Write(mPasskey[17]); 
            s.Write(mPasskey[18]); 
            s.Write(mPasskey[19]); 
            s.Write(mPasskey[20]); 
            s.Write(mPasskey[21]); 
            s.Write(mPasskey[22]); 
            s.Write(mPasskey[23]); 
            s.Write(mPasskey[24]); 
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTargetSystem = s.ReadByte();
            this.mControlRequest = s.ReadByte();
            this.mVersion = s.ReadByte();
            this.mPasskey[0] = s.ReadChar();
            this.mPasskey[1] = s.ReadChar();
            this.mPasskey[2] = s.ReadChar();
            this.mPasskey[3] = s.ReadChar();
            this.mPasskey[4] = s.ReadChar();
            this.mPasskey[5] = s.ReadChar();
            this.mPasskey[6] = s.ReadChar();
            this.mPasskey[7] = s.ReadChar();
            this.mPasskey[8] = s.ReadChar();
            this.mPasskey[9] = s.ReadChar();
            this.mPasskey[10] = s.ReadChar();
            this.mPasskey[11] = s.ReadChar();
            this.mPasskey[12] = s.ReadChar();
            this.mPasskey[13] = s.ReadChar();
            this.mPasskey[14] = s.ReadChar();
            this.mPasskey[15] = s.ReadChar();
            this.mPasskey[16] = s.ReadChar();
            this.mPasskey[17] = s.ReadChar();
            this.mPasskey[18] = s.ReadChar();
            this.mPasskey[19] = s.ReadChar();
            this.mPasskey[20] = s.ReadChar();
            this.mPasskey[21] = s.ReadChar();
            this.mPasskey[22] = s.ReadChar();
            this.mPasskey[23] = s.ReadChar();
            this.mPasskey[24] = s.ReadChar();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mGcsSystemId);
            s.Write(mControlRequest);
            s.Write(mAck);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mGcsSystemId = s.ReadByte();
            this.mControlRequest = s.ReadByte();
            this.mAck = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mKey[0]); 
            s.Write(mKey[1]); 
            s.Write(mKey[2]); 
            s.Write(mKey[3]); 
            s.Write(mKey[4]); 
            s.Write(mKey[5]); 
            s.Write(mKey[6]); 
            s.Write(mKey[7]); 
            s.Write(mKey[8]); 
            s.Write(mKey[9]); 
            s.Write(mKey[10]); 
            s.Write(mKey[11]); 
            s.Write(mKey[12]); 
            s.Write(mKey[13]); 
            s.Write(mKey[14]); 
            s.Write(mKey[15]); 
            s.Write(mKey[16]); 
            s.Write(mKey[17]); 
            s.Write(mKey[18]); 
            s.Write(mKey[19]); 
            s.Write(mKey[20]); 
            s.Write(mKey[21]); 
            s.Write(mKey[22]); 
            s.Write(mKey[23]); 
            s.Write(mKey[24]); 
            s.Write(mKey[25]); 
            s.Write(mKey[26]); 
            s.Write(mKey[27]); 
            s.Write(mKey[28]); 
            s.Write(mKey[29]); 
            s.Write(mKey[30]); 
            s.Write(mKey[31]); 
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mKey[0] = s.ReadChar();
            this.mKey[1] = s.ReadChar();
            this.mKey[2] = s.ReadChar();
            this.mKey[3] = s.ReadChar();
            this.mKey[4] = s.ReadChar();
            this.mKey[5] = s.ReadChar();
            this.mKey[6] = s.ReadChar();
            this.mKey[7] = s.ReadChar();
            this.mKey[8] = s.ReadChar();
            this.mKey[9] = s.ReadChar();
            this.mKey[10] = s.ReadChar();
            this.mKey[11] = s.ReadChar();
            this.mKey[12] = s.ReadChar();
            this.mKey[13] = s.ReadChar();
            this.mKey[14] = s.ReadChar();
            this.mKey[15] = s.ReadChar();
            this.mKey[16] = s.ReadChar();
            this.mKey[17] = s.ReadChar();
            this.mKey[18] = s.ReadChar();
            this.mKey[19] = s.ReadChar();
            this.mKey[20] = s.ReadChar();
            this.mKey[21] = s.ReadChar();
            this.mKey[22] = s.ReadChar();
            this.mKey[23] = s.ReadChar();
            this.mKey[24] = s.ReadChar();
            this.mKey[25] = s.ReadChar();
            this.mKey[26] = s.ReadChar();
            this.mKey[27] = s.ReadChar();
            this.mKey[28] = s.ReadChar();
            this.mKey[29] = s.ReadChar();
            this.mKey[30] = s.ReadChar();
            this.mKey[31] = s.ReadChar();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mCustomMode);
            s.Write(mTargetSystem);
            s.Write(mBaseMode);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mCustomMode = s.ReadUInt32();
            this.mTargetSystem = s.ReadByte();
            this.mBaseMode = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mParamIndex);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
            s.Write(mParamId[0]); 
            s.Write(mParamId[1]); 
            s.Write(mParamId[2]); 
            s.Write(mParamId[3]); 
            s.Write(mParamId[4]); 
            s.Write(mParamId[5]); 
            s.Write(mParamId[6]); 
            s.Write(mParamId[7]); 
            s.Write(mParamId[8]); 
            s.Write(mParamId[9]); 
            s.Write(mParamId[10]); 
            s.Write(mParamId[11]); 
            s.Write(mParamId[12]); 
            s.Write(mParamId[13]); 
            s.Write(mParamId[14]); 
            s.Write(mParamId[15]); 
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mParamIndex = s.ReadInt16();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
            this.mParamId[0] = s.ReadChar();
            this.mParamId[1] = s.ReadChar();
            this.mParamId[2] = s.ReadChar();
            this.mParamId[3] = s.ReadChar();
            this.mParamId[4] = s.ReadChar();
            this.mParamId[5] = s.ReadChar();
            this.mParamId[6] = s.ReadChar();
            this.mParamId[7] = s.ReadChar();
            this.mParamId[8] = s.ReadChar();
            this.mParamId[9] = s.ReadChar();
            this.mParamId[10] = s.ReadChar();
            this.mParamId[11] = s.ReadChar();
            this.mParamId[12] = s.ReadChar();
            this.mParamId[13] = s.ReadChar();
            this.mParamId[14] = s.ReadChar();
            this.mParamId[15] = s.ReadChar();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        public MavParamType ParamType {
            get { return mParamType; }
            set { mParamType = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mParamValue);
            s.Write(mParamCount);
            s.Write(mParamIndex);
            s.Write(mParamId[0]); 
            s.Write(mParamId[1]); 
            s.Write(mParamId[2]); 
            s.Write(mParamId[3]); 
            s.Write(mParamId[4]); 
            s.Write(mParamId[5]); 
            s.Write(mParamId[6]); 
            s.Write(mParamId[7]); 
            s.Write(mParamId[8]); 
            s.Write(mParamId[9]); 
            s.Write(mParamId[10]); 
            s.Write(mParamId[11]); 
            s.Write(mParamId[12]); 
            s.Write(mParamId[13]); 
            s.Write(mParamId[14]); 
            s.Write(mParamId[15]); 
            s.Write((byte)mParamType);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mParamValue = s.ReadSingle();
            this.mParamCount = s.ReadUInt16();
            this.mParamIndex = s.ReadUInt16();
            this.mParamId[0] = s.ReadChar();
            this.mParamId[1] = s.ReadChar();
            this.mParamId[2] = s.ReadChar();
            this.mParamId[3] = s.ReadChar();
            this.mParamId[4] = s.ReadChar();
            this.mParamId[5] = s.ReadChar();
            this.mParamId[6] = s.ReadChar();
            this.mParamId[7] = s.ReadChar();
            this.mParamId[8] = s.ReadChar();
            this.mParamId[9] = s.ReadChar();
            this.mParamId[10] = s.ReadChar();
            this.mParamId[11] = s.ReadChar();
            this.mParamId[12] = s.ReadChar();
            this.mParamId[13] = s.ReadChar();
            this.mParamId[14] = s.ReadChar();
            this.mParamId[15] = s.ReadChar();
            this.mParamType = (MavParamType)s.ReadByte();
        }

        private float mParamValue;
        private UInt16 mParamCount;
        private UInt16 mParamIndex;
        private char[] mParamId = new char[16];
        private MavParamType mParamType;
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

        public MavParamType ParamType {
            get { return mParamType; }
            set { mParamType = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mParamValue);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
            s.Write(mParamId[0]); 
            s.Write(mParamId[1]); 
            s.Write(mParamId[2]); 
            s.Write(mParamId[3]); 
            s.Write(mParamId[4]); 
            s.Write(mParamId[5]); 
            s.Write(mParamId[6]); 
            s.Write(mParamId[7]); 
            s.Write(mParamId[8]); 
            s.Write(mParamId[9]); 
            s.Write(mParamId[10]); 
            s.Write(mParamId[11]); 
            s.Write(mParamId[12]); 
            s.Write(mParamId[13]); 
            s.Write(mParamId[14]); 
            s.Write(mParamId[15]); 
            s.Write((byte)mParamType);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mParamValue = s.ReadSingle();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
            this.mParamId[0] = s.ReadChar();
            this.mParamId[1] = s.ReadChar();
            this.mParamId[2] = s.ReadChar();
            this.mParamId[3] = s.ReadChar();
            this.mParamId[4] = s.ReadChar();
            this.mParamId[5] = s.ReadChar();
            this.mParamId[6] = s.ReadChar();
            this.mParamId[7] = s.ReadChar();
            this.mParamId[8] = s.ReadChar();
            this.mParamId[9] = s.ReadChar();
            this.mParamId[10] = s.ReadChar();
            this.mParamId[11] = s.ReadChar();
            this.mParamId[12] = s.ReadChar();
            this.mParamId[13] = s.ReadChar();
            this.mParamId[14] = s.ReadChar();
            this.mParamId[15] = s.ReadChar();
            this.mParamType = (MavParamType)s.ReadByte();
        }

        private float mParamValue;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private char[] mParamId = new char[16];
        private MavParamType mParamType;
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mLat);
            s.Write(mLon);
            s.Write(mAlt);
            s.Write(mEph);
            s.Write(mEpv);
            s.Write(mVel);
            s.Write(mCog);
            s.Write(mFixType);
            s.Write(mSatellitesVisible);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mLat = s.ReadInt32();
            this.mLon = s.ReadInt32();
            this.mAlt = s.ReadInt32();
            this.mEph = s.ReadUInt16();
            this.mEpv = s.ReadUInt16();
            this.mVel = s.ReadUInt16();
            this.mCog = s.ReadUInt16();
            this.mFixType = s.ReadByte();
            this.mSatellitesVisible = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mSatellitesVisible);
            s.Write(mSatellitePrn[0]); 
            s.Write(mSatellitePrn[1]); 
            s.Write(mSatellitePrn[2]); 
            s.Write(mSatellitePrn[3]); 
            s.Write(mSatellitePrn[4]); 
            s.Write(mSatellitePrn[5]); 
            s.Write(mSatellitePrn[6]); 
            s.Write(mSatellitePrn[7]); 
            s.Write(mSatellitePrn[8]); 
            s.Write(mSatellitePrn[9]); 
            s.Write(mSatellitePrn[10]); 
            s.Write(mSatellitePrn[11]); 
            s.Write(mSatellitePrn[12]); 
            s.Write(mSatellitePrn[13]); 
            s.Write(mSatellitePrn[14]); 
            s.Write(mSatellitePrn[15]); 
            s.Write(mSatellitePrn[16]); 
            s.Write(mSatellitePrn[17]); 
            s.Write(mSatellitePrn[18]); 
            s.Write(mSatellitePrn[19]); 
            s.Write(mSatelliteUsed[0]); 
            s.Write(mSatelliteUsed[1]); 
            s.Write(mSatelliteUsed[2]); 
            s.Write(mSatelliteUsed[3]); 
            s.Write(mSatelliteUsed[4]); 
            s.Write(mSatelliteUsed[5]); 
            s.Write(mSatelliteUsed[6]); 
            s.Write(mSatelliteUsed[7]); 
            s.Write(mSatelliteUsed[8]); 
            s.Write(mSatelliteUsed[9]); 
            s.Write(mSatelliteUsed[10]); 
            s.Write(mSatelliteUsed[11]); 
            s.Write(mSatelliteUsed[12]); 
            s.Write(mSatelliteUsed[13]); 
            s.Write(mSatelliteUsed[14]); 
            s.Write(mSatelliteUsed[15]); 
            s.Write(mSatelliteUsed[16]); 
            s.Write(mSatelliteUsed[17]); 
            s.Write(mSatelliteUsed[18]); 
            s.Write(mSatelliteUsed[19]); 
            s.Write(mSatelliteElevation[0]); 
            s.Write(mSatelliteElevation[1]); 
            s.Write(mSatelliteElevation[2]); 
            s.Write(mSatelliteElevation[3]); 
            s.Write(mSatelliteElevation[4]); 
            s.Write(mSatelliteElevation[5]); 
            s.Write(mSatelliteElevation[6]); 
            s.Write(mSatelliteElevation[7]); 
            s.Write(mSatelliteElevation[8]); 
            s.Write(mSatelliteElevation[9]); 
            s.Write(mSatelliteElevation[10]); 
            s.Write(mSatelliteElevation[11]); 
            s.Write(mSatelliteElevation[12]); 
            s.Write(mSatelliteElevation[13]); 
            s.Write(mSatelliteElevation[14]); 
            s.Write(mSatelliteElevation[15]); 
            s.Write(mSatelliteElevation[16]); 
            s.Write(mSatelliteElevation[17]); 
            s.Write(mSatelliteElevation[18]); 
            s.Write(mSatelliteElevation[19]); 
            s.Write(mSatelliteAzimuth[0]); 
            s.Write(mSatelliteAzimuth[1]); 
            s.Write(mSatelliteAzimuth[2]); 
            s.Write(mSatelliteAzimuth[3]); 
            s.Write(mSatelliteAzimuth[4]); 
            s.Write(mSatelliteAzimuth[5]); 
            s.Write(mSatelliteAzimuth[6]); 
            s.Write(mSatelliteAzimuth[7]); 
            s.Write(mSatelliteAzimuth[8]); 
            s.Write(mSatelliteAzimuth[9]); 
            s.Write(mSatelliteAzimuth[10]); 
            s.Write(mSatelliteAzimuth[11]); 
            s.Write(mSatelliteAzimuth[12]); 
            s.Write(mSatelliteAzimuth[13]); 
            s.Write(mSatelliteAzimuth[14]); 
            s.Write(mSatelliteAzimuth[15]); 
            s.Write(mSatelliteAzimuth[16]); 
            s.Write(mSatelliteAzimuth[17]); 
            s.Write(mSatelliteAzimuth[18]); 
            s.Write(mSatelliteAzimuth[19]); 
            s.Write(mSatelliteSnr[0]); 
            s.Write(mSatelliteSnr[1]); 
            s.Write(mSatelliteSnr[2]); 
            s.Write(mSatelliteSnr[3]); 
            s.Write(mSatelliteSnr[4]); 
            s.Write(mSatelliteSnr[5]); 
            s.Write(mSatelliteSnr[6]); 
            s.Write(mSatelliteSnr[7]); 
            s.Write(mSatelliteSnr[8]); 
            s.Write(mSatelliteSnr[9]); 
            s.Write(mSatelliteSnr[10]); 
            s.Write(mSatelliteSnr[11]); 
            s.Write(mSatelliteSnr[12]); 
            s.Write(mSatelliteSnr[13]); 
            s.Write(mSatelliteSnr[14]); 
            s.Write(mSatelliteSnr[15]); 
            s.Write(mSatelliteSnr[16]); 
            s.Write(mSatelliteSnr[17]); 
            s.Write(mSatelliteSnr[18]); 
            s.Write(mSatelliteSnr[19]); 
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mSatellitesVisible = s.ReadByte();
            this.mSatellitePrn[0] = s.ReadByte();
            this.mSatellitePrn[1] = s.ReadByte();
            this.mSatellitePrn[2] = s.ReadByte();
            this.mSatellitePrn[3] = s.ReadByte();
            this.mSatellitePrn[4] = s.ReadByte();
            this.mSatellitePrn[5] = s.ReadByte();
            this.mSatellitePrn[6] = s.ReadByte();
            this.mSatellitePrn[7] = s.ReadByte();
            this.mSatellitePrn[8] = s.ReadByte();
            this.mSatellitePrn[9] = s.ReadByte();
            this.mSatellitePrn[10] = s.ReadByte();
            this.mSatellitePrn[11] = s.ReadByte();
            this.mSatellitePrn[12] = s.ReadByte();
            this.mSatellitePrn[13] = s.ReadByte();
            this.mSatellitePrn[14] = s.ReadByte();
            this.mSatellitePrn[15] = s.ReadByte();
            this.mSatellitePrn[16] = s.ReadByte();
            this.mSatellitePrn[17] = s.ReadByte();
            this.mSatellitePrn[18] = s.ReadByte();
            this.mSatellitePrn[19] = s.ReadByte();
            this.mSatelliteUsed[0] = s.ReadByte();
            this.mSatelliteUsed[1] = s.ReadByte();
            this.mSatelliteUsed[2] = s.ReadByte();
            this.mSatelliteUsed[3] = s.ReadByte();
            this.mSatelliteUsed[4] = s.ReadByte();
            this.mSatelliteUsed[5] = s.ReadByte();
            this.mSatelliteUsed[6] = s.ReadByte();
            this.mSatelliteUsed[7] = s.ReadByte();
            this.mSatelliteUsed[8] = s.ReadByte();
            this.mSatelliteUsed[9] = s.ReadByte();
            this.mSatelliteUsed[10] = s.ReadByte();
            this.mSatelliteUsed[11] = s.ReadByte();
            this.mSatelliteUsed[12] = s.ReadByte();
            this.mSatelliteUsed[13] = s.ReadByte();
            this.mSatelliteUsed[14] = s.ReadByte();
            this.mSatelliteUsed[15] = s.ReadByte();
            this.mSatelliteUsed[16] = s.ReadByte();
            this.mSatelliteUsed[17] = s.ReadByte();
            this.mSatelliteUsed[18] = s.ReadByte();
            this.mSatelliteUsed[19] = s.ReadByte();
            this.mSatelliteElevation[0] = s.ReadByte();
            this.mSatelliteElevation[1] = s.ReadByte();
            this.mSatelliteElevation[2] = s.ReadByte();
            this.mSatelliteElevation[3] = s.ReadByte();
            this.mSatelliteElevation[4] = s.ReadByte();
            this.mSatelliteElevation[5] = s.ReadByte();
            this.mSatelliteElevation[6] = s.ReadByte();
            this.mSatelliteElevation[7] = s.ReadByte();
            this.mSatelliteElevation[8] = s.ReadByte();
            this.mSatelliteElevation[9] = s.ReadByte();
            this.mSatelliteElevation[10] = s.ReadByte();
            this.mSatelliteElevation[11] = s.ReadByte();
            this.mSatelliteElevation[12] = s.ReadByte();
            this.mSatelliteElevation[13] = s.ReadByte();
            this.mSatelliteElevation[14] = s.ReadByte();
            this.mSatelliteElevation[15] = s.ReadByte();
            this.mSatelliteElevation[16] = s.ReadByte();
            this.mSatelliteElevation[17] = s.ReadByte();
            this.mSatelliteElevation[18] = s.ReadByte();
            this.mSatelliteElevation[19] = s.ReadByte();
            this.mSatelliteAzimuth[0] = s.ReadByte();
            this.mSatelliteAzimuth[1] = s.ReadByte();
            this.mSatelliteAzimuth[2] = s.ReadByte();
            this.mSatelliteAzimuth[3] = s.ReadByte();
            this.mSatelliteAzimuth[4] = s.ReadByte();
            this.mSatelliteAzimuth[5] = s.ReadByte();
            this.mSatelliteAzimuth[6] = s.ReadByte();
            this.mSatelliteAzimuth[7] = s.ReadByte();
            this.mSatelliteAzimuth[8] = s.ReadByte();
            this.mSatelliteAzimuth[9] = s.ReadByte();
            this.mSatelliteAzimuth[10] = s.ReadByte();
            this.mSatelliteAzimuth[11] = s.ReadByte();
            this.mSatelliteAzimuth[12] = s.ReadByte();
            this.mSatelliteAzimuth[13] = s.ReadByte();
            this.mSatelliteAzimuth[14] = s.ReadByte();
            this.mSatelliteAzimuth[15] = s.ReadByte();
            this.mSatelliteAzimuth[16] = s.ReadByte();
            this.mSatelliteAzimuth[17] = s.ReadByte();
            this.mSatelliteAzimuth[18] = s.ReadByte();
            this.mSatelliteAzimuth[19] = s.ReadByte();
            this.mSatelliteSnr[0] = s.ReadByte();
            this.mSatelliteSnr[1] = s.ReadByte();
            this.mSatelliteSnr[2] = s.ReadByte();
            this.mSatelliteSnr[3] = s.ReadByte();
            this.mSatelliteSnr[4] = s.ReadByte();
            this.mSatelliteSnr[5] = s.ReadByte();
            this.mSatelliteSnr[6] = s.ReadByte();
            this.mSatelliteSnr[7] = s.ReadByte();
            this.mSatelliteSnr[8] = s.ReadByte();
            this.mSatelliteSnr[9] = s.ReadByte();
            this.mSatelliteSnr[10] = s.ReadByte();
            this.mSatelliteSnr[11] = s.ReadByte();
            this.mSatelliteSnr[12] = s.ReadByte();
            this.mSatelliteSnr[13] = s.ReadByte();
            this.mSatelliteSnr[14] = s.ReadByte();
            this.mSatelliteSnr[15] = s.ReadByte();
            this.mSatelliteSnr[16] = s.ReadByte();
            this.mSatelliteSnr[17] = s.ReadByte();
            this.mSatelliteSnr[18] = s.ReadByte();
            this.mSatelliteSnr[19] = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mXacc);
            s.Write(mYacc);
            s.Write(mZacc);
            s.Write(mXgyro);
            s.Write(mYgyro);
            s.Write(mZgyro);
            s.Write(mXmag);
            s.Write(mYmag);
            s.Write(mZmag);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mXacc = s.ReadInt16();
            this.mYacc = s.ReadInt16();
            this.mZacc = s.ReadInt16();
            this.mXgyro = s.ReadInt16();
            this.mYgyro = s.ReadInt16();
            this.mZgyro = s.ReadInt16();
            this.mXmag = s.ReadInt16();
            this.mYmag = s.ReadInt16();
            this.mZmag = s.ReadInt16();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mXacc);
            s.Write(mYacc);
            s.Write(mZacc);
            s.Write(mXgyro);
            s.Write(mYgyro);
            s.Write(mZgyro);
            s.Write(mXmag);
            s.Write(mYmag);
            s.Write(mZmag);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mXacc = s.ReadInt16();
            this.mYacc = s.ReadInt16();
            this.mZacc = s.ReadInt16();
            this.mXgyro = s.ReadInt16();
            this.mYgyro = s.ReadInt16();
            this.mZgyro = s.ReadInt16();
            this.mXmag = s.ReadInt16();
            this.mYmag = s.ReadInt16();
            this.mZmag = s.ReadInt16();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mPressAbs);
            s.Write(mPressDiff1);
            s.Write(mPressDiff2);
            s.Write(mTemperature);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mPressAbs = s.ReadInt16();
            this.mPressDiff1 = s.ReadInt16();
            this.mPressDiff2 = s.ReadInt16();
            this.mTemperature = s.ReadInt16();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mPressAbs);
            s.Write(mPressDiff);
            s.Write(mTemperature);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mPressAbs = s.ReadSingle();
            this.mPressDiff = s.ReadSingle();
            this.mTemperature = s.ReadInt16();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mRoll);
            s.Write(mPitch);
            s.Write(mYaw);
            s.Write(mRollspeed);
            s.Write(mPitchspeed);
            s.Write(mYawspeed);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mRoll = s.ReadSingle();
            this.mPitch = s.ReadSingle();
            this.mYaw = s.ReadSingle();
            this.mRollspeed = s.ReadSingle();
            this.mPitchspeed = s.ReadSingle();
            this.mYawspeed = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mQ1);
            s.Write(mQ2);
            s.Write(mQ3);
            s.Write(mQ4);
            s.Write(mRollspeed);
            s.Write(mPitchspeed);
            s.Write(mYawspeed);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mQ1 = s.ReadSingle();
            this.mQ2 = s.ReadSingle();
            this.mQ3 = s.ReadSingle();
            this.mQ4 = s.ReadSingle();
            this.mRollspeed = s.ReadSingle();
            this.mPitchspeed = s.ReadSingle();
            this.mYawspeed = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mX);
            s.Write(mY);
            s.Write(mZ);
            s.Write(mVx);
            s.Write(mVy);
            s.Write(mVz);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mX = s.ReadSingle();
            this.mY = s.ReadSingle();
            this.mZ = s.ReadSingle();
            this.mVx = s.ReadSingle();
            this.mVy = s.ReadSingle();
            this.mVz = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mLat);
            s.Write(mLon);
            s.Write(mAlt);
            s.Write(mRelativeAlt);
            s.Write(mVx);
            s.Write(mVy);
            s.Write(mVz);
            s.Write(mHdg);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mLat = s.ReadInt32();
            this.mLon = s.ReadInt32();
            this.mAlt = s.ReadInt32();
            this.mRelativeAlt = s.ReadInt32();
            this.mVx = s.ReadInt16();
            this.mVy = s.ReadInt16();
            this.mVz = s.ReadInt16();
            this.mHdg = s.ReadUInt16();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mChan1Scaled);
            s.Write(mChan2Scaled);
            s.Write(mChan3Scaled);
            s.Write(mChan4Scaled);
            s.Write(mChan5Scaled);
            s.Write(mChan6Scaled);
            s.Write(mChan7Scaled);
            s.Write(mChan8Scaled);
            s.Write(mPort);
            s.Write(mRssi);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mChan1Scaled = s.ReadInt16();
            this.mChan2Scaled = s.ReadInt16();
            this.mChan3Scaled = s.ReadInt16();
            this.mChan4Scaled = s.ReadInt16();
            this.mChan5Scaled = s.ReadInt16();
            this.mChan6Scaled = s.ReadInt16();
            this.mChan7Scaled = s.ReadInt16();
            this.mChan8Scaled = s.ReadInt16();
            this.mPort = s.ReadByte();
            this.mRssi = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mChan1Raw);
            s.Write(mChan2Raw);
            s.Write(mChan3Raw);
            s.Write(mChan4Raw);
            s.Write(mChan5Raw);
            s.Write(mChan6Raw);
            s.Write(mChan7Raw);
            s.Write(mChan8Raw);
            s.Write(mPort);
            s.Write(mRssi);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mChan1Raw = s.ReadUInt16();
            this.mChan2Raw = s.ReadUInt16();
            this.mChan3Raw = s.ReadUInt16();
            this.mChan4Raw = s.ReadUInt16();
            this.mChan5Raw = s.ReadUInt16();
            this.mChan6Raw = s.ReadUInt16();
            this.mChan7Raw = s.ReadUInt16();
            this.mChan8Raw = s.ReadUInt16();
            this.mPort = s.ReadByte();
            this.mRssi = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mServo1Raw);
            s.Write(mServo2Raw);
            s.Write(mServo3Raw);
            s.Write(mServo4Raw);
            s.Write(mServo5Raw);
            s.Write(mServo6Raw);
            s.Write(mServo7Raw);
            s.Write(mServo8Raw);
            s.Write(mPort);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt32();
            this.mServo1Raw = s.ReadUInt16();
            this.mServo2Raw = s.ReadUInt16();
            this.mServo3Raw = s.ReadUInt16();
            this.mServo4Raw = s.ReadUInt16();
            this.mServo5Raw = s.ReadUInt16();
            this.mServo6Raw = s.ReadUInt16();
            this.mServo7Raw = s.ReadUInt16();
            this.mServo8Raw = s.ReadUInt16();
            this.mPort = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mStartIndex);
            s.Write(mEndIndex);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mStartIndex = s.ReadInt16();
            this.mEndIndex = s.ReadInt16();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mStartIndex);
            s.Write(mEndIndex);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mStartIndex = s.ReadInt16();
            this.mEndIndex = s.ReadInt16();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        public MavCmd Command {
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

        public MavFrame Frame {
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mParam1);
            s.Write(mParam2);
            s.Write(mParam3);
            s.Write(mParam4);
            s.Write(mX);
            s.Write(mY);
            s.Write(mZ);
            s.Write(mSeq);
            s.Write((UInt16)mCommand);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
            s.Write((byte)mFrame);
            s.Write(mCurrent);
            s.Write(mAutocontinue);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mParam1 = s.ReadSingle();
            this.mParam2 = s.ReadSingle();
            this.mParam3 = s.ReadSingle();
            this.mParam4 = s.ReadSingle();
            this.mX = s.ReadSingle();
            this.mY = s.ReadSingle();
            this.mZ = s.ReadSingle();
            this.mSeq = s.ReadUInt16();
            this.mCommand = (MavCmd)s.ReadUInt16();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
            this.mFrame = (MavFrame)s.ReadByte();
            this.mCurrent = s.ReadByte();
            this.mAutocontinue = s.ReadByte();
        }

        private float mParam1;
        private float mParam2;
        private float mParam3;
        private float mParam4;
        private float mX;
        private float mY;
        private float mZ;
        private UInt16 mSeq;
        private MavCmd mCommand;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private MavFrame mFrame;
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mSeq);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mSeq = s.ReadUInt16();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mSeq);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mSeq = s.ReadUInt16();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mSeq);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mSeq = s.ReadUInt16();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mCount);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mCount = s.ReadUInt16();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mSeq);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mSeq = s.ReadUInt16();
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

        public MavMissionResult Type {
            get { return mType; }
            set { mType = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
            s.Write((byte)mType);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
            this.mType = (MavMissionResult)s.ReadByte();
        }

        private byte mTargetSystem;
        private byte mTargetComponent;
        private MavMissionResult mType;
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mLatitude);
            s.Write(mLongitude);
            s.Write(mAltitude);
            s.Write(mTargetSystem);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mLatitude = s.ReadInt32();
            this.mLongitude = s.ReadInt32();
            this.mAltitude = s.ReadInt32();
            this.mTargetSystem = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mLatitude);
            s.Write(mLongitude);
            s.Write(mAltitude);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mLatitude = s.ReadInt32();
            this.mLongitude = s.ReadInt32();
            this.mAltitude = s.ReadInt32();
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

        public MavFrame CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mX);
            s.Write(mY);
            s.Write(mZ);
            s.Write(mYaw);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
            s.Write((byte)mCoordinateFrame);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mX = s.ReadSingle();
            this.mY = s.ReadSingle();
            this.mZ = s.ReadSingle();
            this.mYaw = s.ReadSingle();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
            this.mCoordinateFrame = (MavFrame)s.ReadByte();
        }

        private float mX;
        private float mY;
        private float mZ;
        private float mYaw;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private MavFrame mCoordinateFrame;
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

        public MavFrame CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mX);
            s.Write(mY);
            s.Write(mZ);
            s.Write(mYaw);
            s.Write((byte)mCoordinateFrame);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mX = s.ReadSingle();
            this.mY = s.ReadSingle();
            this.mZ = s.ReadSingle();
            this.mYaw = s.ReadSingle();
            this.mCoordinateFrame = (MavFrame)s.ReadByte();
        }

        private float mX;
        private float mY;
        private float mZ;
        private float mYaw;
        private MavFrame mCoordinateFrame;
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

        public MavFrame CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mLatitude);
            s.Write(mLongitude);
            s.Write(mAltitude);
            s.Write(mYaw);
            s.Write((byte)mCoordinateFrame);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mLatitude = s.ReadInt32();
            this.mLongitude = s.ReadInt32();
            this.mAltitude = s.ReadInt32();
            this.mYaw = s.ReadInt16();
            this.mCoordinateFrame = (MavFrame)s.ReadByte();
        }

        private Int32 mLatitude;
        private Int32 mLongitude;
        private Int32 mAltitude;
        private Int16 mYaw;
        private MavFrame mCoordinateFrame;
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

        public MavFrame CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mLatitude);
            s.Write(mLongitude);
            s.Write(mAltitude);
            s.Write(mYaw);
            s.Write((byte)mCoordinateFrame);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mLatitude = s.ReadInt32();
            this.mLongitude = s.ReadInt32();
            this.mAltitude = s.ReadInt32();
            this.mYaw = s.ReadInt16();
            this.mCoordinateFrame = (MavFrame)s.ReadByte();
        }

        private Int32 mLatitude;
        private Int32 mLongitude;
        private Int32 mAltitude;
        private Int16 mYaw;
        private MavFrame mCoordinateFrame;
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

        public MavFrame Frame {
            get { return mFrame; }
            set { mFrame = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mP1x);
            s.Write(mP1y);
            s.Write(mP1z);
            s.Write(mP2x);
            s.Write(mP2y);
            s.Write(mP2z);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
            s.Write((byte)mFrame);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mP1x = s.ReadSingle();
            this.mP1y = s.ReadSingle();
            this.mP1z = s.ReadSingle();
            this.mP2x = s.ReadSingle();
            this.mP2y = s.ReadSingle();
            this.mP2z = s.ReadSingle();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
            this.mFrame = (MavFrame)s.ReadByte();
        }

        private float mP1x;
        private float mP1y;
        private float mP1z;
        private float mP2x;
        private float mP2y;
        private float mP2z;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private MavFrame mFrame;
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

        public MavFrame Frame {
            get { return mFrame; }
            set { mFrame = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mP1x);
            s.Write(mP1y);
            s.Write(mP1z);
            s.Write(mP2x);
            s.Write(mP2y);
            s.Write(mP2z);
            s.Write((byte)mFrame);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mP1x = s.ReadSingle();
            this.mP1y = s.ReadSingle();
            this.mP1z = s.ReadSingle();
            this.mP2x = s.ReadSingle();
            this.mP2y = s.ReadSingle();
            this.mP2z = s.ReadSingle();
            this.mFrame = (MavFrame)s.ReadByte();
        }

        private float mP1x;
        private float mP1y;
        private float mP1z;
        private float mP2x;
        private float mP2y;
        private float mP2z;
        private MavFrame mFrame;
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mRoll);
            s.Write(mPitch);
            s.Write(mYaw);
            s.Write(mThrust);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mRoll = s.ReadSingle();
            this.mPitch = s.ReadSingle();
            this.mYaw = s.ReadSingle();
            this.mThrust = s.ReadSingle();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mRollSpeed);
            s.Write(mPitchSpeed);
            s.Write(mYawSpeed);
            s.Write(mThrust);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mRollSpeed = s.ReadSingle();
            this.mPitchSpeed = s.ReadSingle();
            this.mYawSpeed = s.ReadSingle();
            this.mThrust = s.ReadSingle();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mRoll);
            s.Write(mPitch);
            s.Write(mYaw);
            s.Write(mThrust);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mRoll = s.ReadSingle();
            this.mPitch = s.ReadSingle();
            this.mYaw = s.ReadSingle();
            this.mThrust = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mRollSpeed);
            s.Write(mPitchSpeed);
            s.Write(mYawSpeed);
            s.Write(mThrust);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mRollSpeed = s.ReadSingle();
            this.mPitchSpeed = s.ReadSingle();
            this.mYawSpeed = s.ReadSingle();
            this.mThrust = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mMotorFrontNw);
            s.Write(mMotorRightNe);
            s.Write(mMotorBackSe);
            s.Write(mMotorLeftSw);
            s.Write(mTargetSystem);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mMotorFrontNw = s.ReadUInt16();
            this.mMotorRightNe = s.ReadUInt16();
            this.mMotorBackSe = s.ReadUInt16();
            this.mMotorLeftSw = s.ReadUInt16();
            this.mTargetSystem = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mRoll[0]); 
            s.Write(mRoll[1]); 
            s.Write(mRoll[2]); 
            s.Write(mRoll[3]); 
            s.Write(mPitch[0]); 
            s.Write(mPitch[1]); 
            s.Write(mPitch[2]); 
            s.Write(mPitch[3]); 
            s.Write(mYaw[0]); 
            s.Write(mYaw[1]); 
            s.Write(mYaw[2]); 
            s.Write(mYaw[3]); 
            s.Write(mThrust[0]); 
            s.Write(mThrust[1]); 
            s.Write(mThrust[2]); 
            s.Write(mThrust[3]); 
            s.Write(mGroup);
            s.Write(mMode);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mRoll[0] = s.ReadInt16();
            this.mRoll[1] = s.ReadInt16();
            this.mRoll[2] = s.ReadInt16();
            this.mRoll[3] = s.ReadInt16();
            this.mPitch[0] = s.ReadInt16();
            this.mPitch[1] = s.ReadInt16();
            this.mPitch[2] = s.ReadInt16();
            this.mPitch[3] = s.ReadInt16();
            this.mYaw[0] = s.ReadInt16();
            this.mYaw[1] = s.ReadInt16();
            this.mYaw[2] = s.ReadInt16();
            this.mYaw[3] = s.ReadInt16();
            this.mThrust[0] = s.ReadUInt16();
            this.mThrust[1] = s.ReadUInt16();
            this.mThrust[2] = s.ReadUInt16();
            this.mThrust[3] = s.ReadUInt16();
            this.mGroup = s.ReadByte();
            this.mMode = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mNavRoll);
            s.Write(mNavPitch);
            s.Write(mAltError);
            s.Write(mAspdError);
            s.Write(mXtrackError);
            s.Write(mNavBearing);
            s.Write(mTargetBearing);
            s.Write(mWpDist);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mNavRoll = s.ReadSingle();
            this.mNavPitch = s.ReadSingle();
            this.mAltError = s.ReadSingle();
            this.mAspdError = s.ReadSingle();
            this.mXtrackError = s.ReadSingle();
            this.mNavBearing = s.ReadInt16();
            this.mTargetBearing = s.ReadInt16();
            this.mWpDist = s.ReadUInt16();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mRoll[0]); 
            s.Write(mRoll[1]); 
            s.Write(mRoll[2]); 
            s.Write(mRoll[3]); 
            s.Write(mPitch[0]); 
            s.Write(mPitch[1]); 
            s.Write(mPitch[2]); 
            s.Write(mPitch[3]); 
            s.Write(mYaw[0]); 
            s.Write(mYaw[1]); 
            s.Write(mYaw[2]); 
            s.Write(mYaw[3]); 
            s.Write(mThrust[0]); 
            s.Write(mThrust[1]); 
            s.Write(mThrust[2]); 
            s.Write(mThrust[3]); 
            s.Write(mGroup);
            s.Write(mMode);
            s.Write(mLedRed[0]); 
            s.Write(mLedRed[1]); 
            s.Write(mLedRed[2]); 
            s.Write(mLedRed[3]); 
            s.Write(mLedBlue[0]); 
            s.Write(mLedBlue[1]); 
            s.Write(mLedBlue[2]); 
            s.Write(mLedBlue[3]); 
            s.Write(mLedGreen[0]); 
            s.Write(mLedGreen[1]); 
            s.Write(mLedGreen[2]); 
            s.Write(mLedGreen[3]); 
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mRoll[0] = s.ReadInt16();
            this.mRoll[1] = s.ReadInt16();
            this.mRoll[2] = s.ReadInt16();
            this.mRoll[3] = s.ReadInt16();
            this.mPitch[0] = s.ReadInt16();
            this.mPitch[1] = s.ReadInt16();
            this.mPitch[2] = s.ReadInt16();
            this.mPitch[3] = s.ReadInt16();
            this.mYaw[0] = s.ReadInt16();
            this.mYaw[1] = s.ReadInt16();
            this.mYaw[2] = s.ReadInt16();
            this.mYaw[3] = s.ReadInt16();
            this.mThrust[0] = s.ReadUInt16();
            this.mThrust[1] = s.ReadUInt16();
            this.mThrust[2] = s.ReadUInt16();
            this.mThrust[3] = s.ReadUInt16();
            this.mGroup = s.ReadByte();
            this.mMode = s.ReadByte();
            this.mLedRed[0] = s.ReadByte();
            this.mLedRed[1] = s.ReadByte();
            this.mLedRed[2] = s.ReadByte();
            this.mLedRed[3] = s.ReadByte();
            this.mLedBlue[0] = s.ReadByte();
            this.mLedBlue[1] = s.ReadByte();
            this.mLedBlue[2] = s.ReadByte();
            this.mLedBlue[3] = s.ReadByte();
            this.mLedGreen[0] = s.ReadByte();
            this.mLedGreen[1] = s.ReadByte();
            this.mLedGreen[2] = s.ReadByte();
            this.mLedGreen[3] = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mXerr);
            s.Write(mYerr);
            s.Write(mZerr);
            s.Write(mRollerr);
            s.Write(mPitcherr);
            s.Write(mYawerr);
            s.Write(mVxerr);
            s.Write(mVyerr);
            s.Write(mVzerr);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mXerr = s.ReadSingle();
            this.mYerr = s.ReadSingle();
            this.mZerr = s.ReadSingle();
            this.mRollerr = s.ReadSingle();
            this.mPitcherr = s.ReadSingle();
            this.mYawerr = s.ReadSingle();
            this.mVxerr = s.ReadSingle();
            this.mVyerr = s.ReadSingle();
            this.mVzerr = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mReqMessageRate);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
            s.Write(mReqStreamId);
            s.Write(mStartStop);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mReqMessageRate = s.ReadUInt16();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
            this.mReqStreamId = s.ReadByte();
            this.mStartStop = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mMessageRate);
            s.Write(mStreamId);
            s.Write(mOnOff);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mMessageRate = s.ReadUInt16();
            this.mStreamId = s.ReadByte();
            this.mOnOff = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mX);
            s.Write(mY);
            s.Write(mZ);
            s.Write(mR);
            s.Write(mButtons);
            s.Write(mTarget);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mX = s.ReadInt16();
            this.mY = s.ReadInt16();
            this.mZ = s.ReadInt16();
            this.mR = s.ReadInt16();
            this.mButtons = s.ReadUInt16();
            this.mTarget = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mChan1Raw);
            s.Write(mChan2Raw);
            s.Write(mChan3Raw);
            s.Write(mChan4Raw);
            s.Write(mChan5Raw);
            s.Write(mChan6Raw);
            s.Write(mChan7Raw);
            s.Write(mChan8Raw);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mChan1Raw = s.ReadUInt16();
            this.mChan2Raw = s.ReadUInt16();
            this.mChan3Raw = s.ReadUInt16();
            this.mChan4Raw = s.ReadUInt16();
            this.mChan5Raw = s.ReadUInt16();
            this.mChan6Raw = s.ReadUInt16();
            this.mChan7Raw = s.ReadUInt16();
            this.mChan8Raw = s.ReadUInt16();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mAirspeed);
            s.Write(mGroundspeed);
            s.Write(mAlt);
            s.Write(mClimb);
            s.Write(mHeading);
            s.Write(mThrottle);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mAirspeed = s.ReadSingle();
            this.mGroundspeed = s.ReadSingle();
            this.mAlt = s.ReadSingle();
            this.mClimb = s.ReadSingle();
            this.mHeading = s.ReadInt16();
            this.mThrottle = s.ReadUInt16();
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
        public MavCmd Param1 {
            get { return mParam1; }
            set { mParam1 = value; NotifyUpdated(); }
        }

        public MavCmd Param2 {
            get { return mParam2; }
            set { mParam2 = value; NotifyUpdated(); }
        }

        public MavCmd Param3 {
            get { return mParam3; }
            set { mParam3 = value; NotifyUpdated(); }
        }

        public MavCmd Param4 {
            get { return mParam4; }
            set { mParam4 = value; NotifyUpdated(); }
        }

        public MavCmd Param5 {
            get { return mParam5; }
            set { mParam5 = value; NotifyUpdated(); }
        }

        public MavCmd Param6 {
            get { return mParam6; }
            set { mParam6 = value; NotifyUpdated(); }
        }

        public MavCmd Param7 {
            get { return mParam7; }
            set { mParam7 = value; NotifyUpdated(); }
        }

        public MavCmd Command {
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write((float)mParam1);
            s.Write((float)mParam2);
            s.Write((float)mParam3);
            s.Write((float)mParam4);
            s.Write((float)mParam5);
            s.Write((float)mParam6);
            s.Write((float)mParam7);
            s.Write((UInt16)mCommand);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
            s.Write(mConfirmation);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mParam1 = (MavCmd)s.ReadSingle();
            this.mParam2 = (MavCmd)s.ReadSingle();
            this.mParam3 = (MavCmd)s.ReadSingle();
            this.mParam4 = (MavCmd)s.ReadSingle();
            this.mParam5 = (MavCmd)s.ReadSingle();
            this.mParam6 = (MavCmd)s.ReadSingle();
            this.mParam7 = (MavCmd)s.ReadSingle();
            this.mCommand = (MavCmd)s.ReadUInt16();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
            this.mConfirmation = s.ReadByte();
        }

        private MavCmd mParam1;
        private MavCmd mParam2;
        private MavCmd mParam3;
        private MavCmd mParam4;
        private MavCmd mParam5;
        private MavCmd mParam6;
        private MavCmd mParam7;
        private MavCmd mCommand;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private byte mConfirmation;
    }


    // ___________________________________________________________________________________


    public class UasCommandAck: UasMessage
    {
        public MavCmd Command {
            get { return mCommand; }
            set { mCommand = value; NotifyUpdated(); }
        }

        public MavResult Result {
            get { return mResult; }
            set { mResult = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write((UInt16)mCommand);
            s.Write((byte)mResult);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mCommand = (MavCmd)s.ReadUInt16();
            this.mResult = (MavResult)s.ReadByte();
        }

        private MavCmd mCommand;
        private MavResult mResult;
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mRollRate);
            s.Write(mPitchRate);
            s.Write(mYawRate);
            s.Write(mThrust);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mRollRate = s.ReadSingle();
            this.mPitchRate = s.ReadSingle();
            this.mYawRate = s.ReadSingle();
            this.mThrust = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mRoll);
            s.Write(mPitch);
            s.Write(mYaw);
            s.Write(mThrust);
            s.Write(mModeSwitch);
            s.Write(mManualOverrideSwitch);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mRoll = s.ReadSingle();
            this.mPitch = s.ReadSingle();
            this.mYaw = s.ReadSingle();
            this.mThrust = s.ReadSingle();
            this.mModeSwitch = s.ReadByte();
            this.mManualOverrideSwitch = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mX);
            s.Write(mY);
            s.Write(mZ);
            s.Write(mRoll);
            s.Write(mPitch);
            s.Write(mYaw);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mX = s.ReadSingle();
            this.mY = s.ReadSingle();
            this.mZ = s.ReadSingle();
            this.mRoll = s.ReadSingle();
            this.mPitch = s.ReadSingle();
            this.mYaw = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mRoll);
            s.Write(mPitch);
            s.Write(mYaw);
            s.Write(mRollspeed);
            s.Write(mPitchspeed);
            s.Write(mYawspeed);
            s.Write(mLat);
            s.Write(mLon);
            s.Write(mAlt);
            s.Write(mVx);
            s.Write(mVy);
            s.Write(mVz);
            s.Write(mXacc);
            s.Write(mYacc);
            s.Write(mZacc);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mRoll = s.ReadSingle();
            this.mPitch = s.ReadSingle();
            this.mYaw = s.ReadSingle();
            this.mRollspeed = s.ReadSingle();
            this.mPitchspeed = s.ReadSingle();
            this.mYawspeed = s.ReadSingle();
            this.mLat = s.ReadInt32();
            this.mLon = s.ReadInt32();
            this.mAlt = s.ReadInt32();
            this.mVx = s.ReadInt16();
            this.mVy = s.ReadInt16();
            this.mVz = s.ReadInt16();
            this.mXacc = s.ReadInt16();
            this.mYacc = s.ReadInt16();
            this.mZacc = s.ReadInt16();
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

        public MavMode Mode {
            get { return mMode; }
            set { mMode = value; NotifyUpdated(); }
        }

        public byte NavMode {
            get { return mNavMode; }
            set { mNavMode = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mRollAilerons);
            s.Write(mPitchElevator);
            s.Write(mYawRudder);
            s.Write(mThrottle);
            s.Write(mAux1);
            s.Write(mAux2);
            s.Write(mAux3);
            s.Write(mAux4);
            s.Write((byte)mMode);
            s.Write(mNavMode);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mRollAilerons = s.ReadSingle();
            this.mPitchElevator = s.ReadSingle();
            this.mYawRudder = s.ReadSingle();
            this.mThrottle = s.ReadSingle();
            this.mAux1 = s.ReadSingle();
            this.mAux2 = s.ReadSingle();
            this.mAux3 = s.ReadSingle();
            this.mAux4 = s.ReadSingle();
            this.mMode = (MavMode)s.ReadByte();
            this.mNavMode = s.ReadByte();
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
        private MavMode mMode;
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mChan1Raw);
            s.Write(mChan2Raw);
            s.Write(mChan3Raw);
            s.Write(mChan4Raw);
            s.Write(mChan5Raw);
            s.Write(mChan6Raw);
            s.Write(mChan7Raw);
            s.Write(mChan8Raw);
            s.Write(mChan9Raw);
            s.Write(mChan10Raw);
            s.Write(mChan11Raw);
            s.Write(mChan12Raw);
            s.Write(mRssi);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mChan1Raw = s.ReadUInt16();
            this.mChan2Raw = s.ReadUInt16();
            this.mChan3Raw = s.ReadUInt16();
            this.mChan4Raw = s.ReadUInt16();
            this.mChan5Raw = s.ReadUInt16();
            this.mChan6Raw = s.ReadUInt16();
            this.mChan7Raw = s.ReadUInt16();
            this.mChan8Raw = s.ReadUInt16();
            this.mChan9Raw = s.ReadUInt16();
            this.mChan10Raw = s.ReadUInt16();
            this.mChan11Raw = s.ReadUInt16();
            this.mChan12Raw = s.ReadUInt16();
            this.mRssi = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mFlowCompMX);
            s.Write(mFlowCompMY);
            s.Write(mGroundDistance);
            s.Write(mFlowX);
            s.Write(mFlowY);
            s.Write(mSensorId);
            s.Write(mQuality);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mFlowCompMX = s.ReadSingle();
            this.mFlowCompMY = s.ReadSingle();
            this.mGroundDistance = s.ReadSingle();
            this.mFlowX = s.ReadInt16();
            this.mFlowY = s.ReadInt16();
            this.mSensorId = s.ReadByte();
            this.mQuality = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mUsec);
            s.Write(mX);
            s.Write(mY);
            s.Write(mZ);
            s.Write(mRoll);
            s.Write(mPitch);
            s.Write(mYaw);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mUsec = s.ReadUInt64();
            this.mX = s.ReadSingle();
            this.mY = s.ReadSingle();
            this.mZ = s.ReadSingle();
            this.mRoll = s.ReadSingle();
            this.mPitch = s.ReadSingle();
            this.mYaw = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mUsec);
            s.Write(mX);
            s.Write(mY);
            s.Write(mZ);
            s.Write(mRoll);
            s.Write(mPitch);
            s.Write(mYaw);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mUsec = s.ReadUInt64();
            this.mX = s.ReadSingle();
            this.mY = s.ReadSingle();
            this.mZ = s.ReadSingle();
            this.mRoll = s.ReadSingle();
            this.mPitch = s.ReadSingle();
            this.mYaw = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mUsec);
            s.Write(mX);
            s.Write(mY);
            s.Write(mZ);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mUsec = s.ReadUInt64();
            this.mX = s.ReadSingle();
            this.mY = s.ReadSingle();
            this.mZ = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mUsec);
            s.Write(mX);
            s.Write(mY);
            s.Write(mZ);
            s.Write(mRoll);
            s.Write(mPitch);
            s.Write(mYaw);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mUsec = s.ReadUInt64();
            this.mX = s.ReadSingle();
            this.mY = s.ReadSingle();
            this.mZ = s.ReadSingle();
            this.mRoll = s.ReadSingle();
            this.mPitch = s.ReadSingle();
            this.mYaw = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mXacc);
            s.Write(mYacc);
            s.Write(mZacc);
            s.Write(mXgyro);
            s.Write(mYgyro);
            s.Write(mZgyro);
            s.Write(mXmag);
            s.Write(mYmag);
            s.Write(mZmag);
            s.Write(mAbsPressure);
            s.Write(mDiffPressure);
            s.Write(mPressureAlt);
            s.Write(mTemperature);
            s.Write(mFieldsUpdated);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mXacc = s.ReadSingle();
            this.mYacc = s.ReadSingle();
            this.mZacc = s.ReadSingle();
            this.mXgyro = s.ReadSingle();
            this.mYgyro = s.ReadSingle();
            this.mZgyro = s.ReadSingle();
            this.mXmag = s.ReadSingle();
            this.mYmag = s.ReadSingle();
            this.mZmag = s.ReadSingle();
            this.mAbsPressure = s.ReadSingle();
            this.mDiffPressure = s.ReadSingle();
            this.mPressureAlt = s.ReadSingle();
            this.mTemperature = s.ReadSingle();
            this.mFieldsUpdated = s.ReadUInt16();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mFrontDistanceM);
            s.Write(mLeft[0]); 
            s.Write(mLeft[1]); 
            s.Write(mLeft[2]); 
            s.Write(mLeft[3]); 
            s.Write(mLeft[4]); 
            s.Write(mLeft[5]); 
            s.Write(mLeft[6]); 
            s.Write(mLeft[7]); 
            s.Write(mLeft[8]); 
            s.Write(mLeft[9]); 
            s.Write(mRight[0]); 
            s.Write(mRight[1]); 
            s.Write(mRight[2]); 
            s.Write(mRight[3]); 
            s.Write(mRight[4]); 
            s.Write(mRight[5]); 
            s.Write(mRight[6]); 
            s.Write(mRight[7]); 
            s.Write(mRight[8]); 
            s.Write(mRight[9]); 
            s.Write(mSensorId);
            s.Write(mQuality);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mFrontDistanceM = s.ReadSingle();
            this.mLeft[0] = s.ReadInt16();
            this.mLeft[1] = s.ReadInt16();
            this.mLeft[2] = s.ReadInt16();
            this.mLeft[3] = s.ReadInt16();
            this.mLeft[4] = s.ReadInt16();
            this.mLeft[5] = s.ReadInt16();
            this.mLeft[6] = s.ReadInt16();
            this.mLeft[7] = s.ReadInt16();
            this.mLeft[8] = s.ReadInt16();
            this.mLeft[9] = s.ReadInt16();
            this.mRight[0] = s.ReadInt16();
            this.mRight[1] = s.ReadInt16();
            this.mRight[2] = s.ReadInt16();
            this.mRight[3] = s.ReadInt16();
            this.mRight[4] = s.ReadInt16();
            this.mRight[5] = s.ReadInt16();
            this.mRight[6] = s.ReadInt16();
            this.mRight[7] = s.ReadInt16();
            this.mRight[8] = s.ReadInt16();
            this.mRight[9] = s.ReadInt16();
            this.mSensorId = s.ReadByte();
            this.mQuality = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mXacc);
            s.Write(mYacc);
            s.Write(mZacc);
            s.Write(mXgyro);
            s.Write(mYgyro);
            s.Write(mZgyro);
            s.Write(mXmag);
            s.Write(mYmag);
            s.Write(mZmag);
            s.Write(mAbsPressure);
            s.Write(mDiffPressure);
            s.Write(mPressureAlt);
            s.Write(mTemperature);
            s.Write(mFieldsUpdated);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mXacc = s.ReadSingle();
            this.mYacc = s.ReadSingle();
            this.mZacc = s.ReadSingle();
            this.mXgyro = s.ReadSingle();
            this.mYgyro = s.ReadSingle();
            this.mZgyro = s.ReadSingle();
            this.mXmag = s.ReadSingle();
            this.mYmag = s.ReadSingle();
            this.mZmag = s.ReadSingle();
            this.mAbsPressure = s.ReadSingle();
            this.mDiffPressure = s.ReadSingle();
            this.mPressureAlt = s.ReadSingle();
            this.mTemperature = s.ReadSingle();
            this.mFieldsUpdated = s.ReadUInt32();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mQ1);
            s.Write(mQ2);
            s.Write(mQ3);
            s.Write(mQ4);
            s.Write(mRoll);
            s.Write(mPitch);
            s.Write(mYaw);
            s.Write(mXacc);
            s.Write(mYacc);
            s.Write(mZacc);
            s.Write(mXgyro);
            s.Write(mYgyro);
            s.Write(mZgyro);
            s.Write(mLat);
            s.Write(mLon);
            s.Write(mAlt);
            s.Write(mStdDevHorz);
            s.Write(mStdDevVert);
            s.Write(mVn);
            s.Write(mVe);
            s.Write(mVd);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mQ1 = s.ReadSingle();
            this.mQ2 = s.ReadSingle();
            this.mQ3 = s.ReadSingle();
            this.mQ4 = s.ReadSingle();
            this.mRoll = s.ReadSingle();
            this.mPitch = s.ReadSingle();
            this.mYaw = s.ReadSingle();
            this.mXacc = s.ReadSingle();
            this.mYacc = s.ReadSingle();
            this.mZacc = s.ReadSingle();
            this.mXgyro = s.ReadSingle();
            this.mYgyro = s.ReadSingle();
            this.mZgyro = s.ReadSingle();
            this.mLat = s.ReadSingle();
            this.mLon = s.ReadSingle();
            this.mAlt = s.ReadSingle();
            this.mStdDevHorz = s.ReadSingle();
            this.mStdDevVert = s.ReadSingle();
            this.mVn = s.ReadSingle();
            this.mVe = s.ReadSingle();
            this.mVd = s.ReadSingle();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mRxerrors);
            s.Write(mFixed);
            s.Write(mRssi);
            s.Write(mRemrssi);
            s.Write(mTxbuf);
            s.Write(mNoise);
            s.Write(mRemnoise);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mRxerrors = s.ReadUInt16();
            this.mFixed = s.ReadUInt16();
            this.mRssi = s.ReadByte();
            this.mRemrssi = s.ReadByte();
            this.mTxbuf = s.ReadByte();
            this.mNoise = s.ReadByte();
            this.mRemnoise = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTransferUid);
            s.Write(mFileSize);
            s.Write(mDestPath[0]); 
            s.Write(mDestPath[1]); 
            s.Write(mDestPath[2]); 
            s.Write(mDestPath[3]); 
            s.Write(mDestPath[4]); 
            s.Write(mDestPath[5]); 
            s.Write(mDestPath[6]); 
            s.Write(mDestPath[7]); 
            s.Write(mDestPath[8]); 
            s.Write(mDestPath[9]); 
            s.Write(mDestPath[10]); 
            s.Write(mDestPath[11]); 
            s.Write(mDestPath[12]); 
            s.Write(mDestPath[13]); 
            s.Write(mDestPath[14]); 
            s.Write(mDestPath[15]); 
            s.Write(mDestPath[16]); 
            s.Write(mDestPath[17]); 
            s.Write(mDestPath[18]); 
            s.Write(mDestPath[19]); 
            s.Write(mDestPath[20]); 
            s.Write(mDestPath[21]); 
            s.Write(mDestPath[22]); 
            s.Write(mDestPath[23]); 
            s.Write(mDestPath[24]); 
            s.Write(mDestPath[25]); 
            s.Write(mDestPath[26]); 
            s.Write(mDestPath[27]); 
            s.Write(mDestPath[28]); 
            s.Write(mDestPath[29]); 
            s.Write(mDestPath[30]); 
            s.Write(mDestPath[31]); 
            s.Write(mDestPath[32]); 
            s.Write(mDestPath[33]); 
            s.Write(mDestPath[34]); 
            s.Write(mDestPath[35]); 
            s.Write(mDestPath[36]); 
            s.Write(mDestPath[37]); 
            s.Write(mDestPath[38]); 
            s.Write(mDestPath[39]); 
            s.Write(mDestPath[40]); 
            s.Write(mDestPath[41]); 
            s.Write(mDestPath[42]); 
            s.Write(mDestPath[43]); 
            s.Write(mDestPath[44]); 
            s.Write(mDestPath[45]); 
            s.Write(mDestPath[46]); 
            s.Write(mDestPath[47]); 
            s.Write(mDestPath[48]); 
            s.Write(mDestPath[49]); 
            s.Write(mDestPath[50]); 
            s.Write(mDestPath[51]); 
            s.Write(mDestPath[52]); 
            s.Write(mDestPath[53]); 
            s.Write(mDestPath[54]); 
            s.Write(mDestPath[55]); 
            s.Write(mDestPath[56]); 
            s.Write(mDestPath[57]); 
            s.Write(mDestPath[58]); 
            s.Write(mDestPath[59]); 
            s.Write(mDestPath[60]); 
            s.Write(mDestPath[61]); 
            s.Write(mDestPath[62]); 
            s.Write(mDestPath[63]); 
            s.Write(mDestPath[64]); 
            s.Write(mDestPath[65]); 
            s.Write(mDestPath[66]); 
            s.Write(mDestPath[67]); 
            s.Write(mDestPath[68]); 
            s.Write(mDestPath[69]); 
            s.Write(mDestPath[70]); 
            s.Write(mDestPath[71]); 
            s.Write(mDestPath[72]); 
            s.Write(mDestPath[73]); 
            s.Write(mDestPath[74]); 
            s.Write(mDestPath[75]); 
            s.Write(mDestPath[76]); 
            s.Write(mDestPath[77]); 
            s.Write(mDestPath[78]); 
            s.Write(mDestPath[79]); 
            s.Write(mDestPath[80]); 
            s.Write(mDestPath[81]); 
            s.Write(mDestPath[82]); 
            s.Write(mDestPath[83]); 
            s.Write(mDestPath[84]); 
            s.Write(mDestPath[85]); 
            s.Write(mDestPath[86]); 
            s.Write(mDestPath[87]); 
            s.Write(mDestPath[88]); 
            s.Write(mDestPath[89]); 
            s.Write(mDestPath[90]); 
            s.Write(mDestPath[91]); 
            s.Write(mDestPath[92]); 
            s.Write(mDestPath[93]); 
            s.Write(mDestPath[94]); 
            s.Write(mDestPath[95]); 
            s.Write(mDestPath[96]); 
            s.Write(mDestPath[97]); 
            s.Write(mDestPath[98]); 
            s.Write(mDestPath[99]); 
            s.Write(mDestPath[100]); 
            s.Write(mDestPath[101]); 
            s.Write(mDestPath[102]); 
            s.Write(mDestPath[103]); 
            s.Write(mDestPath[104]); 
            s.Write(mDestPath[105]); 
            s.Write(mDestPath[106]); 
            s.Write(mDestPath[107]); 
            s.Write(mDestPath[108]); 
            s.Write(mDestPath[109]); 
            s.Write(mDestPath[110]); 
            s.Write(mDestPath[111]); 
            s.Write(mDestPath[112]); 
            s.Write(mDestPath[113]); 
            s.Write(mDestPath[114]); 
            s.Write(mDestPath[115]); 
            s.Write(mDestPath[116]); 
            s.Write(mDestPath[117]); 
            s.Write(mDestPath[118]); 
            s.Write(mDestPath[119]); 
            s.Write(mDestPath[120]); 
            s.Write(mDestPath[121]); 
            s.Write(mDestPath[122]); 
            s.Write(mDestPath[123]); 
            s.Write(mDestPath[124]); 
            s.Write(mDestPath[125]); 
            s.Write(mDestPath[126]); 
            s.Write(mDestPath[127]); 
            s.Write(mDestPath[128]); 
            s.Write(mDestPath[129]); 
            s.Write(mDestPath[130]); 
            s.Write(mDestPath[131]); 
            s.Write(mDestPath[132]); 
            s.Write(mDestPath[133]); 
            s.Write(mDestPath[134]); 
            s.Write(mDestPath[135]); 
            s.Write(mDestPath[136]); 
            s.Write(mDestPath[137]); 
            s.Write(mDestPath[138]); 
            s.Write(mDestPath[139]); 
            s.Write(mDestPath[140]); 
            s.Write(mDestPath[141]); 
            s.Write(mDestPath[142]); 
            s.Write(mDestPath[143]); 
            s.Write(mDestPath[144]); 
            s.Write(mDestPath[145]); 
            s.Write(mDestPath[146]); 
            s.Write(mDestPath[147]); 
            s.Write(mDestPath[148]); 
            s.Write(mDestPath[149]); 
            s.Write(mDestPath[150]); 
            s.Write(mDestPath[151]); 
            s.Write(mDestPath[152]); 
            s.Write(mDestPath[153]); 
            s.Write(mDestPath[154]); 
            s.Write(mDestPath[155]); 
            s.Write(mDestPath[156]); 
            s.Write(mDestPath[157]); 
            s.Write(mDestPath[158]); 
            s.Write(mDestPath[159]); 
            s.Write(mDestPath[160]); 
            s.Write(mDestPath[161]); 
            s.Write(mDestPath[162]); 
            s.Write(mDestPath[163]); 
            s.Write(mDestPath[164]); 
            s.Write(mDestPath[165]); 
            s.Write(mDestPath[166]); 
            s.Write(mDestPath[167]); 
            s.Write(mDestPath[168]); 
            s.Write(mDestPath[169]); 
            s.Write(mDestPath[170]); 
            s.Write(mDestPath[171]); 
            s.Write(mDestPath[172]); 
            s.Write(mDestPath[173]); 
            s.Write(mDestPath[174]); 
            s.Write(mDestPath[175]); 
            s.Write(mDestPath[176]); 
            s.Write(mDestPath[177]); 
            s.Write(mDestPath[178]); 
            s.Write(mDestPath[179]); 
            s.Write(mDestPath[180]); 
            s.Write(mDestPath[181]); 
            s.Write(mDestPath[182]); 
            s.Write(mDestPath[183]); 
            s.Write(mDestPath[184]); 
            s.Write(mDestPath[185]); 
            s.Write(mDestPath[186]); 
            s.Write(mDestPath[187]); 
            s.Write(mDestPath[188]); 
            s.Write(mDestPath[189]); 
            s.Write(mDestPath[190]); 
            s.Write(mDestPath[191]); 
            s.Write(mDestPath[192]); 
            s.Write(mDestPath[193]); 
            s.Write(mDestPath[194]); 
            s.Write(mDestPath[195]); 
            s.Write(mDestPath[196]); 
            s.Write(mDestPath[197]); 
            s.Write(mDestPath[198]); 
            s.Write(mDestPath[199]); 
            s.Write(mDestPath[200]); 
            s.Write(mDestPath[201]); 
            s.Write(mDestPath[202]); 
            s.Write(mDestPath[203]); 
            s.Write(mDestPath[204]); 
            s.Write(mDestPath[205]); 
            s.Write(mDestPath[206]); 
            s.Write(mDestPath[207]); 
            s.Write(mDestPath[208]); 
            s.Write(mDestPath[209]); 
            s.Write(mDestPath[210]); 
            s.Write(mDestPath[211]); 
            s.Write(mDestPath[212]); 
            s.Write(mDestPath[213]); 
            s.Write(mDestPath[214]); 
            s.Write(mDestPath[215]); 
            s.Write(mDestPath[216]); 
            s.Write(mDestPath[217]); 
            s.Write(mDestPath[218]); 
            s.Write(mDestPath[219]); 
            s.Write(mDestPath[220]); 
            s.Write(mDestPath[221]); 
            s.Write(mDestPath[222]); 
            s.Write(mDestPath[223]); 
            s.Write(mDestPath[224]); 
            s.Write(mDestPath[225]); 
            s.Write(mDestPath[226]); 
            s.Write(mDestPath[227]); 
            s.Write(mDestPath[228]); 
            s.Write(mDestPath[229]); 
            s.Write(mDestPath[230]); 
            s.Write(mDestPath[231]); 
            s.Write(mDestPath[232]); 
            s.Write(mDestPath[233]); 
            s.Write(mDestPath[234]); 
            s.Write(mDestPath[235]); 
            s.Write(mDestPath[236]); 
            s.Write(mDestPath[237]); 
            s.Write(mDestPath[238]); 
            s.Write(mDestPath[239]); 
            s.Write(mDirection);
            s.Write(mFlags);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTransferUid = s.ReadUInt64();
            this.mFileSize = s.ReadUInt32();
            this.mDestPath[0] = s.ReadChar();
            this.mDestPath[1] = s.ReadChar();
            this.mDestPath[2] = s.ReadChar();
            this.mDestPath[3] = s.ReadChar();
            this.mDestPath[4] = s.ReadChar();
            this.mDestPath[5] = s.ReadChar();
            this.mDestPath[6] = s.ReadChar();
            this.mDestPath[7] = s.ReadChar();
            this.mDestPath[8] = s.ReadChar();
            this.mDestPath[9] = s.ReadChar();
            this.mDestPath[10] = s.ReadChar();
            this.mDestPath[11] = s.ReadChar();
            this.mDestPath[12] = s.ReadChar();
            this.mDestPath[13] = s.ReadChar();
            this.mDestPath[14] = s.ReadChar();
            this.mDestPath[15] = s.ReadChar();
            this.mDestPath[16] = s.ReadChar();
            this.mDestPath[17] = s.ReadChar();
            this.mDestPath[18] = s.ReadChar();
            this.mDestPath[19] = s.ReadChar();
            this.mDestPath[20] = s.ReadChar();
            this.mDestPath[21] = s.ReadChar();
            this.mDestPath[22] = s.ReadChar();
            this.mDestPath[23] = s.ReadChar();
            this.mDestPath[24] = s.ReadChar();
            this.mDestPath[25] = s.ReadChar();
            this.mDestPath[26] = s.ReadChar();
            this.mDestPath[27] = s.ReadChar();
            this.mDestPath[28] = s.ReadChar();
            this.mDestPath[29] = s.ReadChar();
            this.mDestPath[30] = s.ReadChar();
            this.mDestPath[31] = s.ReadChar();
            this.mDestPath[32] = s.ReadChar();
            this.mDestPath[33] = s.ReadChar();
            this.mDestPath[34] = s.ReadChar();
            this.mDestPath[35] = s.ReadChar();
            this.mDestPath[36] = s.ReadChar();
            this.mDestPath[37] = s.ReadChar();
            this.mDestPath[38] = s.ReadChar();
            this.mDestPath[39] = s.ReadChar();
            this.mDestPath[40] = s.ReadChar();
            this.mDestPath[41] = s.ReadChar();
            this.mDestPath[42] = s.ReadChar();
            this.mDestPath[43] = s.ReadChar();
            this.mDestPath[44] = s.ReadChar();
            this.mDestPath[45] = s.ReadChar();
            this.mDestPath[46] = s.ReadChar();
            this.mDestPath[47] = s.ReadChar();
            this.mDestPath[48] = s.ReadChar();
            this.mDestPath[49] = s.ReadChar();
            this.mDestPath[50] = s.ReadChar();
            this.mDestPath[51] = s.ReadChar();
            this.mDestPath[52] = s.ReadChar();
            this.mDestPath[53] = s.ReadChar();
            this.mDestPath[54] = s.ReadChar();
            this.mDestPath[55] = s.ReadChar();
            this.mDestPath[56] = s.ReadChar();
            this.mDestPath[57] = s.ReadChar();
            this.mDestPath[58] = s.ReadChar();
            this.mDestPath[59] = s.ReadChar();
            this.mDestPath[60] = s.ReadChar();
            this.mDestPath[61] = s.ReadChar();
            this.mDestPath[62] = s.ReadChar();
            this.mDestPath[63] = s.ReadChar();
            this.mDestPath[64] = s.ReadChar();
            this.mDestPath[65] = s.ReadChar();
            this.mDestPath[66] = s.ReadChar();
            this.mDestPath[67] = s.ReadChar();
            this.mDestPath[68] = s.ReadChar();
            this.mDestPath[69] = s.ReadChar();
            this.mDestPath[70] = s.ReadChar();
            this.mDestPath[71] = s.ReadChar();
            this.mDestPath[72] = s.ReadChar();
            this.mDestPath[73] = s.ReadChar();
            this.mDestPath[74] = s.ReadChar();
            this.mDestPath[75] = s.ReadChar();
            this.mDestPath[76] = s.ReadChar();
            this.mDestPath[77] = s.ReadChar();
            this.mDestPath[78] = s.ReadChar();
            this.mDestPath[79] = s.ReadChar();
            this.mDestPath[80] = s.ReadChar();
            this.mDestPath[81] = s.ReadChar();
            this.mDestPath[82] = s.ReadChar();
            this.mDestPath[83] = s.ReadChar();
            this.mDestPath[84] = s.ReadChar();
            this.mDestPath[85] = s.ReadChar();
            this.mDestPath[86] = s.ReadChar();
            this.mDestPath[87] = s.ReadChar();
            this.mDestPath[88] = s.ReadChar();
            this.mDestPath[89] = s.ReadChar();
            this.mDestPath[90] = s.ReadChar();
            this.mDestPath[91] = s.ReadChar();
            this.mDestPath[92] = s.ReadChar();
            this.mDestPath[93] = s.ReadChar();
            this.mDestPath[94] = s.ReadChar();
            this.mDestPath[95] = s.ReadChar();
            this.mDestPath[96] = s.ReadChar();
            this.mDestPath[97] = s.ReadChar();
            this.mDestPath[98] = s.ReadChar();
            this.mDestPath[99] = s.ReadChar();
            this.mDestPath[100] = s.ReadChar();
            this.mDestPath[101] = s.ReadChar();
            this.mDestPath[102] = s.ReadChar();
            this.mDestPath[103] = s.ReadChar();
            this.mDestPath[104] = s.ReadChar();
            this.mDestPath[105] = s.ReadChar();
            this.mDestPath[106] = s.ReadChar();
            this.mDestPath[107] = s.ReadChar();
            this.mDestPath[108] = s.ReadChar();
            this.mDestPath[109] = s.ReadChar();
            this.mDestPath[110] = s.ReadChar();
            this.mDestPath[111] = s.ReadChar();
            this.mDestPath[112] = s.ReadChar();
            this.mDestPath[113] = s.ReadChar();
            this.mDestPath[114] = s.ReadChar();
            this.mDestPath[115] = s.ReadChar();
            this.mDestPath[116] = s.ReadChar();
            this.mDestPath[117] = s.ReadChar();
            this.mDestPath[118] = s.ReadChar();
            this.mDestPath[119] = s.ReadChar();
            this.mDestPath[120] = s.ReadChar();
            this.mDestPath[121] = s.ReadChar();
            this.mDestPath[122] = s.ReadChar();
            this.mDestPath[123] = s.ReadChar();
            this.mDestPath[124] = s.ReadChar();
            this.mDestPath[125] = s.ReadChar();
            this.mDestPath[126] = s.ReadChar();
            this.mDestPath[127] = s.ReadChar();
            this.mDestPath[128] = s.ReadChar();
            this.mDestPath[129] = s.ReadChar();
            this.mDestPath[130] = s.ReadChar();
            this.mDestPath[131] = s.ReadChar();
            this.mDestPath[132] = s.ReadChar();
            this.mDestPath[133] = s.ReadChar();
            this.mDestPath[134] = s.ReadChar();
            this.mDestPath[135] = s.ReadChar();
            this.mDestPath[136] = s.ReadChar();
            this.mDestPath[137] = s.ReadChar();
            this.mDestPath[138] = s.ReadChar();
            this.mDestPath[139] = s.ReadChar();
            this.mDestPath[140] = s.ReadChar();
            this.mDestPath[141] = s.ReadChar();
            this.mDestPath[142] = s.ReadChar();
            this.mDestPath[143] = s.ReadChar();
            this.mDestPath[144] = s.ReadChar();
            this.mDestPath[145] = s.ReadChar();
            this.mDestPath[146] = s.ReadChar();
            this.mDestPath[147] = s.ReadChar();
            this.mDestPath[148] = s.ReadChar();
            this.mDestPath[149] = s.ReadChar();
            this.mDestPath[150] = s.ReadChar();
            this.mDestPath[151] = s.ReadChar();
            this.mDestPath[152] = s.ReadChar();
            this.mDestPath[153] = s.ReadChar();
            this.mDestPath[154] = s.ReadChar();
            this.mDestPath[155] = s.ReadChar();
            this.mDestPath[156] = s.ReadChar();
            this.mDestPath[157] = s.ReadChar();
            this.mDestPath[158] = s.ReadChar();
            this.mDestPath[159] = s.ReadChar();
            this.mDestPath[160] = s.ReadChar();
            this.mDestPath[161] = s.ReadChar();
            this.mDestPath[162] = s.ReadChar();
            this.mDestPath[163] = s.ReadChar();
            this.mDestPath[164] = s.ReadChar();
            this.mDestPath[165] = s.ReadChar();
            this.mDestPath[166] = s.ReadChar();
            this.mDestPath[167] = s.ReadChar();
            this.mDestPath[168] = s.ReadChar();
            this.mDestPath[169] = s.ReadChar();
            this.mDestPath[170] = s.ReadChar();
            this.mDestPath[171] = s.ReadChar();
            this.mDestPath[172] = s.ReadChar();
            this.mDestPath[173] = s.ReadChar();
            this.mDestPath[174] = s.ReadChar();
            this.mDestPath[175] = s.ReadChar();
            this.mDestPath[176] = s.ReadChar();
            this.mDestPath[177] = s.ReadChar();
            this.mDestPath[178] = s.ReadChar();
            this.mDestPath[179] = s.ReadChar();
            this.mDestPath[180] = s.ReadChar();
            this.mDestPath[181] = s.ReadChar();
            this.mDestPath[182] = s.ReadChar();
            this.mDestPath[183] = s.ReadChar();
            this.mDestPath[184] = s.ReadChar();
            this.mDestPath[185] = s.ReadChar();
            this.mDestPath[186] = s.ReadChar();
            this.mDestPath[187] = s.ReadChar();
            this.mDestPath[188] = s.ReadChar();
            this.mDestPath[189] = s.ReadChar();
            this.mDestPath[190] = s.ReadChar();
            this.mDestPath[191] = s.ReadChar();
            this.mDestPath[192] = s.ReadChar();
            this.mDestPath[193] = s.ReadChar();
            this.mDestPath[194] = s.ReadChar();
            this.mDestPath[195] = s.ReadChar();
            this.mDestPath[196] = s.ReadChar();
            this.mDestPath[197] = s.ReadChar();
            this.mDestPath[198] = s.ReadChar();
            this.mDestPath[199] = s.ReadChar();
            this.mDestPath[200] = s.ReadChar();
            this.mDestPath[201] = s.ReadChar();
            this.mDestPath[202] = s.ReadChar();
            this.mDestPath[203] = s.ReadChar();
            this.mDestPath[204] = s.ReadChar();
            this.mDestPath[205] = s.ReadChar();
            this.mDestPath[206] = s.ReadChar();
            this.mDestPath[207] = s.ReadChar();
            this.mDestPath[208] = s.ReadChar();
            this.mDestPath[209] = s.ReadChar();
            this.mDestPath[210] = s.ReadChar();
            this.mDestPath[211] = s.ReadChar();
            this.mDestPath[212] = s.ReadChar();
            this.mDestPath[213] = s.ReadChar();
            this.mDestPath[214] = s.ReadChar();
            this.mDestPath[215] = s.ReadChar();
            this.mDestPath[216] = s.ReadChar();
            this.mDestPath[217] = s.ReadChar();
            this.mDestPath[218] = s.ReadChar();
            this.mDestPath[219] = s.ReadChar();
            this.mDestPath[220] = s.ReadChar();
            this.mDestPath[221] = s.ReadChar();
            this.mDestPath[222] = s.ReadChar();
            this.mDestPath[223] = s.ReadChar();
            this.mDestPath[224] = s.ReadChar();
            this.mDestPath[225] = s.ReadChar();
            this.mDestPath[226] = s.ReadChar();
            this.mDestPath[227] = s.ReadChar();
            this.mDestPath[228] = s.ReadChar();
            this.mDestPath[229] = s.ReadChar();
            this.mDestPath[230] = s.ReadChar();
            this.mDestPath[231] = s.ReadChar();
            this.mDestPath[232] = s.ReadChar();
            this.mDestPath[233] = s.ReadChar();
            this.mDestPath[234] = s.ReadChar();
            this.mDestPath[235] = s.ReadChar();
            this.mDestPath[236] = s.ReadChar();
            this.mDestPath[237] = s.ReadChar();
            this.mDestPath[238] = s.ReadChar();
            this.mDestPath[239] = s.ReadChar();
            this.mDirection = s.ReadByte();
            this.mFlags = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTransferUid);
            s.Write(mDirPath[0]); 
            s.Write(mDirPath[1]); 
            s.Write(mDirPath[2]); 
            s.Write(mDirPath[3]); 
            s.Write(mDirPath[4]); 
            s.Write(mDirPath[5]); 
            s.Write(mDirPath[6]); 
            s.Write(mDirPath[7]); 
            s.Write(mDirPath[8]); 
            s.Write(mDirPath[9]); 
            s.Write(mDirPath[10]); 
            s.Write(mDirPath[11]); 
            s.Write(mDirPath[12]); 
            s.Write(mDirPath[13]); 
            s.Write(mDirPath[14]); 
            s.Write(mDirPath[15]); 
            s.Write(mDirPath[16]); 
            s.Write(mDirPath[17]); 
            s.Write(mDirPath[18]); 
            s.Write(mDirPath[19]); 
            s.Write(mDirPath[20]); 
            s.Write(mDirPath[21]); 
            s.Write(mDirPath[22]); 
            s.Write(mDirPath[23]); 
            s.Write(mDirPath[24]); 
            s.Write(mDirPath[25]); 
            s.Write(mDirPath[26]); 
            s.Write(mDirPath[27]); 
            s.Write(mDirPath[28]); 
            s.Write(mDirPath[29]); 
            s.Write(mDirPath[30]); 
            s.Write(mDirPath[31]); 
            s.Write(mDirPath[32]); 
            s.Write(mDirPath[33]); 
            s.Write(mDirPath[34]); 
            s.Write(mDirPath[35]); 
            s.Write(mDirPath[36]); 
            s.Write(mDirPath[37]); 
            s.Write(mDirPath[38]); 
            s.Write(mDirPath[39]); 
            s.Write(mDirPath[40]); 
            s.Write(mDirPath[41]); 
            s.Write(mDirPath[42]); 
            s.Write(mDirPath[43]); 
            s.Write(mDirPath[44]); 
            s.Write(mDirPath[45]); 
            s.Write(mDirPath[46]); 
            s.Write(mDirPath[47]); 
            s.Write(mDirPath[48]); 
            s.Write(mDirPath[49]); 
            s.Write(mDirPath[50]); 
            s.Write(mDirPath[51]); 
            s.Write(mDirPath[52]); 
            s.Write(mDirPath[53]); 
            s.Write(mDirPath[54]); 
            s.Write(mDirPath[55]); 
            s.Write(mDirPath[56]); 
            s.Write(mDirPath[57]); 
            s.Write(mDirPath[58]); 
            s.Write(mDirPath[59]); 
            s.Write(mDirPath[60]); 
            s.Write(mDirPath[61]); 
            s.Write(mDirPath[62]); 
            s.Write(mDirPath[63]); 
            s.Write(mDirPath[64]); 
            s.Write(mDirPath[65]); 
            s.Write(mDirPath[66]); 
            s.Write(mDirPath[67]); 
            s.Write(mDirPath[68]); 
            s.Write(mDirPath[69]); 
            s.Write(mDirPath[70]); 
            s.Write(mDirPath[71]); 
            s.Write(mDirPath[72]); 
            s.Write(mDirPath[73]); 
            s.Write(mDirPath[74]); 
            s.Write(mDirPath[75]); 
            s.Write(mDirPath[76]); 
            s.Write(mDirPath[77]); 
            s.Write(mDirPath[78]); 
            s.Write(mDirPath[79]); 
            s.Write(mDirPath[80]); 
            s.Write(mDirPath[81]); 
            s.Write(mDirPath[82]); 
            s.Write(mDirPath[83]); 
            s.Write(mDirPath[84]); 
            s.Write(mDirPath[85]); 
            s.Write(mDirPath[86]); 
            s.Write(mDirPath[87]); 
            s.Write(mDirPath[88]); 
            s.Write(mDirPath[89]); 
            s.Write(mDirPath[90]); 
            s.Write(mDirPath[91]); 
            s.Write(mDirPath[92]); 
            s.Write(mDirPath[93]); 
            s.Write(mDirPath[94]); 
            s.Write(mDirPath[95]); 
            s.Write(mDirPath[96]); 
            s.Write(mDirPath[97]); 
            s.Write(mDirPath[98]); 
            s.Write(mDirPath[99]); 
            s.Write(mDirPath[100]); 
            s.Write(mDirPath[101]); 
            s.Write(mDirPath[102]); 
            s.Write(mDirPath[103]); 
            s.Write(mDirPath[104]); 
            s.Write(mDirPath[105]); 
            s.Write(mDirPath[106]); 
            s.Write(mDirPath[107]); 
            s.Write(mDirPath[108]); 
            s.Write(mDirPath[109]); 
            s.Write(mDirPath[110]); 
            s.Write(mDirPath[111]); 
            s.Write(mDirPath[112]); 
            s.Write(mDirPath[113]); 
            s.Write(mDirPath[114]); 
            s.Write(mDirPath[115]); 
            s.Write(mDirPath[116]); 
            s.Write(mDirPath[117]); 
            s.Write(mDirPath[118]); 
            s.Write(mDirPath[119]); 
            s.Write(mDirPath[120]); 
            s.Write(mDirPath[121]); 
            s.Write(mDirPath[122]); 
            s.Write(mDirPath[123]); 
            s.Write(mDirPath[124]); 
            s.Write(mDirPath[125]); 
            s.Write(mDirPath[126]); 
            s.Write(mDirPath[127]); 
            s.Write(mDirPath[128]); 
            s.Write(mDirPath[129]); 
            s.Write(mDirPath[130]); 
            s.Write(mDirPath[131]); 
            s.Write(mDirPath[132]); 
            s.Write(mDirPath[133]); 
            s.Write(mDirPath[134]); 
            s.Write(mDirPath[135]); 
            s.Write(mDirPath[136]); 
            s.Write(mDirPath[137]); 
            s.Write(mDirPath[138]); 
            s.Write(mDirPath[139]); 
            s.Write(mDirPath[140]); 
            s.Write(mDirPath[141]); 
            s.Write(mDirPath[142]); 
            s.Write(mDirPath[143]); 
            s.Write(mDirPath[144]); 
            s.Write(mDirPath[145]); 
            s.Write(mDirPath[146]); 
            s.Write(mDirPath[147]); 
            s.Write(mDirPath[148]); 
            s.Write(mDirPath[149]); 
            s.Write(mDirPath[150]); 
            s.Write(mDirPath[151]); 
            s.Write(mDirPath[152]); 
            s.Write(mDirPath[153]); 
            s.Write(mDirPath[154]); 
            s.Write(mDirPath[155]); 
            s.Write(mDirPath[156]); 
            s.Write(mDirPath[157]); 
            s.Write(mDirPath[158]); 
            s.Write(mDirPath[159]); 
            s.Write(mDirPath[160]); 
            s.Write(mDirPath[161]); 
            s.Write(mDirPath[162]); 
            s.Write(mDirPath[163]); 
            s.Write(mDirPath[164]); 
            s.Write(mDirPath[165]); 
            s.Write(mDirPath[166]); 
            s.Write(mDirPath[167]); 
            s.Write(mDirPath[168]); 
            s.Write(mDirPath[169]); 
            s.Write(mDirPath[170]); 
            s.Write(mDirPath[171]); 
            s.Write(mDirPath[172]); 
            s.Write(mDirPath[173]); 
            s.Write(mDirPath[174]); 
            s.Write(mDirPath[175]); 
            s.Write(mDirPath[176]); 
            s.Write(mDirPath[177]); 
            s.Write(mDirPath[178]); 
            s.Write(mDirPath[179]); 
            s.Write(mDirPath[180]); 
            s.Write(mDirPath[181]); 
            s.Write(mDirPath[182]); 
            s.Write(mDirPath[183]); 
            s.Write(mDirPath[184]); 
            s.Write(mDirPath[185]); 
            s.Write(mDirPath[186]); 
            s.Write(mDirPath[187]); 
            s.Write(mDirPath[188]); 
            s.Write(mDirPath[189]); 
            s.Write(mDirPath[190]); 
            s.Write(mDirPath[191]); 
            s.Write(mDirPath[192]); 
            s.Write(mDirPath[193]); 
            s.Write(mDirPath[194]); 
            s.Write(mDirPath[195]); 
            s.Write(mDirPath[196]); 
            s.Write(mDirPath[197]); 
            s.Write(mDirPath[198]); 
            s.Write(mDirPath[199]); 
            s.Write(mDirPath[200]); 
            s.Write(mDirPath[201]); 
            s.Write(mDirPath[202]); 
            s.Write(mDirPath[203]); 
            s.Write(mDirPath[204]); 
            s.Write(mDirPath[205]); 
            s.Write(mDirPath[206]); 
            s.Write(mDirPath[207]); 
            s.Write(mDirPath[208]); 
            s.Write(mDirPath[209]); 
            s.Write(mDirPath[210]); 
            s.Write(mDirPath[211]); 
            s.Write(mDirPath[212]); 
            s.Write(mDirPath[213]); 
            s.Write(mDirPath[214]); 
            s.Write(mDirPath[215]); 
            s.Write(mDirPath[216]); 
            s.Write(mDirPath[217]); 
            s.Write(mDirPath[218]); 
            s.Write(mDirPath[219]); 
            s.Write(mDirPath[220]); 
            s.Write(mDirPath[221]); 
            s.Write(mDirPath[222]); 
            s.Write(mDirPath[223]); 
            s.Write(mDirPath[224]); 
            s.Write(mDirPath[225]); 
            s.Write(mDirPath[226]); 
            s.Write(mDirPath[227]); 
            s.Write(mDirPath[228]); 
            s.Write(mDirPath[229]); 
            s.Write(mDirPath[230]); 
            s.Write(mDirPath[231]); 
            s.Write(mDirPath[232]); 
            s.Write(mDirPath[233]); 
            s.Write(mDirPath[234]); 
            s.Write(mDirPath[235]); 
            s.Write(mDirPath[236]); 
            s.Write(mDirPath[237]); 
            s.Write(mDirPath[238]); 
            s.Write(mDirPath[239]); 
            s.Write(mFlags);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTransferUid = s.ReadUInt64();
            this.mDirPath[0] = s.ReadChar();
            this.mDirPath[1] = s.ReadChar();
            this.mDirPath[2] = s.ReadChar();
            this.mDirPath[3] = s.ReadChar();
            this.mDirPath[4] = s.ReadChar();
            this.mDirPath[5] = s.ReadChar();
            this.mDirPath[6] = s.ReadChar();
            this.mDirPath[7] = s.ReadChar();
            this.mDirPath[8] = s.ReadChar();
            this.mDirPath[9] = s.ReadChar();
            this.mDirPath[10] = s.ReadChar();
            this.mDirPath[11] = s.ReadChar();
            this.mDirPath[12] = s.ReadChar();
            this.mDirPath[13] = s.ReadChar();
            this.mDirPath[14] = s.ReadChar();
            this.mDirPath[15] = s.ReadChar();
            this.mDirPath[16] = s.ReadChar();
            this.mDirPath[17] = s.ReadChar();
            this.mDirPath[18] = s.ReadChar();
            this.mDirPath[19] = s.ReadChar();
            this.mDirPath[20] = s.ReadChar();
            this.mDirPath[21] = s.ReadChar();
            this.mDirPath[22] = s.ReadChar();
            this.mDirPath[23] = s.ReadChar();
            this.mDirPath[24] = s.ReadChar();
            this.mDirPath[25] = s.ReadChar();
            this.mDirPath[26] = s.ReadChar();
            this.mDirPath[27] = s.ReadChar();
            this.mDirPath[28] = s.ReadChar();
            this.mDirPath[29] = s.ReadChar();
            this.mDirPath[30] = s.ReadChar();
            this.mDirPath[31] = s.ReadChar();
            this.mDirPath[32] = s.ReadChar();
            this.mDirPath[33] = s.ReadChar();
            this.mDirPath[34] = s.ReadChar();
            this.mDirPath[35] = s.ReadChar();
            this.mDirPath[36] = s.ReadChar();
            this.mDirPath[37] = s.ReadChar();
            this.mDirPath[38] = s.ReadChar();
            this.mDirPath[39] = s.ReadChar();
            this.mDirPath[40] = s.ReadChar();
            this.mDirPath[41] = s.ReadChar();
            this.mDirPath[42] = s.ReadChar();
            this.mDirPath[43] = s.ReadChar();
            this.mDirPath[44] = s.ReadChar();
            this.mDirPath[45] = s.ReadChar();
            this.mDirPath[46] = s.ReadChar();
            this.mDirPath[47] = s.ReadChar();
            this.mDirPath[48] = s.ReadChar();
            this.mDirPath[49] = s.ReadChar();
            this.mDirPath[50] = s.ReadChar();
            this.mDirPath[51] = s.ReadChar();
            this.mDirPath[52] = s.ReadChar();
            this.mDirPath[53] = s.ReadChar();
            this.mDirPath[54] = s.ReadChar();
            this.mDirPath[55] = s.ReadChar();
            this.mDirPath[56] = s.ReadChar();
            this.mDirPath[57] = s.ReadChar();
            this.mDirPath[58] = s.ReadChar();
            this.mDirPath[59] = s.ReadChar();
            this.mDirPath[60] = s.ReadChar();
            this.mDirPath[61] = s.ReadChar();
            this.mDirPath[62] = s.ReadChar();
            this.mDirPath[63] = s.ReadChar();
            this.mDirPath[64] = s.ReadChar();
            this.mDirPath[65] = s.ReadChar();
            this.mDirPath[66] = s.ReadChar();
            this.mDirPath[67] = s.ReadChar();
            this.mDirPath[68] = s.ReadChar();
            this.mDirPath[69] = s.ReadChar();
            this.mDirPath[70] = s.ReadChar();
            this.mDirPath[71] = s.ReadChar();
            this.mDirPath[72] = s.ReadChar();
            this.mDirPath[73] = s.ReadChar();
            this.mDirPath[74] = s.ReadChar();
            this.mDirPath[75] = s.ReadChar();
            this.mDirPath[76] = s.ReadChar();
            this.mDirPath[77] = s.ReadChar();
            this.mDirPath[78] = s.ReadChar();
            this.mDirPath[79] = s.ReadChar();
            this.mDirPath[80] = s.ReadChar();
            this.mDirPath[81] = s.ReadChar();
            this.mDirPath[82] = s.ReadChar();
            this.mDirPath[83] = s.ReadChar();
            this.mDirPath[84] = s.ReadChar();
            this.mDirPath[85] = s.ReadChar();
            this.mDirPath[86] = s.ReadChar();
            this.mDirPath[87] = s.ReadChar();
            this.mDirPath[88] = s.ReadChar();
            this.mDirPath[89] = s.ReadChar();
            this.mDirPath[90] = s.ReadChar();
            this.mDirPath[91] = s.ReadChar();
            this.mDirPath[92] = s.ReadChar();
            this.mDirPath[93] = s.ReadChar();
            this.mDirPath[94] = s.ReadChar();
            this.mDirPath[95] = s.ReadChar();
            this.mDirPath[96] = s.ReadChar();
            this.mDirPath[97] = s.ReadChar();
            this.mDirPath[98] = s.ReadChar();
            this.mDirPath[99] = s.ReadChar();
            this.mDirPath[100] = s.ReadChar();
            this.mDirPath[101] = s.ReadChar();
            this.mDirPath[102] = s.ReadChar();
            this.mDirPath[103] = s.ReadChar();
            this.mDirPath[104] = s.ReadChar();
            this.mDirPath[105] = s.ReadChar();
            this.mDirPath[106] = s.ReadChar();
            this.mDirPath[107] = s.ReadChar();
            this.mDirPath[108] = s.ReadChar();
            this.mDirPath[109] = s.ReadChar();
            this.mDirPath[110] = s.ReadChar();
            this.mDirPath[111] = s.ReadChar();
            this.mDirPath[112] = s.ReadChar();
            this.mDirPath[113] = s.ReadChar();
            this.mDirPath[114] = s.ReadChar();
            this.mDirPath[115] = s.ReadChar();
            this.mDirPath[116] = s.ReadChar();
            this.mDirPath[117] = s.ReadChar();
            this.mDirPath[118] = s.ReadChar();
            this.mDirPath[119] = s.ReadChar();
            this.mDirPath[120] = s.ReadChar();
            this.mDirPath[121] = s.ReadChar();
            this.mDirPath[122] = s.ReadChar();
            this.mDirPath[123] = s.ReadChar();
            this.mDirPath[124] = s.ReadChar();
            this.mDirPath[125] = s.ReadChar();
            this.mDirPath[126] = s.ReadChar();
            this.mDirPath[127] = s.ReadChar();
            this.mDirPath[128] = s.ReadChar();
            this.mDirPath[129] = s.ReadChar();
            this.mDirPath[130] = s.ReadChar();
            this.mDirPath[131] = s.ReadChar();
            this.mDirPath[132] = s.ReadChar();
            this.mDirPath[133] = s.ReadChar();
            this.mDirPath[134] = s.ReadChar();
            this.mDirPath[135] = s.ReadChar();
            this.mDirPath[136] = s.ReadChar();
            this.mDirPath[137] = s.ReadChar();
            this.mDirPath[138] = s.ReadChar();
            this.mDirPath[139] = s.ReadChar();
            this.mDirPath[140] = s.ReadChar();
            this.mDirPath[141] = s.ReadChar();
            this.mDirPath[142] = s.ReadChar();
            this.mDirPath[143] = s.ReadChar();
            this.mDirPath[144] = s.ReadChar();
            this.mDirPath[145] = s.ReadChar();
            this.mDirPath[146] = s.ReadChar();
            this.mDirPath[147] = s.ReadChar();
            this.mDirPath[148] = s.ReadChar();
            this.mDirPath[149] = s.ReadChar();
            this.mDirPath[150] = s.ReadChar();
            this.mDirPath[151] = s.ReadChar();
            this.mDirPath[152] = s.ReadChar();
            this.mDirPath[153] = s.ReadChar();
            this.mDirPath[154] = s.ReadChar();
            this.mDirPath[155] = s.ReadChar();
            this.mDirPath[156] = s.ReadChar();
            this.mDirPath[157] = s.ReadChar();
            this.mDirPath[158] = s.ReadChar();
            this.mDirPath[159] = s.ReadChar();
            this.mDirPath[160] = s.ReadChar();
            this.mDirPath[161] = s.ReadChar();
            this.mDirPath[162] = s.ReadChar();
            this.mDirPath[163] = s.ReadChar();
            this.mDirPath[164] = s.ReadChar();
            this.mDirPath[165] = s.ReadChar();
            this.mDirPath[166] = s.ReadChar();
            this.mDirPath[167] = s.ReadChar();
            this.mDirPath[168] = s.ReadChar();
            this.mDirPath[169] = s.ReadChar();
            this.mDirPath[170] = s.ReadChar();
            this.mDirPath[171] = s.ReadChar();
            this.mDirPath[172] = s.ReadChar();
            this.mDirPath[173] = s.ReadChar();
            this.mDirPath[174] = s.ReadChar();
            this.mDirPath[175] = s.ReadChar();
            this.mDirPath[176] = s.ReadChar();
            this.mDirPath[177] = s.ReadChar();
            this.mDirPath[178] = s.ReadChar();
            this.mDirPath[179] = s.ReadChar();
            this.mDirPath[180] = s.ReadChar();
            this.mDirPath[181] = s.ReadChar();
            this.mDirPath[182] = s.ReadChar();
            this.mDirPath[183] = s.ReadChar();
            this.mDirPath[184] = s.ReadChar();
            this.mDirPath[185] = s.ReadChar();
            this.mDirPath[186] = s.ReadChar();
            this.mDirPath[187] = s.ReadChar();
            this.mDirPath[188] = s.ReadChar();
            this.mDirPath[189] = s.ReadChar();
            this.mDirPath[190] = s.ReadChar();
            this.mDirPath[191] = s.ReadChar();
            this.mDirPath[192] = s.ReadChar();
            this.mDirPath[193] = s.ReadChar();
            this.mDirPath[194] = s.ReadChar();
            this.mDirPath[195] = s.ReadChar();
            this.mDirPath[196] = s.ReadChar();
            this.mDirPath[197] = s.ReadChar();
            this.mDirPath[198] = s.ReadChar();
            this.mDirPath[199] = s.ReadChar();
            this.mDirPath[200] = s.ReadChar();
            this.mDirPath[201] = s.ReadChar();
            this.mDirPath[202] = s.ReadChar();
            this.mDirPath[203] = s.ReadChar();
            this.mDirPath[204] = s.ReadChar();
            this.mDirPath[205] = s.ReadChar();
            this.mDirPath[206] = s.ReadChar();
            this.mDirPath[207] = s.ReadChar();
            this.mDirPath[208] = s.ReadChar();
            this.mDirPath[209] = s.ReadChar();
            this.mDirPath[210] = s.ReadChar();
            this.mDirPath[211] = s.ReadChar();
            this.mDirPath[212] = s.ReadChar();
            this.mDirPath[213] = s.ReadChar();
            this.mDirPath[214] = s.ReadChar();
            this.mDirPath[215] = s.ReadChar();
            this.mDirPath[216] = s.ReadChar();
            this.mDirPath[217] = s.ReadChar();
            this.mDirPath[218] = s.ReadChar();
            this.mDirPath[219] = s.ReadChar();
            this.mDirPath[220] = s.ReadChar();
            this.mDirPath[221] = s.ReadChar();
            this.mDirPath[222] = s.ReadChar();
            this.mDirPath[223] = s.ReadChar();
            this.mDirPath[224] = s.ReadChar();
            this.mDirPath[225] = s.ReadChar();
            this.mDirPath[226] = s.ReadChar();
            this.mDirPath[227] = s.ReadChar();
            this.mDirPath[228] = s.ReadChar();
            this.mDirPath[229] = s.ReadChar();
            this.mDirPath[230] = s.ReadChar();
            this.mDirPath[231] = s.ReadChar();
            this.mDirPath[232] = s.ReadChar();
            this.mDirPath[233] = s.ReadChar();
            this.mDirPath[234] = s.ReadChar();
            this.mDirPath[235] = s.ReadChar();
            this.mDirPath[236] = s.ReadChar();
            this.mDirPath[237] = s.ReadChar();
            this.mDirPath[238] = s.ReadChar();
            this.mDirPath[239] = s.ReadChar();
            this.mFlags = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTransferUid);
            s.Write(mResult);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTransferUid = s.ReadUInt64();
            this.mResult = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mLat);
            s.Write(mLon);
            s.Write(mAlt);
            s.Write(mEph);
            s.Write(mEpv);
            s.Write(mVel);
            s.Write(mVn);
            s.Write(mVe);
            s.Write(mVd);
            s.Write(mCog);
            s.Write(mFixType);
            s.Write(mSatellitesVisible);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mLat = s.ReadInt32();
            this.mLon = s.ReadInt32();
            this.mAlt = s.ReadInt32();
            this.mEph = s.ReadUInt16();
            this.mEpv = s.ReadUInt16();
            this.mVel = s.ReadUInt16();
            this.mVn = s.ReadInt16();
            this.mVe = s.ReadInt16();
            this.mVd = s.ReadInt16();
            this.mCog = s.ReadUInt16();
            this.mFixType = s.ReadByte();
            this.mSatellitesVisible = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mFlowCompMX);
            s.Write(mFlowCompMY);
            s.Write(mGroundDistance);
            s.Write(mFlowX);
            s.Write(mFlowY);
            s.Write(mSensorId);
            s.Write(mQuality);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mFlowCompMX = s.ReadSingle();
            this.mFlowCompMY = s.ReadSingle();
            this.mGroundDistance = s.ReadSingle();
            this.mFlowX = s.ReadInt16();
            this.mFlowY = s.ReadInt16();
            this.mSensorId = s.ReadByte();
            this.mQuality = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mAttitudeQuaternion[0]); 
            s.Write(mAttitudeQuaternion[1]); 
            s.Write(mAttitudeQuaternion[2]); 
            s.Write(mAttitudeQuaternion[3]); 
            s.Write(mRollspeed);
            s.Write(mPitchspeed);
            s.Write(mYawspeed);
            s.Write(mLat);
            s.Write(mLon);
            s.Write(mAlt);
            s.Write(mVx);
            s.Write(mVy);
            s.Write(mVz);
            s.Write(mIndAirspeed);
            s.Write(mTrueAirspeed);
            s.Write(mXacc);
            s.Write(mYacc);
            s.Write(mZacc);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mAttitudeQuaternion[0] = s.ReadSingle();
            this.mAttitudeQuaternion[1] = s.ReadSingle();
            this.mAttitudeQuaternion[2] = s.ReadSingle();
            this.mAttitudeQuaternion[3] = s.ReadSingle();
            this.mRollspeed = s.ReadSingle();
            this.mPitchspeed = s.ReadSingle();
            this.mYawspeed = s.ReadSingle();
            this.mLat = s.ReadInt32();
            this.mLon = s.ReadInt32();
            this.mAlt = s.ReadInt32();
            this.mVx = s.ReadInt16();
            this.mVy = s.ReadInt16();
            this.mVz = s.ReadInt16();
            this.mIndAirspeed = s.ReadUInt16();
            this.mTrueAirspeed = s.ReadUInt16();
            this.mXacc = s.ReadInt16();
            this.mYacc = s.ReadInt16();
            this.mZacc = s.ReadInt16();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mCurrentConsumed);
            s.Write(mEnergyConsumed);
            s.Write(mVoltageCell1);
            s.Write(mVoltageCell2);
            s.Write(mVoltageCell3);
            s.Write(mVoltageCell4);
            s.Write(mVoltageCell5);
            s.Write(mVoltageCell6);
            s.Write(mCurrentBattery);
            s.Write(mAccuId);
            s.Write(mBatteryRemaining);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mCurrentConsumed = s.ReadInt32();
            this.mEnergyConsumed = s.ReadInt32();
            this.mVoltageCell1 = s.ReadUInt16();
            this.mVoltageCell2 = s.ReadUInt16();
            this.mVoltageCell3 = s.ReadUInt16();
            this.mVoltageCell4 = s.ReadUInt16();
            this.mVoltageCell5 = s.ReadUInt16();
            this.mVoltageCell6 = s.ReadUInt16();
            this.mCurrentBattery = s.ReadInt16();
            this.mAccuId = s.ReadByte();
            this.mBatteryRemaining = s.ReadSByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mVal1);
            s.Write(mVal2);
            s.Write(mVal3);
            s.Write(mVal4);
            s.Write(mVal5);
            s.Write(mVal6);
            s.Write(mVal7);
            s.Write(mVal8);
            s.Write(mTargetSystem);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mVal1 = s.ReadSingle();
            this.mVal2 = s.ReadSingle();
            this.mVal3 = s.ReadSingle();
            this.mVal4 = s.ReadSingle();
            this.mVal5 = s.ReadSingle();
            this.mVal6 = s.ReadSingle();
            this.mVal7 = s.ReadSingle();
            this.mVal8 = s.ReadSingle();
            this.mTargetSystem = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTransX);
            s.Write(mTransY);
            s.Write(mTransZ);
            s.Write(mRotX);
            s.Write(mRotY);
            s.Write(mRotZ);
            s.Write(mTargetSystem);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTransX = s.ReadSingle();
            this.mTransY = s.ReadSingle();
            this.mTransZ = s.ReadSingle();
            this.mRotX = s.ReadSingle();
            this.mRotY = s.ReadSingle();
            this.mRotZ = s.ReadSingle();
            this.mTargetSystem = s.ReadByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mAddress);
            s.Write(mVer);
            s.Write(mType);
            s.Write(mValue[0]); 
            s.Write(mValue[1]); 
            s.Write(mValue[2]); 
            s.Write(mValue[3]); 
            s.Write(mValue[4]); 
            s.Write(mValue[5]); 
            s.Write(mValue[6]); 
            s.Write(mValue[7]); 
            s.Write(mValue[8]); 
            s.Write(mValue[9]); 
            s.Write(mValue[10]); 
            s.Write(mValue[11]); 
            s.Write(mValue[12]); 
            s.Write(mValue[13]); 
            s.Write(mValue[14]); 
            s.Write(mValue[15]); 
            s.Write(mValue[16]); 
            s.Write(mValue[17]); 
            s.Write(mValue[18]); 
            s.Write(mValue[19]); 
            s.Write(mValue[20]); 
            s.Write(mValue[21]); 
            s.Write(mValue[22]); 
            s.Write(mValue[23]); 
            s.Write(mValue[24]); 
            s.Write(mValue[25]); 
            s.Write(mValue[26]); 
            s.Write(mValue[27]); 
            s.Write(mValue[28]); 
            s.Write(mValue[29]); 
            s.Write(mValue[30]); 
            s.Write(mValue[31]); 
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mAddress = s.ReadUInt16();
            this.mVer = s.ReadByte();
            this.mType = s.ReadByte();
            this.mValue[0] = s.ReadSByte();
            this.mValue[1] = s.ReadSByte();
            this.mValue[2] = s.ReadSByte();
            this.mValue[3] = s.ReadSByte();
            this.mValue[4] = s.ReadSByte();
            this.mValue[5] = s.ReadSByte();
            this.mValue[6] = s.ReadSByte();
            this.mValue[7] = s.ReadSByte();
            this.mValue[8] = s.ReadSByte();
            this.mValue[9] = s.ReadSByte();
            this.mValue[10] = s.ReadSByte();
            this.mValue[11] = s.ReadSByte();
            this.mValue[12] = s.ReadSByte();
            this.mValue[13] = s.ReadSByte();
            this.mValue[14] = s.ReadSByte();
            this.mValue[15] = s.ReadSByte();
            this.mValue[16] = s.ReadSByte();
            this.mValue[17] = s.ReadSByte();
            this.mValue[18] = s.ReadSByte();
            this.mValue[19] = s.ReadSByte();
            this.mValue[20] = s.ReadSByte();
            this.mValue[21] = s.ReadSByte();
            this.mValue[22] = s.ReadSByte();
            this.mValue[23] = s.ReadSByte();
            this.mValue[24] = s.ReadSByte();
            this.mValue[25] = s.ReadSByte();
            this.mValue[26] = s.ReadSByte();
            this.mValue[27] = s.ReadSByte();
            this.mValue[28] = s.ReadSByte();
            this.mValue[29] = s.ReadSByte();
            this.mValue[30] = s.ReadSByte();
            this.mValue[31] = s.ReadSByte();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeUsec);
            s.Write(mX);
            s.Write(mY);
            s.Write(mZ);
            s.Write(mName[0]); 
            s.Write(mName[1]); 
            s.Write(mName[2]); 
            s.Write(mName[3]); 
            s.Write(mName[4]); 
            s.Write(mName[5]); 
            s.Write(mName[6]); 
            s.Write(mName[7]); 
            s.Write(mName[8]); 
            s.Write(mName[9]); 
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeUsec = s.ReadUInt64();
            this.mX = s.ReadSingle();
            this.mY = s.ReadSingle();
            this.mZ = s.ReadSingle();
            this.mName[0] = s.ReadChar();
            this.mName[1] = s.ReadChar();
            this.mName[2] = s.ReadChar();
            this.mName[3] = s.ReadChar();
            this.mName[4] = s.ReadChar();
            this.mName[5] = s.ReadChar();
            this.mName[6] = s.ReadChar();
            this.mName[7] = s.ReadChar();
            this.mName[8] = s.ReadChar();
            this.mName[9] = s.ReadChar();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mValue);
            s.Write(mName[0]); 
            s.Write(mName[1]); 
            s.Write(mName[2]); 
            s.Write(mName[3]); 
            s.Write(mName[4]); 
            s.Write(mName[5]); 
            s.Write(mName[6]); 
            s.Write(mName[7]); 
            s.Write(mName[8]); 
            s.Write(mName[9]); 
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mValue = s.ReadSingle();
            this.mName[0] = s.ReadChar();
            this.mName[1] = s.ReadChar();
            this.mName[2] = s.ReadChar();
            this.mName[3] = s.ReadChar();
            this.mName[4] = s.ReadChar();
            this.mName[5] = s.ReadChar();
            this.mName[6] = s.ReadChar();
            this.mName[7] = s.ReadChar();
            this.mName[8] = s.ReadChar();
            this.mName[9] = s.ReadChar();
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mValue);
            s.Write(mName[0]); 
            s.Write(mName[1]); 
            s.Write(mName[2]); 
            s.Write(mName[3]); 
            s.Write(mName[4]); 
            s.Write(mName[5]); 
            s.Write(mName[6]); 
            s.Write(mName[7]); 
            s.Write(mName[8]); 
            s.Write(mName[9]); 
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mValue = s.ReadInt32();
            this.mName[0] = s.ReadChar();
            this.mName[1] = s.ReadChar();
            this.mName[2] = s.ReadChar();
            this.mName[3] = s.ReadChar();
            this.mName[4] = s.ReadChar();
            this.mName[5] = s.ReadChar();
            this.mName[6] = s.ReadChar();
            this.mName[7] = s.ReadChar();
            this.mName[8] = s.ReadChar();
            this.mName[9] = s.ReadChar();
        }

        private UInt32 mTimeBootMs;
        private Int32 mValue;
        private char[] mName = new char[10];
    }


    // ___________________________________________________________________________________


    public class UasStatustext: UasMessage
    {
        public MavSeverity Severity {
            get { return mSeverity; }
            set { mSeverity = value; NotifyUpdated(); }
        }

        public char[] Text {
            get { return mText; }
            set { mText = value; NotifyUpdated(); }
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write((byte)mSeverity);
            s.Write(mText[0]); 
            s.Write(mText[1]); 
            s.Write(mText[2]); 
            s.Write(mText[3]); 
            s.Write(mText[4]); 
            s.Write(mText[5]); 
            s.Write(mText[6]); 
            s.Write(mText[7]); 
            s.Write(mText[8]); 
            s.Write(mText[9]); 
            s.Write(mText[10]); 
            s.Write(mText[11]); 
            s.Write(mText[12]); 
            s.Write(mText[13]); 
            s.Write(mText[14]); 
            s.Write(mText[15]); 
            s.Write(mText[16]); 
            s.Write(mText[17]); 
            s.Write(mText[18]); 
            s.Write(mText[19]); 
            s.Write(mText[20]); 
            s.Write(mText[21]); 
            s.Write(mText[22]); 
            s.Write(mText[23]); 
            s.Write(mText[24]); 
            s.Write(mText[25]); 
            s.Write(mText[26]); 
            s.Write(mText[27]); 
            s.Write(mText[28]); 
            s.Write(mText[29]); 
            s.Write(mText[30]); 
            s.Write(mText[31]); 
            s.Write(mText[32]); 
            s.Write(mText[33]); 
            s.Write(mText[34]); 
            s.Write(mText[35]); 
            s.Write(mText[36]); 
            s.Write(mText[37]); 
            s.Write(mText[38]); 
            s.Write(mText[39]); 
            s.Write(mText[40]); 
            s.Write(mText[41]); 
            s.Write(mText[42]); 
            s.Write(mText[43]); 
            s.Write(mText[44]); 
            s.Write(mText[45]); 
            s.Write(mText[46]); 
            s.Write(mText[47]); 
            s.Write(mText[48]); 
            s.Write(mText[49]); 
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mSeverity = (MavSeverity)s.ReadByte();
            this.mText[0] = s.ReadChar();
            this.mText[1] = s.ReadChar();
            this.mText[2] = s.ReadChar();
            this.mText[3] = s.ReadChar();
            this.mText[4] = s.ReadChar();
            this.mText[5] = s.ReadChar();
            this.mText[6] = s.ReadChar();
            this.mText[7] = s.ReadChar();
            this.mText[8] = s.ReadChar();
            this.mText[9] = s.ReadChar();
            this.mText[10] = s.ReadChar();
            this.mText[11] = s.ReadChar();
            this.mText[12] = s.ReadChar();
            this.mText[13] = s.ReadChar();
            this.mText[14] = s.ReadChar();
            this.mText[15] = s.ReadChar();
            this.mText[16] = s.ReadChar();
            this.mText[17] = s.ReadChar();
            this.mText[18] = s.ReadChar();
            this.mText[19] = s.ReadChar();
            this.mText[20] = s.ReadChar();
            this.mText[21] = s.ReadChar();
            this.mText[22] = s.ReadChar();
            this.mText[23] = s.ReadChar();
            this.mText[24] = s.ReadChar();
            this.mText[25] = s.ReadChar();
            this.mText[26] = s.ReadChar();
            this.mText[27] = s.ReadChar();
            this.mText[28] = s.ReadChar();
            this.mText[29] = s.ReadChar();
            this.mText[30] = s.ReadChar();
            this.mText[31] = s.ReadChar();
            this.mText[32] = s.ReadChar();
            this.mText[33] = s.ReadChar();
            this.mText[34] = s.ReadChar();
            this.mText[35] = s.ReadChar();
            this.mText[36] = s.ReadChar();
            this.mText[37] = s.ReadChar();
            this.mText[38] = s.ReadChar();
            this.mText[39] = s.ReadChar();
            this.mText[40] = s.ReadChar();
            this.mText[41] = s.ReadChar();
            this.mText[42] = s.ReadChar();
            this.mText[43] = s.ReadChar();
            this.mText[44] = s.ReadChar();
            this.mText[45] = s.ReadChar();
            this.mText[46] = s.ReadChar();
            this.mText[47] = s.ReadChar();
            this.mText[48] = s.ReadChar();
            this.mText[49] = s.ReadChar();
        }

        private MavSeverity mSeverity;
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

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mTimeBootMs);
            s.Write(mInd);
            s.Write(mValue);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mTimeBootMs = s.ReadUInt32();
            this.mInd = s.ReadByte();
            this.mValue = s.ReadSingle();
        }

        private UInt32 mTimeBootMs;
        private byte mInd;
        private float mValue;
    }

}
