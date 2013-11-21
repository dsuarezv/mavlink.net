using System;
using System.IO;

namespace MavLinkNet
{

    /// <summary>
    /// Micro air vehicle / autopilot classes. This identifies the individual model.
    /// </summary>
    public enum MavAutopilot { 

        /// <summary> Generic autopilot, full support for everything </summary>
        Generic = 0, 

        /// <summary> PIXHAWK autopilot, http://pixhawk.ethz.ch </summary>
        Pixhawk = 1, 

        /// <summary> SLUGS autopilot, http://slugsuav.soe.ucsc.edu </summary>
        Slugs = 2, 

        /// <summary> ArduPilotMega / ArduCopter, http://diydrones.com </summary>
        Ardupilotmega = 3, 

        /// <summary> OpenPilot, http://openpilot.org </summary>
        Openpilot = 4, 

        /// <summary> Generic autopilot only supporting simple waypoints </summary>
        GenericWaypointsOnly = 5, 

        /// <summary> Generic autopilot supporting waypoints and other simple navigation commands </summary>
        GenericWaypointsAndSimpleNavigationOnly = 6, 

        /// <summary> Generic autopilot supporting the full mission command set </summary>
        GenericMissionFull = 7, 

        /// <summary> No valid autopilot, e.g. a GCS or other MAVLink component </summary>
        Invalid = 8, 

        /// <summary> PPZ UAV - http://nongnu.org/paparazzi </summary>
        Ppz = 9, 

        /// <summary> UAV Dev Board </summary>
        Udb = 10, 

        /// <summary> FlexiPilot </summary>
        Fp = 11, 

        /// <summary> PX4 Autopilot - http://pixhawk.ethz.ch/px4/ </summary>
        Px4 = 12, 

        /// <summary> SMACCMPilot - http://smaccmpilot.org </summary>
        Smaccmpilot = 13, 

        /// <summary> AutoQuad -- http://autoquad.org </summary>
        Autoquad = 14, 

        /// <summary> Armazila -- http://armazila.com </summary>
        Armazila = 15, 

        /// <summary> Aerob -- http://aerob.ru </summary>
        Aerob = 16 };

    public enum MavType { 

        /// <summary> Generic micro air vehicle. </summary>
        Generic = 0, 

        /// <summary> Fixed wing aircraft. </summary>
        FixedWing = 1, 

        /// <summary> Quadrotor </summary>
        Quadrotor = 2, 

        /// <summary> Coaxial helicopter </summary>
        Coaxial = 3, 

        /// <summary> Normal helicopter with tail rotor. </summary>
        Helicopter = 4, 

        /// <summary> Ground installation </summary>
        AntennaTracker = 5, 

        /// <summary> Operator control unit / ground control station </summary>
        Gcs = 6, 

        /// <summary> Airship, controlled </summary>
        Airship = 7, 

        /// <summary> Free balloon, uncontrolled </summary>
        FreeBalloon = 8, 

        /// <summary> Rocket </summary>
        Rocket = 9, 

        /// <summary> Ground rover </summary>
        GroundRover = 10, 

        /// <summary> Surface vessel, boat, ship </summary>
        SurfaceBoat = 11, 

        /// <summary> Submarine </summary>
        Submarine = 12, 

        /// <summary> Hexarotor </summary>
        Hexarotor = 13, 

        /// <summary> Octorotor </summary>
        Octorotor = 14, 

        /// <summary> Octorotor </summary>
        Tricopter = 15, 

        /// <summary> Flapping wing </summary>
        FlappingWing = 16, 

        /// <summary> Flapping wing </summary>
        Kite = 17 };

    /// <summary>
    /// These flags encode the MAV mode.
    /// </summary>
    public enum MavModeFlag { 

        /// <summary> 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. </summary>
        SafetyArmed = 128, 

        /// <summary> 0b01000000 remote control input is enabled. </summary>
        ManualInputEnabled = 64, 

        /// <summary> 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. </summary>
        HilEnabled = 32, 

        /// <summary> 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. </summary>
        StabilizeEnabled = 16, 

        /// <summary> 0b00001000 guided mode enabled, system flies MISSIONs / mission items. </summary>
        GuidedEnabled = 8, 

        /// <summary> 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. </summary>
        AutoEnabled = 4, 

        /// <summary> 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. </summary>
        TestEnabled = 2, 

        /// <summary> 0b00000001 Reserved for future use. </summary>
        CustomModeEnabled = 1 };

    /// <summary>
    /// These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
    /// </summary>
    public enum MavModeFlagDecodePosition { 

        /// <summary> First bit:  10000000 </summary>
        Safety = 128, 

        /// <summary> Second bit: 01000000 </summary>
        Manual = 64, 

        /// <summary> Third bit:  00100000 </summary>
        Hil = 32, 

        /// <summary> Fourth bit: 00010000 </summary>
        Stabilize = 16, 

        /// <summary> Fifth bit:  00001000 </summary>
        Guided = 8, 

        /// <summary> Sixt bit:   00000100 </summary>
        Auto = 4, 

        /// <summary> Seventh bit: 00000010 </summary>
        Test = 2, 

        /// <summary> Eighth bit: 00000001 </summary>
        CustomMode = 1 };

    /// <summary>
    /// Override command, pauses current mission execution and moves immediately to a position
    /// </summary>
    public enum MavGoto { 

        /// <summary> Hold at the current position. </summary>
        DoHold = 0, 

        /// <summary> Continue with the next item in mission execution. </summary>
        DoContinue = 1, 

        /// <summary> Hold at the current position of the system </summary>
        HoldAtCurrentPosition = 2, 

        /// <summary> Hold at the position specified in the parameters of the DO_HOLD action </summary>
        HoldAtSpecifiedPosition = 3 };

    /// <summary>
    /// These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it                 simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
    /// </summary>
    public enum MavMode { 

        /// <summary> System is not ready to fly, booting, calibrating, etc. No flag is set. </summary>
        Preflight = 0, 

        /// <summary> System is allowed to be active, under assisted RC control. </summary>
        StabilizeDisarmed = 80, 

        /// <summary> System is allowed to be active, under assisted RC control. </summary>
        StabilizeArmed = 208, 

        /// <summary> System is allowed to be active, under manual (RC) control, no stabilization </summary>
        ManualDisarmed = 64, 

        /// <summary> System is allowed to be active, under manual (RC) control, no stabilization </summary>
        ManualArmed = 192, 

        /// <summary> System is allowed to be active, under autonomous control, manual setpoint </summary>
        GuidedDisarmed = 88, 

        /// <summary> System is allowed to be active, under autonomous control, manual setpoint </summary>
        GuidedArmed = 216, 

        /// <summary> System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) </summary>
        AutoDisarmed = 92, 

        /// <summary> System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) </summary>
        AutoArmed = 220, 

        /// <summary> UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. </summary>
        TestDisarmed = 66, 

        /// <summary> UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. </summary>
        TestArmed = 194 };

    public enum MavState { 

        /// <summary> Uninitialized system, state is unknown. </summary>
        Uninit = 0, 

        /// <summary> System is booting up. </summary>
        Boot = 0, 

        /// <summary> System is calibrating and not flight-ready. </summary>
        Calibrating = 0, 

        /// <summary> System is grounded and on standby. It can be launched any time. </summary>
        Standby = 0, 

        /// <summary> System is active and might be already airborne. Motors are engaged. </summary>
        Active = 0, 

        /// <summary> System is in a non-normal flight mode. It can however still navigate. </summary>
        Critical = 0, 

        /// <summary> System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. </summary>
        Emergency = 0, 

        /// <summary> System just initialized its power-down sequence, will shut down now. </summary>
        Poweroff = 0 };

    public enum MavComponent { 

        /// <summary>  </summary>
        MavCompIdAll = 0, 

        /// <summary>  </summary>
        MavCompIdGps = 220, 

        /// <summary>  </summary>
        MavCompIdMissionplanner = 190, 

        /// <summary>  </summary>
        MavCompIdPathplanner = 195, 

        /// <summary>  </summary>
        MavCompIdMapper = 180, 

        /// <summary>  </summary>
        MavCompIdCamera = 100, 

        /// <summary>  </summary>
        MavCompIdImu = 200, 

        /// <summary>  </summary>
        MavCompIdImu2 = 201, 

        /// <summary>  </summary>
        MavCompIdImu3 = 202, 

        /// <summary>  </summary>
        MavCompIdUdpBridge = 240, 

        /// <summary>  </summary>
        MavCompIdUartBridge = 241, 

        /// <summary>  </summary>
        MavCompIdSystemControl = 250, 

        /// <summary>  </summary>
        MavCompIdServo1 = 140, 

        /// <summary>  </summary>
        MavCompIdServo2 = 141, 

        /// <summary>  </summary>
        MavCompIdServo3 = 142, 

        /// <summary>  </summary>
        MavCompIdServo4 = 143, 

        /// <summary>  </summary>
        MavCompIdServo5 = 144, 

        /// <summary>  </summary>
        MavCompIdServo6 = 145, 

        /// <summary>  </summary>
        MavCompIdServo7 = 146, 

        /// <summary>  </summary>
        MavCompIdServo8 = 147, 

        /// <summary>  </summary>
        MavCompIdServo9 = 148, 

        /// <summary>  </summary>
        MavCompIdServo10 = 149, 

        /// <summary>  </summary>
        MavCompIdServo11 = 150, 

        /// <summary>  </summary>
        MavCompIdServo12 = 151, 

        /// <summary>  </summary>
        MavCompIdServo13 = 152, 

        /// <summary>  </summary>
        MavCompIdServo14 = 153 };

    /// <summary>
    /// These encode the sensors whose status is sent as part of the SYS_STATUS message.
    /// </summary>
    public enum MavSysStatusSensor { 

        /// <summary> 0x01 3D gyro </summary>
        _3dGyro = 1, 

        /// <summary> 0x02 3D accelerometer </summary>
        _3dAccel = 2, 

        /// <summary> 0x04 3D magnetometer </summary>
        _3dMag = 4, 

        /// <summary> 0x08 absolute pressure </summary>
        AbsolutePressure = 8, 

        /// <summary> 0x10 differential pressure </summary>
        DifferentialPressure = 16, 

        /// <summary> 0x20 GPS </summary>
        Gps = 32, 

        /// <summary> 0x40 optical flow </summary>
        OpticalFlow = 64, 

        /// <summary> 0x80 computer vision position </summary>
        VisionPosition = 128, 

        /// <summary> 0x100 laser based position </summary>
        LaserPosition = 256, 

        /// <summary> 0x200 external ground truth (Vicon or Leica) </summary>
        ExternalGroundTruth = 512, 

        /// <summary> 0x400 3D angular rate control </summary>
        AngularRateControl = 1024, 

        /// <summary> 0x800 attitude stabilization </summary>
        AttitudeStabilization = 2048, 

        /// <summary> 0x1000 yaw position </summary>
        YawPosition = 4096, 

        /// <summary> 0x2000 z/altitude control </summary>
        ZAltitudeControl = 8192, 

        /// <summary> 0x4000 x/y position control </summary>
        XyPositionControl = 16384, 

        /// <summary> 0x8000 motor outputs / control </summary>
        MotorOutputs = 32768, 

        /// <summary> 0x10000 rc receiver </summary>
        RcReceiver = 65536 };

    public enum MavFrame { 

        /// <summary> Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) </summary>
        Global = 0, 

        /// <summary> Local coordinate frame, Z-up (x: north, y: east, z: down). </summary>
        LocalNed = 1, 

        /// <summary> NOT a coordinate frame, indicates a mission command. </summary>
        Mission = 2, 

        /// <summary> Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. </summary>
        GlobalRelativeAlt = 3, 

        /// <summary> Local coordinate frame, Z-down (x: east, y: north, z: up) </summary>
        LocalEnu = 4 };

    public enum MavlinkDataStreamType { 

        /// <summary>  </summary>
        MavlinkDataStreamImgJpeg = 0, 

        /// <summary>  </summary>
        MavlinkDataStreamImgBmp = 0, 

        /// <summary>  </summary>
        MavlinkDataStreamImgRaw8u = 0, 

        /// <summary>  </summary>
        MavlinkDataStreamImgRaw32u = 0, 

        /// <summary>  </summary>
        MavlinkDataStreamImgPgm = 0, 

        /// <summary>  </summary>
        MavlinkDataStreamImgPng = 0 };

    /// <summary>
    /// Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data.
    /// </summary>
    public enum MavCmd { 

        /// <summary> Navigate to MISSION. </summary>
        NavWaypoint = 16, 

        /// <summary> Loiter around this MISSION an unlimited amount of time </summary>
        NavLoiterUnlim = 17, 

        /// <summary> Loiter around this MISSION for X turns </summary>
        NavLoiterTurns = 18, 

        /// <summary> Loiter around this MISSION for X seconds </summary>
        NavLoiterTime = 19, 

        /// <summary> Return to launch location </summary>
        NavReturnToLaunch = 20, 

        /// <summary> Land at location </summary>
        NavLand = 21, 

        /// <summary> Takeoff from ground / hand </summary>
        NavTakeoff = 22, 

        /// <summary> Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. </summary>
        NavRoi = 80, 

        /// <summary> Control autonomous path planning on the MAV. </summary>
        NavPathplanning = 81, 

        /// <summary> NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration </summary>
        NavLast = 95, 

        /// <summary> Delay mission state machine. </summary>
        ConditionDelay = 112, 

        /// <summary> Ascend/descend at rate.  Delay mission state machine until desired altitude reached. </summary>
        ConditionChangeAlt = 113, 

        /// <summary> Delay mission state machine until within desired distance of next NAV point. </summary>
        ConditionDistance = 114, 

        /// <summary> Reach a certain target angle. </summary>
        ConditionYaw = 115, 

        /// <summary> NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration </summary>
        ConditionLast = 159, 

        /// <summary> Set system mode. </summary>
        DoSetMode = 176, 

        /// <summary> Jump to the desired command in the mission list.  Repeat this action only the specified number of times </summary>
        DoJump = 177, 

        /// <summary> Change speed and/or throttle set points. </summary>
        DoChangeSpeed = 178, 

        /// <summary> Changes the home location either to the current location or a specified location. </summary>
        DoSetHome = 179, 

        /// <summary> Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. </summary>
        DoSetParameter = 180, 

        /// <summary> Set a relay to a condition. </summary>
        DoSetRelay = 181, 

        /// <summary> Cycle a relay on and off for a desired number of cyles with a desired period. </summary>
        DoRepeatRelay = 182, 

        /// <summary> Set a servo to a desired PWM value. </summary>
        DoSetServo = 183, 

        /// <summary> Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. </summary>
        DoRepeatServo = 184, 

        /// <summary> Control onboard camera system. </summary>
        DoControlVideo = 200, 

        /// <summary> Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. </summary>
        DoSetRoi = 201, 

        /// <summary> NOP - This command is only used to mark the upper limit of the DO commands in the enumeration </summary>
        DoLast = 240, 

        /// <summary> Trigger calibration. This command will be only accepted if in pre-flight mode. </summary>
        PreflightCalibration = 241, 

        /// <summary> Set sensor offsets. This command will be only accepted if in pre-flight mode. </summary>
        PreflightSetSensorOffsets = 242, 

        /// <summary> Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. </summary>
        PreflightStorage = 245, 

        /// <summary> Request the reboot or shutdown of system components. </summary>
        PreflightRebootShutdown = 246, 

        /// <summary> Hold / continue the current action </summary>
        OverrideGoto = 252, 

        /// <summary> start running a mission </summary>
        MissionStart = 300, 

        /// <summary> Arms / Disarms a component </summary>
        ComponentArmDisarm = 400, 

        /// <summary> Starts receiver pairing </summary>
        StartRxPair = 500 };

    /// <summary>
    /// Data stream IDs. A data stream is not a fixed set of messages, but rather a       recommendation to the autopilot software. Individual autopilots may or may not obey       the recommended messages.
    /// </summary>
    public enum MavDataStream { 

        /// <summary> Enable all data streams </summary>
        All = 0, 

        /// <summary> Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. </summary>
        RawSensors = 1, 

        /// <summary> Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS </summary>
        ExtendedStatus = 2, 

        /// <summary> Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW </summary>
        RcChannels = 3, 

        /// <summary> Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. </summary>
        RawController = 4, 

        /// <summary> Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. </summary>
        Position = 6, 

        /// <summary> Dependent on the autopilot </summary>
        Extra1 = 10, 

        /// <summary> Dependent on the autopilot </summary>
        Extra2 = 11, 

        /// <summary> Dependent on the autopilot </summary>
        Extra3 = 12 };

    /// <summary>
    ///  The ROI (region of interest) for the vehicle. This can be                  be used by the vehicle for camera/vehicle attitude alignment (see                  MAV_CMD_NAV_ROI).
    /// </summary>
    public enum MavRoi { 

        /// <summary> No region of interest. </summary>
        None = 0, 

        /// <summary> Point toward next MISSION. </summary>
        Wpnext = 1, 

        /// <summary> Point toward given MISSION. </summary>
        Wpindex = 2, 

        /// <summary> Point toward fixed location. </summary>
        Location = 3, 

        /// <summary> Point toward of given id. </summary>
        Target = 4 };

    /// <summary>
    /// ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
    /// </summary>
    public enum MavCmdAck { 

        /// <summary> Command / mission item is ok. </summary>
        Ok = 0, 

        /// <summary> Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. </summary>
        ErrFail = 0, 

        /// <summary> The system is refusing to accept this command from this source / communication partner. </summary>
        ErrAccessDenied = 0, 

        /// <summary> Command or mission item is not supported, other commands would be accepted. </summary>
        ErrNotSupported = 0, 

        /// <summary> The coordinate frame of this command / mission item is not supported. </summary>
        ErrCoordinateFrameNotSupported = 0, 

        /// <summary> The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible. </summary>
        ErrCoordinatesOutOfRange = 0, 

        /// <summary> The X or latitude value is out of range. </summary>
        ErrXLatOutOfRange = 0, 

        /// <summary> The Y or longitude value is out of range. </summary>
        ErrYLonOutOfRange = 0, 

        /// <summary> The Z or altitude value is out of range. </summary>
        ErrZAltOutOfRange = 0 };

    /// <summary>
    /// Specifies the datatype of a MAVLink parameter.
    /// </summary>
    public enum MavParamType { 

        /// <summary> 8-bit unsigned integer </summary>
        Uint8 = 1, 

        /// <summary> 8-bit signed integer </summary>
        Int8 = 2, 

        /// <summary> 16-bit unsigned integer </summary>
        Uint16 = 3, 

        /// <summary> 16-bit signed integer </summary>
        Int16 = 4, 

        /// <summary> 32-bit unsigned integer </summary>
        Uint32 = 5, 

        /// <summary> 32-bit signed integer </summary>
        Int32 = 6, 

        /// <summary> 64-bit unsigned integer </summary>
        Uint64 = 7, 

        /// <summary> 64-bit signed integer </summary>
        Int64 = 8, 

        /// <summary> 32-bit floating-point </summary>
        Real32 = 9, 

        /// <summary> 64-bit floating-point </summary>
        Real64 = 10 };

    /// <summary>
    /// result from a mavlink command
    /// </summary>
    public enum MavResult { 

        /// <summary> Command ACCEPTED and EXECUTED </summary>
        Accepted = 0, 

        /// <summary> Command TEMPORARY REJECTED/DENIED </summary>
        TemporarilyRejected = 1, 

        /// <summary> Command PERMANENTLY DENIED </summary>
        Denied = 2, 

        /// <summary> Command UNKNOWN/UNSUPPORTED </summary>
        Unsupported = 3, 

        /// <summary> Command executed, but failed </summary>
        Failed = 4 };

    /// <summary>
    /// result in a mavlink mission ack
    /// </summary>
    public enum MavMissionResult { 

        /// <summary> mission accepted OK </summary>
        MavMissionAccepted = 0, 

        /// <summary> generic error / not accepting mission commands at all right now </summary>
        MavMissionError = 1, 

        /// <summary> coordinate frame is not supported </summary>
        MavMissionUnsupportedFrame = 2, 

        /// <summary> command is not supported </summary>
        MavMissionUnsupported = 3, 

        /// <summary> mission item exceeds storage space </summary>
        MavMissionNoSpace = 4, 

        /// <summary> one of the parameters has an invalid value </summary>
        MavMissionInvalid = 5, 

        /// <summary> param1 has an invalid value </summary>
        MavMissionInvalidParam1 = 6, 

        /// <summary> param2 has an invalid value </summary>
        MavMissionInvalidParam2 = 7, 

        /// <summary> param3 has an invalid value </summary>
        MavMissionInvalidParam3 = 8, 

        /// <summary> param4 has an invalid value </summary>
        MavMissionInvalidParam4 = 9, 

        /// <summary> x/param5 has an invalid value </summary>
        MavMissionInvalidParam5X = 10, 

        /// <summary> y/param6 has an invalid value </summary>
        MavMissionInvalidParam6Y = 11, 

        /// <summary> param7 has an invalid value </summary>
        MavMissionInvalidParam7 = 12, 

        /// <summary> received waypoint out of sequence </summary>
        MavMissionInvalidSequence = 13, 

        /// <summary> not accepting any mission commands from this communication partner </summary>
        MavMissionDenied = 14 };

    /// <summary>
    /// Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
    /// </summary>
    public enum MavSeverity { 

        /// <summary> System is unusable. This is a "panic" condition. </summary>
        Emergency = 0, 

        /// <summary> Action should be taken immediately. Indicates error in non-critical systems. </summary>
        Alert = 1, 

        /// <summary> Action must be taken immediately. Indicates failure in a primary system. </summary>
        Critical = 2, 

        /// <summary> Indicates an error in secondary/redundant systems. </summary>
        Error = 3, 

        /// <summary> Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. </summary>
        Warning = 4, 

        /// <summary> An unusual event has occured, though not an error condition. This should be investigated for the root cause. </summary>
        Notice = 5, 

        /// <summary> Normal operational messages. Useful for logging. No action is required for these messages. </summary>
        Info = 6, 

        /// <summary> Useful non-operational messages that can assist in debugging. These should not occur during normal operation. </summary>
        Debug = 7 };


    // ___________________________________________________________________________________


    /// <summary>
    /// The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
    /// </summary>
    public class UasHeartbeat: UasMessage
    {
        /// <summary>
        /// A bitfield for use for autopilot-specific flags.
        /// </summary>
        public UInt32 CustomMode {
            get { return mCustomMode; }
            set { mCustomMode = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        /// </summary>
        public MavType Type {
            get { return mType; }
            set { mType = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        /// </summary>
        public MavAutopilot Autopilot {
            get { return mAutopilot; }
            set { mAutopilot = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        /// </summary>
        public MavModeFlag BaseMode {
            get { return mBaseMode; }
            set { mBaseMode = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System status flag, see MAV_STATE ENUM
        /// </summary>
        public MavState SystemStatus {
            get { return mSystemStatus; }
            set { mSystemStatus = value; NotifyUpdated(); }
        }

        /// <summary>
        /// MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
        /// </summary>
        public byte MavlinkVersion {
            get { return mMavlinkVersion; }
            set { mMavlinkVersion = value; NotifyUpdated(); }
        }

        public UasHeartbeat()
        {
            mMessageId = 0;
            CrcExtra = 50;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("Heartbeat \n");
            sb.AppendFormat("    CustomMode: {0}\n", mCustomMode);
            sb.AppendFormat("    Type: {0}\n", mType);
            sb.AppendFormat("    Autopilot: {0}\n", mAutopilot);
            sb.AppendFormat("    BaseMode: {0}\n", mBaseMode);
            sb.AppendFormat("    SystemStatus: {0}\n", mSystemStatus);
            sb.AppendFormat("    MavlinkVersion: {0}\n", mMavlinkVersion);

            return sb.ToString();
        }

        private UInt32 mCustomMode;
        private MavType mType;
        private MavAutopilot mAutopilot;
        private MavModeFlag mBaseMode;
        private MavState mSystemStatus;
        private byte mMavlinkVersion;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows wether the system is currently active or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occured it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
    /// </summary>
    public class UasSysStatus: UasMessage
    {
        /// <summary>
        /// Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        /// </summary>
        public MavSysStatusSensor OnboardControlSensorsPresent {
            get { return mOnboardControlSensorsPresent; }
            set { mOnboardControlSensorsPresent = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        /// </summary>
        public MavSysStatusSensor OnboardControlSensorsEnabled {
            get { return mOnboardControlSensorsEnabled; }
            set { mOnboardControlSensorsEnabled = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        /// </summary>
        public MavSysStatusSensor OnboardControlSensorsHealth {
            get { return mOnboardControlSensorsHealth; }
            set { mOnboardControlSensorsHealth = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        /// </summary>
        public UInt16 Load {
            get { return mLoad; }
            set { mLoad = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Battery voltage, in millivolts (1 = 1 millivolt)
        /// </summary>
        public UInt16 VoltageBattery {
            get { return mVoltageBattery; }
            set { mVoltageBattery = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        /// </summary>
        public Int16 CurrentBattery {
            get { return mCurrentBattery; }
            set { mCurrentBattery = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        /// </summary>
        public UInt16 DropRateComm {
            get { return mDropRateComm; }
            set { mDropRateComm = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        /// </summary>
        public UInt16 ErrorsComm {
            get { return mErrorsComm; }
            set { mErrorsComm = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Autopilot-specific errors
        /// </summary>
        public UInt16 ErrorsCount1 {
            get { return mErrorsCount1; }
            set { mErrorsCount1 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Autopilot-specific errors
        /// </summary>
        public UInt16 ErrorsCount2 {
            get { return mErrorsCount2; }
            set { mErrorsCount2 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Autopilot-specific errors
        /// </summary>
        public UInt16 ErrorsCount3 {
            get { return mErrorsCount3; }
            set { mErrorsCount3 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Autopilot-specific errors
        /// </summary>
        public UInt16 ErrorsCount4 {
            get { return mErrorsCount4; }
            set { mErrorsCount4 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        /// </summary>
        public SByte BatteryRemaining {
            get { return mBatteryRemaining; }
            set { mBatteryRemaining = value; NotifyUpdated(); }
        }

        public UasSysStatus()
        {
            mMessageId = 1;
            CrcExtra = 124;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SysStatus \n");
            sb.AppendFormat("    OnboardControlSensorsPresent: {0}\n", mOnboardControlSensorsPresent);
            sb.AppendFormat("    OnboardControlSensorsEnabled: {0}\n", mOnboardControlSensorsEnabled);
            sb.AppendFormat("    OnboardControlSensorsHealth: {0}\n", mOnboardControlSensorsHealth);
            sb.AppendFormat("    Load: {0}\n", mLoad);
            sb.AppendFormat("    VoltageBattery: {0}\n", mVoltageBattery);
            sb.AppendFormat("    CurrentBattery: {0}\n", mCurrentBattery);
            sb.AppendFormat("    DropRateComm: {0}\n", mDropRateComm);
            sb.AppendFormat("    ErrorsComm: {0}\n", mErrorsComm);
            sb.AppendFormat("    ErrorsCount1: {0}\n", mErrorsCount1);
            sb.AppendFormat("    ErrorsCount2: {0}\n", mErrorsCount2);
            sb.AppendFormat("    ErrorsCount3: {0}\n", mErrorsCount3);
            sb.AppendFormat("    ErrorsCount4: {0}\n", mErrorsCount4);
            sb.AppendFormat("    BatteryRemaining: {0}\n", mBatteryRemaining);

            return sb.ToString();
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


    /// <summary>
    /// The system time is the time of the master clock, typically the computer clock of the main onboard computer.
    /// </summary>
    public class UasSystemTime: UasMessage
    {
        /// <summary>
        /// Timestamp of the master clock in microseconds since UNIX epoch.
        /// </summary>
        public UInt64 TimeUnixUsec {
            get { return mTimeUnixUsec; }
            set { mTimeUnixUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Timestamp of the component clock since boot time in milliseconds.
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        public UasSystemTime()
        {
            mMessageId = 2;
            CrcExtra = 137;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SystemTime \n");
            sb.AppendFormat("    TimeUnixUsec: {0}\n", mTimeUnixUsec);
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);

            return sb.ToString();
        }

        private UInt64 mTimeUnixUsec;
        private UInt32 mTimeBootMs;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
    /// </summary>
    public class UasPing: UasMessage
    {
        /// <summary>
        /// Unix timestamp in microseconds
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// PING sequence
        /// </summary>
        public UInt32 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasPing()
        {
            mMessageId = 4;
            CrcExtra = 237;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("Ping \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    Seq: {0}\n", mSeq);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
        }

        private UInt64 mTimeUsec;
        private UInt32 mSeq;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Request to control this MAV
    /// </summary>
    public class UasChangeOperatorControl: UasMessage
    {
        /// <summary>
        /// System the GCS requests control for
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 0: request control of this MAV, 1: Release control of this MAV
        /// </summary>
        public byte ControlRequest {
            get { return mControlRequest; }
            set { mControlRequest = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
        /// </summary>
        public byte Version {
            get { return mVersion; }
            set { mVersion = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
        /// </summary>
        public char[] Passkey {
            get { return mPasskey; }
            set { mPasskey = value; NotifyUpdated(); }
        }

        public UasChangeOperatorControl()
        {
            mMessageId = 5;
            CrcExtra = 217;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ChangeOperatorControl \n");
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    ControlRequest: {0}\n", mControlRequest);
            sb.AppendFormat("    Version: {0}\n", mVersion);
            sb.Append("    Passkey\n");
            sb.AppendFormat("        [0]: {0}\n", mPasskey[0]);
            sb.AppendFormat("        [1]: {0}\n", mPasskey[1]);
            sb.AppendFormat("        [2]: {0}\n", mPasskey[2]);
            sb.AppendFormat("        [3]: {0}\n", mPasskey[3]);
            sb.AppendFormat("        [4]: {0}\n", mPasskey[4]);
            sb.AppendFormat("        [5]: {0}\n", mPasskey[5]);
            sb.AppendFormat("        [6]: {0}\n", mPasskey[6]);
            sb.AppendFormat("        [7]: {0}\n", mPasskey[7]);
            sb.AppendFormat("        [8]: {0}\n", mPasskey[8]);
            sb.AppendFormat("        [9]: {0}\n", mPasskey[9]);
            sb.AppendFormat("        [10]: {0}\n", mPasskey[10]);
            sb.AppendFormat("        [11]: {0}\n", mPasskey[11]);
            sb.AppendFormat("        [12]: {0}\n", mPasskey[12]);
            sb.AppendFormat("        [13]: {0}\n", mPasskey[13]);
            sb.AppendFormat("        [14]: {0}\n", mPasskey[14]);
            sb.AppendFormat("        [15]: {0}\n", mPasskey[15]);
            sb.AppendFormat("        [16]: {0}\n", mPasskey[16]);
            sb.AppendFormat("        [17]: {0}\n", mPasskey[17]);
            sb.AppendFormat("        [18]: {0}\n", mPasskey[18]);
            sb.AppendFormat("        [19]: {0}\n", mPasskey[19]);
            sb.AppendFormat("        [20]: {0}\n", mPasskey[20]);
            sb.AppendFormat("        [21]: {0}\n", mPasskey[21]);
            sb.AppendFormat("        [22]: {0}\n", mPasskey[22]);
            sb.AppendFormat("        [23]: {0}\n", mPasskey[23]);
            sb.AppendFormat("        [24]: {0}\n", mPasskey[24]);

            return sb.ToString();
        }

        private byte mTargetSystem;
        private byte mControlRequest;
        private byte mVersion;
        private char[] mPasskey = new char[25];
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Accept / deny control of this MAV
    /// </summary>
    public class UasChangeOperatorControlAck: UasMessage
    {
        /// <summary>
        /// ID of the GCS this message 
        /// </summary>
        public byte GcsSystemId {
            get { return mGcsSystemId; }
            set { mGcsSystemId = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 0: request control of this MAV, 1: Release control of this MAV
        /// </summary>
        public byte ControlRequest {
            get { return mControlRequest; }
            set { mControlRequest = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
        /// </summary>
        public byte Ack {
            get { return mAck; }
            set { mAck = value; NotifyUpdated(); }
        }

        public UasChangeOperatorControlAck()
        {
            mMessageId = 6;
            CrcExtra = 104;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ChangeOperatorControlAck \n");
            sb.AppendFormat("    GcsSystemId: {0}\n", mGcsSystemId);
            sb.AppendFormat("    ControlRequest: {0}\n", mControlRequest);
            sb.AppendFormat("    Ack: {0}\n", mAck);

            return sb.ToString();
        }

        private byte mGcsSystemId;
        private byte mControlRequest;
        private byte mAck;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
    /// </summary>
    public class UasAuthKey: UasMessage
    {
        /// <summary>
        /// key
        /// </summary>
        public char[] Key {
            get { return mKey; }
            set { mKey = value; NotifyUpdated(); }
        }

        public UasAuthKey()
        {
            mMessageId = 7;
            CrcExtra = 119;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("AuthKey \n");
            sb.Append("    Key\n");
            sb.AppendFormat("        [0]: {0}\n", mKey[0]);
            sb.AppendFormat("        [1]: {0}\n", mKey[1]);
            sb.AppendFormat("        [2]: {0}\n", mKey[2]);
            sb.AppendFormat("        [3]: {0}\n", mKey[3]);
            sb.AppendFormat("        [4]: {0}\n", mKey[4]);
            sb.AppendFormat("        [5]: {0}\n", mKey[5]);
            sb.AppendFormat("        [6]: {0}\n", mKey[6]);
            sb.AppendFormat("        [7]: {0}\n", mKey[7]);
            sb.AppendFormat("        [8]: {0}\n", mKey[8]);
            sb.AppendFormat("        [9]: {0}\n", mKey[9]);
            sb.AppendFormat("        [10]: {0}\n", mKey[10]);
            sb.AppendFormat("        [11]: {0}\n", mKey[11]);
            sb.AppendFormat("        [12]: {0}\n", mKey[12]);
            sb.AppendFormat("        [13]: {0}\n", mKey[13]);
            sb.AppendFormat("        [14]: {0}\n", mKey[14]);
            sb.AppendFormat("        [15]: {0}\n", mKey[15]);
            sb.AppendFormat("        [16]: {0}\n", mKey[16]);
            sb.AppendFormat("        [17]: {0}\n", mKey[17]);
            sb.AppendFormat("        [18]: {0}\n", mKey[18]);
            sb.AppendFormat("        [19]: {0}\n", mKey[19]);
            sb.AppendFormat("        [20]: {0}\n", mKey[20]);
            sb.AppendFormat("        [21]: {0}\n", mKey[21]);
            sb.AppendFormat("        [22]: {0}\n", mKey[22]);
            sb.AppendFormat("        [23]: {0}\n", mKey[23]);
            sb.AppendFormat("        [24]: {0}\n", mKey[24]);
            sb.AppendFormat("        [25]: {0}\n", mKey[25]);
            sb.AppendFormat("        [26]: {0}\n", mKey[26]);
            sb.AppendFormat("        [27]: {0}\n", mKey[27]);
            sb.AppendFormat("        [28]: {0}\n", mKey[28]);
            sb.AppendFormat("        [29]: {0}\n", mKey[29]);
            sb.AppendFormat("        [30]: {0}\n", mKey[30]);
            sb.AppendFormat("        [31]: {0}\n", mKey[31]);

            return sb.ToString();
        }

        private char[] mKey = new char[32];
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
    /// </summary>
    public class UasSetMode: UasMessage
    {
        /// <summary>
        /// The new autopilot-specific mode. This field can be ignored by an autopilot.
        /// </summary>
        public UInt32 CustomMode {
            get { return mCustomMode; }
            set { mCustomMode = value; NotifyUpdated(); }
        }

        /// <summary>
        /// The system setting the mode
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// The new base mode
        /// </summary>
        public byte BaseMode {
            get { return mBaseMode; }
            set { mBaseMode = value; NotifyUpdated(); }
        }

        public UasSetMode()
        {
            mMessageId = 11;
            CrcExtra = 89;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SetMode \n");
            sb.AppendFormat("    CustomMode: {0}\n", mCustomMode);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    BaseMode: {0}\n", mBaseMode);

            return sb.ToString();
        }

        private UInt32 mCustomMode;
        private byte mTargetSystem;
        private byte mBaseMode;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also http://qgroundcontrol.org/parameter_interface for a full documentation of QGroundControl and IMU code.
    /// </summary>
    public class UasParamRequestRead: UasMessage
    {
        /// <summary>
        /// Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
        /// </summary>
        public Int16 ParamIndex {
            get { return mParamIndex; }
            set { mParamIndex = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// </summary>
        public char[] ParamId {
            get { return mParamId; }
            set { mParamId = value; NotifyUpdated(); }
        }

        public UasParamRequestRead()
        {
            mMessageId = 20;
            CrcExtra = 214;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ParamRequestRead \n");
            sb.AppendFormat("    ParamIndex: {0}\n", mParamIndex);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);
            sb.Append("    ParamId\n");
            sb.AppendFormat("        [0]: {0}\n", mParamId[0]);
            sb.AppendFormat("        [1]: {0}\n", mParamId[1]);
            sb.AppendFormat("        [2]: {0}\n", mParamId[2]);
            sb.AppendFormat("        [3]: {0}\n", mParamId[3]);
            sb.AppendFormat("        [4]: {0}\n", mParamId[4]);
            sb.AppendFormat("        [5]: {0}\n", mParamId[5]);
            sb.AppendFormat("        [6]: {0}\n", mParamId[6]);
            sb.AppendFormat("        [7]: {0}\n", mParamId[7]);
            sb.AppendFormat("        [8]: {0}\n", mParamId[8]);
            sb.AppendFormat("        [9]: {0}\n", mParamId[9]);
            sb.AppendFormat("        [10]: {0}\n", mParamId[10]);
            sb.AppendFormat("        [11]: {0}\n", mParamId[11]);
            sb.AppendFormat("        [12]: {0}\n", mParamId[12]);
            sb.AppendFormat("        [13]: {0}\n", mParamId[13]);
            sb.AppendFormat("        [14]: {0}\n", mParamId[14]);
            sb.AppendFormat("        [15]: {0}\n", mParamId[15]);

            return sb.ToString();
        }

        private Int16 mParamIndex;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private char[] mParamId = new char[16];
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Request all parameters of this component. After his request, all parameters are emitted.
    /// </summary>
    public class UasParamRequestList: UasMessage
    {
        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasParamRequestList()
        {
            mMessageId = 21;
            CrcExtra = 159;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ParamRequestList \n");
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
        }

        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
    /// </summary>
    public class UasParamValue: UasMessage
    {
        /// <summary>
        /// Onboard parameter value
        /// </summary>
        public float ParamValue {
            get { return mParamValue; }
            set { mParamValue = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Total number of onboard parameters
        /// </summary>
        public UInt16 ParamCount {
            get { return mParamCount; }
            set { mParamCount = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Index of this onboard parameter
        /// </summary>
        public UInt16 ParamIndex {
            get { return mParamIndex; }
            set { mParamIndex = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// </summary>
        public char[] ParamId {
            get { return mParamId; }
            set { mParamId = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
        /// </summary>
        public MavParamType ParamType {
            get { return mParamType; }
            set { mParamType = value; NotifyUpdated(); }
        }

        public UasParamValue()
        {
            mMessageId = 22;
            CrcExtra = 220;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ParamValue \n");
            sb.AppendFormat("    ParamValue: {0}\n", mParamValue);
            sb.AppendFormat("    ParamCount: {0}\n", mParamCount);
            sb.AppendFormat("    ParamIndex: {0}\n", mParamIndex);
            sb.Append("    ParamId\n");
            sb.AppendFormat("        [0]: {0}\n", mParamId[0]);
            sb.AppendFormat("        [1]: {0}\n", mParamId[1]);
            sb.AppendFormat("        [2]: {0}\n", mParamId[2]);
            sb.AppendFormat("        [3]: {0}\n", mParamId[3]);
            sb.AppendFormat("        [4]: {0}\n", mParamId[4]);
            sb.AppendFormat("        [5]: {0}\n", mParamId[5]);
            sb.AppendFormat("        [6]: {0}\n", mParamId[6]);
            sb.AppendFormat("        [7]: {0}\n", mParamId[7]);
            sb.AppendFormat("        [8]: {0}\n", mParamId[8]);
            sb.AppendFormat("        [9]: {0}\n", mParamId[9]);
            sb.AppendFormat("        [10]: {0}\n", mParamId[10]);
            sb.AppendFormat("        [11]: {0}\n", mParamId[11]);
            sb.AppendFormat("        [12]: {0}\n", mParamId[12]);
            sb.AppendFormat("        [13]: {0}\n", mParamId[13]);
            sb.AppendFormat("        [14]: {0}\n", mParamId[14]);
            sb.AppendFormat("        [15]: {0}\n", mParamId[15]);
            sb.AppendFormat("    ParamType: {0}\n", mParamType);

            return sb.ToString();
        }

        private float mParamValue;
        private UInt16 mParamCount;
        private UInt16 mParamIndex;
        private char[] mParamId = new char[16];
        private MavParamType mParamType;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
    /// </summary>
    public class UasParamSet: UasMessage
    {
        /// <summary>
        /// Onboard parameter value
        /// </summary>
        public float ParamValue {
            get { return mParamValue; }
            set { mParamValue = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        /// </summary>
        public char[] ParamId {
            get { return mParamId; }
            set { mParamId = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
        /// </summary>
        public MavParamType ParamType {
            get { return mParamType; }
            set { mParamType = value; NotifyUpdated(); }
        }

        public UasParamSet()
        {
            mMessageId = 23;
            CrcExtra = 168;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ParamSet \n");
            sb.AppendFormat("    ParamValue: {0}\n", mParamValue);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);
            sb.Append("    ParamId\n");
            sb.AppendFormat("        [0]: {0}\n", mParamId[0]);
            sb.AppendFormat("        [1]: {0}\n", mParamId[1]);
            sb.AppendFormat("        [2]: {0}\n", mParamId[2]);
            sb.AppendFormat("        [3]: {0}\n", mParamId[3]);
            sb.AppendFormat("        [4]: {0}\n", mParamId[4]);
            sb.AppendFormat("        [5]: {0}\n", mParamId[5]);
            sb.AppendFormat("        [6]: {0}\n", mParamId[6]);
            sb.AppendFormat("        [7]: {0}\n", mParamId[7]);
            sb.AppendFormat("        [8]: {0}\n", mParamId[8]);
            sb.AppendFormat("        [9]: {0}\n", mParamId[9]);
            sb.AppendFormat("        [10]: {0}\n", mParamId[10]);
            sb.AppendFormat("        [11]: {0}\n", mParamId[11]);
            sb.AppendFormat("        [12]: {0}\n", mParamId[12]);
            sb.AppendFormat("        [13]: {0}\n", mParamId[13]);
            sb.AppendFormat("        [14]: {0}\n", mParamId[14]);
            sb.AppendFormat("        [15]: {0}\n", mParamId[15]);
            sb.AppendFormat("    ParamType: {0}\n", mParamType);

            return sb.ToString();
        }

        private float mParamValue;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private char[] mParamId = new char[16];
        private MavParamType mParamType;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// The global position, as returned by the Global Positioning System (GPS). This is                  NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
    /// </summary>
    public class UasGpsRawInt: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// </summary>
        public Int32 Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Longitude (WGS84), in degrees * 1E7
        /// </summary>
        public Int32 Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude (WGS84), in meters * 1000 (positive for up)
        /// </summary>
        public Int32 Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        /// <summary>
        /// GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
        /// </summary>
        public UInt16 Eph {
            get { return mEph; }
            set { mEph = value; NotifyUpdated(); }
        }

        /// <summary>
        /// GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
        /// </summary>
        public UInt16 Epv {
            get { return mEpv; }
            set { mEpv = value; NotifyUpdated(); }
        }

        /// <summary>
        /// GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
        /// </summary>
        public UInt16 Vel {
            get { return mVel; }
            set { mVel = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        /// </summary>
        public UInt16 Cog {
            get { return mCog; }
            set { mCog = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
        /// </summary>
        public byte FixType {
            get { return mFixType; }
            set { mFixType = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Number of satellites visible. If unknown, set to 255
        /// </summary>
        public byte SatellitesVisible {
            get { return mSatellitesVisible; }
            set { mSatellitesVisible = value; NotifyUpdated(); }
        }

        public UasGpsRawInt()
        {
            mMessageId = 24;
            CrcExtra = 24;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("GpsRawInt \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    Lat: {0}\n", mLat);
            sb.AppendFormat("    Lon: {0}\n", mLon);
            sb.AppendFormat("    Alt: {0}\n", mAlt);
            sb.AppendFormat("    Eph: {0}\n", mEph);
            sb.AppendFormat("    Epv: {0}\n", mEpv);
            sb.AppendFormat("    Vel: {0}\n", mVel);
            sb.AppendFormat("    Cog: {0}\n", mCog);
            sb.AppendFormat("    FixType: {0}\n", mFixType);
            sb.AppendFormat("    SatellitesVisible: {0}\n", mSatellitesVisible);

            return sb.ToString();
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


    /// <summary>
    /// The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
    /// </summary>
    public class UasGpsStatus: UasMessage
    {
        /// <summary>
        /// Number of satellites visible
        /// </summary>
        public byte SatellitesVisible {
            get { return mSatellitesVisible; }
            set { mSatellitesVisible = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global satellite ID
        /// </summary>
        public byte[] SatellitePrn {
            get { return mSatellitePrn; }
            set { mSatellitePrn = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 0: Satellite not used, 1: used for localization
        /// </summary>
        public byte[] SatelliteUsed {
            get { return mSatelliteUsed; }
            set { mSatelliteUsed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Elevation (0: right on top of receiver, 90: on the horizon) of satellite
        /// </summary>
        public byte[] SatelliteElevation {
            get { return mSatelliteElevation; }
            set { mSatelliteElevation = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Direction of satellite, 0: 0 deg, 255: 360 deg.
        /// </summary>
        public byte[] SatelliteAzimuth {
            get { return mSatelliteAzimuth; }
            set { mSatelliteAzimuth = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Signal to noise ratio of satellite
        /// </summary>
        public byte[] SatelliteSnr {
            get { return mSatelliteSnr; }
            set { mSatelliteSnr = value; NotifyUpdated(); }
        }

        public UasGpsStatus()
        {
            mMessageId = 25;
            CrcExtra = 23;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("GpsStatus \n");
            sb.AppendFormat("    SatellitesVisible: {0}\n", mSatellitesVisible);
            sb.Append("    SatellitePrn\n");
            sb.AppendFormat("        [0]: {0}\n", mSatellitePrn[0]);
            sb.AppendFormat("        [1]: {0}\n", mSatellitePrn[1]);
            sb.AppendFormat("        [2]: {0}\n", mSatellitePrn[2]);
            sb.AppendFormat("        [3]: {0}\n", mSatellitePrn[3]);
            sb.AppendFormat("        [4]: {0}\n", mSatellitePrn[4]);
            sb.AppendFormat("        [5]: {0}\n", mSatellitePrn[5]);
            sb.AppendFormat("        [6]: {0}\n", mSatellitePrn[6]);
            sb.AppendFormat("        [7]: {0}\n", mSatellitePrn[7]);
            sb.AppendFormat("        [8]: {0}\n", mSatellitePrn[8]);
            sb.AppendFormat("        [9]: {0}\n", mSatellitePrn[9]);
            sb.AppendFormat("        [10]: {0}\n", mSatellitePrn[10]);
            sb.AppendFormat("        [11]: {0}\n", mSatellitePrn[11]);
            sb.AppendFormat("        [12]: {0}\n", mSatellitePrn[12]);
            sb.AppendFormat("        [13]: {0}\n", mSatellitePrn[13]);
            sb.AppendFormat("        [14]: {0}\n", mSatellitePrn[14]);
            sb.AppendFormat("        [15]: {0}\n", mSatellitePrn[15]);
            sb.AppendFormat("        [16]: {0}\n", mSatellitePrn[16]);
            sb.AppendFormat("        [17]: {0}\n", mSatellitePrn[17]);
            sb.AppendFormat("        [18]: {0}\n", mSatellitePrn[18]);
            sb.AppendFormat("        [19]: {0}\n", mSatellitePrn[19]);
            sb.Append("    SatelliteUsed\n");
            sb.AppendFormat("        [0]: {0}\n", mSatelliteUsed[0]);
            sb.AppendFormat("        [1]: {0}\n", mSatelliteUsed[1]);
            sb.AppendFormat("        [2]: {0}\n", mSatelliteUsed[2]);
            sb.AppendFormat("        [3]: {0}\n", mSatelliteUsed[3]);
            sb.AppendFormat("        [4]: {0}\n", mSatelliteUsed[4]);
            sb.AppendFormat("        [5]: {0}\n", mSatelliteUsed[5]);
            sb.AppendFormat("        [6]: {0}\n", mSatelliteUsed[6]);
            sb.AppendFormat("        [7]: {0}\n", mSatelliteUsed[7]);
            sb.AppendFormat("        [8]: {0}\n", mSatelliteUsed[8]);
            sb.AppendFormat("        [9]: {0}\n", mSatelliteUsed[9]);
            sb.AppendFormat("        [10]: {0}\n", mSatelliteUsed[10]);
            sb.AppendFormat("        [11]: {0}\n", mSatelliteUsed[11]);
            sb.AppendFormat("        [12]: {0}\n", mSatelliteUsed[12]);
            sb.AppendFormat("        [13]: {0}\n", mSatelliteUsed[13]);
            sb.AppendFormat("        [14]: {0}\n", mSatelliteUsed[14]);
            sb.AppendFormat("        [15]: {0}\n", mSatelliteUsed[15]);
            sb.AppendFormat("        [16]: {0}\n", mSatelliteUsed[16]);
            sb.AppendFormat("        [17]: {0}\n", mSatelliteUsed[17]);
            sb.AppendFormat("        [18]: {0}\n", mSatelliteUsed[18]);
            sb.AppendFormat("        [19]: {0}\n", mSatelliteUsed[19]);
            sb.Append("    SatelliteElevation\n");
            sb.AppendFormat("        [0]: {0}\n", mSatelliteElevation[0]);
            sb.AppendFormat("        [1]: {0}\n", mSatelliteElevation[1]);
            sb.AppendFormat("        [2]: {0}\n", mSatelliteElevation[2]);
            sb.AppendFormat("        [3]: {0}\n", mSatelliteElevation[3]);
            sb.AppendFormat("        [4]: {0}\n", mSatelliteElevation[4]);
            sb.AppendFormat("        [5]: {0}\n", mSatelliteElevation[5]);
            sb.AppendFormat("        [6]: {0}\n", mSatelliteElevation[6]);
            sb.AppendFormat("        [7]: {0}\n", mSatelliteElevation[7]);
            sb.AppendFormat("        [8]: {0}\n", mSatelliteElevation[8]);
            sb.AppendFormat("        [9]: {0}\n", mSatelliteElevation[9]);
            sb.AppendFormat("        [10]: {0}\n", mSatelliteElevation[10]);
            sb.AppendFormat("        [11]: {0}\n", mSatelliteElevation[11]);
            sb.AppendFormat("        [12]: {0}\n", mSatelliteElevation[12]);
            sb.AppendFormat("        [13]: {0}\n", mSatelliteElevation[13]);
            sb.AppendFormat("        [14]: {0}\n", mSatelliteElevation[14]);
            sb.AppendFormat("        [15]: {0}\n", mSatelliteElevation[15]);
            sb.AppendFormat("        [16]: {0}\n", mSatelliteElevation[16]);
            sb.AppendFormat("        [17]: {0}\n", mSatelliteElevation[17]);
            sb.AppendFormat("        [18]: {0}\n", mSatelliteElevation[18]);
            sb.AppendFormat("        [19]: {0}\n", mSatelliteElevation[19]);
            sb.Append("    SatelliteAzimuth\n");
            sb.AppendFormat("        [0]: {0}\n", mSatelliteAzimuth[0]);
            sb.AppendFormat("        [1]: {0}\n", mSatelliteAzimuth[1]);
            sb.AppendFormat("        [2]: {0}\n", mSatelliteAzimuth[2]);
            sb.AppendFormat("        [3]: {0}\n", mSatelliteAzimuth[3]);
            sb.AppendFormat("        [4]: {0}\n", mSatelliteAzimuth[4]);
            sb.AppendFormat("        [5]: {0}\n", mSatelliteAzimuth[5]);
            sb.AppendFormat("        [6]: {0}\n", mSatelliteAzimuth[6]);
            sb.AppendFormat("        [7]: {0}\n", mSatelliteAzimuth[7]);
            sb.AppendFormat("        [8]: {0}\n", mSatelliteAzimuth[8]);
            sb.AppendFormat("        [9]: {0}\n", mSatelliteAzimuth[9]);
            sb.AppendFormat("        [10]: {0}\n", mSatelliteAzimuth[10]);
            sb.AppendFormat("        [11]: {0}\n", mSatelliteAzimuth[11]);
            sb.AppendFormat("        [12]: {0}\n", mSatelliteAzimuth[12]);
            sb.AppendFormat("        [13]: {0}\n", mSatelliteAzimuth[13]);
            sb.AppendFormat("        [14]: {0}\n", mSatelliteAzimuth[14]);
            sb.AppendFormat("        [15]: {0}\n", mSatelliteAzimuth[15]);
            sb.AppendFormat("        [16]: {0}\n", mSatelliteAzimuth[16]);
            sb.AppendFormat("        [17]: {0}\n", mSatelliteAzimuth[17]);
            sb.AppendFormat("        [18]: {0}\n", mSatelliteAzimuth[18]);
            sb.AppendFormat("        [19]: {0}\n", mSatelliteAzimuth[19]);
            sb.Append("    SatelliteSnr\n");
            sb.AppendFormat("        [0]: {0}\n", mSatelliteSnr[0]);
            sb.AppendFormat("        [1]: {0}\n", mSatelliteSnr[1]);
            sb.AppendFormat("        [2]: {0}\n", mSatelliteSnr[2]);
            sb.AppendFormat("        [3]: {0}\n", mSatelliteSnr[3]);
            sb.AppendFormat("        [4]: {0}\n", mSatelliteSnr[4]);
            sb.AppendFormat("        [5]: {0}\n", mSatelliteSnr[5]);
            sb.AppendFormat("        [6]: {0}\n", mSatelliteSnr[6]);
            sb.AppendFormat("        [7]: {0}\n", mSatelliteSnr[7]);
            sb.AppendFormat("        [8]: {0}\n", mSatelliteSnr[8]);
            sb.AppendFormat("        [9]: {0}\n", mSatelliteSnr[9]);
            sb.AppendFormat("        [10]: {0}\n", mSatelliteSnr[10]);
            sb.AppendFormat("        [11]: {0}\n", mSatelliteSnr[11]);
            sb.AppendFormat("        [12]: {0}\n", mSatelliteSnr[12]);
            sb.AppendFormat("        [13]: {0}\n", mSatelliteSnr[13]);
            sb.AppendFormat("        [14]: {0}\n", mSatelliteSnr[14]);
            sb.AppendFormat("        [15]: {0}\n", mSatelliteSnr[15]);
            sb.AppendFormat("        [16]: {0}\n", mSatelliteSnr[16]);
            sb.AppendFormat("        [17]: {0}\n", mSatelliteSnr[17]);
            sb.AppendFormat("        [18]: {0}\n", mSatelliteSnr[18]);
            sb.AppendFormat("        [19]: {0}\n", mSatelliteSnr[19]);

            return sb.ToString();
        }

        private byte mSatellitesVisible;
        private byte[] mSatellitePrn = new byte[20];
        private byte[] mSatelliteUsed = new byte[20];
        private byte[] mSatelliteElevation = new byte[20];
        private byte[] mSatelliteAzimuth = new byte[20];
        private byte[] mSatelliteSnr = new byte[20];
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
    /// </summary>
    public class UasScaledImu: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X acceleration (mg)
        /// </summary>
        public Int16 Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y acceleration (mg)
        /// </summary>
        public Int16 Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z acceleration (mg)
        /// </summary>
        public Int16 Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around X axis (millirad /sec)
        /// </summary>
        public Int16 Xgyro {
            get { return mXgyro; }
            set { mXgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around Y axis (millirad /sec)
        /// </summary>
        public Int16 Ygyro {
            get { return mYgyro; }
            set { mYgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around Z axis (millirad /sec)
        /// </summary>
        public Int16 Zgyro {
            get { return mZgyro; }
            set { mZgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X Magnetic field (milli tesla)
        /// </summary>
        public Int16 Xmag {
            get { return mXmag; }
            set { mXmag = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y Magnetic field (milli tesla)
        /// </summary>
        public Int16 Ymag {
            get { return mYmag; }
            set { mYmag = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z Magnetic field (milli tesla)
        /// </summary>
        public Int16 Zmag {
            get { return mZmag; }
            set { mZmag = value; NotifyUpdated(); }
        }

        public UasScaledImu()
        {
            mMessageId = 26;
            CrcExtra = 170;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ScaledImu \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    Xacc: {0}\n", mXacc);
            sb.AppendFormat("    Yacc: {0}\n", mYacc);
            sb.AppendFormat("    Zacc: {0}\n", mZacc);
            sb.AppendFormat("    Xgyro: {0}\n", mXgyro);
            sb.AppendFormat("    Ygyro: {0}\n", mYgyro);
            sb.AppendFormat("    Zgyro: {0}\n", mZgyro);
            sb.AppendFormat("    Xmag: {0}\n", mXmag);
            sb.AppendFormat("    Ymag: {0}\n", mYmag);
            sb.AppendFormat("    Zmag: {0}\n", mZmag);

            return sb.ToString();
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


    /// <summary>
    /// The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
    /// </summary>
    public class UasRawImu: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X acceleration (raw)
        /// </summary>
        public Int16 Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y acceleration (raw)
        /// </summary>
        public Int16 Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z acceleration (raw)
        /// </summary>
        public Int16 Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around X axis (raw)
        /// </summary>
        public Int16 Xgyro {
            get { return mXgyro; }
            set { mXgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around Y axis (raw)
        /// </summary>
        public Int16 Ygyro {
            get { return mYgyro; }
            set { mYgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around Z axis (raw)
        /// </summary>
        public Int16 Zgyro {
            get { return mZgyro; }
            set { mZgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X Magnetic field (raw)
        /// </summary>
        public Int16 Xmag {
            get { return mXmag; }
            set { mXmag = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y Magnetic field (raw)
        /// </summary>
        public Int16 Ymag {
            get { return mYmag; }
            set { mYmag = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z Magnetic field (raw)
        /// </summary>
        public Int16 Zmag {
            get { return mZmag; }
            set { mZmag = value; NotifyUpdated(); }
        }

        public UasRawImu()
        {
            mMessageId = 27;
            CrcExtra = 144;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("RawImu \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    Xacc: {0}\n", mXacc);
            sb.AppendFormat("    Yacc: {0}\n", mYacc);
            sb.AppendFormat("    Zacc: {0}\n", mZacc);
            sb.AppendFormat("    Xgyro: {0}\n", mXgyro);
            sb.AppendFormat("    Ygyro: {0}\n", mYgyro);
            sb.AppendFormat("    Zgyro: {0}\n", mZgyro);
            sb.AppendFormat("    Xmag: {0}\n", mXmag);
            sb.AppendFormat("    Ymag: {0}\n", mYmag);
            sb.AppendFormat("    Zmag: {0}\n", mZmag);

            return sb.ToString();
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


    /// <summary>
    /// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
    /// </summary>
    public class UasRawPressure: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Absolute pressure (raw)
        /// </summary>
        public Int16 PressAbs {
            get { return mPressAbs; }
            set { mPressAbs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Differential pressure 1 (raw)
        /// </summary>
        public Int16 PressDiff1 {
            get { return mPressDiff1; }
            set { mPressDiff1 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Differential pressure 2 (raw)
        /// </summary>
        public Int16 PressDiff2 {
            get { return mPressDiff2; }
            set { mPressDiff2 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Raw Temperature measurement (raw)
        /// </summary>
        public Int16 Temperature {
            get { return mTemperature; }
            set { mTemperature = value; NotifyUpdated(); }
        }

        public UasRawPressure()
        {
            mMessageId = 28;
            CrcExtra = 67;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("RawPressure \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    PressAbs: {0}\n", mPressAbs);
            sb.AppendFormat("    PressDiff1: {0}\n", mPressDiff1);
            sb.AppendFormat("    PressDiff2: {0}\n", mPressDiff2);
            sb.AppendFormat("    Temperature: {0}\n", mTemperature);

            return sb.ToString();
        }

        private UInt64 mTimeUsec;
        private Int16 mPressAbs;
        private Int16 mPressDiff1;
        private Int16 mPressDiff2;
        private Int16 mTemperature;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
    /// </summary>
    public class UasScaledPressure: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Absolute pressure (hectopascal)
        /// </summary>
        public float PressAbs {
            get { return mPressAbs; }
            set { mPressAbs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Differential pressure 1 (hectopascal)
        /// </summary>
        public float PressDiff {
            get { return mPressDiff; }
            set { mPressDiff = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Temperature measurement (0.01 degrees celsius)
        /// </summary>
        public Int16 Temperature {
            get { return mTemperature; }
            set { mTemperature = value; NotifyUpdated(); }
        }

        public UasScaledPressure()
        {
            mMessageId = 29;
            CrcExtra = 115;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ScaledPressure \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    PressAbs: {0}\n", mPressAbs);
            sb.AppendFormat("    PressDiff: {0}\n", mPressDiff);
            sb.AppendFormat("    Temperature: {0}\n", mTemperature);

            return sb.ToString();
        }

        private UInt32 mTimeBootMs;
        private float mPressAbs;
        private float mPressDiff;
        private Int16 mTemperature;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
    /// </summary>
    public class UasAttitude: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Roll angle (rad, -pi..+pi)
        /// </summary>
        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Pitch angle (rad, -pi..+pi)
        /// </summary>
        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Yaw angle (rad, -pi..+pi)
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Roll angular speed (rad/s)
        /// </summary>
        public float Rollspeed {
            get { return mRollspeed; }
            set { mRollspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Pitch angular speed (rad/s)
        /// </summary>
        public float Pitchspeed {
            get { return mPitchspeed; }
            set { mPitchspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Yaw angular speed (rad/s)
        /// </summary>
        public float Yawspeed {
            get { return mYawspeed; }
            set { mYawspeed = value; NotifyUpdated(); }
        }

        public UasAttitude()
        {
            mMessageId = 30;
            CrcExtra = 39;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("Attitude \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    Roll: {0}\n", mRoll);
            sb.AppendFormat("    Pitch: {0}\n", mPitch);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);
            sb.AppendFormat("    Rollspeed: {0}\n", mRollspeed);
            sb.AppendFormat("    Pitchspeed: {0}\n", mPitchspeed);
            sb.AppendFormat("    Yawspeed: {0}\n", mYawspeed);

            return sb.ToString();
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


    /// <summary>
    /// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
    /// </summary>
    public class UasAttitudeQuaternion: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Quaternion component 1
        /// </summary>
        public float Q1 {
            get { return mQ1; }
            set { mQ1 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Quaternion component 2
        /// </summary>
        public float Q2 {
            get { return mQ2; }
            set { mQ2 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Quaternion component 3
        /// </summary>
        public float Q3 {
            get { return mQ3; }
            set { mQ3 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Quaternion component 4
        /// </summary>
        public float Q4 {
            get { return mQ4; }
            set { mQ4 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Roll angular speed (rad/s)
        /// </summary>
        public float Rollspeed {
            get { return mRollspeed; }
            set { mRollspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Pitch angular speed (rad/s)
        /// </summary>
        public float Pitchspeed {
            get { return mPitchspeed; }
            set { mPitchspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Yaw angular speed (rad/s)
        /// </summary>
        public float Yawspeed {
            get { return mYawspeed; }
            set { mYawspeed = value; NotifyUpdated(); }
        }

        public UasAttitudeQuaternion()
        {
            mMessageId = 31;
            CrcExtra = 246;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("AttitudeQuaternion \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    Q1: {0}\n", mQ1);
            sb.AppendFormat("    Q2: {0}\n", mQ2);
            sb.AppendFormat("    Q3: {0}\n", mQ3);
            sb.AppendFormat("    Q4: {0}\n", mQ4);
            sb.AppendFormat("    Rollspeed: {0}\n", mRollspeed);
            sb.AppendFormat("    Pitchspeed: {0}\n", mPitchspeed);
            sb.AppendFormat("    Yawspeed: {0}\n", mYawspeed);

            return sb.ToString();
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


    /// <summary>
    /// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    /// </summary>
    public class UasLocalPositionNed: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X Position
        /// </summary>
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y Position
        /// </summary>
        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z Position
        /// </summary>
        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X Speed
        /// </summary>
        public float Vx {
            get { return mVx; }
            set { mVx = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y Speed
        /// </summary>
        public float Vy {
            get { return mVy; }
            set { mVy = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z Speed
        /// </summary>
        public float Vz {
            get { return mVz; }
            set { mVz = value; NotifyUpdated(); }
        }

        public UasLocalPositionNed()
        {
            mMessageId = 32;
            CrcExtra = 185;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("LocalPositionNed \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    X: {0}\n", mX);
            sb.AppendFormat("    Y: {0}\n", mY);
            sb.AppendFormat("    Z: {0}\n", mZ);
            sb.AppendFormat("    Vx: {0}\n", mVx);
            sb.AppendFormat("    Vy: {0}\n", mVy);
            sb.AppendFormat("    Vz: {0}\n", mVz);

            return sb.ToString();
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


    /// <summary>
    /// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It                 is designed as scaled integer message since the resolution of float is not sufficient.
    /// </summary>
    public class UasGlobalPositionInt: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Latitude, expressed as * 1E7
        /// </summary>
        public Int32 Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Longitude, expressed as * 1E7
        /// </summary>
        public Int32 Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude in meters, expressed as * 1000 (millimeters), above MSL
        /// </summary>
        public Int32 Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude above ground in meters, expressed as * 1000 (millimeters)
        /// </summary>
        public Int32 RelativeAlt {
            get { return mRelativeAlt; }
            set { mRelativeAlt = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Ground X Speed (Latitude), expressed as m/s * 100
        /// </summary>
        public Int16 Vx {
            get { return mVx; }
            set { mVx = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Ground Y Speed (Longitude), expressed as m/s * 100
        /// </summary>
        public Int16 Vy {
            get { return mVy; }
            set { mVy = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Ground Z Speed (Altitude), expressed as m/s * 100
        /// </summary>
        public Int16 Vz {
            get { return mVz; }
            set { mVz = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        /// </summary>
        public UInt16 Hdg {
            get { return mHdg; }
            set { mHdg = value; NotifyUpdated(); }
        }

        public UasGlobalPositionInt()
        {
            mMessageId = 33;
            CrcExtra = 104;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("GlobalPositionInt \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    Lat: {0}\n", mLat);
            sb.AppendFormat("    Lon: {0}\n", mLon);
            sb.AppendFormat("    Alt: {0}\n", mAlt);
            sb.AppendFormat("    RelativeAlt: {0}\n", mRelativeAlt);
            sb.AppendFormat("    Vx: {0}\n", mVx);
            sb.AppendFormat("    Vy: {0}\n", mVy);
            sb.AppendFormat("    Vz: {0}\n", mVz);
            sb.AppendFormat("    Hdg: {0}\n", mHdg);

            return sb.ToString();
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


    /// <summary>
    /// The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
    /// </summary>
    public class UasRcChannelsScaled: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// </summary>
        public Int16 Chan1Scaled {
            get { return mChan1Scaled; }
            set { mChan1Scaled = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// </summary>
        public Int16 Chan2Scaled {
            get { return mChan2Scaled; }
            set { mChan2Scaled = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// </summary>
        public Int16 Chan3Scaled {
            get { return mChan3Scaled; }
            set { mChan3Scaled = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// </summary>
        public Int16 Chan4Scaled {
            get { return mChan4Scaled; }
            set { mChan4Scaled = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// </summary>
        public Int16 Chan5Scaled {
            get { return mChan5Scaled; }
            set { mChan5Scaled = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// </summary>
        public Int16 Chan6Scaled {
            get { return mChan6Scaled; }
            set { mChan6Scaled = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// </summary>
        public Int16 Chan7Scaled {
            get { return mChan7Scaled; }
            set { mChan7Scaled = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
        /// </summary>
        public Int16 Chan8Scaled {
            get { return mChan8Scaled; }
            set { mChan8Scaled = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
        /// </summary>
        public byte Port {
            get { return mPort; }
            set { mPort = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
        /// </summary>
        public byte Rssi {
            get { return mRssi; }
            set { mRssi = value; NotifyUpdated(); }
        }

        public UasRcChannelsScaled()
        {
            mMessageId = 34;
            CrcExtra = 237;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("RcChannelsScaled \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    Chan1Scaled: {0}\n", mChan1Scaled);
            sb.AppendFormat("    Chan2Scaled: {0}\n", mChan2Scaled);
            sb.AppendFormat("    Chan3Scaled: {0}\n", mChan3Scaled);
            sb.AppendFormat("    Chan4Scaled: {0}\n", mChan4Scaled);
            sb.AppendFormat("    Chan5Scaled: {0}\n", mChan5Scaled);
            sb.AppendFormat("    Chan6Scaled: {0}\n", mChan6Scaled);
            sb.AppendFormat("    Chan7Scaled: {0}\n", mChan7Scaled);
            sb.AppendFormat("    Chan8Scaled: {0}\n", mChan8Scaled);
            sb.AppendFormat("    Port: {0}\n", mPort);
            sb.AppendFormat("    Rssi: {0}\n", mRssi);

            return sb.ToString();
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


    /// <summary>
    /// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    /// </summary>
    public class UasRcChannelsRaw: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// </summary>
        public UInt16 Chan1Raw {
            get { return mChan1Raw; }
            set { mChan1Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// </summary>
        public UInt16 Chan2Raw {
            get { return mChan2Raw; }
            set { mChan2Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// </summary>
        public UInt16 Chan3Raw {
            get { return mChan3Raw; }
            set { mChan3Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// </summary>
        public UInt16 Chan4Raw {
            get { return mChan4Raw; }
            set { mChan4Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// </summary>
        public UInt16 Chan5Raw {
            get { return mChan5Raw; }
            set { mChan5Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// </summary>
        public UInt16 Chan6Raw {
            get { return mChan6Raw; }
            set { mChan6Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// </summary>
        public UInt16 Chan7Raw {
            get { return mChan7Raw; }
            set { mChan7Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
        /// </summary>
        public UInt16 Chan8Raw {
            get { return mChan8Raw; }
            set { mChan8Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
        /// </summary>
        public byte Port {
            get { return mPort; }
            set { mPort = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
        /// </summary>
        public byte Rssi {
            get { return mRssi; }
            set { mRssi = value; NotifyUpdated(); }
        }

        public UasRcChannelsRaw()
        {
            mMessageId = 35;
            CrcExtra = 244;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("RcChannelsRaw \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    Chan1Raw: {0}\n", mChan1Raw);
            sb.AppendFormat("    Chan2Raw: {0}\n", mChan2Raw);
            sb.AppendFormat("    Chan3Raw: {0}\n", mChan3Raw);
            sb.AppendFormat("    Chan4Raw: {0}\n", mChan4Raw);
            sb.AppendFormat("    Chan5Raw: {0}\n", mChan5Raw);
            sb.AppendFormat("    Chan6Raw: {0}\n", mChan6Raw);
            sb.AppendFormat("    Chan7Raw: {0}\n", mChan7Raw);
            sb.AppendFormat("    Chan8Raw: {0}\n", mChan8Raw);
            sb.AppendFormat("    Port: {0}\n", mPort);
            sb.AppendFormat("    Rssi: {0}\n", mRssi);

            return sb.ToString();
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


    /// <summary>
    /// The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
    /// </summary>
    public class UasServoOutputRaw: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds since system boot)
        /// </summary>
        public UInt32 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Servo output 1 value, in microseconds
        /// </summary>
        public UInt16 Servo1Raw {
            get { return mServo1Raw; }
            set { mServo1Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Servo output 2 value, in microseconds
        /// </summary>
        public UInt16 Servo2Raw {
            get { return mServo2Raw; }
            set { mServo2Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Servo output 3 value, in microseconds
        /// </summary>
        public UInt16 Servo3Raw {
            get { return mServo3Raw; }
            set { mServo3Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Servo output 4 value, in microseconds
        /// </summary>
        public UInt16 Servo4Raw {
            get { return mServo4Raw; }
            set { mServo4Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Servo output 5 value, in microseconds
        /// </summary>
        public UInt16 Servo5Raw {
            get { return mServo5Raw; }
            set { mServo5Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Servo output 6 value, in microseconds
        /// </summary>
        public UInt16 Servo6Raw {
            get { return mServo6Raw; }
            set { mServo6Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Servo output 7 value, in microseconds
        /// </summary>
        public UInt16 Servo7Raw {
            get { return mServo7Raw; }
            set { mServo7Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Servo output 8 value, in microseconds
        /// </summary>
        public UInt16 Servo8Raw {
            get { return mServo8Raw; }
            set { mServo8Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
        /// </summary>
        public byte Port {
            get { return mPort; }
            set { mPort = value; NotifyUpdated(); }
        }

        public UasServoOutputRaw()
        {
            mMessageId = 36;
            CrcExtra = 222;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ServoOutputRaw \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    Servo1Raw: {0}\n", mServo1Raw);
            sb.AppendFormat("    Servo2Raw: {0}\n", mServo2Raw);
            sb.AppendFormat("    Servo3Raw: {0}\n", mServo3Raw);
            sb.AppendFormat("    Servo4Raw: {0}\n", mServo4Raw);
            sb.AppendFormat("    Servo5Raw: {0}\n", mServo5Raw);
            sb.AppendFormat("    Servo6Raw: {0}\n", mServo6Raw);
            sb.AppendFormat("    Servo7Raw: {0}\n", mServo7Raw);
            sb.AppendFormat("    Servo8Raw: {0}\n", mServo8Raw);
            sb.AppendFormat("    Port: {0}\n", mPort);

            return sb.ToString();
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


    /// <summary>
    /// Request a partial list of mission items from the system/component. http://qgroundcontrol.org/mavlink/waypoint_protocol. If start and end index are the same, just send one waypoint.
    /// </summary>
    public class UasMissionRequestPartialList: UasMessage
    {
        /// <summary>
        /// Start index, 0 by default
        /// </summary>
        public Int16 StartIndex {
            get { return mStartIndex; }
            set { mStartIndex = value; NotifyUpdated(); }
        }

        /// <summary>
        /// End index, -1 by default (-1: send list to end). Else a valid index of the list
        /// </summary>
        public Int16 EndIndex {
            get { return mEndIndex; }
            set { mEndIndex = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasMissionRequestPartialList()
        {
            mMessageId = 37;
            CrcExtra = 212;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MissionRequestPartialList \n");
            sb.AppendFormat("    StartIndex: {0}\n", mStartIndex);
            sb.AppendFormat("    EndIndex: {0}\n", mEndIndex);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
        }

        private Int16 mStartIndex;
        private Int16 mEndIndex;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
    /// </summary>
    public class UasMissionWritePartialList: UasMessage
    {
        /// <summary>
        /// Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
        /// </summary>
        public Int16 StartIndex {
            get { return mStartIndex; }
            set { mStartIndex = value; NotifyUpdated(); }
        }

        /// <summary>
        /// End index, equal or greater than start index.
        /// </summary>
        public Int16 EndIndex {
            get { return mEndIndex; }
            set { mEndIndex = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasMissionWritePartialList()
        {
            mMessageId = 38;
            CrcExtra = 9;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MissionWritePartialList \n");
            sb.AppendFormat("    StartIndex: {0}\n", mStartIndex);
            sb.AppendFormat("    EndIndex: {0}\n", mEndIndex);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
        }

        private Int16 mStartIndex;
        private Int16 mEndIndex;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Message encoding a mission item. This message is emitted to announce                  the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
    /// </summary>
    public class UasMissionItem: UasMessage
    {
        /// <summary>
        /// PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
        /// </summary>
        public float Param1 {
            get { return mParam1; }
            set { mParam1 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
        /// </summary>
        public float Param2 {
            get { return mParam2; }
            set { mParam2 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
        /// </summary>
        public float Param3 {
            get { return mParam3; }
            set { mParam3 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
        /// </summary>
        public float Param4 {
            get { return mParam4; }
            set { mParam4 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// PARAM5 / local: x position, global: latitude
        /// </summary>
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// PARAM6 / y position: global: longitude
        /// </summary>
        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// PARAM7 / z position: global: altitude
        /// </summary>
        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Sequence
        /// </summary>
        public UInt16 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        /// <summary>
        /// The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
        /// </summary>
        public MavCmd Command {
            get { return mCommand; }
            set { mCommand = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        /// <summary>
        /// The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
        /// </summary>
        public MavFrame Frame {
            get { return mFrame; }
            set { mFrame = value; NotifyUpdated(); }
        }

        /// <summary>
        /// false:0, true:1
        /// </summary>
        public byte Current {
            get { return mCurrent; }
            set { mCurrent = value; NotifyUpdated(); }
        }

        /// <summary>
        /// autocontinue to next wp
        /// </summary>
        public byte Autocontinue {
            get { return mAutocontinue; }
            set { mAutocontinue = value; NotifyUpdated(); }
        }

        public UasMissionItem()
        {
            mMessageId = 39;
            CrcExtra = 254;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MissionItem \n");
            sb.AppendFormat("    Param1: {0}\n", mParam1);
            sb.AppendFormat("    Param2: {0}\n", mParam2);
            sb.AppendFormat("    Param3: {0}\n", mParam3);
            sb.AppendFormat("    Param4: {0}\n", mParam4);
            sb.AppendFormat("    X: {0}\n", mX);
            sb.AppendFormat("    Y: {0}\n", mY);
            sb.AppendFormat("    Z: {0}\n", mZ);
            sb.AppendFormat("    Seq: {0}\n", mSeq);
            sb.AppendFormat("    Command: {0}\n", mCommand);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);
            sb.AppendFormat("    Frame: {0}\n", mFrame);
            sb.AppendFormat("    Current: {0}\n", mCurrent);
            sb.AppendFormat("    Autocontinue: {0}\n", mAutocontinue);

            return sb.ToString();
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


    /// <summary>
    /// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
    /// </summary>
    public class UasMissionRequest: UasMessage
    {
        /// <summary>
        /// Sequence
        /// </summary>
        public UInt16 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasMissionRequest()
        {
            mMessageId = 40;
            CrcExtra = 230;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MissionRequest \n");
            sb.AppendFormat("    Seq: {0}\n", mSeq);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
        }

        private UInt16 mSeq;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
    /// </summary>
    public class UasMissionSetCurrent: UasMessage
    {
        /// <summary>
        /// Sequence
        /// </summary>
        public UInt16 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasMissionSetCurrent()
        {
            mMessageId = 41;
            CrcExtra = 28;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MissionSetCurrent \n");
            sb.AppendFormat("    Seq: {0}\n", mSeq);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
        }

        private UInt16 mSeq;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
    /// </summary>
    public class UasMissionCurrent: UasMessage
    {
        /// <summary>
        /// Sequence
        /// </summary>
        public UInt16 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        public UasMissionCurrent()
        {
            mMessageId = 42;
            CrcExtra = 28;
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mSeq);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mSeq = s.ReadUInt16();
        }

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MissionCurrent \n");
            sb.AppendFormat("    Seq: {0}\n", mSeq);

            return sb.ToString();
        }

        private UInt16 mSeq;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Request the overall list of mission items from the system/component.
    /// </summary>
    public class UasMissionRequestList: UasMessage
    {
        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasMissionRequestList()
        {
            mMessageId = 43;
            CrcExtra = 132;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MissionRequestList \n");
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
        }

        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
    /// </summary>
    public class UasMissionCount: UasMessage
    {
        /// <summary>
        /// Number of mission items in the sequence
        /// </summary>
        public UInt16 Count {
            get { return mCount; }
            set { mCount = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasMissionCount()
        {
            mMessageId = 44;
            CrcExtra = 221;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MissionCount \n");
            sb.AppendFormat("    Count: {0}\n", mCount);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
        }

        private UInt16 mCount;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Delete all mission items at once.
    /// </summary>
    public class UasMissionClearAll: UasMessage
    {
        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasMissionClearAll()
        {
            mMessageId = 45;
            CrcExtra = 232;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MissionClearAll \n");
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
        }

        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next MISSION.
    /// </summary>
    public class UasMissionItemReached: UasMessage
    {
        /// <summary>
        /// Sequence
        /// </summary>
        public UInt16 Seq {
            get { return mSeq; }
            set { mSeq = value; NotifyUpdated(); }
        }

        public UasMissionItemReached()
        {
            mMessageId = 46;
            CrcExtra = 11;
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mSeq);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mSeq = s.ReadUInt16();
        }

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MissionItemReached \n");
            sb.AppendFormat("    Seq: {0}\n", mSeq);

            return sb.ToString();
        }

        private UInt16 mSeq;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
    /// </summary>
    public class UasMissionAck: UasMessage
    {
        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        /// <summary>
        /// See MAV_MISSION_RESULT enum
        /// </summary>
        public MavMissionResult Type {
            get { return mType; }
            set { mType = value; NotifyUpdated(); }
        }

        public UasMissionAck()
        {
            mMessageId = 47;
            CrcExtra = 153;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MissionAck \n");
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);
            sb.AppendFormat("    Type: {0}\n", mType);

            return sb.ToString();
        }

        private byte mTargetSystem;
        private byte mTargetComponent;
        private MavMissionResult mType;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// As local waypoints exist, the global MISSION reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
    /// </summary>
    public class UasSetGpsGlobalOrigin: UasMessage
    {
        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// </summary>
        public Int32 Latitude {
            get { return mLatitude; }
            set { mLatitude = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Longitude (WGS84, in degrees * 1E7
        /// </summary>
        public Int32 Longitude {
            get { return mLongitude; }
            set { mLongitude = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude (WGS84), in meters * 1000 (positive for up)
        /// </summary>
        public Int32 Altitude {
            get { return mAltitude; }
            set { mAltitude = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public UasSetGpsGlobalOrigin()
        {
            mMessageId = 48;
            CrcExtra = 41;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SetGpsGlobalOrigin \n");
            sb.AppendFormat("    Latitude: {0}\n", mLatitude);
            sb.AppendFormat("    Longitude: {0}\n", mLongitude);
            sb.AppendFormat("    Altitude: {0}\n", mAltitude);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);

            return sb.ToString();
        }

        private Int32 mLatitude;
        private Int32 mLongitude;
        private Int32 mAltitude;
        private byte mTargetSystem;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
    /// </summary>
    public class UasGpsGlobalOrigin: UasMessage
    {
        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// </summary>
        public Int32 Latitude {
            get { return mLatitude; }
            set { mLatitude = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Longitude (WGS84), in degrees * 1E7
        /// </summary>
        public Int32 Longitude {
            get { return mLongitude; }
            set { mLongitude = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude (WGS84), in meters * 1000 (positive for up)
        /// </summary>
        public Int32 Altitude {
            get { return mAltitude; }
            set { mAltitude = value; NotifyUpdated(); }
        }

        public UasGpsGlobalOrigin()
        {
            mMessageId = 49;
            CrcExtra = 39;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("GpsGlobalOrigin \n");
            sb.AppendFormat("    Latitude: {0}\n", mLatitude);
            sb.AppendFormat("    Longitude: {0}\n", mLongitude);
            sb.AppendFormat("    Altitude: {0}\n", mAltitude);

            return sb.ToString();
        }

        private Int32 mLatitude;
        private Int32 mLongitude;
        private Int32 mAltitude;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Set the setpoint for a local position controller. This is the position in local coordinates the MAV should fly to. This message is sent by the path/MISSION planner to the onboard position controller. As some MAVs have a degree of freedom in yaw (e.g. all helicopters/quadrotors), the desired yaw angle is part of the message.
    /// </summary>
    public class UasSetLocalPositionSetpoint: UasMessage
    {
        /// <summary>
        /// x position
        /// </summary>
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// y position
        /// </summary>
        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// z position
        /// </summary>
        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw angle
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
        /// </summary>
        public MavFrame CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        public UasSetLocalPositionSetpoint()
        {
            mMessageId = 50;
            CrcExtra = 214;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SetLocalPositionSetpoint \n");
            sb.AppendFormat("    X: {0}\n", mX);
            sb.AppendFormat("    Y: {0}\n", mY);
            sb.AppendFormat("    Z: {0}\n", mZ);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);
            sb.AppendFormat("    CoordinateFrame: {0}\n", mCoordinateFrame);

            return sb.ToString();
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


    /// <summary>
    /// Transmit the current local setpoint of the controller to other MAVs (collision avoidance) and to the GCS.
    /// </summary>
    public class UasLocalPositionSetpoint: UasMessage
    {
        /// <summary>
        /// x position
        /// </summary>
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// y position
        /// </summary>
        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// z position
        /// </summary>
        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw angle
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
        /// </summary>
        public MavFrame CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        public UasLocalPositionSetpoint()
        {
            mMessageId = 51;
            CrcExtra = 223;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("LocalPositionSetpoint \n");
            sb.AppendFormat("    X: {0}\n", mX);
            sb.AppendFormat("    Y: {0}\n", mY);
            sb.AppendFormat("    Z: {0}\n", mZ);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);
            sb.AppendFormat("    CoordinateFrame: {0}\n", mCoordinateFrame);

            return sb.ToString();
        }

        private float mX;
        private float mY;
        private float mZ;
        private float mYaw;
        private MavFrame mCoordinateFrame;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Transmit the current local setpoint of the controller to other MAVs (collision avoidance) and to the GCS.
    /// </summary>
    public class UasGlobalPositionSetpointInt: UasMessage
    {
        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// </summary>
        public Int32 Latitude {
            get { return mLatitude; }
            set { mLatitude = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Longitude (WGS84), in degrees * 1E7
        /// </summary>
        public Int32 Longitude {
            get { return mLongitude; }
            set { mLongitude = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude (WGS84), in meters * 1000 (positive for up)
        /// </summary>
        public Int32 Altitude {
            get { return mAltitude; }
            set { mAltitude = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw angle in degrees * 100
        /// </summary>
        public Int16 Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
        /// </summary>
        public MavFrame CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        public UasGlobalPositionSetpointInt()
        {
            mMessageId = 52;
            CrcExtra = 141;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("GlobalPositionSetpointInt \n");
            sb.AppendFormat("    Latitude: {0}\n", mLatitude);
            sb.AppendFormat("    Longitude: {0}\n", mLongitude);
            sb.AppendFormat("    Altitude: {0}\n", mAltitude);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);
            sb.AppendFormat("    CoordinateFrame: {0}\n", mCoordinateFrame);

            return sb.ToString();
        }

        private Int32 mLatitude;
        private Int32 mLongitude;
        private Int32 mAltitude;
        private Int16 mYaw;
        private MavFrame mCoordinateFrame;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Set the current global position setpoint.
    /// </summary>
    public class UasSetGlobalPositionSetpointInt: UasMessage
    {
        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// </summary>
        public Int32 Latitude {
            get { return mLatitude; }
            set { mLatitude = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Longitude (WGS84), in degrees * 1E7
        /// </summary>
        public Int32 Longitude {
            get { return mLongitude; }
            set { mLongitude = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude (WGS84), in meters * 1000 (positive for up)
        /// </summary>
        public Int32 Altitude {
            get { return mAltitude; }
            set { mAltitude = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw angle in degrees * 100
        /// </summary>
        public Int16 Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
        /// </summary>
        public MavFrame CoordinateFrame {
            get { return mCoordinateFrame; }
            set { mCoordinateFrame = value; NotifyUpdated(); }
        }

        public UasSetGlobalPositionSetpointInt()
        {
            mMessageId = 53;
            CrcExtra = 33;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SetGlobalPositionSetpointInt \n");
            sb.AppendFormat("    Latitude: {0}\n", mLatitude);
            sb.AppendFormat("    Longitude: {0}\n", mLongitude);
            sb.AppendFormat("    Altitude: {0}\n", mAltitude);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);
            sb.AppendFormat("    CoordinateFrame: {0}\n", mCoordinateFrame);

            return sb.ToString();
        }

        private Int32 mLatitude;
        private Int32 mLongitude;
        private Int32 mAltitude;
        private Int16 mYaw;
        private MavFrame mCoordinateFrame;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/MISSIONs to accept and which to reject. Safety areas are often enforced by national or competition regulations.
    /// </summary>
    public class UasSafetySetAllowedArea: UasMessage
    {
        /// <summary>
        /// x position 1 / Latitude 1
        /// </summary>
        public float P1x {
            get { return mP1x; }
            set { mP1x = value; NotifyUpdated(); }
        }

        /// <summary>
        /// y position 1 / Longitude 1
        /// </summary>
        public float P1y {
            get { return mP1y; }
            set { mP1y = value; NotifyUpdated(); }
        }

        /// <summary>
        /// z position 1 / Altitude 1
        /// </summary>
        public float P1z {
            get { return mP1z; }
            set { mP1z = value; NotifyUpdated(); }
        }

        /// <summary>
        /// x position 2 / Latitude 2
        /// </summary>
        public float P2x {
            get { return mP2x; }
            set { mP2x = value; NotifyUpdated(); }
        }

        /// <summary>
        /// y position 2 / Longitude 2
        /// </summary>
        public float P2y {
            get { return mP2y; }
            set { mP2y = value; NotifyUpdated(); }
        }

        /// <summary>
        /// z position 2 / Altitude 2
        /// </summary>
        public float P2z {
            get { return mP2z; }
            set { mP2z = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
        /// </summary>
        public MavFrame Frame {
            get { return mFrame; }
            set { mFrame = value; NotifyUpdated(); }
        }

        public UasSafetySetAllowedArea()
        {
            mMessageId = 54;
            CrcExtra = 15;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SafetySetAllowedArea \n");
            sb.AppendFormat("    P1x: {0}\n", mP1x);
            sb.AppendFormat("    P1y: {0}\n", mP1y);
            sb.AppendFormat("    P1z: {0}\n", mP1z);
            sb.AppendFormat("    P2x: {0}\n", mP2x);
            sb.AppendFormat("    P2y: {0}\n", mP2y);
            sb.AppendFormat("    P2z: {0}\n", mP2z);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);
            sb.AppendFormat("    Frame: {0}\n", mFrame);

            return sb.ToString();
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


    /// <summary>
    /// Read out the safety zone the MAV currently assumes.
    /// </summary>
    public class UasSafetyAllowedArea: UasMessage
    {
        /// <summary>
        /// x position 1 / Latitude 1
        /// </summary>
        public float P1x {
            get { return mP1x; }
            set { mP1x = value; NotifyUpdated(); }
        }

        /// <summary>
        /// y position 1 / Longitude 1
        /// </summary>
        public float P1y {
            get { return mP1y; }
            set { mP1y = value; NotifyUpdated(); }
        }

        /// <summary>
        /// z position 1 / Altitude 1
        /// </summary>
        public float P1z {
            get { return mP1z; }
            set { mP1z = value; NotifyUpdated(); }
        }

        /// <summary>
        /// x position 2 / Latitude 2
        /// </summary>
        public float P2x {
            get { return mP2x; }
            set { mP2x = value; NotifyUpdated(); }
        }

        /// <summary>
        /// y position 2 / Longitude 2
        /// </summary>
        public float P2y {
            get { return mP2y; }
            set { mP2y = value; NotifyUpdated(); }
        }

        /// <summary>
        /// z position 2 / Altitude 2
        /// </summary>
        public float P2z {
            get { return mP2z; }
            set { mP2z = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
        /// </summary>
        public MavFrame Frame {
            get { return mFrame; }
            set { mFrame = value; NotifyUpdated(); }
        }

        public UasSafetyAllowedArea()
        {
            mMessageId = 55;
            CrcExtra = 3;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SafetyAllowedArea \n");
            sb.AppendFormat("    P1x: {0}\n", mP1x);
            sb.AppendFormat("    P1y: {0}\n", mP1y);
            sb.AppendFormat("    P1z: {0}\n", mP1z);
            sb.AppendFormat("    P2x: {0}\n", mP2x);
            sb.AppendFormat("    P2y: {0}\n", mP2y);
            sb.AppendFormat("    P2z: {0}\n", mP2z);
            sb.AppendFormat("    Frame: {0}\n", mFrame);

            return sb.ToString();
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


    /// <summary>
    /// Set roll, pitch and yaw.
    /// </summary>
    public class UasSetRollPitchYawThrust: UasMessage
    {
        /// <summary>
        /// Desired roll angle in radians
        /// </summary>
        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired pitch angle in radians
        /// </summary>
        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw angle in radians
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Collective thrust, normalized to 0 .. 1
        /// </summary>
        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasSetRollPitchYawThrust()
        {
            mMessageId = 56;
            CrcExtra = 100;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SetRollPitchYawThrust \n");
            sb.AppendFormat("    Roll: {0}\n", mRoll);
            sb.AppendFormat("    Pitch: {0}\n", mPitch);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);
            sb.AppendFormat("    Thrust: {0}\n", mThrust);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
        }

        private float mRoll;
        private float mPitch;
        private float mYaw;
        private float mThrust;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Set roll, pitch and yaw.
    /// </summary>
    public class UasSetRollPitchYawSpeedThrust: UasMessage
    {
        /// <summary>
        /// Desired roll angular speed in rad/s
        /// </summary>
        public float RollSpeed {
            get { return mRollSpeed; }
            set { mRollSpeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired pitch angular speed in rad/s
        /// </summary>
        public float PitchSpeed {
            get { return mPitchSpeed; }
            set { mPitchSpeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw angular speed in rad/s
        /// </summary>
        public float YawSpeed {
            get { return mYawSpeed; }
            set { mYawSpeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Collective thrust, normalized to 0 .. 1
        /// </summary>
        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasSetRollPitchYawSpeedThrust()
        {
            mMessageId = 57;
            CrcExtra = 24;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SetRollPitchYawSpeedThrust \n");
            sb.AppendFormat("    RollSpeed: {0}\n", mRollSpeed);
            sb.AppendFormat("    PitchSpeed: {0}\n", mPitchSpeed);
            sb.AppendFormat("    YawSpeed: {0}\n", mYawSpeed);
            sb.AppendFormat("    Thrust: {0}\n", mThrust);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
        }

        private float mRollSpeed;
        private float mPitchSpeed;
        private float mYawSpeed;
        private float mThrust;
        private byte mTargetSystem;
        private byte mTargetComponent;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Setpoint in roll, pitch, yaw currently active on the system.
    /// </summary>
    public class UasRollPitchYawThrustSetpoint: UasMessage
    {
        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired roll angle in radians
        /// </summary>
        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired pitch angle in radians
        /// </summary>
        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw angle in radians
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Collective thrust, normalized to 0 .. 1
        /// </summary>
        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        public UasRollPitchYawThrustSetpoint()
        {
            mMessageId = 58;
            CrcExtra = 239;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("RollPitchYawThrustSetpoint \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    Roll: {0}\n", mRoll);
            sb.AppendFormat("    Pitch: {0}\n", mPitch);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);
            sb.AppendFormat("    Thrust: {0}\n", mThrust);

            return sb.ToString();
        }

        private UInt32 mTimeBootMs;
        private float mRoll;
        private float mPitch;
        private float mYaw;
        private float mThrust;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Setpoint in rollspeed, pitchspeed, yawspeed currently active on the system.
    /// </summary>
    public class UasRollPitchYawSpeedThrustSetpoint: UasMessage
    {
        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired roll angular speed in rad/s
        /// </summary>
        public float RollSpeed {
            get { return mRollSpeed; }
            set { mRollSpeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired pitch angular speed in rad/s
        /// </summary>
        public float PitchSpeed {
            get { return mPitchSpeed; }
            set { mPitchSpeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw angular speed in rad/s
        /// </summary>
        public float YawSpeed {
            get { return mYawSpeed; }
            set { mYawSpeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Collective thrust, normalized to 0 .. 1
        /// </summary>
        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        public UasRollPitchYawSpeedThrustSetpoint()
        {
            mMessageId = 59;
            CrcExtra = 238;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("RollPitchYawSpeedThrustSetpoint \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    RollSpeed: {0}\n", mRollSpeed);
            sb.AppendFormat("    PitchSpeed: {0}\n", mPitchSpeed);
            sb.AppendFormat("    YawSpeed: {0}\n", mYawSpeed);
            sb.AppendFormat("    Thrust: {0}\n", mThrust);

            return sb.ToString();
        }

        private UInt32 mTimeBootMs;
        private float mRollSpeed;
        private float mPitchSpeed;
        private float mYawSpeed;
        private float mThrust;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Setpoint in the four motor speeds
    /// </summary>
    public class UasSetQuadMotorsSetpoint: UasMessage
    {
        /// <summary>
        /// Front motor in + configuration, front left motor in x configuration
        /// </summary>
        public UInt16 MotorFrontNw {
            get { return mMotorFrontNw; }
            set { mMotorFrontNw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Right motor in + configuration, front right motor in x configuration
        /// </summary>
        public UInt16 MotorRightNe {
            get { return mMotorRightNe; }
            set { mMotorRightNe = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Back motor in + configuration, back right motor in x configuration
        /// </summary>
        public UInt16 MotorBackSe {
            get { return mMotorBackSe; }
            set { mMotorBackSe = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Left motor in + configuration, back left motor in x configuration
        /// </summary>
        public UInt16 MotorLeftSw {
            get { return mMotorLeftSw; }
            set { mMotorLeftSw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID of the system that should set these motor commands
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public UasSetQuadMotorsSetpoint()
        {
            mMessageId = 60;
            CrcExtra = 30;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SetQuadMotorsSetpoint \n");
            sb.AppendFormat("    MotorFrontNw: {0}\n", mMotorFrontNw);
            sb.AppendFormat("    MotorRightNe: {0}\n", mMotorRightNe);
            sb.AppendFormat("    MotorBackSe: {0}\n", mMotorBackSe);
            sb.AppendFormat("    MotorLeftSw: {0}\n", mMotorLeftSw);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);

            return sb.ToString();
        }

        private UInt16 mMotorFrontNw;
        private UInt16 mMotorRightNe;
        private UInt16 mMotorBackSe;
        private UInt16 mMotorLeftSw;
        private byte mTargetSystem;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Setpoint for up to four quadrotors in a group / wing
    /// </summary>
    public class UasSetQuadSwarmRollPitchYawThrust: UasMessage
    {
        /// <summary>
        /// Desired roll angle in radians +-PI (+-INT16_MAX)
        /// </summary>
        public Int16[] Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired pitch angle in radians +-PI (+-INT16_MAX)
        /// </summary>
        public Int16[] Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw angle in radians, scaled to int16 +-PI (+-INT16_MAX)
        /// </summary>
        public Int16[] Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Collective thrust, scaled to uint16 (0..UINT16_MAX)
        /// </summary>
        public UInt16[] Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        /// <summary>
        /// ID of the quadrotor group (0 - 255, up to 256 groups supported)
        /// </summary>
        public byte Group {
            get { return mGroup; }
            set { mGroup = value; NotifyUpdated(); }
        }

        /// <summary>
        /// ID of the flight mode (0 - 255, up to 256 modes supported)
        /// </summary>
        public byte Mode {
            get { return mMode; }
            set { mMode = value; NotifyUpdated(); }
        }

        public UasSetQuadSwarmRollPitchYawThrust()
        {
            mMessageId = 61;
            CrcExtra = 240;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SetQuadSwarmRollPitchYawThrust \n");
            sb.Append("    Roll\n");
            sb.AppendFormat("        [0]: {0}\n", mRoll[0]);
            sb.AppendFormat("        [1]: {0}\n", mRoll[1]);
            sb.AppendFormat("        [2]: {0}\n", mRoll[2]);
            sb.AppendFormat("        [3]: {0}\n", mRoll[3]);
            sb.Append("    Pitch\n");
            sb.AppendFormat("        [0]: {0}\n", mPitch[0]);
            sb.AppendFormat("        [1]: {0}\n", mPitch[1]);
            sb.AppendFormat("        [2]: {0}\n", mPitch[2]);
            sb.AppendFormat("        [3]: {0}\n", mPitch[3]);
            sb.Append("    Yaw\n");
            sb.AppendFormat("        [0]: {0}\n", mYaw[0]);
            sb.AppendFormat("        [1]: {0}\n", mYaw[1]);
            sb.AppendFormat("        [2]: {0}\n", mYaw[2]);
            sb.AppendFormat("        [3]: {0}\n", mYaw[3]);
            sb.Append("    Thrust\n");
            sb.AppendFormat("        [0]: {0}\n", mThrust[0]);
            sb.AppendFormat("        [1]: {0}\n", mThrust[1]);
            sb.AppendFormat("        [2]: {0}\n", mThrust[2]);
            sb.AppendFormat("        [3]: {0}\n", mThrust[3]);
            sb.AppendFormat("    Group: {0}\n", mGroup);
            sb.AppendFormat("    Mode: {0}\n", mMode);

            return sb.ToString();
        }

        private Int16[] mRoll = new Int16[4];
        private Int16[] mPitch = new Int16[4];
        private Int16[] mYaw = new Int16[4];
        private UInt16[] mThrust = new UInt16[4];
        private byte mGroup;
        private byte mMode;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Outputs of the APM navigation controller. The primary use of this message is to check the response and signs of the controller before actual flight and to assist with tuning controller parameters.
    /// </summary>
    public class UasNavControllerOutput: UasMessage
    {
        /// <summary>
        /// Current desired roll in degrees
        /// </summary>
        public float NavRoll {
            get { return mNavRoll; }
            set { mNavRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Current desired pitch in degrees
        /// </summary>
        public float NavPitch {
            get { return mNavPitch; }
            set { mNavPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Current altitude error in meters
        /// </summary>
        public float AltError {
            get { return mAltError; }
            set { mAltError = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Current airspeed error in meters/second
        /// </summary>
        public float AspdError {
            get { return mAspdError; }
            set { mAspdError = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Current crosstrack error on x-y plane in meters
        /// </summary>
        public float XtrackError {
            get { return mXtrackError; }
            set { mXtrackError = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Current desired heading in degrees
        /// </summary>
        public Int16 NavBearing {
            get { return mNavBearing; }
            set { mNavBearing = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Bearing to current MISSION/target in degrees
        /// </summary>
        public Int16 TargetBearing {
            get { return mTargetBearing; }
            set { mTargetBearing = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Distance to active MISSION in meters
        /// </summary>
        public UInt16 WpDist {
            get { return mWpDist; }
            set { mWpDist = value; NotifyUpdated(); }
        }

        public UasNavControllerOutput()
        {
            mMessageId = 62;
            CrcExtra = 183;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("NavControllerOutput \n");
            sb.AppendFormat("    NavRoll: {0}\n", mNavRoll);
            sb.AppendFormat("    NavPitch: {0}\n", mNavPitch);
            sb.AppendFormat("    AltError: {0}\n", mAltError);
            sb.AppendFormat("    AspdError: {0}\n", mAspdError);
            sb.AppendFormat("    XtrackError: {0}\n", mXtrackError);
            sb.AppendFormat("    NavBearing: {0}\n", mNavBearing);
            sb.AppendFormat("    TargetBearing: {0}\n", mTargetBearing);
            sb.AppendFormat("    WpDist: {0}\n", mWpDist);

            return sb.ToString();
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


    /// <summary>
    /// Setpoint for up to four quadrotors in a group / wing
    /// </summary>
    public class UasSetQuadSwarmLedRollPitchYawThrust: UasMessage
    {
        /// <summary>
        /// Desired roll angle in radians +-PI (+-INT16_MAX)
        /// </summary>
        public Int16[] Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired pitch angle in radians +-PI (+-INT16_MAX)
        /// </summary>
        public Int16[] Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw angle in radians, scaled to int16 +-PI (+-INT16_MAX)
        /// </summary>
        public Int16[] Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Collective thrust, scaled to uint16 (0..UINT16_MAX)
        /// </summary>
        public UInt16[] Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        /// <summary>
        /// ID of the quadrotor group (0 - 255, up to 256 groups supported)
        /// </summary>
        public byte Group {
            get { return mGroup; }
            set { mGroup = value; NotifyUpdated(); }
        }

        /// <summary>
        /// ID of the flight mode (0 - 255, up to 256 modes supported)
        /// </summary>
        public byte Mode {
            get { return mMode; }
            set { mMode = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RGB red channel (0-255)
        /// </summary>
        public byte[] LedRed {
            get { return mLedRed; }
            set { mLedRed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RGB green channel (0-255)
        /// </summary>
        public byte[] LedBlue {
            get { return mLedBlue; }
            set { mLedBlue = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RGB blue channel (0-255)
        /// </summary>
        public byte[] LedGreen {
            get { return mLedGreen; }
            set { mLedGreen = value; NotifyUpdated(); }
        }

        public UasSetQuadSwarmLedRollPitchYawThrust()
        {
            mMessageId = 63;
            CrcExtra = 130;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SetQuadSwarmLedRollPitchYawThrust \n");
            sb.Append("    Roll\n");
            sb.AppendFormat("        [0]: {0}\n", mRoll[0]);
            sb.AppendFormat("        [1]: {0}\n", mRoll[1]);
            sb.AppendFormat("        [2]: {0}\n", mRoll[2]);
            sb.AppendFormat("        [3]: {0}\n", mRoll[3]);
            sb.Append("    Pitch\n");
            sb.AppendFormat("        [0]: {0}\n", mPitch[0]);
            sb.AppendFormat("        [1]: {0}\n", mPitch[1]);
            sb.AppendFormat("        [2]: {0}\n", mPitch[2]);
            sb.AppendFormat("        [3]: {0}\n", mPitch[3]);
            sb.Append("    Yaw\n");
            sb.AppendFormat("        [0]: {0}\n", mYaw[0]);
            sb.AppendFormat("        [1]: {0}\n", mYaw[1]);
            sb.AppendFormat("        [2]: {0}\n", mYaw[2]);
            sb.AppendFormat("        [3]: {0}\n", mYaw[3]);
            sb.Append("    Thrust\n");
            sb.AppendFormat("        [0]: {0}\n", mThrust[0]);
            sb.AppendFormat("        [1]: {0}\n", mThrust[1]);
            sb.AppendFormat("        [2]: {0}\n", mThrust[2]);
            sb.AppendFormat("        [3]: {0}\n", mThrust[3]);
            sb.AppendFormat("    Group: {0}\n", mGroup);
            sb.AppendFormat("    Mode: {0}\n", mMode);
            sb.Append("    LedRed\n");
            sb.AppendFormat("        [0]: {0}\n", mLedRed[0]);
            sb.AppendFormat("        [1]: {0}\n", mLedRed[1]);
            sb.AppendFormat("        [2]: {0}\n", mLedRed[2]);
            sb.AppendFormat("        [3]: {0}\n", mLedRed[3]);
            sb.Append("    LedBlue\n");
            sb.AppendFormat("        [0]: {0}\n", mLedBlue[0]);
            sb.AppendFormat("        [1]: {0}\n", mLedBlue[1]);
            sb.AppendFormat("        [2]: {0}\n", mLedBlue[2]);
            sb.AppendFormat("        [3]: {0}\n", mLedBlue[3]);
            sb.Append("    LedGreen\n");
            sb.AppendFormat("        [0]: {0}\n", mLedGreen[0]);
            sb.AppendFormat("        [1]: {0}\n", mLedGreen[1]);
            sb.AppendFormat("        [2]: {0}\n", mLedGreen[2]);
            sb.AppendFormat("        [3]: {0}\n", mLedGreen[3]);

            return sb.ToString();
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


    /// <summary>
    /// Corrects the systems state by adding an error correction term to the position and velocity, and by rotating the attitude by a correction angle.
    /// </summary>
    public class UasStateCorrection: UasMessage
    {
        /// <summary>
        /// x position error
        /// </summary>
        public float Xerr {
            get { return mXerr; }
            set { mXerr = value; NotifyUpdated(); }
        }

        /// <summary>
        /// y position error
        /// </summary>
        public float Yerr {
            get { return mYerr; }
            set { mYerr = value; NotifyUpdated(); }
        }

        /// <summary>
        /// z position error
        /// </summary>
        public float Zerr {
            get { return mZerr; }
            set { mZerr = value; NotifyUpdated(); }
        }

        /// <summary>
        /// roll error (radians)
        /// </summary>
        public float Rollerr {
            get { return mRollerr; }
            set { mRollerr = value; NotifyUpdated(); }
        }

        /// <summary>
        /// pitch error (radians)
        /// </summary>
        public float Pitcherr {
            get { return mPitcherr; }
            set { mPitcherr = value; NotifyUpdated(); }
        }

        /// <summary>
        /// yaw error (radians)
        /// </summary>
        public float Yawerr {
            get { return mYawerr; }
            set { mYawerr = value; NotifyUpdated(); }
        }

        /// <summary>
        /// x velocity
        /// </summary>
        public float Vxerr {
            get { return mVxerr; }
            set { mVxerr = value; NotifyUpdated(); }
        }

        /// <summary>
        /// y velocity
        /// </summary>
        public float Vyerr {
            get { return mVyerr; }
            set { mVyerr = value; NotifyUpdated(); }
        }

        /// <summary>
        /// z velocity
        /// </summary>
        public float Vzerr {
            get { return mVzerr; }
            set { mVzerr = value; NotifyUpdated(); }
        }

        public UasStateCorrection()
        {
            mMessageId = 64;
            CrcExtra = 130;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("StateCorrection \n");
            sb.AppendFormat("    Xerr: {0}\n", mXerr);
            sb.AppendFormat("    Yerr: {0}\n", mYerr);
            sb.AppendFormat("    Zerr: {0}\n", mZerr);
            sb.AppendFormat("    Rollerr: {0}\n", mRollerr);
            sb.AppendFormat("    Pitcherr: {0}\n", mPitcherr);
            sb.AppendFormat("    Yawerr: {0}\n", mYawerr);
            sb.AppendFormat("    Vxerr: {0}\n", mVxerr);
            sb.AppendFormat("    Vyerr: {0}\n", mVyerr);
            sb.AppendFormat("    Vzerr: {0}\n", mVzerr);

            return sb.ToString();
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
        /// <summary>
        /// The requested interval between two messages of this type
        /// </summary>
        public UInt16 ReqMessageRate {
            get { return mReqMessageRate; }
            set { mReqMessageRate = value; NotifyUpdated(); }
        }

        /// <summary>
        /// The target requested to send the message stream.
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// The target requested to send the message stream.
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        /// <summary>
        /// The ID of the requested data stream
        /// </summary>
        public byte ReqStreamId {
            get { return mReqStreamId; }
            set { mReqStreamId = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 1 to start sending, 0 to stop sending.
        /// </summary>
        public byte StartStop {
            get { return mStartStop; }
            set { mStartStop = value; NotifyUpdated(); }
        }

        public UasRequestDataStream()
        {
            mMessageId = 66;
            CrcExtra = 148;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("RequestDataStream \n");
            sb.AppendFormat("    ReqMessageRate: {0}\n", mReqMessageRate);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);
            sb.AppendFormat("    ReqStreamId: {0}\n", mReqStreamId);
            sb.AppendFormat("    StartStop: {0}\n", mStartStop);

            return sb.ToString();
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
        /// <summary>
        /// The requested interval between two messages of this type
        /// </summary>
        public UInt16 MessageRate {
            get { return mMessageRate; }
            set { mMessageRate = value; NotifyUpdated(); }
        }

        /// <summary>
        /// The ID of the requested data stream
        /// </summary>
        public byte StreamId {
            get { return mStreamId; }
            set { mStreamId = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 1 stream is enabled, 0 stream is stopped.
        /// </summary>
        public byte OnOff {
            get { return mOnOff; }
            set { mOnOff = value; NotifyUpdated(); }
        }

        public UasDataStream()
        {
            mMessageId = 67;
            CrcExtra = 21;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("DataStream \n");
            sb.AppendFormat("    MessageRate: {0}\n", mMessageRate);
            sb.AppendFormat("    StreamId: {0}\n", mStreamId);
            sb.AppendFormat("    OnOff: {0}\n", mOnOff);

            return sb.ToString();
        }

        private UInt16 mMessageRate;
        private byte mStreamId;
        private byte mOnOff;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their 
    /// </summary>
    public class UasManualControl: UasMessage
    {
        /// <summary>
        /// X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
        /// </summary>
        public Int16 X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
        /// </summary>
        public Int16 Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle.
        /// </summary>
        public Int16 Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
        /// </summary>
        public Int16 R {
            get { return mR; }
            set { mR = value; NotifyUpdated(); }
        }

        /// <summary>
        /// A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
        /// </summary>
        public UInt16 Buttons {
            get { return mButtons; }
            set { mButtons = value; NotifyUpdated(); }
        }

        /// <summary>
        /// The system to be controlled.
        /// </summary>
        public byte Target {
            get { return mTarget; }
            set { mTarget = value; NotifyUpdated(); }
        }

        public UasManualControl()
        {
            mMessageId = 69;
            CrcExtra = 243;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ManualControl \n");
            sb.AppendFormat("    X: {0}\n", mX);
            sb.AppendFormat("    Y: {0}\n", mY);
            sb.AppendFormat("    Z: {0}\n", mZ);
            sb.AppendFormat("    R: {0}\n", mR);
            sb.AppendFormat("    Buttons: {0}\n", mButtons);
            sb.AppendFormat("    Target: {0}\n", mTarget);

            return sb.ToString();
        }

        private Int16 mX;
        private Int16 mY;
        private Int16 mZ;
        private Int16 mR;
        private UInt16 mButtons;
        private byte mTarget;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    /// </summary>
    public class UasRcChannelsOverride: UasMessage
    {
        /// <summary>
        /// RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// </summary>
        public UInt16 Chan1Raw {
            get { return mChan1Raw; }
            set { mChan1Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// </summary>
        public UInt16 Chan2Raw {
            get { return mChan2Raw; }
            set { mChan2Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// </summary>
        public UInt16 Chan3Raw {
            get { return mChan3Raw; }
            set { mChan3Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// </summary>
        public UInt16 Chan4Raw {
            get { return mChan4Raw; }
            set { mChan4Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// </summary>
        public UInt16 Chan5Raw {
            get { return mChan5Raw; }
            set { mChan5Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// </summary>
        public UInt16 Chan6Raw {
            get { return mChan6Raw; }
            set { mChan6Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// </summary>
        public UInt16 Chan7Raw {
            get { return mChan7Raw; }
            set { mChan7Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
        /// </summary>
        public UInt16 Chan8Raw {
            get { return mChan8Raw; }
            set { mChan8Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component ID
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        public UasRcChannelsOverride()
        {
            mMessageId = 70;
            CrcExtra = 124;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("RcChannelsOverride \n");
            sb.AppendFormat("    Chan1Raw: {0}\n", mChan1Raw);
            sb.AppendFormat("    Chan2Raw: {0}\n", mChan2Raw);
            sb.AppendFormat("    Chan3Raw: {0}\n", mChan3Raw);
            sb.AppendFormat("    Chan4Raw: {0}\n", mChan4Raw);
            sb.AppendFormat("    Chan5Raw: {0}\n", mChan5Raw);
            sb.AppendFormat("    Chan6Raw: {0}\n", mChan6Raw);
            sb.AppendFormat("    Chan7Raw: {0}\n", mChan7Raw);
            sb.AppendFormat("    Chan8Raw: {0}\n", mChan8Raw);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);

            return sb.ToString();
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


    /// <summary>
    /// Metrics typically displayed on a HUD for fixed wing aircraft
    /// </summary>
    public class UasVfrHud: UasMessage
    {
        /// <summary>
        /// Current airspeed in m/s
        /// </summary>
        public float Airspeed {
            get { return mAirspeed; }
            set { mAirspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Current ground speed in m/s
        /// </summary>
        public float Groundspeed {
            get { return mGroundspeed; }
            set { mGroundspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Current altitude (MSL), in meters
        /// </summary>
        public float Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Current climb rate in meters/second
        /// </summary>
        public float Climb {
            get { return mClimb; }
            set { mClimb = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Current heading in degrees, in compass units (0..360, 0=north)
        /// </summary>
        public Int16 Heading {
            get { return mHeading; }
            set { mHeading = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Current throttle setting in integer percent, 0 to 100
        /// </summary>
        public UInt16 Throttle {
            get { return mThrottle; }
            set { mThrottle = value; NotifyUpdated(); }
        }

        public UasVfrHud()
        {
            mMessageId = 74;
            CrcExtra = 20;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("VfrHud \n");
            sb.AppendFormat("    Airspeed: {0}\n", mAirspeed);
            sb.AppendFormat("    Groundspeed: {0}\n", mGroundspeed);
            sb.AppendFormat("    Alt: {0}\n", mAlt);
            sb.AppendFormat("    Climb: {0}\n", mClimb);
            sb.AppendFormat("    Heading: {0}\n", mHeading);
            sb.AppendFormat("    Throttle: {0}\n", mThrottle);

            return sb.ToString();
        }

        private float mAirspeed;
        private float mGroundspeed;
        private float mAlt;
        private float mClimb;
        private Int16 mHeading;
        private UInt16 mThrottle;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Send a command with up to seven parameters to the MAV
    /// </summary>
    public class UasCommandLong: UasMessage
    {
        /// <summary>
        /// Parameter 1, as defined by MAV_CMD enum.
        /// </summary>
        public float Param1 {
            get { return mParam1; }
            set { mParam1 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Parameter 2, as defined by MAV_CMD enum.
        /// </summary>
        public float Param2 {
            get { return mParam2; }
            set { mParam2 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Parameter 3, as defined by MAV_CMD enum.
        /// </summary>
        public float Param3 {
            get { return mParam3; }
            set { mParam3 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Parameter 4, as defined by MAV_CMD enum.
        /// </summary>
        public float Param4 {
            get { return mParam4; }
            set { mParam4 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Parameter 5, as defined by MAV_CMD enum.
        /// </summary>
        public float Param5 {
            get { return mParam5; }
            set { mParam5 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Parameter 6, as defined by MAV_CMD enum.
        /// </summary>
        public float Param6 {
            get { return mParam6; }
            set { mParam6 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Parameter 7, as defined by MAV_CMD enum.
        /// </summary>
        public float Param7 {
            get { return mParam7; }
            set { mParam7 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Command ID, as defined by MAV_CMD enum.
        /// </summary>
        public MavCmd Command {
            get { return mCommand; }
            set { mCommand = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System which should execute the command
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Component which should execute the command, 0 for all components
        /// </summary>
        public byte TargetComponent {
            get { return mTargetComponent; }
            set { mTargetComponent = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
        /// </summary>
        public byte Confirmation {
            get { return mConfirmation; }
            set { mConfirmation = value; NotifyUpdated(); }
        }

        public UasCommandLong()
        {
            mMessageId = 76;
            CrcExtra = 152;
        }

        internal override void SerializeBody(BinaryWriter s)
        {
            s.Write(mParam1);
            s.Write(mParam2);
            s.Write(mParam3);
            s.Write(mParam4);
            s.Write(mParam5);
            s.Write(mParam6);
            s.Write(mParam7);
            s.Write((UInt16)mCommand);
            s.Write(mTargetSystem);
            s.Write(mTargetComponent);
            s.Write(mConfirmation);
        }

        internal override void DeserializeBody(BinaryReader s)
        {
            this.mParam1 = s.ReadSingle();
            this.mParam2 = s.ReadSingle();
            this.mParam3 = s.ReadSingle();
            this.mParam4 = s.ReadSingle();
            this.mParam5 = s.ReadSingle();
            this.mParam6 = s.ReadSingle();
            this.mParam7 = s.ReadSingle();
            this.mCommand = (MavCmd)s.ReadUInt16();
            this.mTargetSystem = s.ReadByte();
            this.mTargetComponent = s.ReadByte();
            this.mConfirmation = s.ReadByte();
        }

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("CommandLong \n");
            sb.AppendFormat("    Param1: {0}\n", mParam1);
            sb.AppendFormat("    Param2: {0}\n", mParam2);
            sb.AppendFormat("    Param3: {0}\n", mParam3);
            sb.AppendFormat("    Param4: {0}\n", mParam4);
            sb.AppendFormat("    Param5: {0}\n", mParam5);
            sb.AppendFormat("    Param6: {0}\n", mParam6);
            sb.AppendFormat("    Param7: {0}\n", mParam7);
            sb.AppendFormat("    Command: {0}\n", mCommand);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);
            sb.AppendFormat("    TargetComponent: {0}\n", mTargetComponent);
            sb.AppendFormat("    Confirmation: {0}\n", mConfirmation);

            return sb.ToString();
        }

        private float mParam1;
        private float mParam2;
        private float mParam3;
        private float mParam4;
        private float mParam5;
        private float mParam6;
        private float mParam7;
        private MavCmd mCommand;
        private byte mTargetSystem;
        private byte mTargetComponent;
        private byte mConfirmation;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Report status of a command. Includes feedback wether the command was executed.
    /// </summary>
    public class UasCommandAck: UasMessage
    {
        /// <summary>
        /// Command ID, as defined by MAV_CMD enum.
        /// </summary>
        public MavCmd Command {
            get { return mCommand; }
            set { mCommand = value; NotifyUpdated(); }
        }

        /// <summary>
        /// See MAV_RESULT enum
        /// </summary>
        public MavResult Result {
            get { return mResult; }
            set { mResult = value; NotifyUpdated(); }
        }

        public UasCommandAck()
        {
            mMessageId = 77;
            CrcExtra = 143;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("CommandAck \n");
            sb.AppendFormat("    Command: {0}\n", mCommand);
            sb.AppendFormat("    Result: {0}\n", mResult);

            return sb.ToString();
        }

        private MavCmd mCommand;
        private MavResult mResult;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Setpoint in roll, pitch, yaw rates and thrust currently active on the system.
    /// </summary>
    public class UasRollPitchYawRatesThrustSetpoint: UasMessage
    {
        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired roll rate in radians per second
        /// </summary>
        public float RollRate {
            get { return mRollRate; }
            set { mRollRate = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired pitch rate in radians per second
        /// </summary>
        public float PitchRate {
            get { return mPitchRate; }
            set { mPitchRate = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw rate in radians per second
        /// </summary>
        public float YawRate {
            get { return mYawRate; }
            set { mYawRate = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Collective thrust, normalized to 0 .. 1
        /// </summary>
        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        public UasRollPitchYawRatesThrustSetpoint()
        {
            mMessageId = 80;
            CrcExtra = 127;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("RollPitchYawRatesThrustSetpoint \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    RollRate: {0}\n", mRollRate);
            sb.AppendFormat("    PitchRate: {0}\n", mPitchRate);
            sb.AppendFormat("    YawRate: {0}\n", mYawRate);
            sb.AppendFormat("    Thrust: {0}\n", mThrust);

            return sb.ToString();
        }

        private UInt32 mTimeBootMs;
        private float mRollRate;
        private float mPitchRate;
        private float mYawRate;
        private float mThrust;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Setpoint in roll, pitch, yaw and thrust from the operator
    /// </summary>
    public class UasManualSetpoint: UasMessage
    {
        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired roll rate in radians per second
        /// </summary>
        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired pitch rate in radians per second
        /// </summary>
        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Desired yaw rate in radians per second
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Collective thrust, normalized to 0 .. 1
        /// </summary>
        public float Thrust {
            get { return mThrust; }
            set { mThrust = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Flight mode switch position, 0.. 255
        /// </summary>
        public byte ModeSwitch {
            get { return mModeSwitch; }
            set { mModeSwitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Override mode switch position, 0.. 255
        /// </summary>
        public byte ManualOverrideSwitch {
            get { return mManualOverrideSwitch; }
            set { mManualOverrideSwitch = value; NotifyUpdated(); }
        }

        public UasManualSetpoint()
        {
            mMessageId = 81;
            CrcExtra = 106;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ManualSetpoint \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    Roll: {0}\n", mRoll);
            sb.AppendFormat("    Pitch: {0}\n", mPitch);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);
            sb.AppendFormat("    Thrust: {0}\n", mThrust);
            sb.AppendFormat("    ModeSwitch: {0}\n", mModeSwitch);
            sb.AppendFormat("    ManualOverrideSwitch: {0}\n", mManualOverrideSwitch);

            return sb.ToString();
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


    /// <summary>
    /// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    /// </summary>
    public class UasLocalPositionNedSystemGlobalOffset: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X Position
        /// </summary>
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y Position
        /// </summary>
        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z Position
        /// </summary>
        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Roll
        /// </summary>
        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Pitch
        /// </summary>
        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Yaw
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public UasLocalPositionNedSystemGlobalOffset()
        {
            mMessageId = 89;
            CrcExtra = 231;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("LocalPositionNedSystemGlobalOffset \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    X: {0}\n", mX);
            sb.AppendFormat("    Y: {0}\n", mY);
            sb.AppendFormat("    Z: {0}\n", mZ);
            sb.AppendFormat("    Roll: {0}\n", mRoll);
            sb.AppendFormat("    Pitch: {0}\n", mPitch);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);

            return sb.ToString();
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


    /// <summary>
    /// DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
    /// </summary>
    public class UasHilState: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Roll angle (rad)
        /// </summary>
        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Pitch angle (rad)
        /// </summary>
        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Yaw angle (rad)
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Body frame roll / phi angular speed (rad/s)
        /// </summary>
        public float Rollspeed {
            get { return mRollspeed; }
            set { mRollspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Body frame pitch / theta angular speed (rad/s)
        /// </summary>
        public float Pitchspeed {
            get { return mPitchspeed; }
            set { mPitchspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Body frame yaw / psi angular speed (rad/s)
        /// </summary>
        public float Yawspeed {
            get { return mYawspeed; }
            set { mYawspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Latitude, expressed as * 1E7
        /// </summary>
        public Int32 Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Longitude, expressed as * 1E7
        /// </summary>
        public Int32 Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude in meters, expressed as * 1000 (millimeters)
        /// </summary>
        public Int32 Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Ground X Speed (Latitude), expressed as m/s * 100
        /// </summary>
        public Int16 Vx {
            get { return mVx; }
            set { mVx = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Ground Y Speed (Longitude), expressed as m/s * 100
        /// </summary>
        public Int16 Vy {
            get { return mVy; }
            set { mVy = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Ground Z Speed (Altitude), expressed as m/s * 100
        /// </summary>
        public Int16 Vz {
            get { return mVz; }
            set { mVz = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X acceleration (mg)
        /// </summary>
        public Int16 Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y acceleration (mg)
        /// </summary>
        public Int16 Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z acceleration (mg)
        /// </summary>
        public Int16 Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        public UasHilState()
        {
            mMessageId = 90;
            CrcExtra = 183;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("HilState \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    Roll: {0}\n", mRoll);
            sb.AppendFormat("    Pitch: {0}\n", mPitch);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);
            sb.AppendFormat("    Rollspeed: {0}\n", mRollspeed);
            sb.AppendFormat("    Pitchspeed: {0}\n", mPitchspeed);
            sb.AppendFormat("    Yawspeed: {0}\n", mYawspeed);
            sb.AppendFormat("    Lat: {0}\n", mLat);
            sb.AppendFormat("    Lon: {0}\n", mLon);
            sb.AppendFormat("    Alt: {0}\n", mAlt);
            sb.AppendFormat("    Vx: {0}\n", mVx);
            sb.AppendFormat("    Vy: {0}\n", mVy);
            sb.AppendFormat("    Vz: {0}\n", mVz);
            sb.AppendFormat("    Xacc: {0}\n", mXacc);
            sb.AppendFormat("    Yacc: {0}\n", mYacc);
            sb.AppendFormat("    Zacc: {0}\n", mZacc);

            return sb.ToString();
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


    /// <summary>
    /// Sent from autopilot to simulation. Hardware in the loop control outputs
    /// </summary>
    public class UasHilControls: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Control output -1 .. 1
        /// </summary>
        public float RollAilerons {
            get { return mRollAilerons; }
            set { mRollAilerons = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Control output -1 .. 1
        /// </summary>
        public float PitchElevator {
            get { return mPitchElevator; }
            set { mPitchElevator = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Control output -1 .. 1
        /// </summary>
        public float YawRudder {
            get { return mYawRudder; }
            set { mYawRudder = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Throttle 0 .. 1
        /// </summary>
        public float Throttle {
            get { return mThrottle; }
            set { mThrottle = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Aux 1, -1 .. 1
        /// </summary>
        public float Aux1 {
            get { return mAux1; }
            set { mAux1 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Aux 2, -1 .. 1
        /// </summary>
        public float Aux2 {
            get { return mAux2; }
            set { mAux2 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Aux 3, -1 .. 1
        /// </summary>
        public float Aux3 {
            get { return mAux3; }
            set { mAux3 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Aux 4, -1 .. 1
        /// </summary>
        public float Aux4 {
            get { return mAux4; }
            set { mAux4 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System mode (MAV_MODE)
        /// </summary>
        public MavMode Mode {
            get { return mMode; }
            set { mMode = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Navigation mode (MAV_NAV_MODE)
        /// </summary>
        public byte NavMode {
            get { return mNavMode; }
            set { mNavMode = value; NotifyUpdated(); }
        }

        public UasHilControls()
        {
            mMessageId = 91;
            CrcExtra = 63;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("HilControls \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    RollAilerons: {0}\n", mRollAilerons);
            sb.AppendFormat("    PitchElevator: {0}\n", mPitchElevator);
            sb.AppendFormat("    YawRudder: {0}\n", mYawRudder);
            sb.AppendFormat("    Throttle: {0}\n", mThrottle);
            sb.AppendFormat("    Aux1: {0}\n", mAux1);
            sb.AppendFormat("    Aux2: {0}\n", mAux2);
            sb.AppendFormat("    Aux3: {0}\n", mAux3);
            sb.AppendFormat("    Aux4: {0}\n", mAux4);
            sb.AppendFormat("    Mode: {0}\n", mMode);
            sb.AppendFormat("    NavMode: {0}\n", mNavMode);

            return sb.ToString();
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


    /// <summary>
    /// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
    /// </summary>
    public class UasHilRcInputsRaw: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 1 value, in microseconds
        /// </summary>
        public UInt16 Chan1Raw {
            get { return mChan1Raw; }
            set { mChan1Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 2 value, in microseconds
        /// </summary>
        public UInt16 Chan2Raw {
            get { return mChan2Raw; }
            set { mChan2Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 3 value, in microseconds
        /// </summary>
        public UInt16 Chan3Raw {
            get { return mChan3Raw; }
            set { mChan3Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 4 value, in microseconds
        /// </summary>
        public UInt16 Chan4Raw {
            get { return mChan4Raw; }
            set { mChan4Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 5 value, in microseconds
        /// </summary>
        public UInt16 Chan5Raw {
            get { return mChan5Raw; }
            set { mChan5Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 6 value, in microseconds
        /// </summary>
        public UInt16 Chan6Raw {
            get { return mChan6Raw; }
            set { mChan6Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 7 value, in microseconds
        /// </summary>
        public UInt16 Chan7Raw {
            get { return mChan7Raw; }
            set { mChan7Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 8 value, in microseconds
        /// </summary>
        public UInt16 Chan8Raw {
            get { return mChan8Raw; }
            set { mChan8Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 9 value, in microseconds
        /// </summary>
        public UInt16 Chan9Raw {
            get { return mChan9Raw; }
            set { mChan9Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 10 value, in microseconds
        /// </summary>
        public UInt16 Chan10Raw {
            get { return mChan10Raw; }
            set { mChan10Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 11 value, in microseconds
        /// </summary>
        public UInt16 Chan11Raw {
            get { return mChan11Raw; }
            set { mChan11Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RC channel 12 value, in microseconds
        /// </summary>
        public UInt16 Chan12Raw {
            get { return mChan12Raw; }
            set { mChan12Raw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Receive signal strength indicator, 0: 0%, 255: 100%
        /// </summary>
        public byte Rssi {
            get { return mRssi; }
            set { mRssi = value; NotifyUpdated(); }
        }

        public UasHilRcInputsRaw()
        {
            mMessageId = 92;
            CrcExtra = 54;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("HilRcInputsRaw \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    Chan1Raw: {0}\n", mChan1Raw);
            sb.AppendFormat("    Chan2Raw: {0}\n", mChan2Raw);
            sb.AppendFormat("    Chan3Raw: {0}\n", mChan3Raw);
            sb.AppendFormat("    Chan4Raw: {0}\n", mChan4Raw);
            sb.AppendFormat("    Chan5Raw: {0}\n", mChan5Raw);
            sb.AppendFormat("    Chan6Raw: {0}\n", mChan6Raw);
            sb.AppendFormat("    Chan7Raw: {0}\n", mChan7Raw);
            sb.AppendFormat("    Chan8Raw: {0}\n", mChan8Raw);
            sb.AppendFormat("    Chan9Raw: {0}\n", mChan9Raw);
            sb.AppendFormat("    Chan10Raw: {0}\n", mChan10Raw);
            sb.AppendFormat("    Chan11Raw: {0}\n", mChan11Raw);
            sb.AppendFormat("    Chan12Raw: {0}\n", mChan12Raw);
            sb.AppendFormat("    Rssi: {0}\n", mRssi);

            return sb.ToString();
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


    /// <summary>
    /// Optical flow from a flow sensor (e.g. optical mouse sensor)
    /// </summary>
    public class UasOpticalFlow: UasMessage
    {
        /// <summary>
        /// Timestamp (UNIX)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Flow in meters in x-sensor direction, angular-speed compensated
        /// </summary>
        public float FlowCompMX {
            get { return mFlowCompMX; }
            set { mFlowCompMX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Flow in meters in y-sensor direction, angular-speed compensated
        /// </summary>
        public float FlowCompMY {
            get { return mFlowCompMY; }
            set { mFlowCompMY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
        /// </summary>
        public float GroundDistance {
            get { return mGroundDistance; }
            set { mGroundDistance = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Flow in pixels * 10 in x-sensor direction (dezi-pixels)
        /// </summary>
        public Int16 FlowX {
            get { return mFlowX; }
            set { mFlowX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Flow in pixels * 10 in y-sensor direction (dezi-pixels)
        /// </summary>
        public Int16 FlowY {
            get { return mFlowY; }
            set { mFlowY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Sensor ID
        /// </summary>
        public byte SensorId {
            get { return mSensorId; }
            set { mSensorId = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Optical flow quality / confidence. 0: bad, 255: maximum quality
        /// </summary>
        public byte Quality {
            get { return mQuality; }
            set { mQuality = value; NotifyUpdated(); }
        }

        public UasOpticalFlow()
        {
            mMessageId = 100;
            CrcExtra = 175;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("OpticalFlow \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    FlowCompMX: {0}\n", mFlowCompMX);
            sb.AppendFormat("    FlowCompMY: {0}\n", mFlowCompMY);
            sb.AppendFormat("    GroundDistance: {0}\n", mGroundDistance);
            sb.AppendFormat("    FlowX: {0}\n", mFlowX);
            sb.AppendFormat("    FlowY: {0}\n", mFlowY);
            sb.AppendFormat("    SensorId: {0}\n", mSensorId);
            sb.AppendFormat("    Quality: {0}\n", mQuality);

            return sb.ToString();
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
        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// </summary>
        public UInt64 Usec {
            get { return mUsec; }
            set { mUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global X position
        /// </summary>
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global Y position
        /// </summary>
        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global Z position
        /// </summary>
        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Roll angle in rad
        /// </summary>
        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Pitch angle in rad
        /// </summary>
        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Yaw angle in rad
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public UasGlobalVisionPositionEstimate()
        {
            mMessageId = 101;
            CrcExtra = 102;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("GlobalVisionPositionEstimate \n");
            sb.AppendFormat("    Usec: {0}\n", mUsec);
            sb.AppendFormat("    X: {0}\n", mX);
            sb.AppendFormat("    Y: {0}\n", mY);
            sb.AppendFormat("    Z: {0}\n", mZ);
            sb.AppendFormat("    Roll: {0}\n", mRoll);
            sb.AppendFormat("    Pitch: {0}\n", mPitch);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);

            return sb.ToString();
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
        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// </summary>
        public UInt64 Usec {
            get { return mUsec; }
            set { mUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global X position
        /// </summary>
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global Y position
        /// </summary>
        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global Z position
        /// </summary>
        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Roll angle in rad
        /// </summary>
        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Pitch angle in rad
        /// </summary>
        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Yaw angle in rad
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public UasVisionPositionEstimate()
        {
            mMessageId = 102;
            CrcExtra = 158;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("VisionPositionEstimate \n");
            sb.AppendFormat("    Usec: {0}\n", mUsec);
            sb.AppendFormat("    X: {0}\n", mX);
            sb.AppendFormat("    Y: {0}\n", mY);
            sb.AppendFormat("    Z: {0}\n", mZ);
            sb.AppendFormat("    Roll: {0}\n", mRoll);
            sb.AppendFormat("    Pitch: {0}\n", mPitch);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);

            return sb.ToString();
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
        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// </summary>
        public UInt64 Usec {
            get { return mUsec; }
            set { mUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global X speed
        /// </summary>
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global Y speed
        /// </summary>
        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global Z speed
        /// </summary>
        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        public UasVisionSpeedEstimate()
        {
            mMessageId = 103;
            CrcExtra = 208;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("VisionSpeedEstimate \n");
            sb.AppendFormat("    Usec: {0}\n", mUsec);
            sb.AppendFormat("    X: {0}\n", mX);
            sb.AppendFormat("    Y: {0}\n", mY);
            sb.AppendFormat("    Z: {0}\n", mZ);

            return sb.ToString();
        }

        private UInt64 mUsec;
        private float mX;
        private float mY;
        private float mZ;
    }


    // ___________________________________________________________________________________


    public class UasViconPositionEstimate: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// </summary>
        public UInt64 Usec {
            get { return mUsec; }
            set { mUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global X position
        /// </summary>
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global Y position
        /// </summary>
        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Global Z position
        /// </summary>
        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Roll angle in rad
        /// </summary>
        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Pitch angle in rad
        /// </summary>
        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Yaw angle in rad
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        public UasViconPositionEstimate()
        {
            mMessageId = 104;
            CrcExtra = 56;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("ViconPositionEstimate \n");
            sb.AppendFormat("    Usec: {0}\n", mUsec);
            sb.AppendFormat("    X: {0}\n", mX);
            sb.AppendFormat("    Y: {0}\n", mY);
            sb.AppendFormat("    Z: {0}\n", mZ);
            sb.AppendFormat("    Roll: {0}\n", mRoll);
            sb.AppendFormat("    Pitch: {0}\n", mPitch);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);

            return sb.ToString();
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


    /// <summary>
    /// The IMU readings in SI units in NED body frame
    /// </summary>
    public class UasHighresImu: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X acceleration (m/s^2)
        /// </summary>
        public float Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y acceleration (m/s^2)
        /// </summary>
        public float Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z acceleration (m/s^2)
        /// </summary>
        public float Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around X axis (rad / sec)
        /// </summary>
        public float Xgyro {
            get { return mXgyro; }
            set { mXgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around Y axis (rad / sec)
        /// </summary>
        public float Ygyro {
            get { return mYgyro; }
            set { mYgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around Z axis (rad / sec)
        /// </summary>
        public float Zgyro {
            get { return mZgyro; }
            set { mZgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X Magnetic field (Gauss)
        /// </summary>
        public float Xmag {
            get { return mXmag; }
            set { mXmag = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y Magnetic field (Gauss)
        /// </summary>
        public float Ymag {
            get { return mYmag; }
            set { mYmag = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z Magnetic field (Gauss)
        /// </summary>
        public float Zmag {
            get { return mZmag; }
            set { mZmag = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Absolute pressure in millibar
        /// </summary>
        public float AbsPressure {
            get { return mAbsPressure; }
            set { mAbsPressure = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Differential pressure in millibar
        /// </summary>
        public float DiffPressure {
            get { return mDiffPressure; }
            set { mDiffPressure = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude calculated from pressure
        /// </summary>
        public float PressureAlt {
            get { return mPressureAlt; }
            set { mPressureAlt = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Temperature in degrees celsius
        /// </summary>
        public float Temperature {
            get { return mTemperature; }
            set { mTemperature = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
        /// </summary>
        public UInt16 FieldsUpdated {
            get { return mFieldsUpdated; }
            set { mFieldsUpdated = value; NotifyUpdated(); }
        }

        public UasHighresImu()
        {
            mMessageId = 105;
            CrcExtra = 93;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("HighresImu \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    Xacc: {0}\n", mXacc);
            sb.AppendFormat("    Yacc: {0}\n", mYacc);
            sb.AppendFormat("    Zacc: {0}\n", mZacc);
            sb.AppendFormat("    Xgyro: {0}\n", mXgyro);
            sb.AppendFormat("    Ygyro: {0}\n", mYgyro);
            sb.AppendFormat("    Zgyro: {0}\n", mZgyro);
            sb.AppendFormat("    Xmag: {0}\n", mXmag);
            sb.AppendFormat("    Ymag: {0}\n", mYmag);
            sb.AppendFormat("    Zmag: {0}\n", mZmag);
            sb.AppendFormat("    AbsPressure: {0}\n", mAbsPressure);
            sb.AppendFormat("    DiffPressure: {0}\n", mDiffPressure);
            sb.AppendFormat("    PressureAlt: {0}\n", mPressureAlt);
            sb.AppendFormat("    Temperature: {0}\n", mTemperature);
            sb.AppendFormat("    FieldsUpdated: {0}\n", mFieldsUpdated);

            return sb.ToString();
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


    /// <summary>
    /// Optical flow from an omnidirectional flow sensor (e.g. PX4FLOW with wide angle lens)
    /// </summary>
    public class UasOmnidirectionalFlow: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Front distance in meters. Positive value (including zero): distance known. Negative value: Unknown distance
        /// </summary>
        public float FrontDistanceM {
            get { return mFrontDistanceM; }
            set { mFrontDistanceM = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Flow in deci pixels (1 = 0.1 pixel) on left hemisphere
        /// </summary>
        public Int16[] Left {
            get { return mLeft; }
            set { mLeft = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Flow in deci pixels (1 = 0.1 pixel) on right hemisphere
        /// </summary>
        public Int16[] Right {
            get { return mRight; }
            set { mRight = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Sensor ID
        /// </summary>
        public byte SensorId {
            get { return mSensorId; }
            set { mSensorId = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Optical flow quality / confidence. 0: bad, 255: maximum quality
        /// </summary>
        public byte Quality {
            get { return mQuality; }
            set { mQuality = value; NotifyUpdated(); }
        }

        public UasOmnidirectionalFlow()
        {
            mMessageId = 106;
            CrcExtra = 211;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("OmnidirectionalFlow \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    FrontDistanceM: {0}\n", mFrontDistanceM);
            sb.Append("    Left\n");
            sb.AppendFormat("        [0]: {0}\n", mLeft[0]);
            sb.AppendFormat("        [1]: {0}\n", mLeft[1]);
            sb.AppendFormat("        [2]: {0}\n", mLeft[2]);
            sb.AppendFormat("        [3]: {0}\n", mLeft[3]);
            sb.AppendFormat("        [4]: {0}\n", mLeft[4]);
            sb.AppendFormat("        [5]: {0}\n", mLeft[5]);
            sb.AppendFormat("        [6]: {0}\n", mLeft[6]);
            sb.AppendFormat("        [7]: {0}\n", mLeft[7]);
            sb.AppendFormat("        [8]: {0}\n", mLeft[8]);
            sb.AppendFormat("        [9]: {0}\n", mLeft[9]);
            sb.Append("    Right\n");
            sb.AppendFormat("        [0]: {0}\n", mRight[0]);
            sb.AppendFormat("        [1]: {0}\n", mRight[1]);
            sb.AppendFormat("        [2]: {0}\n", mRight[2]);
            sb.AppendFormat("        [3]: {0}\n", mRight[3]);
            sb.AppendFormat("        [4]: {0}\n", mRight[4]);
            sb.AppendFormat("        [5]: {0}\n", mRight[5]);
            sb.AppendFormat("        [6]: {0}\n", mRight[6]);
            sb.AppendFormat("        [7]: {0}\n", mRight[7]);
            sb.AppendFormat("        [8]: {0}\n", mRight[8]);
            sb.AppendFormat("        [9]: {0}\n", mRight[9]);
            sb.AppendFormat("    SensorId: {0}\n", mSensorId);
            sb.AppendFormat("    Quality: {0}\n", mQuality);

            return sb.ToString();
        }

        private UInt64 mTimeUsec;
        private float mFrontDistanceM;
        private Int16[] mLeft = new Int16[10];
        private Int16[] mRight = new Int16[10];
        private byte mSensorId;
        private byte mQuality;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// The IMU readings in SI units in NED body frame
    /// </summary>
    public class UasHilSensor: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds, synced to UNIX time or since system boot)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X acceleration (m/s^2)
        /// </summary>
        public float Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y acceleration (m/s^2)
        /// </summary>
        public float Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z acceleration (m/s^2)
        /// </summary>
        public float Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around X axis in body frame (rad / sec)
        /// </summary>
        public float Xgyro {
            get { return mXgyro; }
            set { mXgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around Y axis in body frame (rad / sec)
        /// </summary>
        public float Ygyro {
            get { return mYgyro; }
            set { mYgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around Z axis in body frame (rad / sec)
        /// </summary>
        public float Zgyro {
            get { return mZgyro; }
            set { mZgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X Magnetic field (Gauss)
        /// </summary>
        public float Xmag {
            get { return mXmag; }
            set { mXmag = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y Magnetic field (Gauss)
        /// </summary>
        public float Ymag {
            get { return mYmag; }
            set { mYmag = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z Magnetic field (Gauss)
        /// </summary>
        public float Zmag {
            get { return mZmag; }
            set { mZmag = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Absolute pressure in millibar
        /// </summary>
        public float AbsPressure {
            get { return mAbsPressure; }
            set { mAbsPressure = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Differential pressure (airspeed) in millibar
        /// </summary>
        public float DiffPressure {
            get { return mDiffPressure; }
            set { mDiffPressure = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude calculated from pressure
        /// </summary>
        public float PressureAlt {
            get { return mPressureAlt; }
            set { mPressureAlt = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Temperature in degrees celsius
        /// </summary>
        public float Temperature {
            get { return mTemperature; }
            set { mTemperature = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
        /// </summary>
        public UInt32 FieldsUpdated {
            get { return mFieldsUpdated; }
            set { mFieldsUpdated = value; NotifyUpdated(); }
        }

        public UasHilSensor()
        {
            mMessageId = 107;
            CrcExtra = 108;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("HilSensor \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    Xacc: {0}\n", mXacc);
            sb.AppendFormat("    Yacc: {0}\n", mYacc);
            sb.AppendFormat("    Zacc: {0}\n", mZacc);
            sb.AppendFormat("    Xgyro: {0}\n", mXgyro);
            sb.AppendFormat("    Ygyro: {0}\n", mYgyro);
            sb.AppendFormat("    Zgyro: {0}\n", mZgyro);
            sb.AppendFormat("    Xmag: {0}\n", mXmag);
            sb.AppendFormat("    Ymag: {0}\n", mYmag);
            sb.AppendFormat("    Zmag: {0}\n", mZmag);
            sb.AppendFormat("    AbsPressure: {0}\n", mAbsPressure);
            sb.AppendFormat("    DiffPressure: {0}\n", mDiffPressure);
            sb.AppendFormat("    PressureAlt: {0}\n", mPressureAlt);
            sb.AppendFormat("    Temperature: {0}\n", mTemperature);
            sb.AppendFormat("    FieldsUpdated: {0}\n", mFieldsUpdated);

            return sb.ToString();
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


    /// <summary>
    /// Status of simulation environment, if used
    /// </summary>
    public class UasSimState: UasMessage
    {
        /// <summary>
        /// True attitude quaternion component 1
        /// </summary>
        public float Q1 {
            get { return mQ1; }
            set { mQ1 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// True attitude quaternion component 2
        /// </summary>
        public float Q2 {
            get { return mQ2; }
            set { mQ2 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// True attitude quaternion component 3
        /// </summary>
        public float Q3 {
            get { return mQ3; }
            set { mQ3 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// True attitude quaternion component 4
        /// </summary>
        public float Q4 {
            get { return mQ4; }
            set { mQ4 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
        /// </summary>
        public float Roll {
            get { return mRoll; }
            set { mRoll = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
        /// </summary>
        public float Pitch {
            get { return mPitch; }
            set { mPitch = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
        /// </summary>
        public float Yaw {
            get { return mYaw; }
            set { mYaw = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X acceleration m/s/s
        /// </summary>
        public float Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y acceleration m/s/s
        /// </summary>
        public float Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z acceleration m/s/s
        /// </summary>
        public float Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around X axis rad/s
        /// </summary>
        public float Xgyro {
            get { return mXgyro; }
            set { mXgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around Y axis rad/s
        /// </summary>
        public float Ygyro {
            get { return mYgyro; }
            set { mYgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Angular speed around Z axis rad/s
        /// </summary>
        public float Zgyro {
            get { return mZgyro; }
            set { mZgyro = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Latitude in degrees
        /// </summary>
        public float Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Longitude in degrees
        /// </summary>
        public float Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude in meters
        /// </summary>
        public float Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Horizontal position standard deviation
        /// </summary>
        public float StdDevHorz {
            get { return mStdDevHorz; }
            set { mStdDevHorz = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Vertical position standard deviation
        /// </summary>
        public float StdDevVert {
            get { return mStdDevVert; }
            set { mStdDevVert = value; NotifyUpdated(); }
        }

        /// <summary>
        /// True velocity in m/s in NORTH direction in earth-fixed NED frame
        /// </summary>
        public float Vn {
            get { return mVn; }
            set { mVn = value; NotifyUpdated(); }
        }

        /// <summary>
        /// True velocity in m/s in EAST direction in earth-fixed NED frame
        /// </summary>
        public float Ve {
            get { return mVe; }
            set { mVe = value; NotifyUpdated(); }
        }

        /// <summary>
        /// True velocity in m/s in DOWN direction in earth-fixed NED frame
        /// </summary>
        public float Vd {
            get { return mVd; }
            set { mVd = value; NotifyUpdated(); }
        }

        public UasSimState()
        {
            mMessageId = 108;
            CrcExtra = 32;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("SimState \n");
            sb.AppendFormat("    Q1: {0}\n", mQ1);
            sb.AppendFormat("    Q2: {0}\n", mQ2);
            sb.AppendFormat("    Q3: {0}\n", mQ3);
            sb.AppendFormat("    Q4: {0}\n", mQ4);
            sb.AppendFormat("    Roll: {0}\n", mRoll);
            sb.AppendFormat("    Pitch: {0}\n", mPitch);
            sb.AppendFormat("    Yaw: {0}\n", mYaw);
            sb.AppendFormat("    Xacc: {0}\n", mXacc);
            sb.AppendFormat("    Yacc: {0}\n", mYacc);
            sb.AppendFormat("    Zacc: {0}\n", mZacc);
            sb.AppendFormat("    Xgyro: {0}\n", mXgyro);
            sb.AppendFormat("    Ygyro: {0}\n", mYgyro);
            sb.AppendFormat("    Zgyro: {0}\n", mZgyro);
            sb.AppendFormat("    Lat: {0}\n", mLat);
            sb.AppendFormat("    Lon: {0}\n", mLon);
            sb.AppendFormat("    Alt: {0}\n", mAlt);
            sb.AppendFormat("    StdDevHorz: {0}\n", mStdDevHorz);
            sb.AppendFormat("    StdDevVert: {0}\n", mStdDevVert);
            sb.AppendFormat("    Vn: {0}\n", mVn);
            sb.AppendFormat("    Ve: {0}\n", mVe);
            sb.AppendFormat("    Vd: {0}\n", mVd);

            return sb.ToString();
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


    /// <summary>
    /// Status generated by radio
    /// </summary>
    public class UasRadioStatus: UasMessage
    {
        /// <summary>
        /// receive errors
        /// </summary>
        public UInt16 Rxerrors {
            get { return mRxerrors; }
            set { mRxerrors = value; NotifyUpdated(); }
        }

        /// <summary>
        /// count of error corrected packets
        /// </summary>
        public UInt16 Fixed {
            get { return mFixed; }
            set { mFixed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// local signal strength
        /// </summary>
        public byte Rssi {
            get { return mRssi; }
            set { mRssi = value; NotifyUpdated(); }
        }

        /// <summary>
        /// remote signal strength
        /// </summary>
        public byte Remrssi {
            get { return mRemrssi; }
            set { mRemrssi = value; NotifyUpdated(); }
        }

        /// <summary>
        /// how full the tx buffer is as a percentage
        /// </summary>
        public byte Txbuf {
            get { return mTxbuf; }
            set { mTxbuf = value; NotifyUpdated(); }
        }

        /// <summary>
        /// background noise level
        /// </summary>
        public byte Noise {
            get { return mNoise; }
            set { mNoise = value; NotifyUpdated(); }
        }

        /// <summary>
        /// remote background noise level
        /// </summary>
        public byte Remnoise {
            get { return mRemnoise; }
            set { mRemnoise = value; NotifyUpdated(); }
        }

        public UasRadioStatus()
        {
            mMessageId = 109;
            CrcExtra = 185;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("RadioStatus \n");
            sb.AppendFormat("    Rxerrors: {0}\n", mRxerrors);
            sb.AppendFormat("    Fixed: {0}\n", mFixed);
            sb.AppendFormat("    Rssi: {0}\n", mRssi);
            sb.AppendFormat("    Remrssi: {0}\n", mRemrssi);
            sb.AppendFormat("    Txbuf: {0}\n", mTxbuf);
            sb.AppendFormat("    Noise: {0}\n", mNoise);
            sb.AppendFormat("    Remnoise: {0}\n", mRemnoise);

            return sb.ToString();
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


    /// <summary>
    /// Begin file transfer
    /// </summary>
    public class UasFileTransferStart: UasMessage
    {
        /// <summary>
        /// Unique transfer ID
        /// </summary>
        public UInt64 TransferUid {
            get { return mTransferUid; }
            set { mTransferUid = value; NotifyUpdated(); }
        }

        /// <summary>
        /// File size in bytes
        /// </summary>
        public UInt32 FileSize {
            get { return mFileSize; }
            set { mFileSize = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Destination path
        /// </summary>
        public char[] DestPath {
            get { return mDestPath; }
            set { mDestPath = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Transfer direction: 0: from requester, 1: to requester
        /// </summary>
        public byte Direction {
            get { return mDirection; }
            set { mDirection = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RESERVED
        /// </summary>
        public byte Flags {
            get { return mFlags; }
            set { mFlags = value; NotifyUpdated(); }
        }

        public UasFileTransferStart()
        {
            mMessageId = 110;
            CrcExtra = 235;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("FileTransferStart \n");
            sb.AppendFormat("    TransferUid: {0}\n", mTransferUid);
            sb.AppendFormat("    FileSize: {0}\n", mFileSize);
            sb.Append("    DestPath\n");
            sb.AppendFormat("        [0]: {0}\n", mDestPath[0]);
            sb.AppendFormat("        [1]: {0}\n", mDestPath[1]);
            sb.AppendFormat("        [2]: {0}\n", mDestPath[2]);
            sb.AppendFormat("        [3]: {0}\n", mDestPath[3]);
            sb.AppendFormat("        [4]: {0}\n", mDestPath[4]);
            sb.AppendFormat("        [5]: {0}\n", mDestPath[5]);
            sb.AppendFormat("        [6]: {0}\n", mDestPath[6]);
            sb.AppendFormat("        [7]: {0}\n", mDestPath[7]);
            sb.AppendFormat("        [8]: {0}\n", mDestPath[8]);
            sb.AppendFormat("        [9]: {0}\n", mDestPath[9]);
            sb.AppendFormat("        [10]: {0}\n", mDestPath[10]);
            sb.AppendFormat("        [11]: {0}\n", mDestPath[11]);
            sb.AppendFormat("        [12]: {0}\n", mDestPath[12]);
            sb.AppendFormat("        [13]: {0}\n", mDestPath[13]);
            sb.AppendFormat("        [14]: {0}\n", mDestPath[14]);
            sb.AppendFormat("        [15]: {0}\n", mDestPath[15]);
            sb.AppendFormat("        [16]: {0}\n", mDestPath[16]);
            sb.AppendFormat("        [17]: {0}\n", mDestPath[17]);
            sb.AppendFormat("        [18]: {0}\n", mDestPath[18]);
            sb.AppendFormat("        [19]: {0}\n", mDestPath[19]);
            sb.AppendFormat("        [20]: {0}\n", mDestPath[20]);
            sb.AppendFormat("        [21]: {0}\n", mDestPath[21]);
            sb.AppendFormat("        [22]: {0}\n", mDestPath[22]);
            sb.AppendFormat("        [23]: {0}\n", mDestPath[23]);
            sb.AppendFormat("        [24]: {0}\n", mDestPath[24]);
            sb.AppendFormat("        [25]: {0}\n", mDestPath[25]);
            sb.AppendFormat("        [26]: {0}\n", mDestPath[26]);
            sb.AppendFormat("        [27]: {0}\n", mDestPath[27]);
            sb.AppendFormat("        [28]: {0}\n", mDestPath[28]);
            sb.AppendFormat("        [29]: {0}\n", mDestPath[29]);
            sb.AppendFormat("        [30]: {0}\n", mDestPath[30]);
            sb.AppendFormat("        [31]: {0}\n", mDestPath[31]);
            sb.AppendFormat("        [32]: {0}\n", mDestPath[32]);
            sb.AppendFormat("        [33]: {0}\n", mDestPath[33]);
            sb.AppendFormat("        [34]: {0}\n", mDestPath[34]);
            sb.AppendFormat("        [35]: {0}\n", mDestPath[35]);
            sb.AppendFormat("        [36]: {0}\n", mDestPath[36]);
            sb.AppendFormat("        [37]: {0}\n", mDestPath[37]);
            sb.AppendFormat("        [38]: {0}\n", mDestPath[38]);
            sb.AppendFormat("        [39]: {0}\n", mDestPath[39]);
            sb.AppendFormat("        [40]: {0}\n", mDestPath[40]);
            sb.AppendFormat("        [41]: {0}\n", mDestPath[41]);
            sb.AppendFormat("        [42]: {0}\n", mDestPath[42]);
            sb.AppendFormat("        [43]: {0}\n", mDestPath[43]);
            sb.AppendFormat("        [44]: {0}\n", mDestPath[44]);
            sb.AppendFormat("        [45]: {0}\n", mDestPath[45]);
            sb.AppendFormat("        [46]: {0}\n", mDestPath[46]);
            sb.AppendFormat("        [47]: {0}\n", mDestPath[47]);
            sb.AppendFormat("        [48]: {0}\n", mDestPath[48]);
            sb.AppendFormat("        [49]: {0}\n", mDestPath[49]);
            sb.AppendFormat("        [50]: {0}\n", mDestPath[50]);
            sb.AppendFormat("        [51]: {0}\n", mDestPath[51]);
            sb.AppendFormat("        [52]: {0}\n", mDestPath[52]);
            sb.AppendFormat("        [53]: {0}\n", mDestPath[53]);
            sb.AppendFormat("        [54]: {0}\n", mDestPath[54]);
            sb.AppendFormat("        [55]: {0}\n", mDestPath[55]);
            sb.AppendFormat("        [56]: {0}\n", mDestPath[56]);
            sb.AppendFormat("        [57]: {0}\n", mDestPath[57]);
            sb.AppendFormat("        [58]: {0}\n", mDestPath[58]);
            sb.AppendFormat("        [59]: {0}\n", mDestPath[59]);
            sb.AppendFormat("        [60]: {0}\n", mDestPath[60]);
            sb.AppendFormat("        [61]: {0}\n", mDestPath[61]);
            sb.AppendFormat("        [62]: {0}\n", mDestPath[62]);
            sb.AppendFormat("        [63]: {0}\n", mDestPath[63]);
            sb.AppendFormat("        [64]: {0}\n", mDestPath[64]);
            sb.AppendFormat("        [65]: {0}\n", mDestPath[65]);
            sb.AppendFormat("        [66]: {0}\n", mDestPath[66]);
            sb.AppendFormat("        [67]: {0}\n", mDestPath[67]);
            sb.AppendFormat("        [68]: {0}\n", mDestPath[68]);
            sb.AppendFormat("        [69]: {0}\n", mDestPath[69]);
            sb.AppendFormat("        [70]: {0}\n", mDestPath[70]);
            sb.AppendFormat("        [71]: {0}\n", mDestPath[71]);
            sb.AppendFormat("        [72]: {0}\n", mDestPath[72]);
            sb.AppendFormat("        [73]: {0}\n", mDestPath[73]);
            sb.AppendFormat("        [74]: {0}\n", mDestPath[74]);
            sb.AppendFormat("        [75]: {0}\n", mDestPath[75]);
            sb.AppendFormat("        [76]: {0}\n", mDestPath[76]);
            sb.AppendFormat("        [77]: {0}\n", mDestPath[77]);
            sb.AppendFormat("        [78]: {0}\n", mDestPath[78]);
            sb.AppendFormat("        [79]: {0}\n", mDestPath[79]);
            sb.AppendFormat("        [80]: {0}\n", mDestPath[80]);
            sb.AppendFormat("        [81]: {0}\n", mDestPath[81]);
            sb.AppendFormat("        [82]: {0}\n", mDestPath[82]);
            sb.AppendFormat("        [83]: {0}\n", mDestPath[83]);
            sb.AppendFormat("        [84]: {0}\n", mDestPath[84]);
            sb.AppendFormat("        [85]: {0}\n", mDestPath[85]);
            sb.AppendFormat("        [86]: {0}\n", mDestPath[86]);
            sb.AppendFormat("        [87]: {0}\n", mDestPath[87]);
            sb.AppendFormat("        [88]: {0}\n", mDestPath[88]);
            sb.AppendFormat("        [89]: {0}\n", mDestPath[89]);
            sb.AppendFormat("        [90]: {0}\n", mDestPath[90]);
            sb.AppendFormat("        [91]: {0}\n", mDestPath[91]);
            sb.AppendFormat("        [92]: {0}\n", mDestPath[92]);
            sb.AppendFormat("        [93]: {0}\n", mDestPath[93]);
            sb.AppendFormat("        [94]: {0}\n", mDestPath[94]);
            sb.AppendFormat("        [95]: {0}\n", mDestPath[95]);
            sb.AppendFormat("        [96]: {0}\n", mDestPath[96]);
            sb.AppendFormat("        [97]: {0}\n", mDestPath[97]);
            sb.AppendFormat("        [98]: {0}\n", mDestPath[98]);
            sb.AppendFormat("        [99]: {0}\n", mDestPath[99]);
            sb.AppendFormat("        [100]: {0}\n", mDestPath[100]);
            sb.AppendFormat("        [101]: {0}\n", mDestPath[101]);
            sb.AppendFormat("        [102]: {0}\n", mDestPath[102]);
            sb.AppendFormat("        [103]: {0}\n", mDestPath[103]);
            sb.AppendFormat("        [104]: {0}\n", mDestPath[104]);
            sb.AppendFormat("        [105]: {0}\n", mDestPath[105]);
            sb.AppendFormat("        [106]: {0}\n", mDestPath[106]);
            sb.AppendFormat("        [107]: {0}\n", mDestPath[107]);
            sb.AppendFormat("        [108]: {0}\n", mDestPath[108]);
            sb.AppendFormat("        [109]: {0}\n", mDestPath[109]);
            sb.AppendFormat("        [110]: {0}\n", mDestPath[110]);
            sb.AppendFormat("        [111]: {0}\n", mDestPath[111]);
            sb.AppendFormat("        [112]: {0}\n", mDestPath[112]);
            sb.AppendFormat("        [113]: {0}\n", mDestPath[113]);
            sb.AppendFormat("        [114]: {0}\n", mDestPath[114]);
            sb.AppendFormat("        [115]: {0}\n", mDestPath[115]);
            sb.AppendFormat("        [116]: {0}\n", mDestPath[116]);
            sb.AppendFormat("        [117]: {0}\n", mDestPath[117]);
            sb.AppendFormat("        [118]: {0}\n", mDestPath[118]);
            sb.AppendFormat("        [119]: {0}\n", mDestPath[119]);
            sb.AppendFormat("        [120]: {0}\n", mDestPath[120]);
            sb.AppendFormat("        [121]: {0}\n", mDestPath[121]);
            sb.AppendFormat("        [122]: {0}\n", mDestPath[122]);
            sb.AppendFormat("        [123]: {0}\n", mDestPath[123]);
            sb.AppendFormat("        [124]: {0}\n", mDestPath[124]);
            sb.AppendFormat("        [125]: {0}\n", mDestPath[125]);
            sb.AppendFormat("        [126]: {0}\n", mDestPath[126]);
            sb.AppendFormat("        [127]: {0}\n", mDestPath[127]);
            sb.AppendFormat("        [128]: {0}\n", mDestPath[128]);
            sb.AppendFormat("        [129]: {0}\n", mDestPath[129]);
            sb.AppendFormat("        [130]: {0}\n", mDestPath[130]);
            sb.AppendFormat("        [131]: {0}\n", mDestPath[131]);
            sb.AppendFormat("        [132]: {0}\n", mDestPath[132]);
            sb.AppendFormat("        [133]: {0}\n", mDestPath[133]);
            sb.AppendFormat("        [134]: {0}\n", mDestPath[134]);
            sb.AppendFormat("        [135]: {0}\n", mDestPath[135]);
            sb.AppendFormat("        [136]: {0}\n", mDestPath[136]);
            sb.AppendFormat("        [137]: {0}\n", mDestPath[137]);
            sb.AppendFormat("        [138]: {0}\n", mDestPath[138]);
            sb.AppendFormat("        [139]: {0}\n", mDestPath[139]);
            sb.AppendFormat("        [140]: {0}\n", mDestPath[140]);
            sb.AppendFormat("        [141]: {0}\n", mDestPath[141]);
            sb.AppendFormat("        [142]: {0}\n", mDestPath[142]);
            sb.AppendFormat("        [143]: {0}\n", mDestPath[143]);
            sb.AppendFormat("        [144]: {0}\n", mDestPath[144]);
            sb.AppendFormat("        [145]: {0}\n", mDestPath[145]);
            sb.AppendFormat("        [146]: {0}\n", mDestPath[146]);
            sb.AppendFormat("        [147]: {0}\n", mDestPath[147]);
            sb.AppendFormat("        [148]: {0}\n", mDestPath[148]);
            sb.AppendFormat("        [149]: {0}\n", mDestPath[149]);
            sb.AppendFormat("        [150]: {0}\n", mDestPath[150]);
            sb.AppendFormat("        [151]: {0}\n", mDestPath[151]);
            sb.AppendFormat("        [152]: {0}\n", mDestPath[152]);
            sb.AppendFormat("        [153]: {0}\n", mDestPath[153]);
            sb.AppendFormat("        [154]: {0}\n", mDestPath[154]);
            sb.AppendFormat("        [155]: {0}\n", mDestPath[155]);
            sb.AppendFormat("        [156]: {0}\n", mDestPath[156]);
            sb.AppendFormat("        [157]: {0}\n", mDestPath[157]);
            sb.AppendFormat("        [158]: {0}\n", mDestPath[158]);
            sb.AppendFormat("        [159]: {0}\n", mDestPath[159]);
            sb.AppendFormat("        [160]: {0}\n", mDestPath[160]);
            sb.AppendFormat("        [161]: {0}\n", mDestPath[161]);
            sb.AppendFormat("        [162]: {0}\n", mDestPath[162]);
            sb.AppendFormat("        [163]: {0}\n", mDestPath[163]);
            sb.AppendFormat("        [164]: {0}\n", mDestPath[164]);
            sb.AppendFormat("        [165]: {0}\n", mDestPath[165]);
            sb.AppendFormat("        [166]: {0}\n", mDestPath[166]);
            sb.AppendFormat("        [167]: {0}\n", mDestPath[167]);
            sb.AppendFormat("        [168]: {0}\n", mDestPath[168]);
            sb.AppendFormat("        [169]: {0}\n", mDestPath[169]);
            sb.AppendFormat("        [170]: {0}\n", mDestPath[170]);
            sb.AppendFormat("        [171]: {0}\n", mDestPath[171]);
            sb.AppendFormat("        [172]: {0}\n", mDestPath[172]);
            sb.AppendFormat("        [173]: {0}\n", mDestPath[173]);
            sb.AppendFormat("        [174]: {0}\n", mDestPath[174]);
            sb.AppendFormat("        [175]: {0}\n", mDestPath[175]);
            sb.AppendFormat("        [176]: {0}\n", mDestPath[176]);
            sb.AppendFormat("        [177]: {0}\n", mDestPath[177]);
            sb.AppendFormat("        [178]: {0}\n", mDestPath[178]);
            sb.AppendFormat("        [179]: {0}\n", mDestPath[179]);
            sb.AppendFormat("        [180]: {0}\n", mDestPath[180]);
            sb.AppendFormat("        [181]: {0}\n", mDestPath[181]);
            sb.AppendFormat("        [182]: {0}\n", mDestPath[182]);
            sb.AppendFormat("        [183]: {0}\n", mDestPath[183]);
            sb.AppendFormat("        [184]: {0}\n", mDestPath[184]);
            sb.AppendFormat("        [185]: {0}\n", mDestPath[185]);
            sb.AppendFormat("        [186]: {0}\n", mDestPath[186]);
            sb.AppendFormat("        [187]: {0}\n", mDestPath[187]);
            sb.AppendFormat("        [188]: {0}\n", mDestPath[188]);
            sb.AppendFormat("        [189]: {0}\n", mDestPath[189]);
            sb.AppendFormat("        [190]: {0}\n", mDestPath[190]);
            sb.AppendFormat("        [191]: {0}\n", mDestPath[191]);
            sb.AppendFormat("        [192]: {0}\n", mDestPath[192]);
            sb.AppendFormat("        [193]: {0}\n", mDestPath[193]);
            sb.AppendFormat("        [194]: {0}\n", mDestPath[194]);
            sb.AppendFormat("        [195]: {0}\n", mDestPath[195]);
            sb.AppendFormat("        [196]: {0}\n", mDestPath[196]);
            sb.AppendFormat("        [197]: {0}\n", mDestPath[197]);
            sb.AppendFormat("        [198]: {0}\n", mDestPath[198]);
            sb.AppendFormat("        [199]: {0}\n", mDestPath[199]);
            sb.AppendFormat("        [200]: {0}\n", mDestPath[200]);
            sb.AppendFormat("        [201]: {0}\n", mDestPath[201]);
            sb.AppendFormat("        [202]: {0}\n", mDestPath[202]);
            sb.AppendFormat("        [203]: {0}\n", mDestPath[203]);
            sb.AppendFormat("        [204]: {0}\n", mDestPath[204]);
            sb.AppendFormat("        [205]: {0}\n", mDestPath[205]);
            sb.AppendFormat("        [206]: {0}\n", mDestPath[206]);
            sb.AppendFormat("        [207]: {0}\n", mDestPath[207]);
            sb.AppendFormat("        [208]: {0}\n", mDestPath[208]);
            sb.AppendFormat("        [209]: {0}\n", mDestPath[209]);
            sb.AppendFormat("        [210]: {0}\n", mDestPath[210]);
            sb.AppendFormat("        [211]: {0}\n", mDestPath[211]);
            sb.AppendFormat("        [212]: {0}\n", mDestPath[212]);
            sb.AppendFormat("        [213]: {0}\n", mDestPath[213]);
            sb.AppendFormat("        [214]: {0}\n", mDestPath[214]);
            sb.AppendFormat("        [215]: {0}\n", mDestPath[215]);
            sb.AppendFormat("        [216]: {0}\n", mDestPath[216]);
            sb.AppendFormat("        [217]: {0}\n", mDestPath[217]);
            sb.AppendFormat("        [218]: {0}\n", mDestPath[218]);
            sb.AppendFormat("        [219]: {0}\n", mDestPath[219]);
            sb.AppendFormat("        [220]: {0}\n", mDestPath[220]);
            sb.AppendFormat("        [221]: {0}\n", mDestPath[221]);
            sb.AppendFormat("        [222]: {0}\n", mDestPath[222]);
            sb.AppendFormat("        [223]: {0}\n", mDestPath[223]);
            sb.AppendFormat("        [224]: {0}\n", mDestPath[224]);
            sb.AppendFormat("        [225]: {0}\n", mDestPath[225]);
            sb.AppendFormat("        [226]: {0}\n", mDestPath[226]);
            sb.AppendFormat("        [227]: {0}\n", mDestPath[227]);
            sb.AppendFormat("        [228]: {0}\n", mDestPath[228]);
            sb.AppendFormat("        [229]: {0}\n", mDestPath[229]);
            sb.AppendFormat("        [230]: {0}\n", mDestPath[230]);
            sb.AppendFormat("        [231]: {0}\n", mDestPath[231]);
            sb.AppendFormat("        [232]: {0}\n", mDestPath[232]);
            sb.AppendFormat("        [233]: {0}\n", mDestPath[233]);
            sb.AppendFormat("        [234]: {0}\n", mDestPath[234]);
            sb.AppendFormat("        [235]: {0}\n", mDestPath[235]);
            sb.AppendFormat("        [236]: {0}\n", mDestPath[236]);
            sb.AppendFormat("        [237]: {0}\n", mDestPath[237]);
            sb.AppendFormat("        [238]: {0}\n", mDestPath[238]);
            sb.AppendFormat("        [239]: {0}\n", mDestPath[239]);
            sb.AppendFormat("    Direction: {0}\n", mDirection);
            sb.AppendFormat("    Flags: {0}\n", mFlags);

            return sb.ToString();
        }

        private UInt64 mTransferUid;
        private UInt32 mFileSize;
        private char[] mDestPath = new char[240];
        private byte mDirection;
        private byte mFlags;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Get directory listing
    /// </summary>
    public class UasFileTransferDirList: UasMessage
    {
        /// <summary>
        /// Unique transfer ID
        /// </summary>
        public UInt64 TransferUid {
            get { return mTransferUid; }
            set { mTransferUid = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Directory path to list
        /// </summary>
        public char[] DirPath {
            get { return mDirPath; }
            set { mDirPath = value; NotifyUpdated(); }
        }

        /// <summary>
        /// RESERVED
        /// </summary>
        public byte Flags {
            get { return mFlags; }
            set { mFlags = value; NotifyUpdated(); }
        }

        public UasFileTransferDirList()
        {
            mMessageId = 111;
            CrcExtra = 93;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("FileTransferDirList \n");
            sb.AppendFormat("    TransferUid: {0}\n", mTransferUid);
            sb.Append("    DirPath\n");
            sb.AppendFormat("        [0]: {0}\n", mDirPath[0]);
            sb.AppendFormat("        [1]: {0}\n", mDirPath[1]);
            sb.AppendFormat("        [2]: {0}\n", mDirPath[2]);
            sb.AppendFormat("        [3]: {0}\n", mDirPath[3]);
            sb.AppendFormat("        [4]: {0}\n", mDirPath[4]);
            sb.AppendFormat("        [5]: {0}\n", mDirPath[5]);
            sb.AppendFormat("        [6]: {0}\n", mDirPath[6]);
            sb.AppendFormat("        [7]: {0}\n", mDirPath[7]);
            sb.AppendFormat("        [8]: {0}\n", mDirPath[8]);
            sb.AppendFormat("        [9]: {0}\n", mDirPath[9]);
            sb.AppendFormat("        [10]: {0}\n", mDirPath[10]);
            sb.AppendFormat("        [11]: {0}\n", mDirPath[11]);
            sb.AppendFormat("        [12]: {0}\n", mDirPath[12]);
            sb.AppendFormat("        [13]: {0}\n", mDirPath[13]);
            sb.AppendFormat("        [14]: {0}\n", mDirPath[14]);
            sb.AppendFormat("        [15]: {0}\n", mDirPath[15]);
            sb.AppendFormat("        [16]: {0}\n", mDirPath[16]);
            sb.AppendFormat("        [17]: {0}\n", mDirPath[17]);
            sb.AppendFormat("        [18]: {0}\n", mDirPath[18]);
            sb.AppendFormat("        [19]: {0}\n", mDirPath[19]);
            sb.AppendFormat("        [20]: {0}\n", mDirPath[20]);
            sb.AppendFormat("        [21]: {0}\n", mDirPath[21]);
            sb.AppendFormat("        [22]: {0}\n", mDirPath[22]);
            sb.AppendFormat("        [23]: {0}\n", mDirPath[23]);
            sb.AppendFormat("        [24]: {0}\n", mDirPath[24]);
            sb.AppendFormat("        [25]: {0}\n", mDirPath[25]);
            sb.AppendFormat("        [26]: {0}\n", mDirPath[26]);
            sb.AppendFormat("        [27]: {0}\n", mDirPath[27]);
            sb.AppendFormat("        [28]: {0}\n", mDirPath[28]);
            sb.AppendFormat("        [29]: {0}\n", mDirPath[29]);
            sb.AppendFormat("        [30]: {0}\n", mDirPath[30]);
            sb.AppendFormat("        [31]: {0}\n", mDirPath[31]);
            sb.AppendFormat("        [32]: {0}\n", mDirPath[32]);
            sb.AppendFormat("        [33]: {0}\n", mDirPath[33]);
            sb.AppendFormat("        [34]: {0}\n", mDirPath[34]);
            sb.AppendFormat("        [35]: {0}\n", mDirPath[35]);
            sb.AppendFormat("        [36]: {0}\n", mDirPath[36]);
            sb.AppendFormat("        [37]: {0}\n", mDirPath[37]);
            sb.AppendFormat("        [38]: {0}\n", mDirPath[38]);
            sb.AppendFormat("        [39]: {0}\n", mDirPath[39]);
            sb.AppendFormat("        [40]: {0}\n", mDirPath[40]);
            sb.AppendFormat("        [41]: {0}\n", mDirPath[41]);
            sb.AppendFormat("        [42]: {0}\n", mDirPath[42]);
            sb.AppendFormat("        [43]: {0}\n", mDirPath[43]);
            sb.AppendFormat("        [44]: {0}\n", mDirPath[44]);
            sb.AppendFormat("        [45]: {0}\n", mDirPath[45]);
            sb.AppendFormat("        [46]: {0}\n", mDirPath[46]);
            sb.AppendFormat("        [47]: {0}\n", mDirPath[47]);
            sb.AppendFormat("        [48]: {0}\n", mDirPath[48]);
            sb.AppendFormat("        [49]: {0}\n", mDirPath[49]);
            sb.AppendFormat("        [50]: {0}\n", mDirPath[50]);
            sb.AppendFormat("        [51]: {0}\n", mDirPath[51]);
            sb.AppendFormat("        [52]: {0}\n", mDirPath[52]);
            sb.AppendFormat("        [53]: {0}\n", mDirPath[53]);
            sb.AppendFormat("        [54]: {0}\n", mDirPath[54]);
            sb.AppendFormat("        [55]: {0}\n", mDirPath[55]);
            sb.AppendFormat("        [56]: {0}\n", mDirPath[56]);
            sb.AppendFormat("        [57]: {0}\n", mDirPath[57]);
            sb.AppendFormat("        [58]: {0}\n", mDirPath[58]);
            sb.AppendFormat("        [59]: {0}\n", mDirPath[59]);
            sb.AppendFormat("        [60]: {0}\n", mDirPath[60]);
            sb.AppendFormat("        [61]: {0}\n", mDirPath[61]);
            sb.AppendFormat("        [62]: {0}\n", mDirPath[62]);
            sb.AppendFormat("        [63]: {0}\n", mDirPath[63]);
            sb.AppendFormat("        [64]: {0}\n", mDirPath[64]);
            sb.AppendFormat("        [65]: {0}\n", mDirPath[65]);
            sb.AppendFormat("        [66]: {0}\n", mDirPath[66]);
            sb.AppendFormat("        [67]: {0}\n", mDirPath[67]);
            sb.AppendFormat("        [68]: {0}\n", mDirPath[68]);
            sb.AppendFormat("        [69]: {0}\n", mDirPath[69]);
            sb.AppendFormat("        [70]: {0}\n", mDirPath[70]);
            sb.AppendFormat("        [71]: {0}\n", mDirPath[71]);
            sb.AppendFormat("        [72]: {0}\n", mDirPath[72]);
            sb.AppendFormat("        [73]: {0}\n", mDirPath[73]);
            sb.AppendFormat("        [74]: {0}\n", mDirPath[74]);
            sb.AppendFormat("        [75]: {0}\n", mDirPath[75]);
            sb.AppendFormat("        [76]: {0}\n", mDirPath[76]);
            sb.AppendFormat("        [77]: {0}\n", mDirPath[77]);
            sb.AppendFormat("        [78]: {0}\n", mDirPath[78]);
            sb.AppendFormat("        [79]: {0}\n", mDirPath[79]);
            sb.AppendFormat("        [80]: {0}\n", mDirPath[80]);
            sb.AppendFormat("        [81]: {0}\n", mDirPath[81]);
            sb.AppendFormat("        [82]: {0}\n", mDirPath[82]);
            sb.AppendFormat("        [83]: {0}\n", mDirPath[83]);
            sb.AppendFormat("        [84]: {0}\n", mDirPath[84]);
            sb.AppendFormat("        [85]: {0}\n", mDirPath[85]);
            sb.AppendFormat("        [86]: {0}\n", mDirPath[86]);
            sb.AppendFormat("        [87]: {0}\n", mDirPath[87]);
            sb.AppendFormat("        [88]: {0}\n", mDirPath[88]);
            sb.AppendFormat("        [89]: {0}\n", mDirPath[89]);
            sb.AppendFormat("        [90]: {0}\n", mDirPath[90]);
            sb.AppendFormat("        [91]: {0}\n", mDirPath[91]);
            sb.AppendFormat("        [92]: {0}\n", mDirPath[92]);
            sb.AppendFormat("        [93]: {0}\n", mDirPath[93]);
            sb.AppendFormat("        [94]: {0}\n", mDirPath[94]);
            sb.AppendFormat("        [95]: {0}\n", mDirPath[95]);
            sb.AppendFormat("        [96]: {0}\n", mDirPath[96]);
            sb.AppendFormat("        [97]: {0}\n", mDirPath[97]);
            sb.AppendFormat("        [98]: {0}\n", mDirPath[98]);
            sb.AppendFormat("        [99]: {0}\n", mDirPath[99]);
            sb.AppendFormat("        [100]: {0}\n", mDirPath[100]);
            sb.AppendFormat("        [101]: {0}\n", mDirPath[101]);
            sb.AppendFormat("        [102]: {0}\n", mDirPath[102]);
            sb.AppendFormat("        [103]: {0}\n", mDirPath[103]);
            sb.AppendFormat("        [104]: {0}\n", mDirPath[104]);
            sb.AppendFormat("        [105]: {0}\n", mDirPath[105]);
            sb.AppendFormat("        [106]: {0}\n", mDirPath[106]);
            sb.AppendFormat("        [107]: {0}\n", mDirPath[107]);
            sb.AppendFormat("        [108]: {0}\n", mDirPath[108]);
            sb.AppendFormat("        [109]: {0}\n", mDirPath[109]);
            sb.AppendFormat("        [110]: {0}\n", mDirPath[110]);
            sb.AppendFormat("        [111]: {0}\n", mDirPath[111]);
            sb.AppendFormat("        [112]: {0}\n", mDirPath[112]);
            sb.AppendFormat("        [113]: {0}\n", mDirPath[113]);
            sb.AppendFormat("        [114]: {0}\n", mDirPath[114]);
            sb.AppendFormat("        [115]: {0}\n", mDirPath[115]);
            sb.AppendFormat("        [116]: {0}\n", mDirPath[116]);
            sb.AppendFormat("        [117]: {0}\n", mDirPath[117]);
            sb.AppendFormat("        [118]: {0}\n", mDirPath[118]);
            sb.AppendFormat("        [119]: {0}\n", mDirPath[119]);
            sb.AppendFormat("        [120]: {0}\n", mDirPath[120]);
            sb.AppendFormat("        [121]: {0}\n", mDirPath[121]);
            sb.AppendFormat("        [122]: {0}\n", mDirPath[122]);
            sb.AppendFormat("        [123]: {0}\n", mDirPath[123]);
            sb.AppendFormat("        [124]: {0}\n", mDirPath[124]);
            sb.AppendFormat("        [125]: {0}\n", mDirPath[125]);
            sb.AppendFormat("        [126]: {0}\n", mDirPath[126]);
            sb.AppendFormat("        [127]: {0}\n", mDirPath[127]);
            sb.AppendFormat("        [128]: {0}\n", mDirPath[128]);
            sb.AppendFormat("        [129]: {0}\n", mDirPath[129]);
            sb.AppendFormat("        [130]: {0}\n", mDirPath[130]);
            sb.AppendFormat("        [131]: {0}\n", mDirPath[131]);
            sb.AppendFormat("        [132]: {0}\n", mDirPath[132]);
            sb.AppendFormat("        [133]: {0}\n", mDirPath[133]);
            sb.AppendFormat("        [134]: {0}\n", mDirPath[134]);
            sb.AppendFormat("        [135]: {0}\n", mDirPath[135]);
            sb.AppendFormat("        [136]: {0}\n", mDirPath[136]);
            sb.AppendFormat("        [137]: {0}\n", mDirPath[137]);
            sb.AppendFormat("        [138]: {0}\n", mDirPath[138]);
            sb.AppendFormat("        [139]: {0}\n", mDirPath[139]);
            sb.AppendFormat("        [140]: {0}\n", mDirPath[140]);
            sb.AppendFormat("        [141]: {0}\n", mDirPath[141]);
            sb.AppendFormat("        [142]: {0}\n", mDirPath[142]);
            sb.AppendFormat("        [143]: {0}\n", mDirPath[143]);
            sb.AppendFormat("        [144]: {0}\n", mDirPath[144]);
            sb.AppendFormat("        [145]: {0}\n", mDirPath[145]);
            sb.AppendFormat("        [146]: {0}\n", mDirPath[146]);
            sb.AppendFormat("        [147]: {0}\n", mDirPath[147]);
            sb.AppendFormat("        [148]: {0}\n", mDirPath[148]);
            sb.AppendFormat("        [149]: {0}\n", mDirPath[149]);
            sb.AppendFormat("        [150]: {0}\n", mDirPath[150]);
            sb.AppendFormat("        [151]: {0}\n", mDirPath[151]);
            sb.AppendFormat("        [152]: {0}\n", mDirPath[152]);
            sb.AppendFormat("        [153]: {0}\n", mDirPath[153]);
            sb.AppendFormat("        [154]: {0}\n", mDirPath[154]);
            sb.AppendFormat("        [155]: {0}\n", mDirPath[155]);
            sb.AppendFormat("        [156]: {0}\n", mDirPath[156]);
            sb.AppendFormat("        [157]: {0}\n", mDirPath[157]);
            sb.AppendFormat("        [158]: {0}\n", mDirPath[158]);
            sb.AppendFormat("        [159]: {0}\n", mDirPath[159]);
            sb.AppendFormat("        [160]: {0}\n", mDirPath[160]);
            sb.AppendFormat("        [161]: {0}\n", mDirPath[161]);
            sb.AppendFormat("        [162]: {0}\n", mDirPath[162]);
            sb.AppendFormat("        [163]: {0}\n", mDirPath[163]);
            sb.AppendFormat("        [164]: {0}\n", mDirPath[164]);
            sb.AppendFormat("        [165]: {0}\n", mDirPath[165]);
            sb.AppendFormat("        [166]: {0}\n", mDirPath[166]);
            sb.AppendFormat("        [167]: {0}\n", mDirPath[167]);
            sb.AppendFormat("        [168]: {0}\n", mDirPath[168]);
            sb.AppendFormat("        [169]: {0}\n", mDirPath[169]);
            sb.AppendFormat("        [170]: {0}\n", mDirPath[170]);
            sb.AppendFormat("        [171]: {0}\n", mDirPath[171]);
            sb.AppendFormat("        [172]: {0}\n", mDirPath[172]);
            sb.AppendFormat("        [173]: {0}\n", mDirPath[173]);
            sb.AppendFormat("        [174]: {0}\n", mDirPath[174]);
            sb.AppendFormat("        [175]: {0}\n", mDirPath[175]);
            sb.AppendFormat("        [176]: {0}\n", mDirPath[176]);
            sb.AppendFormat("        [177]: {0}\n", mDirPath[177]);
            sb.AppendFormat("        [178]: {0}\n", mDirPath[178]);
            sb.AppendFormat("        [179]: {0}\n", mDirPath[179]);
            sb.AppendFormat("        [180]: {0}\n", mDirPath[180]);
            sb.AppendFormat("        [181]: {0}\n", mDirPath[181]);
            sb.AppendFormat("        [182]: {0}\n", mDirPath[182]);
            sb.AppendFormat("        [183]: {0}\n", mDirPath[183]);
            sb.AppendFormat("        [184]: {0}\n", mDirPath[184]);
            sb.AppendFormat("        [185]: {0}\n", mDirPath[185]);
            sb.AppendFormat("        [186]: {0}\n", mDirPath[186]);
            sb.AppendFormat("        [187]: {0}\n", mDirPath[187]);
            sb.AppendFormat("        [188]: {0}\n", mDirPath[188]);
            sb.AppendFormat("        [189]: {0}\n", mDirPath[189]);
            sb.AppendFormat("        [190]: {0}\n", mDirPath[190]);
            sb.AppendFormat("        [191]: {0}\n", mDirPath[191]);
            sb.AppendFormat("        [192]: {0}\n", mDirPath[192]);
            sb.AppendFormat("        [193]: {0}\n", mDirPath[193]);
            sb.AppendFormat("        [194]: {0}\n", mDirPath[194]);
            sb.AppendFormat("        [195]: {0}\n", mDirPath[195]);
            sb.AppendFormat("        [196]: {0}\n", mDirPath[196]);
            sb.AppendFormat("        [197]: {0}\n", mDirPath[197]);
            sb.AppendFormat("        [198]: {0}\n", mDirPath[198]);
            sb.AppendFormat("        [199]: {0}\n", mDirPath[199]);
            sb.AppendFormat("        [200]: {0}\n", mDirPath[200]);
            sb.AppendFormat("        [201]: {0}\n", mDirPath[201]);
            sb.AppendFormat("        [202]: {0}\n", mDirPath[202]);
            sb.AppendFormat("        [203]: {0}\n", mDirPath[203]);
            sb.AppendFormat("        [204]: {0}\n", mDirPath[204]);
            sb.AppendFormat("        [205]: {0}\n", mDirPath[205]);
            sb.AppendFormat("        [206]: {0}\n", mDirPath[206]);
            sb.AppendFormat("        [207]: {0}\n", mDirPath[207]);
            sb.AppendFormat("        [208]: {0}\n", mDirPath[208]);
            sb.AppendFormat("        [209]: {0}\n", mDirPath[209]);
            sb.AppendFormat("        [210]: {0}\n", mDirPath[210]);
            sb.AppendFormat("        [211]: {0}\n", mDirPath[211]);
            sb.AppendFormat("        [212]: {0}\n", mDirPath[212]);
            sb.AppendFormat("        [213]: {0}\n", mDirPath[213]);
            sb.AppendFormat("        [214]: {0}\n", mDirPath[214]);
            sb.AppendFormat("        [215]: {0}\n", mDirPath[215]);
            sb.AppendFormat("        [216]: {0}\n", mDirPath[216]);
            sb.AppendFormat("        [217]: {0}\n", mDirPath[217]);
            sb.AppendFormat("        [218]: {0}\n", mDirPath[218]);
            sb.AppendFormat("        [219]: {0}\n", mDirPath[219]);
            sb.AppendFormat("        [220]: {0}\n", mDirPath[220]);
            sb.AppendFormat("        [221]: {0}\n", mDirPath[221]);
            sb.AppendFormat("        [222]: {0}\n", mDirPath[222]);
            sb.AppendFormat("        [223]: {0}\n", mDirPath[223]);
            sb.AppendFormat("        [224]: {0}\n", mDirPath[224]);
            sb.AppendFormat("        [225]: {0}\n", mDirPath[225]);
            sb.AppendFormat("        [226]: {0}\n", mDirPath[226]);
            sb.AppendFormat("        [227]: {0}\n", mDirPath[227]);
            sb.AppendFormat("        [228]: {0}\n", mDirPath[228]);
            sb.AppendFormat("        [229]: {0}\n", mDirPath[229]);
            sb.AppendFormat("        [230]: {0}\n", mDirPath[230]);
            sb.AppendFormat("        [231]: {0}\n", mDirPath[231]);
            sb.AppendFormat("        [232]: {0}\n", mDirPath[232]);
            sb.AppendFormat("        [233]: {0}\n", mDirPath[233]);
            sb.AppendFormat("        [234]: {0}\n", mDirPath[234]);
            sb.AppendFormat("        [235]: {0}\n", mDirPath[235]);
            sb.AppendFormat("        [236]: {0}\n", mDirPath[236]);
            sb.AppendFormat("        [237]: {0}\n", mDirPath[237]);
            sb.AppendFormat("        [238]: {0}\n", mDirPath[238]);
            sb.AppendFormat("        [239]: {0}\n", mDirPath[239]);
            sb.AppendFormat("    Flags: {0}\n", mFlags);

            return sb.ToString();
        }

        private UInt64 mTransferUid;
        private char[] mDirPath = new char[240];
        private byte mFlags;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// File transfer result
    /// </summary>
    public class UasFileTransferRes: UasMessage
    {
        /// <summary>
        /// Unique transfer ID
        /// </summary>
        public UInt64 TransferUid {
            get { return mTransferUid; }
            set { mTransferUid = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 0: OK, 1: not permitted, 2: bad path / file name, 3: no space left on device
        /// </summary>
        public byte Result {
            get { return mResult; }
            set { mResult = value; NotifyUpdated(); }
        }

        public UasFileTransferRes()
        {
            mMessageId = 112;
            CrcExtra = 124;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("FileTransferRes \n");
            sb.AppendFormat("    TransferUid: {0}\n", mTransferUid);
            sb.AppendFormat("    Result: {0}\n", mResult);

            return sb.ToString();
        }

        private UInt64 mTransferUid;
        private byte mResult;
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// The global position, as returned by the Global Positioning System (GPS). This is                   NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
    /// </summary>
    public class UasHilGps: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Latitude (WGS84), in degrees * 1E7
        /// </summary>
        public Int32 Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Longitude (WGS84), in degrees * 1E7
        /// </summary>
        public Int32 Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude (WGS84), in meters * 1000 (positive for up)
        /// </summary>
        public Int32 Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        /// <summary>
        /// GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        /// </summary>
        public UInt16 Eph {
            get { return mEph; }
            set { mEph = value; NotifyUpdated(); }
        }

        /// <summary>
        /// GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
        /// </summary>
        public UInt16 Epv {
            get { return mEpv; }
            set { mEpv = value; NotifyUpdated(); }
        }

        /// <summary>
        /// GPS ground speed (m/s * 100). If unknown, set to: 65535
        /// </summary>
        public UInt16 Vel {
            get { return mVel; }
            set { mVel = value; NotifyUpdated(); }
        }

        /// <summary>
        /// GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
        /// </summary>
        public Int16 Vn {
            get { return mVn; }
            set { mVn = value; NotifyUpdated(); }
        }

        /// <summary>
        /// GPS velocity in cm/s in EAST direction in earth-fixed NED frame
        /// </summary>
        public Int16 Ve {
            get { return mVe; }
            set { mVe = value; NotifyUpdated(); }
        }

        /// <summary>
        /// GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
        /// </summary>
        public Int16 Vd {
            get { return mVd; }
            set { mVd = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
        /// </summary>
        public UInt16 Cog {
            get { return mCog; }
            set { mCog = value; NotifyUpdated(); }
        }

        /// <summary>
        /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
        /// </summary>
        public byte FixType {
            get { return mFixType; }
            set { mFixType = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Number of satellites visible. If unknown, set to 255
        /// </summary>
        public byte SatellitesVisible {
            get { return mSatellitesVisible; }
            set { mSatellitesVisible = value; NotifyUpdated(); }
        }

        public UasHilGps()
        {
            mMessageId = 113;
            CrcExtra = 124;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("HilGps \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    Lat: {0}\n", mLat);
            sb.AppendFormat("    Lon: {0}\n", mLon);
            sb.AppendFormat("    Alt: {0}\n", mAlt);
            sb.AppendFormat("    Eph: {0}\n", mEph);
            sb.AppendFormat("    Epv: {0}\n", mEpv);
            sb.AppendFormat("    Vel: {0}\n", mVel);
            sb.AppendFormat("    Vn: {0}\n", mVn);
            sb.AppendFormat("    Ve: {0}\n", mVe);
            sb.AppendFormat("    Vd: {0}\n", mVd);
            sb.AppendFormat("    Cog: {0}\n", mCog);
            sb.AppendFormat("    FixType: {0}\n", mFixType);
            sb.AppendFormat("    SatellitesVisible: {0}\n", mSatellitesVisible);

            return sb.ToString();
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


    /// <summary>
    /// Simulated optical flow from a flow sensor (e.g. optical mouse sensor)
    /// </summary>
    public class UasHilOpticalFlow: UasMessage
    {
        /// <summary>
        /// Timestamp (UNIX)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Flow in meters in x-sensor direction, angular-speed compensated
        /// </summary>
        public float FlowCompMX {
            get { return mFlowCompMX; }
            set { mFlowCompMX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Flow in meters in y-sensor direction, angular-speed compensated
        /// </summary>
        public float FlowCompMY {
            get { return mFlowCompMY; }
            set { mFlowCompMY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
        /// </summary>
        public float GroundDistance {
            get { return mGroundDistance; }
            set { mGroundDistance = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Flow in pixels in x-sensor direction
        /// </summary>
        public Int16 FlowX {
            get { return mFlowX; }
            set { mFlowX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Flow in pixels in y-sensor direction
        /// </summary>
        public Int16 FlowY {
            get { return mFlowY; }
            set { mFlowY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Sensor ID
        /// </summary>
        public byte SensorId {
            get { return mSensorId; }
            set { mSensorId = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Optical flow quality / confidence. 0: bad, 255: maximum quality
        /// </summary>
        public byte Quality {
            get { return mQuality; }
            set { mQuality = value; NotifyUpdated(); }
        }

        public UasHilOpticalFlow()
        {
            mMessageId = 114;
            CrcExtra = 119;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("HilOpticalFlow \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    FlowCompMX: {0}\n", mFlowCompMX);
            sb.AppendFormat("    FlowCompMY: {0}\n", mFlowCompMY);
            sb.AppendFormat("    GroundDistance: {0}\n", mGroundDistance);
            sb.AppendFormat("    FlowX: {0}\n", mFlowX);
            sb.AppendFormat("    FlowY: {0}\n", mFlowY);
            sb.AppendFormat("    SensorId: {0}\n", mSensorId);
            sb.AppendFormat("    Quality: {0}\n", mQuality);

            return sb.ToString();
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


    /// <summary>
    /// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
    /// </summary>
    public class UasHilStateQuaternion: UasMessage
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Vehicle attitude expressed as normalized quaternion
        /// </summary>
        public float[] AttitudeQuaternion {
            get { return mAttitudeQuaternion; }
            set { mAttitudeQuaternion = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Body frame roll / phi angular speed (rad/s)
        /// </summary>
        public float Rollspeed {
            get { return mRollspeed; }
            set { mRollspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Body frame pitch / theta angular speed (rad/s)
        /// </summary>
        public float Pitchspeed {
            get { return mPitchspeed; }
            set { mPitchspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Body frame yaw / psi angular speed (rad/s)
        /// </summary>
        public float Yawspeed {
            get { return mYawspeed; }
            set { mYawspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Latitude, expressed as * 1E7
        /// </summary>
        public Int32 Lat {
            get { return mLat; }
            set { mLat = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Longitude, expressed as * 1E7
        /// </summary>
        public Int32 Lon {
            get { return mLon; }
            set { mLon = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Altitude in meters, expressed as * 1000 (millimeters)
        /// </summary>
        public Int32 Alt {
            get { return mAlt; }
            set { mAlt = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Ground X Speed (Latitude), expressed as m/s * 100
        /// </summary>
        public Int16 Vx {
            get { return mVx; }
            set { mVx = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Ground Y Speed (Longitude), expressed as m/s * 100
        /// </summary>
        public Int16 Vy {
            get { return mVy; }
            set { mVy = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Ground Z Speed (Altitude), expressed as m/s * 100
        /// </summary>
        public Int16 Vz {
            get { return mVz; }
            set { mVz = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Indicated airspeed, expressed as m/s * 100
        /// </summary>
        public UInt16 IndAirspeed {
            get { return mIndAirspeed; }
            set { mIndAirspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// True airspeed, expressed as m/s * 100
        /// </summary>
        public UInt16 TrueAirspeed {
            get { return mTrueAirspeed; }
            set { mTrueAirspeed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// X acceleration (mg)
        /// </summary>
        public Int16 Xacc {
            get { return mXacc; }
            set { mXacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Y acceleration (mg)
        /// </summary>
        public Int16 Yacc {
            get { return mYacc; }
            set { mYacc = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Z acceleration (mg)
        /// </summary>
        public Int16 Zacc {
            get { return mZacc; }
            set { mZacc = value; NotifyUpdated(); }
        }

        public UasHilStateQuaternion()
        {
            mMessageId = 115;
            CrcExtra = 4;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("HilStateQuaternion \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.Append("    AttitudeQuaternion\n");
            sb.AppendFormat("        [0]: {0}\n", mAttitudeQuaternion[0]);
            sb.AppendFormat("        [1]: {0}\n", mAttitudeQuaternion[1]);
            sb.AppendFormat("        [2]: {0}\n", mAttitudeQuaternion[2]);
            sb.AppendFormat("        [3]: {0}\n", mAttitudeQuaternion[3]);
            sb.AppendFormat("    Rollspeed: {0}\n", mRollspeed);
            sb.AppendFormat("    Pitchspeed: {0}\n", mPitchspeed);
            sb.AppendFormat("    Yawspeed: {0}\n", mYawspeed);
            sb.AppendFormat("    Lat: {0}\n", mLat);
            sb.AppendFormat("    Lon: {0}\n", mLon);
            sb.AppendFormat("    Alt: {0}\n", mAlt);
            sb.AppendFormat("    Vx: {0}\n", mVx);
            sb.AppendFormat("    Vy: {0}\n", mVy);
            sb.AppendFormat("    Vz: {0}\n", mVz);
            sb.AppendFormat("    IndAirspeed: {0}\n", mIndAirspeed);
            sb.AppendFormat("    TrueAirspeed: {0}\n", mTrueAirspeed);
            sb.AppendFormat("    Xacc: {0}\n", mXacc);
            sb.AppendFormat("    Yacc: {0}\n", mYacc);
            sb.AppendFormat("    Zacc: {0}\n", mZacc);

            return sb.ToString();
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


    /// <summary>
    /// Transmitte battery informations for a accu pack.
    /// </summary>
    public class UasBatteryStatus: UasMessage
    {
        /// <summary>
        /// Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
        /// </summary>
        public Int32 CurrentConsumed {
            get { return mCurrentConsumed; }
            set { mCurrentConsumed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Consumed energy, in 100*Joules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
        /// </summary>
        public Int32 EnergyConsumed {
            get { return mEnergyConsumed; }
            set { mEnergyConsumed = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Battery voltage of cell 1, in millivolts (1 = 1 millivolt)
        /// </summary>
        public UInt16 VoltageCell1 {
            get { return mVoltageCell1; }
            set { mVoltageCell1 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Battery voltage of cell 2, in millivolts (1 = 1 millivolt), -1: no cell
        /// </summary>
        public UInt16 VoltageCell2 {
            get { return mVoltageCell2; }
            set { mVoltageCell2 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Battery voltage of cell 3, in millivolts (1 = 1 millivolt), -1: no cell
        /// </summary>
        public UInt16 VoltageCell3 {
            get { return mVoltageCell3; }
            set { mVoltageCell3 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Battery voltage of cell 4, in millivolts (1 = 1 millivolt), -1: no cell
        /// </summary>
        public UInt16 VoltageCell4 {
            get { return mVoltageCell4; }
            set { mVoltageCell4 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Battery voltage of cell 5, in millivolts (1 = 1 millivolt), -1: no cell
        /// </summary>
        public UInt16 VoltageCell5 {
            get { return mVoltageCell5; }
            set { mVoltageCell5 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Battery voltage of cell 6, in millivolts (1 = 1 millivolt), -1: no cell
        /// </summary>
        public UInt16 VoltageCell6 {
            get { return mVoltageCell6; }
            set { mVoltageCell6 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        /// </summary>
        public Int16 CurrentBattery {
            get { return mCurrentBattery; }
            set { mCurrentBattery = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Accupack ID
        /// </summary>
        public byte AccuId {
            get { return mAccuId; }
            set { mAccuId = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
        /// </summary>
        public SByte BatteryRemaining {
            get { return mBatteryRemaining; }
            set { mBatteryRemaining = value; NotifyUpdated(); }
        }

        public UasBatteryStatus()
        {
            mMessageId = 147;
            CrcExtra = 177;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("BatteryStatus \n");
            sb.AppendFormat("    CurrentConsumed: {0}\n", mCurrentConsumed);
            sb.AppendFormat("    EnergyConsumed: {0}\n", mEnergyConsumed);
            sb.AppendFormat("    VoltageCell1: {0}\n", mVoltageCell1);
            sb.AppendFormat("    VoltageCell2: {0}\n", mVoltageCell2);
            sb.AppendFormat("    VoltageCell3: {0}\n", mVoltageCell3);
            sb.AppendFormat("    VoltageCell4: {0}\n", mVoltageCell4);
            sb.AppendFormat("    VoltageCell5: {0}\n", mVoltageCell5);
            sb.AppendFormat("    VoltageCell6: {0}\n", mVoltageCell6);
            sb.AppendFormat("    CurrentBattery: {0}\n", mCurrentBattery);
            sb.AppendFormat("    AccuId: {0}\n", mAccuId);
            sb.AppendFormat("    BatteryRemaining: {0}\n", mBatteryRemaining);

            return sb.ToString();
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


    /// <summary>
    /// Set the 8 DOF setpoint for a controller.
    /// </summary>
    public class UasSetpoint8dof: UasMessage
    {
        /// <summary>
        /// Value 1
        /// </summary>
        public float Val1 {
            get { return mVal1; }
            set { mVal1 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Value 2
        /// </summary>
        public float Val2 {
            get { return mVal2; }
            set { mVal2 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Value 3
        /// </summary>
        public float Val3 {
            get { return mVal3; }
            set { mVal3 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Value 4
        /// </summary>
        public float Val4 {
            get { return mVal4; }
            set { mVal4 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Value 5
        /// </summary>
        public float Val5 {
            get { return mVal5; }
            set { mVal5 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Value 6
        /// </summary>
        public float Val6 {
            get { return mVal6; }
            set { mVal6 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Value 7
        /// </summary>
        public float Val7 {
            get { return mVal7; }
            set { mVal7 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Value 8
        /// </summary>
        public float Val8 {
            get { return mVal8; }
            set { mVal8 = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public UasSetpoint8dof()
        {
            mMessageId = 148;
            CrcExtra = 241;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("Setpoint8dof \n");
            sb.AppendFormat("    Val1: {0}\n", mVal1);
            sb.AppendFormat("    Val2: {0}\n", mVal2);
            sb.AppendFormat("    Val3: {0}\n", mVal3);
            sb.AppendFormat("    Val4: {0}\n", mVal4);
            sb.AppendFormat("    Val5: {0}\n", mVal5);
            sb.AppendFormat("    Val6: {0}\n", mVal6);
            sb.AppendFormat("    Val7: {0}\n", mVal7);
            sb.AppendFormat("    Val8: {0}\n", mVal8);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);

            return sb.ToString();
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


    /// <summary>
    /// Set the 6 DOF setpoint for a attitude and position controller.
    /// </summary>
    public class UasSetpoint6dof: UasMessage
    {
        /// <summary>
        /// Translational Component in x
        /// </summary>
        public float TransX {
            get { return mTransX; }
            set { mTransX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Translational Component in y
        /// </summary>
        public float TransY {
            get { return mTransY; }
            set { mTransY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Translational Component in z
        /// </summary>
        public float TransZ {
            get { return mTransZ; }
            set { mTransZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Rotational Component in x
        /// </summary>
        public float RotX {
            get { return mRotX; }
            set { mRotX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Rotational Component in y
        /// </summary>
        public float RotY {
            get { return mRotY; }
            set { mRotY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Rotational Component in z
        /// </summary>
        public float RotZ {
            get { return mRotZ; }
            set { mRotZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// System ID
        /// </summary>
        public byte TargetSystem {
            get { return mTargetSystem; }
            set { mTargetSystem = value; NotifyUpdated(); }
        }

        public UasSetpoint6dof()
        {
            mMessageId = 149;
            CrcExtra = 15;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("Setpoint6dof \n");
            sb.AppendFormat("    TransX: {0}\n", mTransX);
            sb.AppendFormat("    TransY: {0}\n", mTransY);
            sb.AppendFormat("    TransZ: {0}\n", mTransZ);
            sb.AppendFormat("    RotX: {0}\n", mRotX);
            sb.AppendFormat("    RotY: {0}\n", mRotY);
            sb.AppendFormat("    RotZ: {0}\n", mRotZ);
            sb.AppendFormat("    TargetSystem: {0}\n", mTargetSystem);

            return sb.ToString();
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


    /// <summary>
    /// Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    /// </summary>
    public class UasMemoryVect: UasMessage
    {
        /// <summary>
        /// Starting address of the debug variables
        /// </summary>
        public UInt16 Address {
            get { return mAddress; }
            set { mAddress = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
        /// </summary>
        public byte Ver {
            get { return mVer; }
            set { mVer = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
        /// </summary>
        public byte Type {
            get { return mType; }
            set { mType = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Memory contents at specified address
        /// </summary>
        public SByte[] Value {
            get { return mValue; }
            set { mValue = value; NotifyUpdated(); }
        }

        public UasMemoryVect()
        {
            mMessageId = 249;
            CrcExtra = 204;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("MemoryVect \n");
            sb.AppendFormat("    Address: {0}\n", mAddress);
            sb.AppendFormat("    Ver: {0}\n", mVer);
            sb.AppendFormat("    Type: {0}\n", mType);
            sb.Append("    Value\n");
            sb.AppendFormat("        [0]: {0}\n", mValue[0]);
            sb.AppendFormat("        [1]: {0}\n", mValue[1]);
            sb.AppendFormat("        [2]: {0}\n", mValue[2]);
            sb.AppendFormat("        [3]: {0}\n", mValue[3]);
            sb.AppendFormat("        [4]: {0}\n", mValue[4]);
            sb.AppendFormat("        [5]: {0}\n", mValue[5]);
            sb.AppendFormat("        [6]: {0}\n", mValue[6]);
            sb.AppendFormat("        [7]: {0}\n", mValue[7]);
            sb.AppendFormat("        [8]: {0}\n", mValue[8]);
            sb.AppendFormat("        [9]: {0}\n", mValue[9]);
            sb.AppendFormat("        [10]: {0}\n", mValue[10]);
            sb.AppendFormat("        [11]: {0}\n", mValue[11]);
            sb.AppendFormat("        [12]: {0}\n", mValue[12]);
            sb.AppendFormat("        [13]: {0}\n", mValue[13]);
            sb.AppendFormat("        [14]: {0}\n", mValue[14]);
            sb.AppendFormat("        [15]: {0}\n", mValue[15]);
            sb.AppendFormat("        [16]: {0}\n", mValue[16]);
            sb.AppendFormat("        [17]: {0}\n", mValue[17]);
            sb.AppendFormat("        [18]: {0}\n", mValue[18]);
            sb.AppendFormat("        [19]: {0}\n", mValue[19]);
            sb.AppendFormat("        [20]: {0}\n", mValue[20]);
            sb.AppendFormat("        [21]: {0}\n", mValue[21]);
            sb.AppendFormat("        [22]: {0}\n", mValue[22]);
            sb.AppendFormat("        [23]: {0}\n", mValue[23]);
            sb.AppendFormat("        [24]: {0}\n", mValue[24]);
            sb.AppendFormat("        [25]: {0}\n", mValue[25]);
            sb.AppendFormat("        [26]: {0}\n", mValue[26]);
            sb.AppendFormat("        [27]: {0}\n", mValue[27]);
            sb.AppendFormat("        [28]: {0}\n", mValue[28]);
            sb.AppendFormat("        [29]: {0}\n", mValue[29]);
            sb.AppendFormat("        [30]: {0}\n", mValue[30]);
            sb.AppendFormat("        [31]: {0}\n", mValue[31]);

            return sb.ToString();
        }

        private UInt16 mAddress;
        private byte mVer;
        private byte mType;
        private SByte[] mValue = new SByte[32];
    }


    // ___________________________________________________________________________________


    public class UasDebugVect: UasMessage
    {
        /// <summary>
        /// Timestamp
        /// </summary>
        public UInt64 TimeUsec {
            get { return mTimeUsec; }
            set { mTimeUsec = value; NotifyUpdated(); }
        }

        /// <summary>
        /// x
        /// </summary>
        public float X {
            get { return mX; }
            set { mX = value; NotifyUpdated(); }
        }

        /// <summary>
        /// y
        /// </summary>
        public float Y {
            get { return mY; }
            set { mY = value; NotifyUpdated(); }
        }

        /// <summary>
        /// z
        /// </summary>
        public float Z {
            get { return mZ; }
            set { mZ = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Name
        /// </summary>
        public char[] Name {
            get { return mName; }
            set { mName = value; NotifyUpdated(); }
        }

        public UasDebugVect()
        {
            mMessageId = 250;
            CrcExtra = 49;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("DebugVect \n");
            sb.AppendFormat("    TimeUsec: {0}\n", mTimeUsec);
            sb.AppendFormat("    X: {0}\n", mX);
            sb.AppendFormat("    Y: {0}\n", mY);
            sb.AppendFormat("    Z: {0}\n", mZ);
            sb.Append("    Name\n");
            sb.AppendFormat("        [0]: {0}\n", mName[0]);
            sb.AppendFormat("        [1]: {0}\n", mName[1]);
            sb.AppendFormat("        [2]: {0}\n", mName[2]);
            sb.AppendFormat("        [3]: {0}\n", mName[3]);
            sb.AppendFormat("        [4]: {0}\n", mName[4]);
            sb.AppendFormat("        [5]: {0}\n", mName[5]);
            sb.AppendFormat("        [6]: {0}\n", mName[6]);
            sb.AppendFormat("        [7]: {0}\n", mName[7]);
            sb.AppendFormat("        [8]: {0}\n", mName[8]);
            sb.AppendFormat("        [9]: {0}\n", mName[9]);

            return sb.ToString();
        }

        private UInt64 mTimeUsec;
        private float mX;
        private float mY;
        private float mZ;
        private char[] mName = new char[10];
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    /// </summary>
    public class UasNamedValueFloat: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Floating point value
        /// </summary>
        public float Value {
            get { return mValue; }
            set { mValue = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Name of the debug variable
        /// </summary>
        public char[] Name {
            get { return mName; }
            set { mName = value; NotifyUpdated(); }
        }

        public UasNamedValueFloat()
        {
            mMessageId = 251;
            CrcExtra = 170;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("NamedValueFloat \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    Value: {0}\n", mValue);
            sb.Append("    Name\n");
            sb.AppendFormat("        [0]: {0}\n", mName[0]);
            sb.AppendFormat("        [1]: {0}\n", mName[1]);
            sb.AppendFormat("        [2]: {0}\n", mName[2]);
            sb.AppendFormat("        [3]: {0}\n", mName[3]);
            sb.AppendFormat("        [4]: {0}\n", mName[4]);
            sb.AppendFormat("        [5]: {0}\n", mName[5]);
            sb.AppendFormat("        [6]: {0}\n", mName[6]);
            sb.AppendFormat("        [7]: {0}\n", mName[7]);
            sb.AppendFormat("        [8]: {0}\n", mName[8]);
            sb.AppendFormat("        [9]: {0}\n", mName[9]);

            return sb.ToString();
        }

        private UInt32 mTimeBootMs;
        private float mValue;
        private char[] mName = new char[10];
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    /// </summary>
    public class UasNamedValueInt: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Signed integer value
        /// </summary>
        public Int32 Value {
            get { return mValue; }
            set { mValue = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Name of the debug variable
        /// </summary>
        public char[] Name {
            get { return mName; }
            set { mName = value; NotifyUpdated(); }
        }

        public UasNamedValueInt()
        {
            mMessageId = 252;
            CrcExtra = 44;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("NamedValueInt \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    Value: {0}\n", mValue);
            sb.Append("    Name\n");
            sb.AppendFormat("        [0]: {0}\n", mName[0]);
            sb.AppendFormat("        [1]: {0}\n", mName[1]);
            sb.AppendFormat("        [2]: {0}\n", mName[2]);
            sb.AppendFormat("        [3]: {0}\n", mName[3]);
            sb.AppendFormat("        [4]: {0}\n", mName[4]);
            sb.AppendFormat("        [5]: {0}\n", mName[5]);
            sb.AppendFormat("        [6]: {0}\n", mName[6]);
            sb.AppendFormat("        [7]: {0}\n", mName[7]);
            sb.AppendFormat("        [8]: {0}\n", mName[8]);
            sb.AppendFormat("        [9]: {0}\n", mName[9]);

            return sb.ToString();
        }

        private UInt32 mTimeBootMs;
        private Int32 mValue;
        private char[] mName = new char[10];
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
    /// </summary>
    public class UasStatustext: UasMessage
    {
        /// <summary>
        /// Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
        /// </summary>
        public MavSeverity Severity {
            get { return mSeverity; }
            set { mSeverity = value; NotifyUpdated(); }
        }

        /// <summary>
        /// Status text message, without null termination character
        /// </summary>
        public char[] Text {
            get { return mText; }
            set { mText = value; NotifyUpdated(); }
        }

        public UasStatustext()
        {
            mMessageId = 253;
            CrcExtra = 83;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("Statustext \n");
            sb.AppendFormat("    Severity: {0}\n", mSeverity);
            sb.Append("    Text\n");
            sb.AppendFormat("        [0]: {0}\n", mText[0]);
            sb.AppendFormat("        [1]: {0}\n", mText[1]);
            sb.AppendFormat("        [2]: {0}\n", mText[2]);
            sb.AppendFormat("        [3]: {0}\n", mText[3]);
            sb.AppendFormat("        [4]: {0}\n", mText[4]);
            sb.AppendFormat("        [5]: {0}\n", mText[5]);
            sb.AppendFormat("        [6]: {0}\n", mText[6]);
            sb.AppendFormat("        [7]: {0}\n", mText[7]);
            sb.AppendFormat("        [8]: {0}\n", mText[8]);
            sb.AppendFormat("        [9]: {0}\n", mText[9]);
            sb.AppendFormat("        [10]: {0}\n", mText[10]);
            sb.AppendFormat("        [11]: {0}\n", mText[11]);
            sb.AppendFormat("        [12]: {0}\n", mText[12]);
            sb.AppendFormat("        [13]: {0}\n", mText[13]);
            sb.AppendFormat("        [14]: {0}\n", mText[14]);
            sb.AppendFormat("        [15]: {0}\n", mText[15]);
            sb.AppendFormat("        [16]: {0}\n", mText[16]);
            sb.AppendFormat("        [17]: {0}\n", mText[17]);
            sb.AppendFormat("        [18]: {0}\n", mText[18]);
            sb.AppendFormat("        [19]: {0}\n", mText[19]);
            sb.AppendFormat("        [20]: {0}\n", mText[20]);
            sb.AppendFormat("        [21]: {0}\n", mText[21]);
            sb.AppendFormat("        [22]: {0}\n", mText[22]);
            sb.AppendFormat("        [23]: {0}\n", mText[23]);
            sb.AppendFormat("        [24]: {0}\n", mText[24]);
            sb.AppendFormat("        [25]: {0}\n", mText[25]);
            sb.AppendFormat("        [26]: {0}\n", mText[26]);
            sb.AppendFormat("        [27]: {0}\n", mText[27]);
            sb.AppendFormat("        [28]: {0}\n", mText[28]);
            sb.AppendFormat("        [29]: {0}\n", mText[29]);
            sb.AppendFormat("        [30]: {0}\n", mText[30]);
            sb.AppendFormat("        [31]: {0}\n", mText[31]);
            sb.AppendFormat("        [32]: {0}\n", mText[32]);
            sb.AppendFormat("        [33]: {0}\n", mText[33]);
            sb.AppendFormat("        [34]: {0}\n", mText[34]);
            sb.AppendFormat("        [35]: {0}\n", mText[35]);
            sb.AppendFormat("        [36]: {0}\n", mText[36]);
            sb.AppendFormat("        [37]: {0}\n", mText[37]);
            sb.AppendFormat("        [38]: {0}\n", mText[38]);
            sb.AppendFormat("        [39]: {0}\n", mText[39]);
            sb.AppendFormat("        [40]: {0}\n", mText[40]);
            sb.AppendFormat("        [41]: {0}\n", mText[41]);
            sb.AppendFormat("        [42]: {0}\n", mText[42]);
            sb.AppendFormat("        [43]: {0}\n", mText[43]);
            sb.AppendFormat("        [44]: {0}\n", mText[44]);
            sb.AppendFormat("        [45]: {0}\n", mText[45]);
            sb.AppendFormat("        [46]: {0}\n", mText[46]);
            sb.AppendFormat("        [47]: {0}\n", mText[47]);
            sb.AppendFormat("        [48]: {0}\n", mText[48]);
            sb.AppendFormat("        [49]: {0}\n", mText[49]);

            return sb.ToString();
        }

        private MavSeverity mSeverity;
        private char[] mText = new char[50];
    }


    // ___________________________________________________________________________________


    /// <summary>
    /// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
    /// </summary>
    public class UasDebug: UasMessage
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public UInt32 TimeBootMs {
            get { return mTimeBootMs; }
            set { mTimeBootMs = value; NotifyUpdated(); }
        }

        /// <summary>
        /// index of debug variable
        /// </summary>
        public byte Ind {
            get { return mInd; }
            set { mInd = value; NotifyUpdated(); }
        }

        /// <summary>
        /// DEBUG value
        /// </summary>
        public float Value {
            get { return mValue; }
            set { mValue = value; NotifyUpdated(); }
        }

        public UasDebug()
        {
            mMessageId = 254;
            CrcExtra = 86;
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

        public override string ToString()
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.Append("Debug \n");
            sb.AppendFormat("    TimeBootMs: {0}\n", mTimeBootMs);
            sb.AppendFormat("    Ind: {0}\n", mInd);
            sb.AppendFormat("    Value: {0}\n", mValue);

            return sb.ToString();
        }

        private UInt32 mTimeBootMs;
        private byte mInd;
        private float mValue;
    }


    // ___________________________________________________________________________________


    public class UasSummary
    {
        public static UasMessage CreateFromId(byte id)
        {
            switch (id)
            {
               case 0: return new UasHeartbeat();
               case 1: return new UasSysStatus();
               case 2: return new UasSystemTime();
               case 4: return new UasPing();
               case 5: return new UasChangeOperatorControl();
               case 6: return new UasChangeOperatorControlAck();
               case 7: return new UasAuthKey();
               case 11: return new UasSetMode();
               case 20: return new UasParamRequestRead();
               case 21: return new UasParamRequestList();
               case 22: return new UasParamValue();
               case 23: return new UasParamSet();
               case 24: return new UasGpsRawInt();
               case 25: return new UasGpsStatus();
               case 26: return new UasScaledImu();
               case 27: return new UasRawImu();
               case 28: return new UasRawPressure();
               case 29: return new UasScaledPressure();
               case 30: return new UasAttitude();
               case 31: return new UasAttitudeQuaternion();
               case 32: return new UasLocalPositionNed();
               case 33: return new UasGlobalPositionInt();
               case 34: return new UasRcChannelsScaled();
               case 35: return new UasRcChannelsRaw();
               case 36: return new UasServoOutputRaw();
               case 37: return new UasMissionRequestPartialList();
               case 38: return new UasMissionWritePartialList();
               case 39: return new UasMissionItem();
               case 40: return new UasMissionRequest();
               case 41: return new UasMissionSetCurrent();
               case 42: return new UasMissionCurrent();
               case 43: return new UasMissionRequestList();
               case 44: return new UasMissionCount();
               case 45: return new UasMissionClearAll();
               case 46: return new UasMissionItemReached();
               case 47: return new UasMissionAck();
               case 48: return new UasSetGpsGlobalOrigin();
               case 49: return new UasGpsGlobalOrigin();
               case 50: return new UasSetLocalPositionSetpoint();
               case 51: return new UasLocalPositionSetpoint();
               case 52: return new UasGlobalPositionSetpointInt();
               case 53: return new UasSetGlobalPositionSetpointInt();
               case 54: return new UasSafetySetAllowedArea();
               case 55: return new UasSafetyAllowedArea();
               case 56: return new UasSetRollPitchYawThrust();
               case 57: return new UasSetRollPitchYawSpeedThrust();
               case 58: return new UasRollPitchYawThrustSetpoint();
               case 59: return new UasRollPitchYawSpeedThrustSetpoint();
               case 60: return new UasSetQuadMotorsSetpoint();
               case 61: return new UasSetQuadSwarmRollPitchYawThrust();
               case 62: return new UasNavControllerOutput();
               case 63: return new UasSetQuadSwarmLedRollPitchYawThrust();
               case 64: return new UasStateCorrection();
               case 66: return new UasRequestDataStream();
               case 67: return new UasDataStream();
               case 69: return new UasManualControl();
               case 70: return new UasRcChannelsOverride();
               case 74: return new UasVfrHud();
               case 76: return new UasCommandLong();
               case 77: return new UasCommandAck();
               case 80: return new UasRollPitchYawRatesThrustSetpoint();
               case 81: return new UasManualSetpoint();
               case 89: return new UasLocalPositionNedSystemGlobalOffset();
               case 90: return new UasHilState();
               case 91: return new UasHilControls();
               case 92: return new UasHilRcInputsRaw();
               case 100: return new UasOpticalFlow();
               case 101: return new UasGlobalVisionPositionEstimate();
               case 102: return new UasVisionPositionEstimate();
               case 103: return new UasVisionSpeedEstimate();
               case 104: return new UasViconPositionEstimate();
               case 105: return new UasHighresImu();
               case 106: return new UasOmnidirectionalFlow();
               case 107: return new UasHilSensor();
               case 108: return new UasSimState();
               case 109: return new UasRadioStatus();
               case 110: return new UasFileTransferStart();
               case 111: return new UasFileTransferDirList();
               case 112: return new UasFileTransferRes();
               case 113: return new UasHilGps();
               case 114: return new UasHilOpticalFlow();
               case 115: return new UasHilStateQuaternion();
               case 147: return new UasBatteryStatus();
               case 148: return new UasSetpoint8dof();
               case 149: return new UasSetpoint6dof();
               case 249: return new UasMemoryVect();
               case 250: return new UasDebugVect();
               case 251: return new UasNamedValueFloat();
               case 252: return new UasNamedValueInt();
               case 253: return new UasStatustext();
               case 254: return new UasDebug();
               default: return null;
            }
        }

        public static byte GetCrcExtraForId(byte id)
        {
            switch (id)
            {
               case 0: return 50;
               case 1: return 124;
               case 2: return 137;
               case 4: return 237;
               case 5: return 217;
               case 6: return 104;
               case 7: return 119;
               case 11: return 89;
               case 20: return 214;
               case 21: return 159;
               case 22: return 220;
               case 23: return 168;
               case 24: return 24;
               case 25: return 23;
               case 26: return 170;
               case 27: return 144;
               case 28: return 67;
               case 29: return 115;
               case 30: return 39;
               case 31: return 246;
               case 32: return 185;
               case 33: return 104;
               case 34: return 237;
               case 35: return 244;
               case 36: return 222;
               case 37: return 212;
               case 38: return 9;
               case 39: return 254;
               case 40: return 230;
               case 41: return 28;
               case 42: return 28;
               case 43: return 132;
               case 44: return 221;
               case 45: return 232;
               case 46: return 11;
               case 47: return 153;
               case 48: return 41;
               case 49: return 39;
               case 50: return 214;
               case 51: return 223;
               case 52: return 141;
               case 53: return 33;
               case 54: return 15;
               case 55: return 3;
               case 56: return 100;
               case 57: return 24;
               case 58: return 239;
               case 59: return 238;
               case 60: return 30;
               case 61: return 240;
               case 62: return 183;
               case 63: return 130;
               case 64: return 130;
               case 66: return 148;
               case 67: return 21;
               case 69: return 243;
               case 70: return 124;
               case 74: return 20;
               case 76: return 152;
               case 77: return 143;
               case 80: return 127;
               case 81: return 106;
               case 89: return 231;
               case 90: return 183;
               case 91: return 63;
               case 92: return 54;
               case 100: return 175;
               case 101: return 102;
               case 102: return 158;
               case 103: return 208;
               case 104: return 56;
               case 105: return 93;
               case 106: return 211;
               case 107: return 108;
               case 108: return 32;
               case 109: return 185;
               case 110: return 235;
               case 111: return 93;
               case 112: return 124;
               case 113: return 124;
               case 114: return 119;
               case 115: return 4;
               case 147: return 177;
               case 148: return 241;
               case 149: return 15;
               case 249: return 204;
               case 250: return 49;
               case 251: return 170;
               case 252: return 44;
               case 253: return 83;
               case 254: return 86;
               default: return 0;
            }
        }
    }

}
