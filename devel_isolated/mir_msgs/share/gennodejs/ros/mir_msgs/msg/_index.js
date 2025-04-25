
"use strict";

let HookExtendedStatus = require('./HookExtendedStatus.js');
let HeightState = require('./HeightState.js');
let RobotState = require('./RobotState.js');
let Twist2D = require('./Twist2D.js');
let SkidDetectionDiff = require('./SkidDetectionDiff.js');
let Gpio = require('./Gpio.js');
let EncoderTestEntry = require('./EncoderTestEntry.js');
let Devices = require('./Devices.js');
let ResourceState = require('./ResourceState.js');
let PowerBoardMotorStatus = require('./PowerBoardMotorStatus.js');
let HookStatus = require('./HookStatus.js');
let GripperState = require('./GripperState.js');
let ExternalRobots = require('./ExternalRobots.js');
let WorldMap = require('./WorldMap.js');
let ExternalRobot = require('./ExternalRobot.js');
let MissionCtrlCommand = require('./MissionCtrlCommand.js');
let ResourcesAcquisition = require('./ResourcesAcquisition.js');
let MissionCtrlState = require('./MissionCtrlState.js');
let Path = require('./Path.js');
let BatteryCurrents = require('./BatteryCurrents.js');
let Encoders = require('./Encoders.js');
let SkidDetectionStampedFloat = require('./SkidDetectionStampedFloat.js');
let MirLocalPlannerPathTypes = require('./MirLocalPlannerPathTypes.js');
let PlanSegment = require('./PlanSegment.js');
let HookData = require('./HookData.js');
let Serial = require('./Serial.js');
let StampedEncoders = require('./StampedEncoders.js');
let MirExtra = require('./MirExtra.js');
let RobotStatus = require('./RobotStatus.js');
let Device = require('./Device.js');
let PalletLifterStatus = require('./PalletLifterStatus.js');
let WebPath = require('./WebPath.js');
let SoundEvent = require('./SoundEvent.js');
let ResourcesState = require('./ResourcesState.js');
let BMSData = require('./BMSData.js');
let PlanSegments = require('./PlanSegments.js');
let LocalMapStat = require('./LocalMapStat.js');
let IOs = require('./IOs.js');
let Proximity = require('./Proximity.js');
let RobotMode = require('./RobotMode.js');
let Event = require('./Event.js');
let MovingState = require('./MovingState.js');
let Error = require('./Error.js');
let ServiceResponseHeader = require('./ServiceResponseHeader.js');
let AngleMeasurment = require('./AngleMeasurment.js');
let ChargingState = require('./ChargingState.js');
let Pendant = require('./Pendant.js');
let JoystickVel = require('./JoystickVel.js');
let Events = require('./Events.js');
let WorldModel = require('./WorldModel.js');
let Pose2D = require('./Pose2D.js');
let TimeDebug = require('./TimeDebug.js');
let UserPrompt = require('./UserPrompt.js');
let SafetyStatus = require('./SafetyStatus.js');
let PrecisionDockingStatus = require('./PrecisionDockingStatus.js');
let BrakeState = require('./BrakeState.js');
let Trolley = require('./Trolley.js');

module.exports = {
  HookExtendedStatus: HookExtendedStatus,
  HeightState: HeightState,
  RobotState: RobotState,
  Twist2D: Twist2D,
  SkidDetectionDiff: SkidDetectionDiff,
  Gpio: Gpio,
  EncoderTestEntry: EncoderTestEntry,
  Devices: Devices,
  ResourceState: ResourceState,
  PowerBoardMotorStatus: PowerBoardMotorStatus,
  HookStatus: HookStatus,
  GripperState: GripperState,
  ExternalRobots: ExternalRobots,
  WorldMap: WorldMap,
  ExternalRobot: ExternalRobot,
  MissionCtrlCommand: MissionCtrlCommand,
  ResourcesAcquisition: ResourcesAcquisition,
  MissionCtrlState: MissionCtrlState,
  Path: Path,
  BatteryCurrents: BatteryCurrents,
  Encoders: Encoders,
  SkidDetectionStampedFloat: SkidDetectionStampedFloat,
  MirLocalPlannerPathTypes: MirLocalPlannerPathTypes,
  PlanSegment: PlanSegment,
  HookData: HookData,
  Serial: Serial,
  StampedEncoders: StampedEncoders,
  MirExtra: MirExtra,
  RobotStatus: RobotStatus,
  Device: Device,
  PalletLifterStatus: PalletLifterStatus,
  WebPath: WebPath,
  SoundEvent: SoundEvent,
  ResourcesState: ResourcesState,
  BMSData: BMSData,
  PlanSegments: PlanSegments,
  LocalMapStat: LocalMapStat,
  IOs: IOs,
  Proximity: Proximity,
  RobotMode: RobotMode,
  Event: Event,
  MovingState: MovingState,
  Error: Error,
  ServiceResponseHeader: ServiceResponseHeader,
  AngleMeasurment: AngleMeasurment,
  ChargingState: ChargingState,
  Pendant: Pendant,
  JoystickVel: JoystickVel,
  Events: Events,
  WorldModel: WorldModel,
  Pose2D: Pose2D,
  TimeDebug: TimeDebug,
  UserPrompt: UserPrompt,
  SafetyStatus: SafetyStatus,
  PrecisionDockingStatus: PrecisionDockingStatus,
  BrakeState: BrakeState,
  Trolley: Trolley,
};
