// Copyright (c) 2021-2024 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

import {
  Subject,
  MappedSubject,
  DebounceTimer,
  ConsumerValue,
  EventBus,
  ConsumerSubject,
  SimVarValueType,
  SubscribableMapFunctions,
  StallWarningEvents,
  Instrument,
} from '@microsoft/msfs-sdk';

import {
  Arinc429Register,
  Arinc429RegisterSubject,
  Arinc429SignStatusMatrix,
  Arinc429Word,
  FrequencyMode,
  NXLogicClockNode,
  NXLogicConfirmNode,
  NXLogicMemoryNode,
  NXLogicPulseNode,
  NXLogicTriggeredMonostableNode,
} from '@flybywiresim/fbw-sdk';
import { VerticalMode } from '@shared/autopilot';
import { VhfComManagerDataEvents } from '@flybywiresim/rmp';
import { PseudoFwcSimvars } from 'instruments/src/MsfsAvionicsCommon/providers/PseudoFwcPublisher';
import { FuelSystemEvents } from 'instruments/src/MsfsAvionicsCommon/providers/FuelSystemPublisher';
import {
  AbnormalProcedure,
  EcamAbnormalSensedProcedures,
  EcamMemos,
  isChecklistAction,
  pfdMemoDisplay,
} from '@instruments/common/EcamMessages';
import PitchTrimUtils from '@shared/PitchTrimUtils';
import {
  FwsCdsAbnormalSensedEntry,
  FwsCdsAbnormalSensedList,
  FwsCdsEvents,
} from '../../../instruments/src/MsfsAvionicsCommon/providers/FwsCdsPublisher';
import { FwsMemos } from 'systems-host/systems/FlightWarningSystem/FwsMemos';
import { FwsNormalChecklists } from 'systems-host/systems/FlightWarningSystem/FwsNormalChecklists';
import { EwdAbnormalItem, FwsAbnormalSensed } from 'systems-host/systems/FlightWarningSystem/FwsAbnormalSensed';
import { FwsAbnormalNonSensed } from 'systems-host/systems/FlightWarningSystem/FwsAbnormalNonSensed';

export function xor(a: boolean, b: boolean): boolean {
  return !!((a ? 1 : 0) ^ (b ? 1 : 0));
}

/**
 * Counts the number of truthy values in an array of booleans
 * @param args
 * @returns
 */
function countTrue(...args: boolean[]): number {
  return args.reduce((accu, b) => (b ? accu + 1 : accu), 0);
}

export enum FwcAuralWarning {
  None,
  SingleChime,
  Crc,
}

type InternalAbnormalSensedList = Map<string, FwsCdsAbnormalSensedEntry>;

export class FwsCore implements Instrument {
  public readonly sub = this.bus.getSubscriber<PseudoFwcSimvars & StallWarningEvents>();
  public readonly vhfSub = this.bus.getSubscriber<VhfComManagerDataEvents>();
  /** Time to inhibit master warnings and cautions during startup in ms */
  private static readonly FWC_STARTUP_TIME = 5000;

  /** Time to inhibit SCs after one is trigger in ms */
  private static readonly AURAL_SC_INHIBIT_TIME = 2000;

  /** The time to play the single chime sound in ms */
  private static readonly AURAL_SC_PLAY_TIME = 500;

  private static readonly EWD_MESSAGE_LINES = 10;

  private static readonly PFD_MEMO_LINES = 3;

  private static readonly PFD_LIMITATIONS_LINES = 8;

  private static readonly EWD_LIMITATIONS_LINES = 10;

  private static readonly SD_STATUS_INFO_MAX_LINES = 5;

  private static readonly SD_STATUS_INOP_SYS_MAX_LINES = 10;

  private static readonly ewdMessageSimVarsLeft = Array.from(
    { length: FwsCore.EWD_MESSAGE_LINES },
    (_, i) => `L:A32NX_EWD_LOWER_LEFT_LINE_${i + 1}`,
  );

  private readonly ewdMessageLinesLeft = Array.from({ length: FwsCore.EWD_MESSAGE_LINES }, (_, _i) =>
    Subject.create(''),
  );

  private static readonly ewdMessageSimVarsRight = Array.from(
    { length: FwsCore.EWD_MESSAGE_LINES },
    (_, i) => `L:A32NX_EWD_LOWER_RIGHT_LINE_${i + 1}`,
  );

  private readonly ewdMessageLinesRight = Array.from({ length: FwsCore.EWD_MESSAGE_LINES }, (_, _i) =>
    Subject.create(''),
  );

  private static readonly pfdMemoSimVars = Array.from(
    { length: FwsCore.PFD_MEMO_LINES },
    (_, i) => `L:A32NX_PFD_MEMO_LINE_${i + 1}`,
  );

  private readonly pfdMemoLines = Array.from({ length: FwsCore.PFD_MEMO_LINES }, (_, _i) => Subject.create(''));

  private static readonly sdStatusInfoSimVars = Array.from(
    { length: FwsCore.SD_STATUS_INFO_MAX_LINES },
    (_, i) => `L:A32NX_SD_STATUS_INFO_LINE_${i + 1}`,
  );

  private readonly sdStatusInfoLines = Array.from({ length: FwsCore.SD_STATUS_INFO_MAX_LINES }, (_, _i) =>
    Subject.create(''),
  );

  private static readonly sdStatusInopAllPhasesSimVars = Array.from(
    { length: FwsCore.SD_STATUS_INOP_SYS_MAX_LINES },
    (_, i) => `L:A32NX_SD_STATUS_INOP_ALL_LINE_${i + 1}`,
  );

  private readonly sdStatusInopAllPhasesLines = Array.from({ length: FwsCore.SD_STATUS_INOP_SYS_MAX_LINES }, (_, _i) =>
    Subject.create(''),
  );

  private static readonly sdStatusInopApprLdgSimVars = Array.from(
    { length: FwsCore.SD_STATUS_INOP_SYS_MAX_LINES },
    (_, i) => `L:A32NX_SD_STATUS_INOP_LDG_LINE_${i + 1}`,
  );

  private readonly sdStatusInopApprLdgLines = Array.from({ length: FwsCore.SD_STATUS_INOP_SYS_MAX_LINES }, (_, _i) =>
    Subject.create(''),
  );

  private static readonly pfdLimitationsSimVars = Array.from(
    { length: FwsCore.PFD_LIMITATIONS_LINES },
    (_, i) => `L:A32NX_PFD_LIMITATIONS_LINE_${i + 1}`,
  );

  private readonly pfdLimitationsLines = Array.from({ length: FwsCore.PFD_LIMITATIONS_LINES }, (_, _i) =>
    Subject.create(''),
  );

  private static readonly ewdLimitationsAllPhasesSimVars = Array.from(
    { length: FwsCore.EWD_LIMITATIONS_LINES },
    (_, i) => `L:A32NX_EWD_LIMITATIONS_ALL_LINE_${i + 1}`,
  );

  private readonly ewdLimitationsAllPhasesLines = Array.from({ length: FwsCore.EWD_LIMITATIONS_LINES }, (_, _i) =>
    Subject.create(''),
  );

  private static readonly ewdLimitationsApprLdgSimVars = Array.from(
    { length: FwsCore.EWD_LIMITATIONS_LINES },
    (_, i) => `L:A32NX_EWD_LIMITATIONS_LDG_LINE_${i + 1}`,
  );

  private readonly ewdLimitationsApprLdgLines = Array.from({ length: FwsCore.EWD_LIMITATIONS_LINES }, (_, _i) =>
    Subject.create(''),
  );

  /* PSEUDO FWC VARIABLES */
  private readonly startupTimer = new DebounceTimer();

  private readonly startupCompleted = Subject.create(false);

  /** Keys/IDs of all failures currently active, irrespective they are already cleared or not */
  public readonly allCurrentFailures: string[] = [];

  /** Keys/IDs of only the failures which are currently presented on the EWD */
  public readonly presentedFailures: string[] = [];

  /** Map to hold all failures which are currently active */
  public readonly activeAbnormalSensedList: InternalAbnormalSensedList = new Map();

  private recallFailures: string[] = [];

  private auralCrcKeys: string[] = [];

  private auralScKeys: string[] = [];

  public readonly auralCrcActive = Subject.create(false);

  private auralSingleChimePending = false;

  public readonly auralSingleChimeInhibitTimer = new DebounceTimer();

  public readonly auralSingleChimePlayingTimer = new DebounceTimer();

  public readonly masterWarning = Subject.create(false);

  public readonly masterCaution = Subject.create(false);

  public readonly fireActive = Subject.create(false);

  private nonCancellableWarningCount = 0;

  public readonly stallWarning = Subject.create(false);

  public readonly masterWarningOutput = MappedSubject.create(
    SubscribableMapFunctions.or(),
    this.masterWarning,
    this.fireActive,
    this.stallWarning,
  );

  public readonly auralCrcOutput = MappedSubject.create(
    ([auralCrc, fireActive]) => auralCrc || fireActive,
    this.auralCrcActive,
    this.fireActive,
  );

  public readonly fwcOut126 = Arinc429RegisterSubject.createEmpty();

  /* 21 - AIR CONDITIONING AND PRESSURIZATION */

  public readonly acsc1DiscreteWord1 = Arinc429Register.empty();

  public readonly acsc1DiscreteWord2 = Arinc429Register.empty();

  public readonly acsc2DiscreteWord1 = Arinc429Register.empty();

  public readonly acsc2DiscreteWord2 = Arinc429Register.empty();

  public readonly cpc1DiscreteWord = Arinc429Register.empty();

  public readonly cpc2DiscreteWord = Arinc429Register.empty();

  public readonly apuBleedValveOpen = Subject.create(false);

  public readonly cabAltSetReset1 = new NXLogicMemoryNode();

  public readonly cabAltSetReset2 = new NXLogicMemoryNode();

  public readonly cabAltSetResetState1 = Subject.create(false);

  public readonly cabAltSetResetState2 = Subject.create(false);

  public readonly cabFanHasFault1 = Subject.create(false);

  public readonly cabFanHasFault2 = Subject.create(false);

  public readonly excessPressure = Subject.create(false);

  public readonly enginesOffAndOnGroundSignal = new NXLogicConfirmNode(7);

  public readonly excessResidualPrConfirm = new NXLogicConfirmNode(5);

  public readonly excessResidualPr = Subject.create(false);

  public readonly lowDiffPress = Subject.create(false);

  public readonly acsc1Lane1Fault = Subject.create(false);

  public readonly acsc1Lane2Fault = Subject.create(false);

  public readonly acsc2Lane1Fault = Subject.create(false);

  public readonly acsc2Lane2Fault = Subject.create(false);

  public readonly acsc1Fault = Subject.create(false);

  public readonly acsc2Fault = Subject.create(false);

  public readonly pack1And2Fault = Subject.create(false);

  public readonly ramAirOn = Subject.create(false);

  public readonly hotAirDisagrees = Subject.create(false);

  public readonly hotAirOpen = Subject.create(false);

  public readonly hotAirPbOn = Subject.create(false);

  public readonly trimAirFault = Subject.create(false);

  public readonly ckptTrimFault = Subject.create(false);

  public readonly fwdTrimFault = Subject.create(false);

  public readonly aftTrimFault = Subject.create(false);

  public readonly trimAirHighPressure = Subject.create(false);

  public readonly ckptDuctOvht = Subject.create(false);

  public readonly fwdDuctOvht = Subject.create(false);

  public readonly aftDuctOvht = Subject.create(false);

  public readonly anyDuctOvht = Subject.create(false);

  public readonly lavGalleyFanFault = Subject.create(false);

  public readonly pack1On = Subject.create(false);

  public readonly pack2On = Subject.create(false);

  public readonly packOffBleedAvailable1 = new NXLogicConfirmNode(5, false);

  public readonly packOffBleedAvailable2 = new NXLogicConfirmNode(5, false);

  public readonly packOffNotFailed1 = new NXLogicConfirmNode(60);

  public readonly packOffNotFailed1Status = Subject.create(false);

  public readonly packOffNotFailed2 = new NXLogicConfirmNode(60);

  public readonly packOffNotFailed2Status = Subject.create(false);

  public readonly cpc1Fault = Subject.create(false);

  public readonly cpc2Fault = Subject.create(false);

  public readonly bothCpcFault = new NXLogicConfirmNode(3, false);

  public readonly bothCpcFaultOutput = Subject.create(false);

  public readonly pressurizationAuto = Subject.create(false);

  public readonly outflowValveOpenAmount = Subject.create(0);

  public readonly outflowValveNotOpen = new NXLogicConfirmNode(70);

  public readonly outflowValveResetCondition = new NXLogicConfirmNode(30);

  public readonly outflowValveNotOpenOutput = Subject.create(false);

  public readonly outflowValveNotOpenSetReset = new NXLogicMemoryNode();

  public readonly safetyValveNotClosedAir = new NXLogicConfirmNode(60);

  public readonly safetyValveNotClosedOutput = Subject.create(false);

  public readonly cabinDeltaPressure = Subject.create(0);

  /* 22 - AUTOFLIGHT */

  public readonly toConfigAndNoToSpeedsPulseNode = new NXLogicPulseNode();

  /** TO speeds not inserted RS */
  public toSpeedsNotInserted = false;

  public readonly toSpeedsNotInsertedWarning = Subject.create(false);

  /** TO CONF pressed in phase 2 or 3 SR */
  private toConfigCheckedInPhase2Or3 = false;

  public readonly toSpeedsTooLowWarning = Subject.create(false);

  public readonly toV2VRV2DisagreeWarning = Subject.create(false);

  public readonly fmcAFault = Subject.create(false);

  public readonly fmcBFault = Subject.create(false);

  public readonly fmcCFault = Subject.create(false);

  public readonly fms1Fault = Subject.create(false);

  public readonly fms2Fault = Subject.create(false);

  /* 23 - COMMUNICATION */
  public readonly rmp1Fault = Subject.create(false);

  public readonly rmp2Fault = Subject.create(false);

  public readonly rmp3Fault = Subject.create(false);

  public readonly rmp1Off = Subject.create(false);

  public readonly rmp2Off = Subject.create(false);

  public readonly rmp3Off = Subject.create(false);

  public readonly rmp3ActiveMode = ConsumerSubject.create(this.vhfSub.on(`vhf_com_active_mode_3`), FrequencyMode.Data);

  /* 24 - ELECTRICAL */

  public readonly ac1BusPowered = Subject.create(false);

  public readonly ac2BusPowered = Subject.create(false);

  public readonly ac3BusPowered = Subject.create(false);

  public readonly ac4BusPowered = Subject.create(false);

  public readonly acESSBusPowered = Subject.create(false);

  public readonly dcESSBusPowered = Subject.create(false);

  public readonly dc2BusPowered = Subject.create(false);

  public readonly extPwrConnected = Subject.create(false);

  public readonly engine1Running = Subject.create(false);

  public readonly engine2Running = Subject.create(false);

  public readonly engine3Running = Subject.create(false);

  public readonly engine4Running = Subject.create(false);

  /* 27 - FLIGHT CONTROLS */

  public readonly altn1LawConfirmNode = new NXLogicConfirmNode(0.3, true);

  public readonly altn1LawConfirmNodeOutput = Subject.create(false);

  public readonly altn2LawConfirmNode = new NXLogicConfirmNode(0.3, true);

  public readonly altn2LawConfirmNodeOutput = Subject.create(false);

  public readonly directLawCondition = Subject.create(false);

  public readonly elac1HydConfirmNode = new NXLogicConfirmNode(3, false);

  public readonly elac1FaultConfirmNode = new NXLogicConfirmNode(0.6, true);

  public readonly elac1FaultConfirmNodeOutput = Subject.create(false);

  public readonly elac1FaultLine123Display = Subject.create(false);

  public readonly elac1FaultLine45Display = Subject.create(false);

  public readonly elac1HydConfirmNodeOutput = Subject.create(false);

  public readonly elac2FaultConfirmNode = new NXLogicConfirmNode(0.6, true);

  public readonly elac2FaultConfirmNodeOutput = Subject.create(false);

  public readonly elac2FaultLine123Display = Subject.create(false);

  public readonly elac2FaultLine45Display = Subject.create(false);

  public readonly elac2HydConfirmNode = new NXLogicConfirmNode(3, false);

  public readonly elac2HydConfirmNodeOutput = Subject.create(false);

  public readonly fcdc1FaultCondition = Subject.create(false);

  public readonly fcdc12FaultCondition = Subject.create(false);

  public readonly fcdc2FaultCondition = Subject.create(false);

  public readonly flapsAngle = Subject.create(0);

  public readonly flapsHandle = Subject.create(0);

  public readonly lrElevFaultCondition = Subject.create(false);

  public readonly sec1FaultCondition = Subject.create(false);

  public readonly sec2FaultCondition = Subject.create(false);

  public readonly sec3FaultCondition = Subject.create(false);

  public readonly sec1FaultLine123Display = Subject.create(false);

  public readonly sec1FaultLine45Display = Subject.create(false);

  public readonly sec2FaultLine123Display = Subject.create(false);

  public readonly sec3FaultLine123Display = Subject.create(false);

  public readonly showLandingInhibit = Subject.create(false);

  public readonly showTakeoffInhibit = Subject.create(false);

  public readonly slatsAngle = Subject.create(0);

  public readonly speedBrakeCommand = Subject.create(false);

  public readonly spoilersArmed = Subject.create(false);

  public slatFlapSelectionS0F0 = false;

  public slatFlapSelectionS18F10 = false;

  public slatFlapSelectionS22F15 = false;

  public slatFlapSelectionS22F20 = false;

  public readonly flapsInferiorToPositionA = Subject.create(false);

  public readonly flapsSuperiorToPositionD = Subject.create(false);

  public readonly flapsSuperiorToPositionF = Subject.create(false);

  public readonly slatsInferiorToPositionD = Subject.create(false);

  public readonly slatsSuperiorToPositionG = Subject.create(false);

  public readonly flapsSuperiorToPositionDOrSlatsSuperiorToPositionC = Subject.create(false);

  public readonly flapsNotTo = Subject.create(false);

  public readonly flapsNotToMemo = Subject.create(false);

  public readonly flapConfigSr = new NXLogicMemoryNode(true);

  public readonly flapConfigAural = Subject.create(false);

  public readonly flapConfigWarning = Subject.create(false);

  public readonly slatsNotTo = Subject.create(false);

  public readonly slatConfigSr = new NXLogicMemoryNode(true);

  public readonly slatConfigAural = Subject.create(false);

  public readonly slatConfigWarning = Subject.create(false);

  public readonly speedbrakesNotTo = Subject.create(false);

  public readonly speedbrakesConfigSr = new NXLogicMemoryNode(true);

  public readonly speedbrakesConfigAural = Subject.create(false);

  public readonly speedbrakesConfigWarning = Subject.create(false);

  public readonly flapsMcduDisagree = Subject.create(false);

  public readonly flapsAndPitchMcduDisagreeEnable = Subject.create(false);

  public readonly pitchConfigInPhase3or4Sr = new NXLogicMemoryNode(true);

  public readonly pitchTrimNotTo = Subject.create(false);

  public readonly pitchTrimNotToAudio = Subject.create(false);

  public readonly pitchTrimNotToWarning = Subject.create(false);

  public readonly pitchTrimMcduCgDisagree = Subject.create(false);

  public readonly trimDisagreeMcduStabConf = new NXLogicConfirmNode(1, true);

  public readonly rudderTrimConfigInPhase3or4Sr = new NXLogicMemoryNode(true);

  public readonly rudderTrimNotTo = Subject.create(false);

  public readonly rudderTrimNotToAudio = Subject.create(false);

  public readonly rudderTrimNotToWarning = Subject.create(false);

  public readonly flapsLeverNotZeroWarning = Subject.create(false);

  public readonly speedBrakeCommand5sConfirm = new NXLogicConfirmNode(5, true);

  public readonly speedBrakeCommand50sConfirm = new NXLogicConfirmNode(50, true);

  public readonly speedBrakeCaution1Confirm = new NXLogicConfirmNode(30, true);

  public readonly engAboveIdleWithSpeedBrakeConfirm = new NXLogicConfirmNode(10, false);

  public readonly apTcasRaNoseUpConfirm = new NXLogicConfirmNode(4, true);

  public readonly speedBrakeCaution3Confirm = new NXLogicConfirmNode(3, true);

  public readonly speedBrakeCaution3Monostable = new NXLogicTriggeredMonostableNode(1.5, true);

  public readonly speedBrakeCaution1Pulse = new NXLogicPulseNode(true);

  public readonly speedBrakeCaution2Pulse = new NXLogicPulseNode(true);

  public readonly speedBrakeStillOutWarning = Subject.create(false);

  public readonly amberSpeedBrake = Subject.create(false);

  public readonly phase84s5Trigger = new NXLogicTriggeredMonostableNode(4.5, false);

  public readonly groundSpoiler5sDelayed = new NXLogicConfirmNode(5, false);

  public readonly speedBrake5sDelayed = new NXLogicConfirmNode(5, false);

  public readonly groundSpoilerNotArmedWarning = Subject.create(false);

  /* FUEL */

  public readonly centerFuelPump1Auto = ConsumerValue.create(null, false);

  public readonly centerFuelPump2Auto = ConsumerValue.create(null, false);

  public readonly centerFuelQuantity = Subject.create(0);

  public readonly fuelXFeedPBOn = Subject.create(false);

  public readonly leftOuterInnerValve = ConsumerSubject.create(null, 0);

  public readonly leftFuelLow = Subject.create(false);

  public readonly leftFuelLowConfirm = new NXLogicConfirmNode(30, true);

  public readonly leftFuelPump1Auto = ConsumerValue.create(null, false);

  public readonly leftFuelPump2Auto = ConsumerValue.create(null, false);

  public readonly lrTankLow = Subject.create(false);

  public readonly lrTankLowConfirm = new NXLogicConfirmNode(30, true);

  public readonly rightOuterInnerValve = ConsumerSubject.create(null, 0);

  public readonly rightFuelLow = Subject.create(false);

  public readonly rightFuelLowConfirm = new NXLogicConfirmNode(30, true);

  public readonly rightFuelPump1Auto = ConsumerValue.create(null, false);

  public readonly rightFuelPump2Auto = ConsumerValue.create(null, false);

  public readonly fuelCtrTankModeSelMan = ConsumerValue.create(null, false);

  /* HYDRAULICS */

  public readonly blueElecPumpPBAuto = Subject.create(false);

  public readonly blueLP = Subject.create(false);

  public readonly blueRvrLow = Subject.create(false);

  public readonly blueRvrOvht = Subject.create(false);

  public readonly eng1pumpPBisAuto = Subject.create(false);

  public readonly eng2pumpPBisAuto = Subject.create(false);

  public readonly greenHydEng1PBAuto = Subject.create(false);

  public readonly greenLP = Subject.create(false);

  public readonly greenRvrOvht = Subject.create(false);

  public readonly hydPTU = Subject.create(false);

  public readonly ptuAuto = Subject.create(false);

  public readonly ratDeployed = Subject.create(0);

  public readonly yellowLP = Subject.create(false);

  public readonly yellowRvrOvht = Subject.create(false);

  public readonly yepumpPBisAuto = Subject.create(false);

  public readonly greenAPumpOn = Subject.create(false);

  public readonly greenBPumpOn = Subject.create(false);

  public readonly yellowAPumpOn = Subject.create(false);

  public readonly yellowBPumpOn = Subject.create(false);

  /* 31 - FWS */

  public readonly fwcFlightPhase = Subject.create(-1);

  public readonly flightPhase126 = Subject.create(false);

  public readonly flightPhase23 = Subject.create(false);

  public readonly flightPhase34 = Subject.create(false);

  public readonly flightPhase345 = Subject.create(false);

  public readonly flightPhase129 = Subject.create(false);

  public readonly flightPhase67 = Subject.create(false);

  public readonly flightPhase78 = Subject.create(false);

  public readonly ldgInhibitTimer = new NXLogicConfirmNode(3);

  public readonly toInhibitTimer = new NXLogicConfirmNode(3);

  /** TO CONFIG TEST raw button input */
  private toConfigTestRaw = false;

  /** TO CONFIG TEST pulse with 0.5s monostable trigger */
  private toConfigTest = false;

  public readonly toConfigPulseNode = new NXLogicPulseNode();

  public readonly toConfigTriggerNode = new NXLogicTriggeredMonostableNode(0.5, true);

  public readonly toConfigTestHeldMin1s5PulseNode = new NXLogicTriggeredMonostableNode(1.5, true);

  /** this will be true whenever the TO CONFIG TEST button is pressed, and stays on for a minimum of 1.5s */
  public readonly toConfigTestHeldMin1s5Pulse = Subject.create(false);

  private toConfigNormalConf = new NXLogicConfirmNode(0.3, false);

  public readonly clr1PulseNode = new NXLogicPulseNode();

  public readonly clr2PulseNode = new NXLogicPulseNode();

  public readonly clrPulseUpTrigger = new NXLogicTriggeredMonostableNode(0.5, true);

  private clrTriggerRisingEdge = false;

  public readonly rclUpPulseNode = new NXLogicPulseNode();

  public readonly rclUpTriggerNode = new NXLogicTriggeredMonostableNode(0.5, true);

  private recallTriggerRisingEdge = false;

  public readonly flightPhase3PulseNode = new NXLogicPulseNode();

  public readonly flightPhaseEndedPulseNode = new NXLogicPulseNode();

  public readonly flightPhaseInhibitOverrideNode = new NXLogicMemoryNode(false);

  /* LANDING GEAR AND LIGHTS */

  public readonly aircraftOnGround = Subject.create(false);

  public readonly antiskidActive = Subject.create(false);

  public readonly brakeFan = Subject.create(false);

  public readonly brakesHot = Subject.create(false);

  public readonly leftLandingLightExtended = Subject.create(false);

  public readonly rightlandingLightExtended = Subject.create(false);

  public readonly lgciu1Fault = Subject.create(false);

  public readonly lgciu2Fault = Subject.create(false);

  public readonly lgciu1DiscreteWord1 = Arinc429Register.empty();

  public readonly lgciu2DiscreteWord1 = Arinc429Register.empty();

  public readonly lgciu1DiscreteWord2 = Arinc429Register.empty();

  public readonly lgciu2DiscreteWord2 = Arinc429Register.empty();

  public readonly nwSteeringDisc = Subject.create(false);

  public readonly parkBrake = Subject.create(false);

  public readonly lgNotDown = Subject.create(false);

  public readonly lgNotDownNoCancel = Subject.create(false);

  public readonly lgLeverRedArrow = Subject.create(false);

  public readonly lgNotDownPulse1 = new NXLogicPulseNode();

  public readonly lgNotDownPulse2 = new NXLogicPulseNode();

  public readonly lgciu1OnGroundDisagreeConf = new NXLogicConfirmNode(1, true);

  public readonly lgciu1OnGroundAgreeConf = new NXLogicConfirmNode(0.5, true);

  public readonly lgciu1OnGroundDisagreeMem = new NXLogicMemoryNode(true);

  public readonly lgciu2OnGroundDisagreeConf = new NXLogicConfirmNode(1, true);

  public readonly lgciu2OnGroundAgreeConf = new NXLogicConfirmNode(0.5, true);

  public readonly lgciu2OnGroundDisagreeMem = new NXLogicMemoryNode(true);

  public readonly ra1OnGroundMem = new NXLogicMemoryNode(true);

  public readonly ra2OnGroundMem = new NXLogicMemoryNode(true);

  public readonly ra3OnGroundMem = new NXLogicMemoryNode(true);

  public readonly ignoreRaOnGroundTrigger = new NXLogicTriggeredMonostableNode(10, true);

  public readonly onGroundConf = new NXLogicConfirmNode(1, true);

  private onGroundImmediate = false;

  /* NAVIGATION */

  public readonly adirsRemainingAlignTime = Subject.create(0);

  public readonly adiru1State = Subject.create(0);

  public readonly adiru2State = Subject.create(0);

  public readonly adiru3State = Subject.create(0);

  public readonly adr1Cas = Subject.create(Arinc429Word.empty());
  public readonly adr2Cas = Arinc429Register.empty();
  public readonly adr3Cas = Arinc429Register.empty();

  public readonly adr1Fault = Subject.create(false);
  public readonly adr2Fault = Subject.create(false);
  public readonly adr3Fault = Subject.create(false);

  public readonly computedAirSpeedToNearest2 = this.adr1Cas.map((it) => Math.round(it.value / 2) * 2);

  public readonly ir1MaintWord = Arinc429Register.empty();
  public readonly ir2MaintWord = Arinc429Register.empty();
  public readonly ir3MaintWord = Arinc429Register.empty();

  public readonly ir1Pitch = Arinc429Register.empty();
  public readonly ir2Pitch = Arinc429Register.empty();
  public readonly ir3Pitch = Arinc429Register.empty();

  public readonly ir1Fault = Subject.create(false);
  public readonly ir2Fault = Subject.create(false);
  public readonly ir3Fault = Subject.create(false);

  public readonly irExcessMotion = Subject.create(false);

  public readonly extremeLatitudeAlert = Subject.create(false);

  public readonly height1Failed = Subject.create(false);

  public readonly height2Failed = Subject.create(false);

  public readonly height3Failed = Subject.create(false);

  private adr3OverspeedWarning = new NXLogicMemoryNode(false, false);

  public readonly overspeedVmo = Subject.create(false);

  public readonly overspeedVle = Subject.create(false);

  public readonly overspeedVfeConf1 = Subject.create(false);

  public readonly overspeedVfeConf1F = Subject.create(false);

  public readonly overspeedVfeConf2 = Subject.create(false);

  public readonly overspeedVfeConf3 = Subject.create(false);

  public readonly overspeedVfeConfFull = Subject.create(false);

  public readonly flapsIndex = Subject.create(0);

  private stallWarningRaw = ConsumerValue.create(this.sub.on('stall_warning_on'), false);

  public readonly trueNorthRef = Subject.create(false);

  /* SURVEILLANCE */

  public readonly gpwsFlapMode = Subject.create(0);

  public readonly gpwsTerrOff = Subject.create(false);

  /** ENGINE AND THROTTLE */

  public readonly engine1Master = ConsumerSubject.create(this.sub.on('engine1Master'), 0);

  public readonly engine2Master = ConsumerSubject.create(this.sub.on('engine2Master'), 0);

  public readonly engine1State = Subject.create(0);

  public readonly engine2State = Subject.create(0);

  public readonly engine3State = Subject.create(0);

  public readonly engine4State = Subject.create(0);

  public readonly N1Eng1 = Subject.create(0);

  public readonly N1Eng2 = Subject.create(0);

  public readonly N2Eng1 = Subject.create(0);

  public readonly N2Eng2 = Subject.create(0);

  public readonly N1IdleEng = Subject.create(0);

  // FIXME ECU should provide this in a discrete word
  public readonly engine1AboveIdle = MappedSubject.create(
    ([n1, idleN1]) => n1 > idleN1 + 2,
    this.N1Eng1,
    this.N1IdleEng,
  );

  public readonly engine2AboveIdle = MappedSubject.create(
    ([n1, idleN1]) => n1 > idleN1 + 2,
    this.N1Eng2,
    this.N1IdleEng,
  );

  // FIXME ECU should provide this in a discrete word, and calculate based on f(OAT)
  // this is absolute min at low temperatures
  public readonly engine1CoreAtOrAboveMinIdle = MappedSubject.create(
    ([n2]) => n2 >= (100 * 10630) / 16645,
    this.N2Eng1,
  );

  public readonly engine2CoreAtOrAboveMinIdle = MappedSubject.create(
    ([n2]) => n2 >= (100 * 10630) / 16645,
    this.N2Eng2,
  );

  public readonly engDualFault = Subject.create(false);

  public readonly engine1Generator = Subject.create(false);

  public readonly engine2Generator = Subject.create(false);

  public readonly emergencyElectricGeneratorPotential = Subject.create(0);

  public readonly emergencyGeneratorOn = this.emergencyElectricGeneratorPotential.map((it) => it > 0);

  public readonly apuMasterSwitch = Subject.create(0);

  public readonly apuAvail = Subject.create(0);

  public readonly radioHeight1 = Arinc429Register.empty();

  public readonly radioHeight2 = Arinc429Register.empty();

  public readonly radioHeight3 = Arinc429Register.empty();

  public readonly fac1Failed = Subject.create(0);

  public readonly toMemo = Subject.create(0);

  public readonly ldgMemo = Subject.create(0);

  public readonly autoBrake = Subject.create(0);

  public readonly fuel = Subject.create(0);

  public readonly usrStartRefueling = Subject.create(0);

  public readonly engSelectorPosition = Subject.create(0);

  public readonly eng1AntiIce = Subject.create(false);

  public readonly eng2AntiIce = Subject.create(false);

  public readonly throttle1Position = Subject.create(0);

  public readonly throttle2Position = Subject.create(0);

  public readonly engine1ValueSwitch = ConsumerValue.create(null, false);

  public readonly engine2ValueSwitch = ConsumerValue.create(null, false);

  public readonly autoThrustStatus = Subject.create(0);

  public readonly autothrustLeverWarningFlex = Subject.create(false);

  public readonly autothrustLeverWarningToga = Subject.create(false);

  public readonly thrustLeverNotSet = Subject.create(false);

  public readonly eng1Or2TakeoffPowerConfirm = new NXLogicConfirmNode(60, false);

  public readonly eng1Or2TakeoffPower = Subject.create(false);

  /* FIRE */

  public readonly agent1Eng1Discharge = Subject.create(0);

  public readonly agent1Eng1DischargeTimer = new NXLogicClockNode(10, 0);

  public readonly agent2Eng1Discharge = Subject.create(0);

  public readonly agent2Eng1DischargeTimer = new NXLogicClockNode(30, 0);

  public readonly agent1Eng2Discharge = Subject.create(0);

  public readonly agent1Eng2DischargeTimer = new NXLogicClockNode(10, 0);

  public readonly agent2Eng2Discharge = Subject.create(0);

  public readonly agent2Eng2DischargeTimer = new NXLogicClockNode(30, 0);

  public readonly agentAPUDischarge = Subject.create(0);

  public readonly agentAPUDischargeTimer = new NXLogicClockNode(10, 0);

  public readonly apuAgentPB = Subject.create(false);

  public readonly apuFireTest = Subject.create(false);

  public readonly cargoFireAgentDisch = Subject.create(false);

  public readonly cargoFireTest = Subject.create(false);

  public readonly eng1Agent1PB = Subject.create(false);

  public readonly eng1Agent2PB = Subject.create(false);

  public readonly eng1FireTest = Subject.create(false);

  public readonly eng2Agent1PB = Subject.create(false);

  public readonly eng2Agent2PB = Subject.create(false);

  public readonly eng2FireTest = Subject.create(false);

  public readonly fireButton1 = Subject.create(false);

  public readonly fireButton2 = Subject.create(false);

  public readonly fireButtonAPU = Subject.create(false);

  /* ICE */

  public readonly iceDetectedTimer1 = new NXLogicConfirmNode(40, false);

  public readonly iceDetectedTimer2 = new NXLogicConfirmNode(5);

  public readonly iceDetectedTimer2Status = Subject.create(false);

  public readonly iceNotDetTimer1 = new NXLogicConfirmNode(60);

  public readonly iceNotDetTimer2 = new NXLogicConfirmNode(130);

  public readonly iceNotDetTimer2Status = Subject.create(false);

  public readonly iceSevereDetectedTimer = new NXLogicConfirmNode(40, false);

  public readonly iceSevereDetectedTimerStatus = Subject.create(false);

  /* OTHER STUFF */

  public readonly airKnob = Subject.create(0);

  public readonly attKnob = Subject.create(0);

  public readonly compMesgCount = Subject.create(0);

  public readonly fmsSwitchingKnob = Subject.create(0);

  public readonly landAsapRed = Subject.create(false);

  public readonly ndXfrKnob = Subject.create(0);

  public readonly manLandingElevation = Subject.create(false);

  public readonly noMobileSwitchPosition = Subject.create(0);

  public readonly predWSOn = Subject.create(false);

  public readonly seatBelt = Subject.create(0);

  public readonly strobeLightsOn = Subject.create(0);

  public readonly tcasFault = Subject.create(false);

  public readonly tcasSensitivity = Subject.create(0);

  public readonly toConfigNormal = Subject.create(false);

  public readonly wingAntiIce = Subject.create(false);

  public readonly voiceVhf3 = Subject.create(false);

  private static pushKeyUnique(val: () => string[] | undefined, pushTo: string[]) {
    if (val) {
      // Push only unique keys
      for (const key of val()) {
        if (!pushTo.includes(key)) {
          pushTo.push(key);
        }
      }
    }
  }

  private readonly memos = new FwsMemos(this);
  private readonly normalChecklists = new FwsNormalChecklists(this);
  private readonly abnormalSensed = new FwsAbnormalSensed(this);
  private readonly abnormalNonSensed = new FwsAbnormalNonSensed(this);

  constructor(
    public readonly bus: EventBus,
    public readonly instrument: BaseInstrument,
  ) {
    this.ewdMessageLinesLeft.forEach((ls, i) =>
      ls.sub((l) => {
        SimVar.SetSimVarValue(FwsCore.ewdMessageSimVarsLeft[i], 'string', l ?? '');
      }),
    );

    this.ewdMessageLinesRight.forEach((ls, i) =>
      ls.sub((l) => {
        SimVar.SetSimVarValue(FwsCore.ewdMessageSimVarsRight[i], 'string', l ?? '');
      }),
    );

    this.pfdMemoLines.forEach((ls, i) =>
      ls.sub((l) => {
        SimVar.SetSimVarValue(FwsCore.pfdMemoSimVars[i], 'string', l ?? '');
      }),
    );

    this.sdStatusInfoLines.forEach((ls, i) =>
      ls.sub((l) => {
        SimVar.SetSimVarValue(FwsCore.sdStatusInfoSimVars[i], 'string', l ?? '');
      }),
    );

    this.sdStatusInopAllPhasesLines.forEach((ls, i) =>
      ls.sub((l) => {
        SimVar.SetSimVarValue(FwsCore.sdStatusInopAllPhasesSimVars[i], 'string', l ?? '');
      }),
    );

    this.sdStatusInopApprLdgLines.forEach((ls, i) =>
      ls.sub((l) => {
        SimVar.SetSimVarValue(FwsCore.sdStatusInopApprLdgSimVars[i], 'string', l ?? '');
      }),
    );

    this.pfdLimitationsLines.forEach((ls, i) =>
      ls.sub((l) => {
        SimVar.SetSimVarValue(FwsCore.pfdLimitationsSimVars[i], 'string', l ?? '');
      }),
    );

    this.ewdLimitationsAllPhasesLines.forEach((ls, i) =>
      ls.sub((l) => {
        SimVar.SetSimVarValue(FwsCore.ewdLimitationsAllPhasesSimVars[i], 'string', l ?? '');
      }),
    );

    this.ewdLimitationsApprLdgLines.forEach((ls, i) =>
      ls.sub((l) => {
        SimVar.SetSimVarValue(FwsCore.ewdLimitationsApprLdgSimVars[i], 'string', l ?? '');
      }),
    );

    SimVar.SetSimVarValue('L:A32NX_STATUS_LEFT_LINE_8', 'string', '000000001');

    const ecamMemoKeys = Object.keys(EcamMemos);
    Object.keys(this.memos.ewdToLdgMemos).forEach((key) => {
      this.memos.ewdToLdgMemos[key].codesToReturn.forEach((code) => {
        const found = ecamMemoKeys.find((it) => it === code);
        if (!found) {
          console.log(
            `ECAM message from PseudoFWC not found in EcamMemos: ${key}.\nIf MEMO, delete from PseudoFWC. If ECAM alert / ABN proc, move to ABN procs in the future.`,
          );
        }
      });
    });
  }

  init(): void {
    this.toConfigNormal.sub((normal) => SimVar.SetSimVarValue('L:A32NX_TO_CONFIG_NORMAL', 'bool', normal));
    this.fwcFlightPhase.sub(() => this.flightPhaseEndedPulseNode.write(true, 0));

    this.auralCrcOutput.sub((crc) => SimVar.SetSimVarValue('L:A32NX_FWC_CRC', 'bool', crc), true);

    this.masterCaution.sub((caution) => {
      // Inhibit master warning/cautions until FWC startup has been completed
      SimVar.SetSimVarValue('L:A32NX_MASTER_CAUTION', 'bool', this.startupCompleted.get() ? caution : false);
    }, true);

    this.masterWarningOutput.sub((warning) => {
      // Inhibit master warning/cautions until FWC startup has been completed
      SimVar.SetSimVarValue('L:A32NX_MASTER_WARNING', 'bool', this.startupCompleted.get() ? warning : false);
    }, true);

    this.startupCompleted.sub((completed) => {
      if (completed) {
        // Check if warnings or cautions have to be set
        SimVar.SetSimVarValue('L:A32NX_MASTER_CAUTION', 'bool', this.masterCaution.get());
        SimVar.SetSimVarValue('L:A32NX_MASTER_WARNING', 'bool', this.masterWarningOutput.get());
      }
    }, true);

    // L/G lever red arrow sinking outputs
    this.lgLeverRedArrow.sub((on) => {
      // TODO FWCs need to be powered...
      SimVar.SetSimVarValue('L:A32NX_FWC_1_LG_RED_ARROW', SimVarValueType.Bool, on);
      SimVar.SetSimVarValue('L:A32NX_FWC_2_LG_RED_ARROW', SimVarValueType.Bool, on);
    }, true);

    this.stallWarning.sub((v) => {
      this.fwcOut126.setBitValue(17, v);
      // set the sound on/off
      SimVar.SetSimVarValue('L:A32NX_AUDIO_STALL_WARNING', 'bool', v);
    }, true);
    this.aircraftOnGround.sub((v) => this.fwcOut126.setBitValue(28, v));

    this.fwcOut126.sub((v) => {
      Arinc429Word.toSimVarValue('L:A32NX_FWC_1_DISCRETE_WORD_126', v.value, v.ssm);
      Arinc429Word.toSimVarValue('L:A32NX_FWC_2_DISCRETE_WORD_126', v.value, v.ssm);
    }, true);

    // FIXME depend on FWC state
    this.fwcOut126.setSsm(Arinc429SignStatusMatrix.NormalOperation);

    const sub = this.bus.getSubscriber<FuelSystemEvents>();

    this.fuelCtrTankModeSelMan.setConsumer(sub.on('fuel_ctr_tk_mode_sel_man'));
    this.engine1ValueSwitch.setConsumer(sub.on('fuel_valve_switch_1'));
    this.engine2ValueSwitch.setConsumer(sub.on('fuel_valve_switch_2'));
    this.centerFuelPump1Auto.setConsumer(sub.on('fuel_pump_switch_1'));
    this.centerFuelPump2Auto.setConsumer(sub.on('fuel_pump_switch_4'));
    this.leftOuterInnerValve.setConsumer(sub.on('fuel_valve_open_4'));
    this.leftFuelPump1Auto.setConsumer(sub.on('fuel_pump_switch_2'));
    this.leftFuelPump2Auto.setConsumer(sub.on('fuel_pump_switch_5'));
    this.rightOuterInnerValve.setConsumer(sub.on('fuel_valve_open_5'));
    this.rightFuelPump1Auto.setConsumer(sub.on('fuel_pump_switch_3'));
    this.rightFuelPump2Auto.setConsumer(sub.on('fuel_pump_switch_6'));

    // Inhibit single chimes for the first two seconds after power-on
    this.auralSingleChimeInhibitTimer.schedule(
      () => (this.auralSingleChimePending = false),
      FwsCore.AURAL_SC_INHIBIT_TIME,
    );

    this.acESSBusPowered.sub((v) => {
      if (v) {
        this.startupTimer.schedule(() => {
          this.startupCompleted.set(true);
          console.log('PseudoFWC startup completed.');
        }, FwsCore.FWC_STARTUP_TIME);
      } else {
        this.startupTimer.clear();
        this.startupCompleted.set(false);
        console.log('PseudoFWC shut down.');
      }
    });
  }

  healthInjector(): void {
    SimVar.SetSimVarValue('L:A32NX_NO_SMOKING_MEMO', SimVarValueType.Bool, true);
    SimVar.SetSimVarValue('L:A32NX_CABIN_READY', SimVarValueType.Bool, true);

    const empty = Arinc429Register.empty();
    empty.setSsm(Arinc429SignStatusMatrix.NormalOperation);

    Arinc429Word.toSimVarValue('L:A32NX_COND_ACSC_1_DISCRETE_WORD_1', empty.value, empty.ssm);
    Arinc429Word.toSimVarValue('L:A32NX_COND_ACSC_1_DISCRETE_WORD_2', empty.value, empty.ssm);
    Arinc429Word.toSimVarValue('L:A32NX_COND_ACSC_2_DISCRETE_WORD_1', empty.value, empty.ssm);
    Arinc429Word.toSimVarValue('L:A32NX_COND_ACSC_2_DISCRETE_WORD_2', empty.value, empty.ssm);

    Arinc429Word.toSimVarValue('L:A32NX_PRESS_CPC_1_DISCRETE_WORD', empty.value, empty.ssm);
    Arinc429Word.toSimVarValue('L:A32NX_PRESS_CPC_2_DISCRETE_WORD', empty.value, empty.ssm);

    [1, 2].forEach((i) => {
      const dw = Arinc429Register.empty();
      dw.setSsm(Arinc429SignStatusMatrix.NormalOperation);
      dw.setBitValue(11, true);
      dw.setBitValue(16, true);
      Arinc429Word.toSimVarValue(`L:A32NX_FCDC_${i}_DISCRETE_WORD_1`, dw.value, dw.ssm);
      dw.setValue(0);
      Arinc429Word.toSimVarValue(`L:A32NX_FCDC_${i}_DISCRETE_WORD_2`, dw.value, dw.ssm);
      [11, 12, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24, 25].forEach((i) => dw.setBitValue(i, true));
      Arinc429Word.toSimVarValue(`L:A32NX_FCDC_${i}_DISCRETE_WORD_3`, dw.value, dw.ssm);
      dw.setValue(0);
      dw.setBitValue(27, SimVar.GetSimVarValue('L:A32NX_SPOILERS_ARMED', SimVarValueType.Bool));
      Arinc429Word.toSimVarValue(`L:A32NX_FCDC_${i}_DISCRETE_WORD_4`, dw.value, dw.ssm);
      Arinc429Word.toSimVarValue(`L:A32NX_FCDC_${i}_DISCRETE_WORD_5`, dw.value, dw.ssm);
    });
  }

  mapOrder(array, order): [] {
    array.sort((a, b) => {
      if (order.indexOf(a) > order.indexOf(b)) {
        return 1;
      }
      return -1;
    });
    return array;
  }

  public adirsMessage1(adirs, engineRunning) {
    let rowChoice = 0;

    switch (true) {
      case Math.ceil(adirs / 60) >= 7 && !engineRunning:
        rowChoice = 0;
        break;
      case Math.ceil(adirs / 60) >= 7 && engineRunning:
        rowChoice = 1;
        break;
      case Math.ceil(adirs / 60) === 6 && !engineRunning:
        rowChoice = 2;
        break;
      case Math.ceil(adirs / 60) === 6 && engineRunning:
        rowChoice = 3;
        break;
      case Math.ceil(adirs / 60) === 5 && !engineRunning:
        rowChoice = 4;
        break;
      case Math.ceil(adirs / 60) === 5 && engineRunning:
        rowChoice = 5;
        break;
      case Math.ceil(adirs / 60) === 4 && !engineRunning:
        rowChoice = 6;
        break;
      case Math.ceil(adirs / 60) === 4 && engineRunning:
        rowChoice = 7;
        break;
      default:
        break;
    }

    return rowChoice;
  }

  public adirsMessage2(adirs, engineRunning) {
    let rowChoice = 0;

    switch (true) {
      case Math.ceil(adirs / 60) === 3 && !engineRunning:
        rowChoice = 0;
        break;
      case Math.ceil(adirs / 60) === 3 && engineRunning:
        rowChoice = 1;
        break;
      case Math.ceil(adirs / 60) === 2 && !engineRunning:
        rowChoice = 2;
        break;
      case Math.ceil(adirs / 60) === 2 && engineRunning:
        rowChoice = 3;
        break;
      case Math.ceil(adirs / 60) === 1 && !engineRunning:
        rowChoice = 4;
        break;
      case Math.ceil(adirs / 60) === 1 && engineRunning:
        rowChoice = 5;
        break;
      default:
        break;
    }

    return rowChoice;
  }

  /**
   * Periodic update
   */
  onUpdate() {
    const deltaTime = this.instrument.deltaTime;

    // A380X hack: Inject healthy messages for some systems which are not yet implemented
    this.healthInjector();

    // Inputs update

    this.flightPhaseEndedPulseNode.write(false, deltaTime);

    this.fwcFlightPhase.set(SimVar.GetSimVarValue('L:A32NX_FWC_FLIGHT_PHASE', 'Enum'));
    this.flightPhase3PulseNode.write(this.fwcFlightPhase.get() === 3, deltaTime);
    // flight phase convenience vars
    this.flightPhase126.set([1, 2, 6].includes(this.fwcFlightPhase.get()));
    this.flightPhase23.set([2, 3].includes(this.fwcFlightPhase.get()));
    this.flightPhase34.set([3, 4].includes(this.fwcFlightPhase.get()));
    this.flightPhase345.set(this.flightPhase34.get() || this.fwcFlightPhase.get() === 5);
    this.flightPhase129.set([1, 2, 9].includes(this.fwcFlightPhase.get()));
    this.flightPhase67.set([6, 7].includes(this.fwcFlightPhase.get()));
    this.flightPhase78.set([7, 8].includes(this.fwcFlightPhase.get()));
    const flightPhase567 = [5, 6, 7].includes(this.fwcFlightPhase.get());

    // TO CONFIG button
    this.toConfigTestRaw = SimVar.GetSimVarValue('L:A32NX_BTN_TOCONFIG', 'bool') > 0;
    this.toConfigPulseNode.write(this.toConfigTestRaw, deltaTime);
    const toConfigTest = this.toConfigTriggerNode.write(this.toConfigPulseNode.read(), deltaTime);
    if (toConfigTest !== this.toConfigTest) {
      // temporary var for the old FWC stuff
      SimVar.SetSimVarValue('L:A32NX_FWS_TO_CONFIG_TEST', 'boolean', toConfigTest);
      this.toConfigTest = toConfigTest;
    }
    this.toConfigTestHeldMin1s5Pulse.set(
      this.toConfigTestHeldMin1s5PulseNode.write(this.toConfigTestRaw, deltaTime) || this.toConfigTestRaw,
    );

    // CLR buttons
    const clearButtonLeft = SimVar.GetSimVarValue('L:A32NX_BTN_CLR', 'bool');
    const clearButtonRight = SimVar.GetSimVarValue('L:A32NX_BTN_CLR2', 'bool');
    this.clr1PulseNode.write(clearButtonLeft, deltaTime);
    this.clr2PulseNode.write(clearButtonRight, deltaTime);
    const previousClrPulseUpTrigger = this.clrPulseUpTrigger.read();
    this.clrPulseUpTrigger.write(this.clr1PulseNode.read() || this.clr2PulseNode.read(), deltaTime);
    this.clrTriggerRisingEdge = !previousClrPulseUpTrigger && this.clrPulseUpTrigger.read();

    // RCL button
    const recallButton = SimVar.GetSimVarValue('L:A32NX_BTN_RCL', 'bool');
    this.rclUpPulseNode.write(recallButton, deltaTime);
    const previousRclUpTriggerNode = this.rclUpTriggerNode.read();
    this.rclUpTriggerNode.write(recallButton, deltaTime);

    this.recallTriggerRisingEdge = !previousRclUpTriggerNode && this.rclUpTriggerNode.read();

    this.flightPhaseInhibitOverrideNode.write(this.rclUpPulseNode.read(), this.flightPhaseEndedPulseNode.read());

    this.showTakeoffInhibit.set(
      this.toInhibitTimer.write(this.flightPhase345.get() && !this.flightPhaseInhibitOverrideNode.read(), deltaTime),
    );
    this.showLandingInhibit.set(
      this.ldgInhibitTimer.write(this.flightPhase78.get() && !this.flightPhaseInhibitOverrideNode.read(), deltaTime),
    );

    this.flapsIndex.set(SimVar.GetSimVarValue('L:A32NX_FLAPS_CONF_INDEX', 'number'));

    this.adr1Cas.set(Arinc429Word.fromSimVarValue('L:A32NX_ADIRS_ADR_1_COMPUTED_AIRSPEED'));
    this.adr2Cas.setFromSimVar('L:A32NX_ADIRS_ADR_2_COMPUTED_AIRSPEED');
    this.adr3Cas.setFromSimVar('L:A32NX_ADIRS_ADR_3_COMPUTED_AIRSPEED');

    this.ir1Pitch.setFromSimVar('L:A32NX_ADIRS_IR_1_PITCH');
    this.ir2Pitch.setFromSimVar('L:A32NX_ADIRS_IR_2_PITCH');
    this.ir3Pitch.setFromSimVar('L:A32NX_ADIRS_IR_3_PITCH');

    this.ir1MaintWord.setFromSimVar('L:A32NX_ADIRS_IR_1_MAINT_WORD');
    this.ir2MaintWord.setFromSimVar('L:A32NX_ADIRS_IR_2_MAINT_WORD');
    this.ir3MaintWord.setFromSimVar('L:A32NX_ADIRS_IR_3_MAINT_WORD');

    this.extremeLatitudeAlert.set(
      (this.ir1MaintWord.bitValueOr(15, false) ||
        this.ir2MaintWord.bitValueOr(15, false) ||
        this.ir3MaintWord.bitValueOr(15, false)) &&
        !SimVar.GetSimVarValue('L:A32NX_PUSH_TRUE_REF', 'bool'),
    );

    /* ELECTRICAL acquisition */
    this.dcESSBusPowered.set(SimVar.GetSimVarValue('L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED', 'bool'));
    this.dc2BusPowered.set(SimVar.GetSimVarValue('L:A32NX_ELEC_DC_2_BUS_IS_POWERED', 'bool'));
    this.ac1BusPowered.set(SimVar.GetSimVarValue('L:A32NX_ELEC_AC_1_BUS_IS_POWERED', 'bool'));
    this.ac2BusPowered.set(SimVar.GetSimVarValue('L:A32NX_ELEC_AC_2_BUS_IS_POWERED', 'bool'));
    this.ac3BusPowered.set(SimVar.GetSimVarValue('L:A32NX_ELEC_AC_3_BUS_IS_POWERED', 'bool'));
    this.ac4BusPowered.set(SimVar.GetSimVarValue('L:A32NX_ELEC_AC_4_BUS_IS_POWERED', 'bool'));
    this.acESSBusPowered.set(SimVar.GetSimVarValue('L:A32NX_ELEC_AC_ESS_BUS_IS_POWERED', 'bool'));

    /* ENGINE AND THROTTLE acquisition */

    const engine1StatesimVar = SimVar.GetSimVarValue('L:A32NX_ENGINE_STATE:1', 'Enum');
    const engine2StateSimVar = SimVar.GetSimVarValue('L:A32NX_ENGINE_STATE:2', 'Enum');
    const engine3StateSiMVar = SimVar.GetSimVarValue('L:A32NX_ENGINE_STATE:3', 'Enum');
    const engine4StateSiMVar = SimVar.GetSimVarValue('L:A32NX_ENGINE_STATE:4', 'Enum');

    this.engine1State.set(engine1StatesimVar);
    this.engine2State.set(engine2StateSimVar);
    this.engine3State.set(engine3StateSiMVar);
    this.engine4State.set(engine4StateSiMVar);

    this.engine1Running.set(engine1StatesimVar == 1);
    this.engine2Running.set(engine2StateSimVar == 1);
    this.engine3Running.set(engine3StateSiMVar == 1);
    this.engine4Running.set(engine4StateSiMVar == 1);

    this.N1Eng1.set(SimVar.GetSimVarValue('L:A32NX_ENGINE_N1:1', 'number'));
    this.N1Eng2.set(SimVar.GetSimVarValue('L:A32NX_ENGINE_N1:2', 'number'));
    this.N2Eng1.set(SimVar.GetSimVarValue('L:A32NX_ENGINE_N2:1', 'number'));
    this.N2Eng2.set(SimVar.GetSimVarValue('L:A32NX_ENGINE_N2:2', 'number'));
    this.N1IdleEng.set(SimVar.GetSimVarValue('L:A32NX_ENGINE_IDLE_N1', 'number'));

    // Flaps
    this.flapsAngle.set(SimVar.GetSimVarValue('L:A32NX_LEFT_FLAPS_ANGLE', 'degrees'));
    this.flapsHandle.set(SimVar.GetSimVarValue('L:A32NX_FLAPS_HANDLE_INDEX', 'enum'));
    this.slatsAngle.set(SimVar.GetSimVarValue('L:A32NX_LEFT_SLATS_ANGLE', 'degrees'));

    // FIXME move out of acquisition to logic below
    const oneEngineAboveMinPower = this.engine1AboveIdle.get() || this.engine2AboveIdle.get();

    this.engine1Generator.set(SimVar.GetSimVarValue('L:A32NX_ELEC_ENG_GEN_1_POTENTIAL_NORMAL', 'bool'));
    this.engine2Generator.set(SimVar.GetSimVarValue('L:A32NX_ELEC_ENG_GEN_2_POTENTIAL_NORMAL', 'bool'));
    this.emergencyElectricGeneratorPotential.set(SimVar.GetSimVarValue('L:A32NX_ELEC_EMER_GEN_POTENTIAL', 'number'));

    this.apuMasterSwitch.set(SimVar.GetSimVarValue('L:A32NX_OVHD_APU_MASTER_SW_PB_IS_ON', 'bool'));

    this.apuAvail.set(SimVar.GetSimVarValue('L:A32NX_OVHD_APU_START_PB_IS_AVAILABLE', 'bool'));
    this.apuBleedValveOpen.set(SimVar.GetSimVarValue('L:A32NX_APU_BLEED_AIR_VALVE_OPEN', 'bool'));

    this.fac1Failed.set(SimVar.GetSimVarValue('L:A32NX_FBW_FAC_FAILED:1', 'boost psi'));

    this.toMemo.set(SimVar.GetSimVarValue('L:A32NX_FWC_TOMEMO', 'bool'));

    this.autoBrake.set(SimVar.GetSimVarValue('L:A32NX_AUTOBRAKES_ARMED_MODE', 'enum'));

    this.ldgMemo.set(SimVar.GetSimVarValue('L:A32NX_FWC_LDGMEMO', 'bool'));

    this.fuel.set(SimVar.GetSimVarValue('A:INTERACTIVE POINT OPEN:9', 'percent'));
    this.usrStartRefueling.set(SimVar.GetSimVarValue('L:A32NX_REFUEL_STARTED_BY_USR', 'bool'));
    this.engSelectorPosition.set(SimVar.GetSimVarValue('L:XMLVAR_ENG_MODE_SEL', 'Enum'));
    this.eng1AntiIce.set(SimVar.GetSimVarValue('ENG ANTI ICE:1', 'bool'));
    this.eng2AntiIce.set(SimVar.GetSimVarValue('ENG ANTI ICE:2', 'bool'));
    this.throttle1Position.set(SimVar.GetSimVarValue('L:A32NX_AUTOTHRUST_TLA:1', 'number'));
    this.throttle2Position.set(SimVar.GetSimVarValue('L:A32NX_AUTOTHRUST_TLA:2', 'number'));
    this.autoThrustStatus.set(SimVar.GetSimVarValue('L:A32NX_AUTOTHRUST_STATUS', 'enum'));
    this.autothrustLeverWarningFlex.set(SimVar.GetSimVarValue('L:A32NX_AUTOTHRUST_THRUST_LEVER_WARNING_FLEX', 'bool'));
    this.autothrustLeverWarningToga.set(SimVar.GetSimVarValue('L:A32NX_AUTOTHRUST_THRUST_LEVER_WARNING_TOGA', 'bool'));

    /* HYDRAULICS acquisition */

    this.greenAPumpOn.set(SimVar.GetSimVarValue('L:A32NX_HYD_GA_EPUMP_ACTIVE', 'bool'));
    this.greenBPumpOn.set(SimVar.GetSimVarValue('L:A32NX_HYD_GB_EPUMP_ACTIVE', 'bool'));
    this.yellowAPumpOn.set(SimVar.GetSimVarValue('L:A32NX_HYD_YA_EPUMP_ACTIVE', 'bool'));
    this.yellowBPumpOn.set(SimVar.GetSimVarValue('L:A32NX_HYD_YB_EPUMP_ACTIVE', 'bool'));

    this.blueElecPumpPBAuto.set(SimVar.GetSimVarValue('L:A32NX_OVHD_HYD_EPUMPB_PB_IS_AUTO', 'bool'));
    this.blueLP.set(SimVar.GetSimVarValue('L:A32NX_HYD_BLUE_EDPUMP_LOW_PRESS', 'bool'));
    this.blueRvrLow.set(SimVar.GetSimVarValue('L:A32NX_HYD_BLUE_RESERVOIR_LEVEL_IS_LOW', 'bool'));
    this.blueRvrOvht.set(SimVar.GetSimVarValue('L:A32NX_HYD_BLUE_RESERVOIR_OVHT', 'bool'));
    this.eng1pumpPBisAuto.set(SimVar.GetSimVarValue('L:A32NX_OVHD_HYD_ENG_1_PUMP_PB_IS_AUTO', 'bool'));
    this.eng2pumpPBisAuto.set(SimVar.GetSimVarValue('L:A32NX_OVHD_HYD_ENG_2_PUMP_PB_IS_AUTO', 'bool'));
    this.greenLP.set(SimVar.GetSimVarValue('L:A32NX_HYD_GREEN_EDPUMP_LOW_PRESS', 'bool'));
    this.greenRvrOvht.set(SimVar.GetSimVarValue('L:A32NX_HYD_GREEN_RESERVOIR_OVHT', 'bool'));
    this.hydPTU.set(SimVar.GetSimVarValue('L:A32NX_HYD_PTU_ON_ECAM_MEMO', 'Bool'));
    this.ptuAuto.set(SimVar.GetSimVarValue('L:A32NX_OVHD_HYD_PTU_PB_IS_AUTO', 'bool'));
    this.ratDeployed.set(SimVar.GetSimVarValue('L:A32NX_RAT_STOW_POSITION', 'percent over 100'));
    this.yellowLP.set(SimVar.GetSimVarValue('L:A32NX_HYD_YELLOW_EDPUMP_LOW_PRESS', 'bool'));
    this.yellowRvrOvht.set(SimVar.GetSimVarValue('L:A32NX_HYD_YELLOW_RESERVOIR_OVHT', 'bool'));
    this.yepumpPBisAuto.set(SimVar.GetSimVarValue('L:A32NX_OVHD_HYD_EPUMPY_PB_IS_AUTO', 'bool'));

    const blueSysPressurised = SimVar.GetSimVarValue('L:A32NX_HYD_BLUE_SYSTEM_1_SECTION_PRESSURE_SWITCH', 'bool');
    const greenSysPressurised = SimVar.GetSimVarValue('L:A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE_SWITCH', 'bool');
    const yellowSysPressurised = SimVar.GetSimVarValue('L:A32NX_HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE_SWITCH', 'bool');

    /* ADIRS acquisition */
    /* NAVIGATION */

    this.ir1Fault.set(!this.ir1Pitch.isNormalOperation() || this.ir1MaintWord.bitValueOr(9, true));
    this.ir2Fault.set(!this.ir2Pitch.isNormalOperation() || this.ir2MaintWord.bitValueOr(9, true));
    this.ir3Fault.set(!this.ir3Pitch.isNormalOperation() || this.ir3MaintWord.bitValueOr(9, true));

    this.irExcessMotion.set(
      this.ir1MaintWord.bitValueOr(13, false) ||
        this.ir2MaintWord.bitValueOr(13, false) ||
        this.ir3MaintWord.bitValueOr(13, false),
    );

    this.adr1Fault.set(this.adr1Cas.get().isFailureWarning() || this.ir1MaintWord.bitValueOr(8, false));
    this.adr2Fault.set(this.adr2Cas.isFailureWarning() || this.ir2MaintWord.bitValueOr(8, false));
    this.adr3Fault.set(this.adr3Cas.isFailureWarning() || this.ir3MaintWord.bitValueOr(8, false));

    this.adirsRemainingAlignTime.set(SimVar.GetSimVarValue('L:A32NX_ADIRS_REMAINING_IR_ALIGNMENT_TIME', 'Seconds'));

    const adr1PressureAltitude = Arinc429Word.fromSimVarValue('L:A32NX_ADIRS_ADR_1_ALTITUDE');
    const adr2PressureAltitude = Arinc429Word.fromSimVarValue('L:A32NX_ADIRS_ADR_2_ALTITUDE');
    const adr3PressureAltitude = Arinc429Word.fromSimVarValue('L:A32NX_ADIRS_ADR_3_ALTITUDE');
    // TODO use GPS alt if ADRs not available
    const pressureAltitude =
      adr1PressureAltitude.valueOr(null) ?? adr2PressureAltitude.valueOr(null) ?? adr3PressureAltitude.valueOr(null);
    this.adiru1State.set(SimVar.GetSimVarValue('L:A32NX_ADIRS_ADIRU_1_STATE', 'enum'));
    this.adiru2State.set(SimVar.GetSimVarValue('L:A32NX_ADIRS_ADIRU_2_STATE', 'enum'));
    this.adiru3State.set(SimVar.GetSimVarValue('L:A32NX_ADIRS_ADIRU_3_STATE', 'enum'));
    // RA acquisition
    this.radioHeight1.setFromSimVar('L:A32NX_RA_1_RADIO_ALTITUDE');
    this.radioHeight2.setFromSimVar('L:A32NX_RA_2_RADIO_ALTITUDE');
    this.radioHeight3.setFromSimVar('L:A32NX_RA_3_RADIO_ALTITUDE');
    this.height1Failed.set(this.radioHeight1.isFailureWarning());
    this.height2Failed.set(this.radioHeight2.isFailureWarning());
    this.height3Failed.set(this.radioHeight3.isFailureWarning());
    // overspeed
    const adr3MaxCas = Arinc429Word.fromSimVarValue('L:A32NX_ADIRS_ADR_3_MAX_AIRSPEED');
    const adr1Discrete1 = Arinc429Word.fromSimVarValue('L:A32NX_ADIRS_ADR_1_DISCRETE_WORD_1');
    const adr2Discrete1 = Arinc429Word.fromSimVarValue('L:A32NX_ADIRS_ADR_2_DISCRETE_WORD_1');

    this.trueNorthRef.set(SimVar.GetSimVarValue('L:A32NX_PUSH_TRUE_REF', 'number'));

    /* LANDING GEAR AND LIGHTS acquisition */

    this.antiskidActive.set(SimVar.GetSimVarValue('ANTISKID BRAKES ACTIVE', 'bool'));
    this.brakeFan.set(SimVar.GetSimVarValue('L:A32NX_BRAKE_FAN', 'bool'));
    this.brakesHot.set(SimVar.GetSimVarValue('L:A32NX_BRAKES_HOT', 'bool'));
    // FIX ME ldg lt extended signal should come from SDAC
    const leftLdgLtPosition = SimVar.GetSimVarValue('L:A32NX_LANDING_2_POSITION', 'number');
    const rightLdgLtPosition = SimVar.GetSimVarValue('L:A32NX_LANDING_3_POSITION', 'number');
    this.leftLandingLightExtended.set(leftLdgLtPosition >= 30);
    this.rightlandingLightExtended.set(rightLdgLtPosition >= 30);
    this.lgciu1Fault.set(SimVar.GetSimVarValue('L:A32NX_LGCIU_1_FAULT', 'bool'));
    this.lgciu2Fault.set(SimVar.GetSimVarValue('L:A32NX_LGCIU_2_FAULT', 'bool'));
    this.lgciu1DiscreteWord1.setFromSimVar('L:A32NX_LGCIU_1_DISCRETE_WORD_1');
    this.lgciu2DiscreteWord1.setFromSimVar('L:A32NX_LGCIU_2_DISCRETE_WORD_1');
    this.lgciu1DiscreteWord2.setFromSimVar('L:A32NX_LGCIU_1_DISCRETE_WORD_2');
    this.lgciu2DiscreteWord2.setFromSimVar('L:A32NX_LGCIU_2_DISCRETE_WORD_2');
    this.parkBrake.set(SimVar.GetSimVarValue('L:A32NX_PARK_BRAKE_LEVER_POS', 'Bool'));
    this.nwSteeringDisc.set(SimVar.GetSimVarValue('L:A32NX_HYD_NW_STRG_DISC_ECAM_MEMO', 'Bool'));
    const leftCompressedHardwireLgciu1 =
      this.dcESSBusPowered.get() && SimVar.GetSimVarValue('A32NX_LGCIU_1_L_GEAR_COMPRESSED', 'bool') > 0;
    const leftCompressedHardwireLgciu2 =
      this.dc2BusPowered.get() && SimVar.GetSimVarValue('A32NX_LGCIU_2_L_GEAR_COMPRESSED', 'bool') > 0;

    // General logic
    const mainGearDownlocked =
      (this.lgciu1DiscreteWord1.bitValueOr(23, false) || this.lgciu2DiscreteWord1.bitValueOr(23, false)) &&
      (this.lgciu1DiscreteWord1.bitValueOr(24, false) || this.lgciu2DiscreteWord1.bitValueOr(24, false));
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    const gearDownlocked =
      mainGearDownlocked &&
      (this.lgciu1DiscreteWord1.bitValueOr(25, false) || this.lgciu2DiscreteWord1.bitValueOr(25, false));

    // on ground logic
    const lgciu1Disagree = xor(leftCompressedHardwireLgciu1, this.lgciu1DiscreteWord2.bitValue(13));
    this.lgciu1OnGroundDisagreeConf.write(lgciu1Disagree, deltaTime);
    this.lgciu1OnGroundAgreeConf.write(!lgciu1Disagree, deltaTime);
    this.lgciu1OnGroundDisagreeMem.write(
      (!this.lgciu1DiscreteWord2.isNormalOperation() && !this.lgciu1DiscreteWord2.isFunctionalTest()) ||
        this.lgciu1OnGroundDisagreeConf.read(),
      this.lgciu1OnGroundAgreeConf.read(),
    );
    const lgciu2Disagree = xor(leftCompressedHardwireLgciu2, this.lgciu2DiscreteWord2.bitValue(13));
    this.lgciu2OnGroundDisagreeConf.write(lgciu2Disagree, deltaTime);
    this.lgciu2OnGroundAgreeConf.write(!lgciu2Disagree, deltaTime);
    this.lgciu2OnGroundDisagreeMem.write(
      (!this.lgciu2DiscreteWord2.isNormalOperation() && !this.lgciu2DiscreteWord2.isFunctionalTest()) ||
        this.lgciu2OnGroundDisagreeConf.read(),
      this.lgciu2OnGroundAgreeConf.read(),
    );
    const lgciuOnGroundDisagree = this.lgciu1OnGroundDisagreeMem.read() || this.lgciu2OnGroundDisagreeMem.read();
    const onGroundA =
      leftCompressedHardwireLgciu1 &&
      this.lgciu1DiscreteWord2.bitValue(13) &&
      leftCompressedHardwireLgciu2 &&
      this.lgciu2DiscreteWord2.bitValue(13);

    // TODO some renaming
    this.ignoreRaOnGroundTrigger.write(
      this.radioHeight1.isNoComputedData() &&
        this.radioHeight2.isNoComputedData() &&
        this.radioHeight3.isNoComputedData() &&
        !lgciuOnGroundDisagree,
      deltaTime,
    );
    this.ra1OnGroundMem.write(
      this.radioHeight1.value < 5,
      !leftCompressedHardwireLgciu1 || !leftCompressedHardwireLgciu2,
    );
    this.ra2OnGroundMem.write(
      this.radioHeight2.value < 5,
      !leftCompressedHardwireLgciu1 || !leftCompressedHardwireLgciu2,
    );
    this.ra3OnGroundMem.write(
      this.radioHeight3.value < 5,
      !leftCompressedHardwireLgciu1 || !leftCompressedHardwireLgciu2,
    );
    const ra1OnGround =
      (this.radioHeight1.isNormalOperation() || this.radioHeight1.isFunctionalTest()) &&
      (this.radioHeight1.value < 5 || this.ra1OnGroundMem.read());
    const ra2OnGround =
      (this.radioHeight2.isNormalOperation() || this.radioHeight2.isFunctionalTest()) &&
      (this.radioHeight2.value < 5 || this.ra2OnGroundMem.read());
    const ra3OnGround =
      (this.radioHeight3.isNormalOperation() || this.radioHeight3.isFunctionalTest()) &&
      (this.radioHeight3.value < 5 || this.ra3OnGroundMem.read());
    const onGroundCount = countTrue(
      leftCompressedHardwireLgciu1,
      leftCompressedHardwireLgciu2,
      ra1OnGround,
      ra2OnGround,
      ra3OnGround,
    );
    const raInvalid =
      this.radioHeight1.isFailureWarning() ||
      this.radioHeight2.isFailureWarning() ||
      this.radioHeight3.isFailureWarning();
    this.onGroundImmediate =
      (onGroundA && this.ignoreRaOnGroundTrigger.read()) ||
      (onGroundCount > 2 && !raInvalid) ||
      (onGroundCount > 1 && raInvalid);
    this.aircraftOnGround.set(this.onGroundConf.write(this.onGroundImmediate, deltaTime));

    // Engine Logic
    this.thrustLeverNotSet.set(this.autothrustLeverWarningFlex.get() || this.autothrustLeverWarningToga.get());
    // FIXME ECU doesn't have the necessary output words so we go purely on TLA
    const flexThrustLimit = SimVar.GetSimVarValue('L:A32NX_AUTOTHRUST_THRUST_LIMIT_TYPE', 'number') === 3;
    const toPower =
      this.throttle1Position.get() >= 45 ||
      (this.throttle1Position.get() >= 35 && flexThrustLimit) ||
      this.throttle2Position.get() >= 45 ||
      (this.throttle2Position.get() >= 35 && flexThrustLimit);
    this.eng1Or2TakeoffPowerConfirm.write(toPower, deltaTime);
    const raAbove1500 =
      this.radioHeight1.valueOr(0) > 1500 || this.radioHeight2.valueOr(0) > 1500 || this.radioHeight3.valueOr(0) > 1500;
    this.eng1Or2TakeoffPower.set(toPower || (this.eng1Or2TakeoffPowerConfirm.read() && !raAbove1500));

    this.engDualFault.set(
      !this.aircraftOnGround.get() &&
        ((this.fireButton1.get() && this.fireButton2.get()) ||
          (!this.engine1ValueSwitch.get() && !this.engine2ValueSwitch.get()) ||
          (this.engine1State.get() === 0 && this.engine2State.get() === 0) ||
          (!this.engine1CoreAtOrAboveMinIdle.get() && !this.engine2CoreAtOrAboveMinIdle.get())),
    );

    /* 22 - AUTOFLIGHT */
    const fm1DiscreteWord3 = Arinc429Word.fromSimVarValue('L:A32NX_FM1_DISCRETE_WORD_3');
    const fm2DiscreteWord3 = Arinc429Word.fromSimVarValue('L:A32NX_FM2_DISCRETE_WORD_3');

    if (!this.flightPhase23.get()) {
      this.toConfigCheckedInPhase2Or3 = false;
    } else if (this.toConfigTestRaw) {
      this.toConfigCheckedInPhase2Or3 = true;
    }

    let overspeedWarning = this.adr3OverspeedWarning.write(
      this.adr3Cas.isNormalOperation() && adr3MaxCas.isNormalOperation() && this.adr3Cas.value > adr3MaxCas.value + 8,
      this.aircraftOnGround.get() ||
        !(this.adr3Cas.isNormalOperation() && adr3MaxCas.isNormalOperation()) ||
        this.adr3Cas.value < adr3MaxCas.value + 4,
    );
    if (
      !(adr1Discrete1.isNormalOperation() || adr1Discrete1.isFunctionalTest()) ||
      !(adr2Discrete1.isNormalOperation() || adr2Discrete1.isFunctionalTest())
    ) {
      const adr3Discrete1 = Arinc429Word.fromSimVarValue('L:A32NX_ADIRS_ADR_3_DISCRETE_WORD_1');
      overspeedWarning ||= adr3Discrete1.getBitValueOr(9, false);
    }
    overspeedWarning ||= adr1Discrete1.getBitValueOr(9, false) || adr2Discrete1.getBitValueOr(9, false);
    const isOverspeed = (limit: number) => this.computedAirSpeedToNearest2.get() > limit + 4;
    this.overspeedVmo.set(!gearDownlocked && this.flapsHandle.get() === 0 && isOverspeed(340));
    this.overspeedVle.set(gearDownlocked && this.flapsHandle.get() === 0 && isOverspeed(250));
    this.overspeedVfeConf1.set(this.flapsHandle.get() === 1 && isOverspeed(263)); // FIXME
    // this.overspeedVfeConf1F.set(this.flapsHandle.get() === 1 && isOverspeed(222));
    this.overspeedVfeConf2.set(this.flapsHandle.get() === 2 && isOverspeed(220));
    this.overspeedVfeConf3.set(this.flapsHandle.get() === 3 && isOverspeed(196));
    this.overspeedVfeConfFull.set(this.flapsHandle.get() === 4 && isOverspeed(182));

    // TO SPEEDS NOT INSERTED
    const fmToSpeedsNotInserted =
      fm1DiscreteWord3.getBitValueOr(18, false) && fm2DiscreteWord3.getBitValueOr(18, false);

    this.toConfigAndNoToSpeedsPulseNode.write(fmToSpeedsNotInserted && this.toConfigTestRaw, deltaTime);

    if (
      fmToSpeedsNotInserted &&
      (this.toConfigTestRaw || this.fwcFlightPhase.get() === 3) &&
      !this.toSpeedsNotInserted
    ) {
      this.toSpeedsNotInserted = true;
    }
    if (!(this.flightPhase23.get() && fmToSpeedsNotInserted) && this.toSpeedsNotInserted) {
      this.toSpeedsNotInserted = false;
    }

    this.toSpeedsNotInsertedWarning.set(
      !this.toConfigAndNoToSpeedsPulseNode.read() && this.toSpeedsNotInserted && !this.flightPhase3PulseNode.read(),
    );

    // TO SPEEDS TOO LOW
    const toSpeedsTooLow = fm1DiscreteWord3.getBitValueOr(17, false) && fm2DiscreteWord3.getBitValueOr(17, false);
    this.toSpeedsTooLowWarning.set(
      (this.toConfigCheckedInPhase2Or3 || this.fwcFlightPhase.get() === 3) &&
        !this.toConfigPulseNode.read() &&
        !this.flightPhase3PulseNode.read() &&
        toSpeedsTooLow,
    );

    // TO V1/VR/V2 DISAGREE
    const toV2VRV2Disagree = fm1DiscreteWord3.getBitValueOr(16, false) && fm2DiscreteWord3.getBitValueOr(16, false);
    this.toV2VRV2DisagreeWarning.set(
      (this.toConfigCheckedInPhase2Or3 || this.fwcFlightPhase.get() === 3) &&
        !this.toConfigPulseNode.read() &&
        !this.flightPhase3PulseNode.read() &&
        toV2VRV2Disagree,
    );

    // FMS takeoff flap settings
    const fm1DiscreteWord2 = Arinc429Word.fromSimVarValue('L:A32NX_FM1_DISCRETE_WORD_2');
    const fm2DiscreteWord2 = Arinc429Word.fromSimVarValue('L:A32NX_FM2_DISCRETE_WORD_2');

    /** MCDU TO CONF 0 selected */
    const mcduToFlapPos0 = fm1DiscreteWord2.getBitValueOr(13, false) || fm2DiscreteWord2.getBitValueOr(13, false);
    /** MCDU TO CONF 1 selected */
    const mcduToFlapPos1 = fm1DiscreteWord2.getBitValueOr(14, false) || fm2DiscreteWord2.getBitValueOr(14, false);
    /** MCDU TO CONF 2 selected */
    const mcduToFlapPos2 = fm1DiscreteWord2.getBitValueOr(15, false) || fm2DiscreteWord2.getBitValueOr(15, false);
    /** MCDU TO CONF 3 selected */
    const mcduToFlapPos3 = fm1DiscreteWord2.getBitValueOr(16, false) || fm2DiscreteWord2.getBitValueOr(16, false);

    this.fmcAFault.set(!SimVar.GetSimVarValue('L:A32NX_FMC_A_IS_HEALTHY', 'bool'));
    this.fmcBFault.set(!SimVar.GetSimVarValue('L:A32NX_FMC_B_IS_HEALTHY', 'bool'));
    this.fmcCFault.set(!SimVar.GetSimVarValue('L:A32NX_FMC_C_IS_HEALTHY', 'bool'));
    this.fms1Fault.set(this.fmcAFault.get() && this.fmcCFault.get());
    this.fms2Fault.set(this.fmcBFault.get() && this.fmcCFault.get());

    /* 21 - AIR CONDITIONING AND PRESSURIZATION */

    this.acsc1DiscreteWord1.setFromSimVar('L:A32NX_COND_ACSC_1_DISCRETE_WORD_1');
    this.acsc1DiscreteWord2.setFromSimVar('L:A32NX_COND_ACSC_1_DISCRETE_WORD_2');
    this.acsc2DiscreteWord1.setFromSimVar('L:A32NX_COND_ACSC_2_DISCRETE_WORD_1');
    this.acsc2DiscreteWord2.setFromSimVar('L:A32NX_COND_ACSC_2_DISCRETE_WORD_2');

    this.acsc1Lane1Fault.set(this.acsc1DiscreteWord1.bitValueOr(21, false));
    this.acsc1Lane2Fault.set(this.acsc1DiscreteWord1.bitValueOr(22, false));
    this.acsc2Lane1Fault.set(this.acsc2DiscreteWord1.bitValueOr(21, false));
    this.acsc2Lane2Fault.set(this.acsc2DiscreteWord1.bitValueOr(22, false));

    const acsc1FT = this.acsc1DiscreteWord1.isFailureWarning();
    const acsc2FT = this.acsc2DiscreteWord1.isFailureWarning();
    this.acsc1Fault.set(acsc1FT && !acsc2FT);
    this.acsc2Fault.set(!acsc1FT && acsc2FT);
    const acscBothFault = acsc1FT && acsc2FT;

    this.ramAirOn.set(SimVar.GetSimVarValue('L:A32NX_AIRCOND_RAMAIR_TOGGLE', 'bool'));

    this.cabFanHasFault1.set(
      this.acsc1DiscreteWord1.bitValueOr(25, false) || this.acsc2DiscreteWord1.bitValueOr(25, false),
    );
    this.cabFanHasFault2.set(
      this.acsc1DiscreteWord1.bitValueOr(26, false) || this.acsc2DiscreteWord1.bitValueOr(26, false),
    );

    this.hotAirDisagrees.set(
      this.acsc1DiscreteWord1.bitValueOr(27, false) && this.acsc2DiscreteWord1.bitValueOr(27, false),
    );
    this.hotAirOpen.set(
      !this.acsc1DiscreteWord1.bitValueOr(20, false) || !this.acsc2DiscreteWord1.bitValueOr(20, false),
    );
    this.hotAirPbOn.set(this.acsc1DiscreteWord1.bitValueOr(23, false) || this.acsc2DiscreteWord1.bitValueOr(23, false));

    this.trimAirFault.set(
      this.acsc1DiscreteWord1.bitValueOr(28, false) || this.acsc2DiscreteWord1.bitValueOr(28, false),
    );
    this.ckptTrimFault.set(
      this.acsc1DiscreteWord2.bitValueOr(18, false) || this.acsc2DiscreteWord2.bitValueOr(18, false),
    );
    this.fwdTrimFault.set(
      this.acsc1DiscreteWord2.bitValueOr(19, false) || this.acsc2DiscreteWord2.bitValueOr(19, false),
    );
    this.aftTrimFault.set(
      this.acsc1DiscreteWord2.bitValueOr(20, false) || this.acsc2DiscreteWord2.bitValueOr(20, false),
    );
    this.trimAirHighPressure.set(
      this.acsc1DiscreteWord1.bitValueOr(18, false) || this.acsc2DiscreteWord1.bitValueOr(18, false),
    );

    this.ckptDuctOvht.set(
      this.acsc1DiscreteWord1.bitValueOr(11, false) || this.acsc2DiscreteWord1.bitValueOr(11, false),
    );
    this.fwdDuctOvht.set(
      this.acsc1DiscreteWord1.bitValueOr(12, false) || this.acsc2DiscreteWord1.bitValueOr(12, false),
    );
    this.aftDuctOvht.set(
      this.acsc1DiscreteWord1.bitValueOr(13, false) || this.acsc2DiscreteWord1.bitValueOr(13, false),
    );
    this.anyDuctOvht.set(this.ckptDuctOvht.get() || this.fwdDuctOvht.get() || this.aftDuctOvht.get());

    this.lavGalleyFanFault.set(
      this.acsc1DiscreteWord1.bitValueOr(24, false) || this.acsc2DiscreteWord1.bitValueOr(24, false),
    );

    const crossbleedFullyClosed = SimVar.GetSimVarValue('L:A32NX_PNEU_XBLEED_VALVE_FULLY_CLOSED', 'bool');
    const eng1Bleed = SimVar.GetSimVarValue('L:A32NX_OVHD_PNEU_ENG_1_BLEED_PB_IS_AUTO', 'bool');
    const eng1BleedPbFault = SimVar.GetSimVarValue('L:A32NX_OVHD_PNEU_ENG_1_BLEED_PB_HAS_FAULT', 'bool');
    const eng2Bleed = SimVar.GetSimVarValue('L:A32NX_OVHD_PNEU_ENG_1_BLEED_PB_IS_AUTO', 'bool');
    const eng2BleedPbFault = SimVar.GetSimVarValue('L:A32NX_OVHD_PNEU_ENG_2_BLEED_PB_HAS_FAULT', 'bool');
    const pack1Fault = SimVar.GetSimVarValue('L:A32NX_OVHD_COND_PACK_1_PB_HAS_FAULT', 'bool');
    const pack2Fault = SimVar.GetSimVarValue('L:A32NX_OVHD_COND_PACK_2_PB_HAS_FAULT', 'bool');
    this.pack1On.set(SimVar.GetSimVarValue('L:A32NX_OVHD_COND_PACK_1_PB_IS_ON', 'bool'));
    this.pack2On.set(SimVar.GetSimVarValue('L:A32NX_OVHD_COND_PACK_2_PB_IS_ON', 'bool'));

    this.cpc1DiscreteWord.setFromSimVar('L:A32NX_PRESS_CPC_1_DISCRETE_WORD');
    this.cpc2DiscreteWord.setFromSimVar('L:A32NX_PRESS_CPC_2_DISCRETE_WORD');

    const activeCpcNumber = this.cpc1DiscreteWord.bitValueOr(11, false) ? 1 : 2;
    const activeCpc = activeCpcNumber === 1 ? this.cpc1DiscreteWord : this.cpc2DiscreteWord;

    this.cpc1Fault.set(this.cpc1DiscreteWord.isFailureWarning());
    this.cpc2Fault.set(this.cpc2DiscreteWord.isFailureWarning());
    this.bothCpcFaultOutput.set(this.bothCpcFault.write(this.cpc1Fault.get() && this.cpc2Fault.get(), deltaTime));

    const manExcessAltitude = SimVar.GetSimVarValue('L:A32NX_PRESS_MAN_EXCESSIVE_CABIN_ALTITUDE', 'bool');
    this.excessPressure.set(activeCpc.bitValueOr(14, false) || manExcessAltitude);

    const eng1And2NotRunning = !this.engine1CoreAtOrAboveMinIdle.get() && !this.engine2CoreAtOrAboveMinIdle.get();
    this.enginesOffAndOnGroundSignal.write(this.aircraftOnGround.get() && eng1And2NotRunning, deltaTime);
    const residualPressureSignal = SimVar.GetSimVarValue('L:A32NX_PRESS_EXCESS_RESIDUAL_PR', 'bool');
    this.excessResidualPr.set(
      this.excessResidualPrConfirm.write(this.enginesOffAndOnGroundSignal.read() && residualPressureSignal, deltaTime),
    );

    this.lowDiffPress.set(activeCpc.bitValueOr(15, false));

    this.pressurizationAuto.set(SimVar.GetSimVarValue('L:A32NX_OVHD_PRESS_MODE_SEL_PB_IS_AUTO', 'bool'));

    this.cabAltSetResetState1.set(
      this.cabAltSetReset1.write(
        (pressureAltitude ?? 0) > 10000 && this.excessPressure.get(),
        this.excessPressure.get() && [3, 10].includes(this.fwcFlightPhase.get()),
      ),
    );
    this.cabAltSetResetState2.set(
      this.cabAltSetReset2.write(
        (pressureAltitude ?? 0) > 16000 && this.excessPressure.get(),
        this.excessPressure.get() && [3, 10].includes(this.fwcFlightPhase.get()),
      ),
    );
    this.packOffBleedAvailable1.write((eng1Bleed === 1 && !eng1BleedPbFault) || !crossbleedFullyClosed, deltaTime);
    this.packOffBleedAvailable2.write((eng2Bleed === 1 && !eng2BleedPbFault) || !crossbleedFullyClosed, deltaTime);
    this.packOffNotFailed1Status.set(
      this.packOffNotFailed1.write(
        !this.pack1On.get() && !pack1Fault && this.packOffBleedAvailable1.read() && this.fwcFlightPhase.get() === 6,
        deltaTime,
      ),
    );
    this.packOffNotFailed2Status.set(
      this.packOffNotFailed2.write(
        !this.pack2On.get() && !pack2Fault && this.packOffBleedAvailable2.read() && this.fwcFlightPhase.get() === 6,
        deltaTime,
      ),
    );
    this.pack1And2Fault.set(
      acscBothFault ||
        (this.packOffNotFailed1Status.get() && this.acsc2Fault.get()) ||
        (this.packOffNotFailed2Status.get() && this.acsc1Fault.get()),
    );

    const manOutflowValueOpenPercentage = SimVar.GetSimVarValue(
      'L:A32NX_PRESS_MAN_OUTFLOW_VALVE_OPEN_PERCENTAGE',
      'percent',
    );
    this.outflowValveOpenAmount.set(
      Arinc429Word.fromSimVarValue(`L:A32NX_PRESS_CPC_${activeCpcNumber}_OUTFLOW_VALVE_OPEN_PERCENTAGE`).valueOr(
        manOutflowValueOpenPercentage,
      ),
    );
    this.outflowValveNotOpenOutput.set(
      this.outflowValveNotOpenSetReset.write(
        this.outflowValveNotOpen.write(
          this.outflowValveOpenAmount.get() < 85 && [8, 9, 10].includes(this.fwcFlightPhase.get()),
          deltaTime,
        ),
        this.outflowValveOpenAmount.get() > 95 ||
          this.outflowValveResetCondition.write(this.fwcFlightPhase.get() === 1, deltaTime),
      ),
    );

    const safetyValveNotClosed = SimVar.GetSimVarValue('L:A32NX_PRESS_SAFETY_VALVE_OPEN_PERCENTAGE', 'percent') > 0;
    this.safetyValveNotClosedAir.write(safetyValveNotClosed, deltaTime);
    this.safetyValveNotClosedOutput.set(
      (safetyValveNotClosed && [1, 2, 3].includes(this.fwcFlightPhase.get())) ||
        (this.safetyValveNotClosedAir.read() && this.fwcFlightPhase.get() === 6),
    );

    const manCabinDeltaPressure = SimVar.GetSimVarValue('L:A32NX_PRESS_MAN_CABIN_DELTA_PRESSURE', 'percent');
    this.cabinDeltaPressure.set(
      Arinc429Word.fromSimVarValue(`L:A32NX_PRESS_CPC_${activeCpcNumber}_CABIN_DELTA_PRESSURE`).valueOr(
        manCabinDeltaPressure,
      ),
    );

    /* 23 - COMMUNICATION */
    this.rmp1Fault.set(false); // Don't want to use failure consumer here, rather use health signal
    this.rmp1Off.set(SimVar.GetSimVarValue('L:A380X_RMP_1_BRIGHTNESS_KNOB', 'number') === 0);

    this.rmp2Fault.set(false);
    this.rmp2Off.set(SimVar.GetSimVarValue('L:A380X_RMP_2_BRIGHTNESS_KNOB', 'number') === 0);

    this.rmp3Fault.set(false);
    this.rmp3Off.set(SimVar.GetSimVarValue('L:A380X_RMP_3_BRIGHTNESS_KNOB', 'number') === 0);

    /* 24 - Electrical */
    this.extPwrConnected.set(
      SimVar.GetSimVarValue('L:A32NX_ELEC_CONTACTOR_990XG1_IS_CLOSED', 'bool') ||
        SimVar.GetSimVarValue('L:A32NX_ELEC_CONTACTOR_990XG2_IS_CLOSED', 'bool') ||
        SimVar.GetSimVarValue('L:A32NX_ELEC_CONTACTOR_990XG3_IS_CLOSED', 'bool') ||
        SimVar.GetSimVarValue('L:A32NX_ELEC_CONTACTOR_990XG4_IS_CLOSED', 'bool'),
    );

    /* OTHER STUFF */

    this.airKnob.set(SimVar.GetSimVarValue('L:A32NX_AIR_DATA_SWITCHING_KNOB', 'enum'));
    this.attKnob.set(SimVar.GetSimVarValue('L:A32NX_ATT_HDG_SWITCHING_KNOB', 'enum'));
    this.compMesgCount.set(SimVar.GetSimVarValue('L:A32NX_COMPANY_MSG_COUNT', 'number'));
    this.fmsSwitchingKnob.set(SimVar.GetSimVarValue('L:A32NX_FMS_SWITCHING_KNOB', 'enum'));
    this.manLandingElevation.set(activeCpc.bitValueOr(17, false));
    this.seatBelt.set(SimVar.GetSimVarValue('A:CABIN SEATBELTS ALERT SWITCH', 'bool'));
    this.ndXfrKnob.set(SimVar.GetSimVarValue('L:A32NX_ECAM_ND_XFR_SWITCHING_KNOB', 'enum'));
    this.noMobileSwitchPosition.set(SimVar.GetSimVarValue('L:XMLVAR_SWITCH_OVHD_INTLT_NOSMOKING_Position', 'number'));
    this.strobeLightsOn.set(SimVar.GetSimVarValue('L:LIGHTING_STROBE_0', 'Bool'));
    this.gpwsFlapMode.set(SimVar.GetSimVarValue('L:A32NX_GPWS_FLAP_OFF', 'Bool'));
    this.gpwsTerrOff.set(SimVar.GetSimVarValue('L:A32NX_GPWS_TERR_OFF', 'Bool'));
    this.predWSOn.set(SimVar.GetSimVarValue('L:A32NX_SWITCH_RADAR_PWS_Position', 'Bool'));
    this.tcasFault.set(SimVar.GetSimVarValue('L:A32NX_TCAS_FAULT', 'bool'));
    this.tcasSensitivity.set(SimVar.GetSimVarValue('L:A32NX_TCAS_SENSITIVITY', 'Enum'));
    this.wingAntiIce.set(SimVar.GetSimVarValue('L:A32NX_PNEU_WING_ANTI_ICE_SYSTEM_SELECTED', 'bool'));
    this.voiceVhf3.set(this.rmp3ActiveMode.get() !== FrequencyMode.Data);

    /* FUEL */
    const fuelGallonsToKg = SimVar.GetSimVarValue('FUEL WEIGHT PER GALLON', 'kilogram');
    this.centerFuelQuantity.set(SimVar.GetSimVarValue('FUEL TANK CENTER QUANTITY', 'gallons') * fuelGallonsToKg);
    this.fuelXFeedPBOn.set(SimVar.GetSimVarValue('L:XMLVAR_Momentary_PUSH_OVHD_FUEL_XFEED_Pressed', 'bool'));

    const leftInnerFuelQuantity = SimVar.GetSimVarValue('FUEL TANK LEFT MAIN QUANTITY', 'gallons') * fuelGallonsToKg;
    const rightInnerFuelQuantity = SimVar.GetSimVarValue('FUEL TANK RIGHT MAIN QUANTITY', 'gallons') * fuelGallonsToKg;
    const leftFuelLow = leftInnerFuelQuantity < 750;
    const rightFuelLow = rightInnerFuelQuantity < 750;
    this.lrTankLow.set(this.lrTankLowConfirm.write(leftFuelLow && rightFuelLow, deltaTime));
    this.leftFuelLow.set(this.leftFuelLowConfirm.write(leftFuelLow && !this.lrTankLow.get(), deltaTime));
    this.rightFuelLow.set(this.rightFuelLowConfirm.write(rightFuelLow && !this.lrTankLow.get(), deltaTime));

    /* F/CTL */
    const fcdc1DiscreteWord1 = Arinc429Word.fromSimVarValue('L:A32NX_FCDC_1_DISCRETE_WORD_1');
    const fcdc2DiscreteWord1 = Arinc429Word.fromSimVarValue('L:A32NX_FCDC_2_DISCRETE_WORD_1');
    const fcdc1DiscreteWord2 = Arinc429Word.fromSimVarValue('L:A32NX_FCDC_1_DISCRETE_WORD_2');
    const fcdc2DiscreteWord2 = Arinc429Word.fromSimVarValue('L:A32NX_FCDC_2_DISCRETE_WORD_2');
    const fcdc1DiscreteWord3 = Arinc429Word.fromSimVarValue('L:A32NX_FCDC_1_DISCRETE_WORD_3');
    const fcdc2DiscreteWord3 = Arinc429Word.fromSimVarValue('L:A32NX_FCDC_2_DISCRETE_WORD_3');
    const fcdc1DiscreteWord4 = Arinc429Word.fromSimVarValue('L:A32NX_FCDC_1_DISCRETE_WORD_4');
    const fcdc2DiscreteWord4 = Arinc429Word.fromSimVarValue('L:A32NX_FCDC_2_DISCRETE_WORD_4');
    const fcdc1DiscreteWord5 = Arinc429Word.fromSimVarValue('L:A32NX_FCDC_1_DISCRETE_WORD_5');
    const fcdc2DiscreteWord5 = Arinc429Word.fromSimVarValue('L:A32NX_FCDC_2_DISCRETE_WORD_5');

    // ELAC 1 FAULT computation
    const se1f =
      (fcdc1DiscreteWord1.getBitValueOr(19, false) || fcdc2DiscreteWord1.getBitValueOr(19, false)) &&
      (fcdc1DiscreteWord1.getBitValueOr(20, false) || fcdc2DiscreteWord1.getBitValueOr(20, false));
    const elac1FaultCondition =
      !(
        [1, 10].includes(this.fwcFlightPhase.get()) &&
        (fcdc1DiscreteWord3.getBitValueOr(19, false) || fcdc2DiscreteWord3.getBitValueOr(19, false))
      ) &&
      this.dcESSBusPowered.get() &&
      (fcdc1DiscreteWord1.getBitValueOr(23, false) ||
        fcdc2DiscreteWord1.getBitValueOr(23, false) ||
        (!this.elac1HydConfirmNodeOutput.get() && se1f));
    this.elac1FaultLine123Display.set(
      !(fcdc1DiscreteWord3.getBitValueOr(19, false) || fcdc2DiscreteWord3.getBitValueOr(19, false)) &&
        (fcdc1DiscreteWord1.getBitValueOr(23, false) || fcdc2DiscreteWord1.getBitValueOr(23, false)),
    );
    this.elac1HydConfirmNodeOutput.set(
      this.elac1HydConfirmNode.write(!greenSysPressurised && !blueSysPressurised, deltaTime),
    );
    this.elac1FaultConfirmNodeOutput.set(this.elac1FaultConfirmNode.write(elac1FaultCondition, deltaTime));

    // ELAC 2 FAULT computation
    const se2f =
      (fcdc1DiscreteWord1.getBitValueOr(21, false) || fcdc2DiscreteWord1.getBitValueOr(21, false)) &&
      (fcdc1DiscreteWord1.getBitValueOr(22, false) || fcdc2DiscreteWord1.getBitValueOr(22, false));
    const elac2FaultCondition =
      !(
        [1, 10].includes(this.fwcFlightPhase.get()) &&
        (fcdc1DiscreteWord3.getBitValueOr(20, false) || fcdc2DiscreteWord3.getBitValueOr(20, false))
      ) &&
      this.dc2BusPowered.get() &&
      (fcdc1DiscreteWord1.getBitValueOr(24, false) ||
        fcdc2DiscreteWord1.getBitValueOr(24, false) ||
        (!this.elac2HydConfirmNodeOutput.get() && se2f));
    this.elac2FaultLine123Display.set(
      !(fcdc1DiscreteWord3.getBitValueOr(20, false) || fcdc2DiscreteWord3.getBitValueOr(20, false)) &&
        (fcdc1DiscreteWord1.getBitValueOr(24, false) || fcdc2DiscreteWord1.getBitValueOr(24, false)),
    );
    this.elac2HydConfirmNodeOutput.set(
      this.elac2HydConfirmNode.write((!greenSysPressurised || !yellowSysPressurised) && !blueSysPressurised, deltaTime),
    );
    this.elac2FaultConfirmNodeOutput.set(this.elac2FaultConfirmNode.write(elac2FaultCondition, deltaTime));

    // SEC 1 FAULT computation
    const ss1f = fcdc1DiscreteWord1.getBitValueOr(25, false) || fcdc2DiscreteWord1.getBitValueOr(25, false);
    this.sec1FaultCondition.set(
      !(
        [1, 10].includes(this.fwcFlightPhase.get()) &&
        (fcdc1DiscreteWord3.getBitValueOr(27, false) || fcdc2DiscreteWord3.getBitValueOr(27, false))
      ) &&
        this.dcESSBusPowered.get() &&
        ss1f,
    );
    this.sec1FaultLine123Display.set(
      !(fcdc1DiscreteWord3.getBitValueOr(27, false) || fcdc2DiscreteWord3.getBitValueOr(27, false)),
    );

    // SEC 2 FAULT computation
    const ss2f = fcdc1DiscreteWord1.getBitValueOr(26, false) || fcdc2DiscreteWord1.getBitValueOr(26, false);
    this.sec2FaultCondition.set(
      !(
        [1, 10].includes(this.fwcFlightPhase.get()) &&
        (fcdc1DiscreteWord3.getBitValueOr(28, false) || fcdc2DiscreteWord3.getBitValueOr(28, false))
      ) &&
        this.dc2BusPowered.get() &&
        ss2f,
    );
    this.sec2FaultLine123Display.set(
      !(fcdc1DiscreteWord3.getBitValueOr(28, false) || fcdc2DiscreteWord3.getBitValueOr(28, false)),
    );

    // SEC 3 FAULT computation
    const ss3f = fcdc1DiscreteWord1.getBitValueOr(29, false) || fcdc2DiscreteWord1.getBitValueOr(29, false);
    this.sec3FaultCondition.set(
      !(
        [1, 10].includes(this.fwcFlightPhase.get()) &&
        (fcdc1DiscreteWord3.getBitValueOr(29, false) || fcdc2DiscreteWord3.getBitValueOr(29, false))
      ) &&
        this.dc2BusPowered.get() &&
        ss3f,
    );
    this.sec3FaultLine123Display.set(
      !(fcdc1DiscreteWord3.getBitValueOr(29, false) || fcdc2DiscreteWord3.getBitValueOr(29, false)),
    );

    // FCDC 1+2 FAULT computation
    const SFCDC1FT =
      fcdc1DiscreteWord1.isFailureWarning() &&
      fcdc1DiscreteWord2.isFailureWarning() &&
      fcdc1DiscreteWord3.isFailureWarning();
    const SFCDC2FT =
      fcdc2DiscreteWord1.isFailureWarning() &&
      fcdc2DiscreteWord2.isFailureWarning() &&
      fcdc2DiscreteWord3.isFailureWarning();
    const SFCDC12FT = SFCDC1FT && SFCDC2FT;
    this.fcdc12FaultCondition.set(SFCDC12FT && this.dc2BusPowered.get());
    this.fcdc1FaultCondition.set(SFCDC1FT && !SFCDC12FT);
    this.fcdc2FaultCondition.set(SFCDC2FT && !(SFCDC12FT || !this.dc2BusPowered.get()));

    // ALTN LAW 2 computation
    const SPA2 = fcdc1DiscreteWord1.getBitValueOr(13, false) || fcdc2DiscreteWord1.getBitValueOr(13, false);
    this.altn2LawConfirmNodeOutput.set(
      this.altn2LawConfirmNode.write(SPA2 && ![1, 10].includes(this.fwcFlightPhase.get()), deltaTime),
    );

    // ALTN LAW 1 computation
    const SPA1 = fcdc1DiscreteWord1.getBitValueOr(12, false) || fcdc2DiscreteWord1.getBitValueOr(12, false);
    this.altn1LawConfirmNodeOutput.set(
      this.altn1LawConfirmNode.write(SPA1 && ![1, 10].includes(this.fwcFlightPhase.get()), deltaTime),
    );

    // DIRECT LAW computation
    const SPBUL =
      (false && SFCDC12FT) ||
      fcdc1DiscreteWord1.getBitValueOr(15, false) ||
      fcdc2DiscreteWord1.getBitValueOr(15, false);
    this.directLawCondition.set(SPBUL && ![1, 10].includes(this.fwcFlightPhase.get()));

    // L+R ELEV FAULT computation
    const lhElevBlueFail =
      (fcdc1DiscreteWord3.isNormalOperation() && !fcdc1DiscreteWord3.getBitValueOr(15, false)) ||
      (fcdc2DiscreteWord3.isNormalOperation() && !fcdc2DiscreteWord3.getBitValueOr(15, false));
    const lhElevGreenFail =
      (fcdc1DiscreteWord3.isNormalOperation() && !fcdc1DiscreteWord3.getBitValueOr(16, false)) ||
      (fcdc2DiscreteWord3.isNormalOperation() && !fcdc2DiscreteWord3.getBitValueOr(16, false));
    const rhElevBlueFail =
      (fcdc1DiscreteWord3.isNormalOperation() && !fcdc1DiscreteWord3.getBitValueOr(17, false)) ||
      (fcdc2DiscreteWord3.isNormalOperation() && !fcdc2DiscreteWord3.getBitValueOr(17, false));
    const rhElevGreenFail =
      (fcdc1DiscreteWord3.isNormalOperation() && !fcdc1DiscreteWord3.getBitValueOr(18, false)) ||
      (fcdc2DiscreteWord3.isNormalOperation() && !fcdc2DiscreteWord3.getBitValueOr(18, false));
    this.lrElevFaultCondition.set(
      lhElevBlueFail &&
        lhElevGreenFail &&
        rhElevBlueFail &&
        rhElevGreenFail &&
        ![1, 10].includes(this.fwcFlightPhase.get()),
    );

    // GND SPLRS FAULT status
    const sec1GroundSpoilerFault = fcdc1DiscreteWord5.getBitValue(14) || fcdc2DiscreteWord5.getBitValue(14);
    const sec2GroundSpoilerFault = fcdc1DiscreteWord5.getBitValue(15) || fcdc2DiscreteWord5.getBitValue(15);
    const sec3GroundSpoilerFault = fcdc1DiscreteWord5.getBitValue(16) || fcdc2DiscreteWord5.getBitValue(16);
    const sec1SpeedbrakeLeverFault = fcdc1DiscreteWord5.getBitValue(11) || fcdc2DiscreteWord5.getBitValue(11);
    const sec2SpeedbrakeLeverFault = fcdc1DiscreteWord5.getBitValue(12) || fcdc2DiscreteWord5.getBitValue(12);
    const sec3SpeedbrakeLeverFault = fcdc1DiscreteWord5.getBitValue(13) || fcdc2DiscreteWord5.getBitValue(13);
    const allGroundSpoilersInop =
      (sec1GroundSpoilerFault || sec1SpeedbrakeLeverFault) &&
      (sec2GroundSpoilerFault || sec2SpeedbrakeLeverFault) &&
      (sec3GroundSpoilerFault || sec3SpeedbrakeLeverFault);

    this.spoilersArmed.set(fcdc1DiscreteWord4.getBitValueOr(27, false) || fcdc2DiscreteWord4.getBitValueOr(27, false));
    this.speedBrakeCommand.set(
      fcdc1DiscreteWord4.getBitValueOr(28, false) || fcdc2DiscreteWord4.getBitValueOr(28, false),
    );

    // FIXME these should be split between the two systems and the two sides
    const flapsPos = Arinc429Word.fromSimVarValue('L:A32NX_SFCC_FLAP_ACTUAL_POSITION_WORD');
    const slatsPos = Arinc429Word.fromSimVarValue('L:A32NX_SFCC_SLAT_ACTUAL_POSITION_WORD');

    // WARNING these vary for other variants... A320 CFM LEAP values here
    // flap/slat internal signals
    this.flapsInferiorToPositionA.set(flapsPos.isNormalOperation() && flapsPos.value < 65);
    this.flapsSuperiorToPositionD.set(flapsPos.isNormalOperation() && flapsPos.value > 152);
    this.flapsSuperiorToPositionF.set(flapsPos.isNormalOperation() && flapsPos.value > 179);
    this.slatsInferiorToPositionD.set(slatsPos.isNormalOperation() && slatsPos.value < 210.46);
    this.slatsSuperiorToPositionG.set(slatsPos.isNormalOperation() && slatsPos.value > 309.53);
    this.flapsSuperiorToPositionDOrSlatsSuperiorToPositionC.set(
      this.flapsSuperiorToPositionD.get() || (slatsPos.isNormalOperation() && slatsPos.value > 198.1),
    );

    // flap, slat and speedbrake config warning logic
    const flapsNotInToPos = this.flapsSuperiorToPositionF.get() || this.flapsInferiorToPositionA.get();
    this.flapConfigSr.write(
      this.flightPhase34.get() && flapsNotInToPos,
      !flapsNotInToPos || this.fwcFlightPhase.get() === 5,
    );
    this.flapsNotTo.set(this.flightPhase129.get() && flapsNotInToPos);
    this.flapsNotToMemo.set(this.flapConfigSr.read() || this.flapsNotTo.get());
    this.flapConfigAural.set(
      (this.toConfigTestHeldMin1s5Pulse.get() && this.flapsNotTo.get()) ||
        (this.flightPhase34.get() && flapsNotInToPos),
    );
    this.flapConfigWarning.set(
      (this.toConfigTestHeldMin1s5Pulse.get() && this.flapsNotTo.get()) || this.slatConfigSr.read(),
    );

    const slatsNotInToPos = this.slatsInferiorToPositionD.get() || this.slatsSuperiorToPositionG.get();
    this.slatConfigSr.write(
      this.flightPhase34.get() && slatsNotInToPos,
      !slatsNotInToPos || this.fwcFlightPhase.get() === 5,
    );
    this.slatsNotTo.set(this.flightPhase129.get() && slatsNotInToPos);
    this.slatConfigAural.set(
      (this.toConfigTestHeldMin1s5Pulse.get() && this.slatsNotTo.get()) ||
        (this.flightPhase34.get() && slatsNotInToPos),
    );
    this.slatConfigWarning.set(
      (this.toConfigTestHeldMin1s5Pulse.get() && this.slatsNotTo.get()) || this.slatConfigSr.read(),
    );

    const speedbrakesNotInToPos =
      fcdc1DiscreteWord4.getBitValueOr(28, false) || fcdc2DiscreteWord4.getBitValueOr(28, false);
    this.speedbrakesConfigSr.write(
      this.flightPhase34.get() && speedbrakesNotInToPos,
      !speedbrakesNotInToPos || this.fwcFlightPhase.get() === 5,
    );
    this.speedbrakesNotTo.set(this.flightPhase129.get() && speedbrakesNotInToPos);
    this.speedbrakesConfigAural.set(
      (this.toConfigTestHeldMin1s5Pulse.get() && this.speedbrakesNotTo.get()) ||
        (this.flightPhase34.get() && speedbrakesNotInToPos),
    );
    this.speedbrakesConfigWarning.set(
      (this.toConfigTestHeldMin1s5Pulse.get() && this.speedbrakesNotTo.get()) || this.speedbrakesConfigSr.read(),
    );

    // flap/slat MCDU disagree
    // FIXME should come from SDAC via ARINC429
    this.slatFlapSelectionS0F0 = this.flapsHandle.get() === 0;
    this.slatFlapSelectionS18F10 = this.flapsHandle.get() === 1; // FIXME assuming 1+F and not considering 1
    this.slatFlapSelectionS22F15 = this.flapsHandle.get() === 2;
    this.slatFlapSelectionS22F20 = this.flapsHandle.get() === 3;

    const flapsMcduPos1Disagree = xor(this.slatFlapSelectionS18F10, mcduToFlapPos1);
    const flapsMcduPos2Disagree = xor(this.slatFlapSelectionS22F15, mcduToFlapPos2);
    const flapsMcduPos3Disagree = xor(this.slatFlapSelectionS22F20, mcduToFlapPos3);

    this.flapsMcduDisagree.set(
      (flapsMcduPos1Disagree || flapsMcduPos2Disagree || flapsMcduPos3Disagree) &&
        (mcduToFlapPos0 || mcduToFlapPos1 || mcduToFlapPos2 || mcduToFlapPos3),
    );

    this.flapsAndPitchMcduDisagreeEnable.set(
      !this.flightPhase3PulseNode.read() &&
        !this.toConfigPulseNode.read() &&
        (this.fwcFlightPhase.get() === 3 || this.toConfigCheckedInPhase2Or3),
    );

    // pitch trim not takeoff
    const stabPos = SimVar.GetSimVarValue('ELEVATOR TRIM POSITION', 'degree');
    const cgPercent = SimVar.GetSimVarValue('CG PERCENT', 'number') * 100;

    // A320neo config
    const pitchConfig = !PitchTrimUtils.pitchTrimInGreenBand(stabPos);
    this.pitchTrimNotTo.set(this.flightPhase129.get() && pitchConfig);
    const pitchConfigTestInPhase129 =
      pitchConfig && this.toConfigTestHeldMin1s5Pulse.get() && this.flightPhase129.get();
    const pitchConfigInPhase3or4 = this.flightPhase34.get() && pitchConfig;
    this.pitchConfigInPhase3or4Sr.write(pitchConfigInPhase3or4, this.fwcFlightPhase.get() === 5 || !pitchConfig);
    this.pitchTrimNotToAudio.set(pitchConfigTestInPhase129 || pitchConfigInPhase3or4);
    this.pitchTrimNotToWarning.set(pitchConfigTestInPhase129 || this.pitchConfigInPhase3or4Sr.read());

    // pitch trim/mcdu disagree
    // we don't check the trim calculated from CG as it's not available yet. Need FQMS implementation for that
    const fm1PitchTrim = Arinc429Word.fromSimVarValue('L:A32NX_FM1_TO_PITCH_TRIM');
    const fm2PitchTrim = Arinc429Word.fromSimVarValue('L:A32NX_FM2_TO_PITCH_TRIM');
    const fmPitchTrim =
      !fm1PitchTrim.isNormalOperation() && fm2PitchTrim.isNormalOperation() ? fm2PitchTrim : fm1PitchTrim;
    this.trimDisagreeMcduStabConf.write(
      fmPitchTrim.isNormalOperation() &&
        (!PitchTrimUtils.pitchTrimInCyanBand(cgPercent, stabPos) ||
          !(Math.abs(fmPitchTrim.valueOr(0) - cgPercent) < 1)),
      deltaTime,
    );
    this.pitchTrimMcduCgDisagree.set(!this.pitchTrimNotToWarning.get() && this.trimDisagreeMcduStabConf.read());

    // rudder trim not takeoff
    const fac1RudderTrimPosition = Arinc429Word.fromSimVarValue('L:A32NX_FAC_1_RUDDER_TRIM_POS');
    const fac2RudderTrimPosition = Arinc429Word.fromSimVarValue('L:A32NX_FAC_2_RUDDER_TRIM_POS');
    const fac1Healthy = SimVar.GetSimVarValue('L:A32NX_FAC_1_HEALTHY', 'boolean') > 0;
    const fac2Healthy = SimVar.GetSimVarValue('L:A32NX_FAC_2_HEALTHY', 'boolean') > 0;

    const rudderTrimConfig =
      (fac1Healthy && Math.abs(fac1RudderTrimPosition.valueOr(0)) > 3.6) ||
      (fac2Healthy && Math.abs(fac2RudderTrimPosition.valueOr(0)) > 3.6);

    this.rudderTrimNotTo.set(this.flightPhase129.get() && rudderTrimConfig);
    const rudderTrimConfigTestInPhase129 =
      this.toConfigTestHeldMin1s5Pulse.get() && this.flightPhase129.get() && rudderTrimConfig;
    const rudderTrimConfigInPhase3or4 = this.flightPhase34.get() && rudderTrimConfig;
    this.rudderTrimConfigInPhase3or4Sr.write(
      rudderTrimConfigInPhase3or4,
      this.fwcFlightPhase.get() === 5 || !rudderTrimConfig,
    );
    this.rudderTrimNotToAudio.set(rudderTrimConfigTestInPhase129 || rudderTrimConfigInPhase3or4);
    this.rudderTrimNotToWarning.set(rudderTrimConfigTestInPhase129 || this.rudderTrimConfigInPhase3or4Sr.read());

    // flaps lvr not zero
    this.flapsLeverNotZeroWarning.set(
      (adr1PressureAltitude.valueOr(0) >= 22000 ||
        adr2PressureAltitude.valueOr(0) >= 22000 ||
        adr3PressureAltitude.valueOr(0) >= 22000) &&
        this.fwcFlightPhase.get() === 6 &&
        !this.slatFlapSelectionS0F0,
    );

    // spd brk still out
    this.speedBrakeCommand5sConfirm.write(this.speedBrakeCommand.get(), deltaTime);
    this.speedBrakeCommand50sConfirm.write(this.speedBrakeCommand.get(), deltaTime);
    this.engAboveIdleWithSpeedBrakeConfirm.write(
      this.speedBrakeCommand50sConfirm.read() && !oneEngineAboveMinPower,
      deltaTime,
    );
    this.speedBrakeCaution1Confirm.write(
      this.fwcFlightPhase.get() === 6 &&
        this.speedBrakeCommand50sConfirm.read() &&
        !this.engAboveIdleWithSpeedBrakeConfirm.read(),
      deltaTime,
    );
    const speedBrakeCaution1 = this.speedBrakeCaution1Confirm.read();
    const speedBrakeCaution2 = this.fwcFlightPhase.get() === 7 && this.speedBrakeCommand5sConfirm.read();
    // FIXME FCU does not provide the bit, so we synthesize it
    const apVerticalMode = SimVar.GetSimVarValue('L:A32NX_FMA_VERTICAL_MODE', 'number');
    const apTcasRaNoseUp =
      apVerticalMode === VerticalMode.TCAS &&
      SimVar.GetSimVarValue('L:A32NX_TCAS_RA_CORRECTIVE', 'bool') > 0 &&
      SimVar.GetSimVarValue('L:A32NX_TCAS_VSPEED_GREEN:1', 'number') > 0;
    this.apTcasRaNoseUpConfirm.write(apTcasRaNoseUp, deltaTime);
    this.speedBrakeCaution3Confirm.write(
      this.speedBrakeCommand.get() &&
        this.fwcFlightPhase.get() === 6 &&
        oneEngineAboveMinPower &&
        this.apTcasRaNoseUpConfirm.read(),
      deltaTime,
    );
    this.speedBrakeCaution3Monostable.write(this.speedBrakeCaution3Confirm.read(), deltaTime);
    const speedBrakeCaution3 = this.speedBrakeCaution3Confirm.read() || this.speedBrakeCaution3Monostable.read();
    this.amberSpeedBrake.set(
      this.speedBrakeCaution1Confirm.previousInput ||
        speedBrakeCaution2 ||
        speedBrakeCaution3 ||
        !this.flightPhase67.get(),
    );
    const speedBrakeDoNotUse = fcdc1DiscreteWord5.getBitValue(27) || fcdc2DiscreteWord5.getBitValue(27);
    this.speedBrakeCaution1Pulse.write(speedBrakeCaution1, deltaTime);
    this.speedBrakeCaution2Pulse.write(speedBrakeCaution2, deltaTime);
    const speedBrakeCaution = speedBrakeCaution1 || speedBrakeCaution2 || speedBrakeCaution3;
    this.speedBrakeStillOutWarning.set(
      !this.speedBrakeCaution1Pulse.read() &&
        !this.speedBrakeCaution2Pulse.read() &&
        speedBrakeCaution &&
        !speedBrakeDoNotUse,
    );

    // gnd splr not armed
    const raBelow500 =
      this.radioHeight1.valueOr(Infinity) < 500 ||
      this.radioHeight2.valueOr(Infinity) < 500 ||
      this.radioHeight3.valueOr(Infinity) < 500;

    const lgDown =
      this.lgciu1DiscreteWord1.bitValueOr(29, false) ||
      (this.lgciu2DiscreteWord1.bitValueOr(29, false) && mainGearDownlocked);
    this.phase84s5Trigger.write(this.fwcFlightPhase.get() === 8, deltaTime);
    this.groundSpoiler5sDelayed.write(
      fcdc1DiscreteWord4.getBitValueOr(27, false) || fcdc2DiscreteWord4.getBitValueOr(27, false),
      deltaTime,
    );
    this.speedBrake5sDelayed.write(
      fcdc1DiscreteWord4.getBitValueOr(28, false) || fcdc2DiscreteWord4.getBitValueOr(28, false),
      deltaTime,
    );

    this.groundSpoilerNotArmedWarning.set(
      raBelow500 &&
        lgDown &&
        this.flightPhase67.get() &&
        !this.phase84s5Trigger.read() &&
        !this.eng1Or2TakeoffPower.get() &&
        !allGroundSpoilersInop &&
        !(this.groundSpoiler5sDelayed.read() || this.speedBrake5sDelayed.read()) &&
        (fcdc1DiscreteWord4.isNormalOperation() || fcdc2DiscreteWord4.isNormalOperation()),
    );

    // l/g gear not down
    const fwcFlightPhase = this.fwcFlightPhase.get();
    const flightPhase45 = fwcFlightPhase === 4 || fwcFlightPhase === 5;
    const flightPhase6 = fwcFlightPhase === 6;
    const below750Ra =
      Math.min(
        this.radioHeight1.valueOr(Infinity),
        this.radioHeight2.valueOr(Infinity),
        this.radioHeight3.valueOr(Infinity),
      ) < 750;
    const altInhibit =
      (pressureAltitude ?? 0) > 18500 &&
      !this.radioHeight1.isNoComputedData() &&
      !this.radioHeight1.isNormalOperation() &&
      !this.radioHeight2.isNoComputedData() &&
      !this.radioHeight2.isNormalOperation() &&
      !this.radioHeight3.isNoComputedData() &&
      !this.radioHeight3.isNormalOperation();
    const gearNotDownlocked = !mainGearDownlocked && (!this.lgciu1Fault.get() || !this.lgciu2Fault.get());
    const below750Condition =
      this.flapsSuperiorToPositionDOrSlatsSuperiorToPositionC.get() &&
      !this.eng1Or2TakeoffPower.get() &&
      below750Ra &&
      gearNotDownlocked;
    const allRaInvalid =
      this.radioHeight1.isFailureWarning() &&
      this.radioHeight2.isFailureWarning() &&
      this.radioHeight3.isFailureWarning();
    const allRaInvalidOrNcd =
      (this.radioHeight1.isNoComputedData || this.radioHeight1.isFailureWarning()) &&
      (this.radioHeight2.isNoComputedData() || this.radioHeight2.isFailureWarning()) &&
      (this.radioHeight3.isNoComputedData() || this.radioHeight3.isFailureWarning());
    const flapsApprCondition =
      ((this.flapsSuperiorToPositionD.get() && !this.flapsSuperiorToPositionF.get() && allRaInvalid) ||
        (this.flapsSuperiorToPositionF.get() && allRaInvalidOrNcd)) &&
      flightPhase6 &&
      gearNotDownlocked;
    const lgNotDownResetPulse =
      this.lgNotDownPulse1.write(below750Condition, deltaTime) ||
      this.lgNotDownPulse2.write(flapsApprCondition, deltaTime);
    this.lgNotDownNoCancel.set((below750Condition || flapsApprCondition) && !lgNotDownResetPulse);
    const n1Eng1 = this.N1Eng1.get();
    const n1Eng2 = this.N1Eng2.get();
    const apprN1 =
      (n1Eng1 < 75 && n1Eng2 < 75) ||
      (n1Eng1 < 97 && n1Eng2 < 97 && !this.engine1Master.get() && !this.engine2Master.get());
    this.lgNotDown.set(gearNotDownlocked && !altInhibit && !this.eng1Or2TakeoffPower.get() && apprN1 && below750Ra);
    // goes to discrete out (RMP02B) and out word 126-11/25
    const redArrow =
      !((flightPhase6 && !allRaInvalid) || flightPhase45) && (this.lgNotDownNoCancel.get() || this.lgNotDown.get());
    this.lgLeverRedArrow.set(redArrow);

    // 32 - Surveillance Logic
    const isNormalLaw = fcdc1DiscreteWord1.getBitValue(11) || fcdc2DiscreteWord1.getBitValue(11);
    // we need to check this since the MSFS SDK stall warning does not.
    const isCasAbove60 =
      this.adr1Cas.get().valueOr(0) > 60 || this.adr2Cas.valueOr(0) > 60 || this.adr3Cas.valueOr(0) > 60;
    this.stallWarning.set(
      !isNormalLaw &&
        isCasAbove60 &&
        this.stallWarningRaw.get() &&
        flightPhase567 &&
        this.radioHeight1.valueOr(Infinity) > 1500 &&
        this.radioHeight2.valueOr(Infinity) > 1500 &&
        this.radioHeight3.valueOr(Infinity) > 1500,
    );

    /* FIRE */

    this.fireButton1.set(SimVar.GetSimVarValue('L:A32NX_FIRE_BUTTON_ENG1', 'bool'));
    this.fireButton2.set(SimVar.GetSimVarValue('L:A32NX_FIRE_BUTTON_ENG2', 'bool'));
    this.fireButtonAPU.set(SimVar.GetSimVarValue('L:A32NX_FIRE_BUTTON_APU', 'bool'));
    this.eng1FireTest.set(SimVar.GetSimVarValue('L:A32NX_FIRE_TEST_ENG1', 'bool'));
    this.eng2FireTest.set(SimVar.GetSimVarValue('L:A32NX_FIRE_TEST_ENG2', 'bool'));
    this.apuFireTest.set(SimVar.GetSimVarValue('L:A32NX_FIRE_TEST_APU', 'bool'));
    this.eng1Agent1PB.set(SimVar.GetSimVarValue('L:A32NX_FIRE_ENG1_AGENT1_Discharge', 'bool'));
    this.eng1Agent2PB.set(SimVar.GetSimVarValue('L:A32NX_FIRE_ENG1_AGENT2_Discharge', 'bool'));
    this.eng2Agent1PB.set(SimVar.GetSimVarValue('L:A32NX_FIRE_ENG2_AGENT1_Discharge', 'bool'));
    this.eng2Agent2PB.set(SimVar.GetSimVarValue('L:A32NX_FIRE_ENG2_AGENT2_Discharge', 'bool'));
    this.apuAgentPB.set(SimVar.GetSimVarValue('L:A32NX_FIRE_APU_AGENT1_Discharge', 'bool'));
    this.cargoFireTest.set(SimVar.GetSimVarValue('L:A32NX_FIRE_TEST_CARGO', 'bool'));
    this.cargoFireAgentDisch.set(SimVar.GetSimVarValue('L:A32NX_CARGOSMOKE_FWD_DISCHARGED', 'bool'));

    this.agent1Eng1Discharge.set(this.agent1Eng1DischargeTimer.write(this.fireButton1.get(), deltaTime));
    this.agent2Eng1Discharge.set(
      this.agent2Eng1DischargeTimer.write(
        this.fireButton1.get() && this.eng1Agent1PB.get() && !this.aircraftOnGround.get(),
        deltaTime,
      ),
    );
    this.agent1Eng2Discharge.set(
      this.agent1Eng2DischargeTimer.write(this.fireButton2.get() && !this.eng1Agent1PB.get(), deltaTime),
    );
    this.agent2Eng2Discharge.set(
      this.agent2Eng2DischargeTimer.write(this.fireButton2.get() && this.eng1Agent1PB.get(), deltaTime),
    );
    this.agentAPUDischarge.set(
      this.agentAPUDischargeTimer.write(this.fireButton2.get() && this.eng1Agent1PB.get(), deltaTime),
    );

    /* ANTI ICE */

    const icePercentage = SimVar.GetSimVarValue('STRUCTURAL ICE PCT', 'percent over 100');
    const tat = SimVar.GetSimVarValue('TOTAL AIR TEMPERATURE', 'celsius');
    const inCloud = SimVar.GetSimVarValue('AMBIENT IN CLOUD', 'boolean');
    const iceDetected1 = this.iceDetectedTimer1.write(
      icePercentage >= 0.1 && tat < 10 && !this.aircraftOnGround.get(),
      deltaTime,
    );
    this.iceDetectedTimer2Status.set(
      this.iceDetectedTimer2.write(iceDetected1 && !(this.eng1AntiIce.get() && this.eng2AntiIce.get()), deltaTime),
    );
    this.iceSevereDetectedTimerStatus.set(
      this.iceSevereDetectedTimer.write(icePercentage >= 0.5 && tat < 10 && !this.aircraftOnGround.get(), deltaTime),
    );
    const iceNotDetected1 = this.iceNotDetTimer1.write(
      this.eng1AntiIce.get() || this.eng2AntiIce.get() || this.wingAntiIce.get(),
      deltaTime,
    );
    this.iceNotDetTimer2Status.set(
      this.iceNotDetTimer2.write(iceNotDetected1 && !(icePercentage >= 0.1 || (tat < 10 && inCloud === 1)), deltaTime),
    );

    /* CABIN READY */

    const callPushAft = SimVar.GetSimVarValue('L:PUSH_OVHD_CALLS_AFT', 'bool');
    const callPushAll = SimVar.GetSimVarValue('L:PUSH_OVHD_CALLS_ALL', 'bool');
    const callPushFwd = SimVar.GetSimVarValue('L:PUSH_OVHD_CALLS_FWD', 'bool');
    if (callPushAft || callPushAll || callPushFwd) {
      SimVar.SetSimVarValue('L:A32NX_CABIN_READY', 'bool', 1);
    }

    /* MASTER CAUT/WARN BUTTONS */

    const masterCautionButtonLeft = SimVar.GetSimVarValue('L:PUSH_AUTOPILOT_MASTERCAUT_L', 'bool');
    const masterCautionButtonRight = SimVar.GetSimVarValue('L:PUSH_AUTOPILOT_MASTERCAUT_R', 'bool');
    const masterWarningButtonLeft = SimVar.GetSimVarValue('L:PUSH_AUTOPILOT_MASTERAWARN_L', 'bool');
    const masterWarningButtonRight = SimVar.GetSimVarValue('L:PUSH_AUTOPILOT_MASTERAWARN_R', 'bool');
    if (masterCautionButtonLeft || masterCautionButtonRight) {
      this.masterCaution.set(false);
      this.auralSingleChimePending = false;
    }
    if ((masterWarningButtonLeft || masterWarningButtonRight) && this.nonCancellableWarningCount === 0) {
      this.masterWarning.set(false);
      this.auralCrcActive.set(false);
    }

    /* T.O. CONFIG CHECK */

    if (this.toMemo.get() && this.toConfigTestRaw) {
      // TODO Note that fuel tank low pressure and gravity feed warnings are not included
      const systemStatus =
        this.engine1Generator.get() &&
        this.engine2Generator.get() &&
        !this.greenLP.get() &&
        !this.yellowLP.get() &&
        !this.blueLP.get() &&
        this.eng1pumpPBisAuto.get() &&
        this.eng2pumpPBisAuto.get();

      const cabin = SimVar.GetSimVarValue('INTERACTIVE POINT OPEN:0', 'percent');
      const catering = SimVar.GetSimVarValue('INTERACTIVE POINT OPEN:3', 'percent');
      const cargofwdLocked = SimVar.GetSimVarValue('L:A32NX_FWD_DOOR_CARGO_LOCKED', 'bool');
      const cargoaftLocked = SimVar.GetSimVarValue('L:A32NX_AFT_DOOR_CARGO_LOCKED', 'bool');
      const brakesHot = SimVar.GetSimVarValue('L:A32NX_BRAKES_HOT', 'bool');

      const speeds = !toSpeedsTooLow && !toV2VRV2Disagree && !fmToSpeedsNotInserted;
      const doors = !!(cabin === 0 && catering === 0 && cargoaftLocked && cargofwdLocked);
      const surfacesNotTo =
        flapsNotInToPos ||
        slatsNotInToPos ||
        this.speedbrakesNotTo.get() ||
        this.rudderTrimNotTo.get() ||
        this.pitchTrimNotTo.get();

      const toConfigNormal =
        systemStatus && speeds && !brakesHot && doors && !this.flapsMcduDisagree.get() && !surfacesNotTo;

      this.toConfigNormal.set(this.toConfigNormalConf.write(toConfigNormal, deltaTime));
    }

    /* CLEAR AND RECALL */
    if (this.clrTriggerRisingEdge) {
      // delete the first failure
      this.presentedFailures.splice(0, 1);
      this.recallFailures = this.allCurrentFailures.filter((item) => !this.presentedFailures.includes(item));
    }

    if (this.recallTriggerRisingEdge) {
      if (this.recallFailures.length > 0) {
        this.presentedFailures.push(this.recallFailures.shift());
      }
    }

    // Output logic

    this.landAsapRed.set(
      !this.aircraftOnGround.get() &&
        (this.fireButton1.get() ||
          this.eng1FireTest.get() ||
          this.fireButton2.get() ||
          this.eng2FireTest.get() ||
          this.fireButtonAPU.get() ||
          this.apuFireTest.get() ||
          this.emergencyGeneratorOn.get() ||
          (this.engine1State.get() === 0 && this.engine2State.get() === 0) ||
          (this.greenLP.get() && this.yellowLP.get()) ||
          (this.yellowLP.get() && this.blueLP.get()) ||
          (this.greenLP.get() && this.blueLP.get())),
    );

    // fire always forces the master warning and SC aural on
    this.fireActive.set(
      [this.eng1FireTest.get(), this.eng2FireTest.get(), this.apuFireTest.get(), this.cargoFireTest.get()].some(
        (e) => e,
      ),
    );

    const flightPhase = this.fwcFlightPhase.get();
    let tempMemoArrayLeft: string[] = [];
    let tempMemoArrayRight: string[] = [];
    const allFailureKeys: string[] = [];
    const stsInfoKeys: string[] = [];
    const stsInopAllPhasesKeys: string[] = [];
    const stsInopApprLdgKeys: string[] = [];
    const ewdLimitationsAllPhasesKeys: string[] = [];
    const ewdLimitationsApprLdgKeys: string[] = [];
    const pfdLimitationsKeys: string[] = [];
    let failureKeys: string[] = this.presentedFailures;
    let recallFailureKeys: string[] = this.recallFailures;
    let failureSystemCount = 0;
    const rightFailureSystemCount = 0;
    const auralCrcKeys: string[] = [];
    const auralScKeys: string[] = [];

    // Update memos and failures list in case failure has been resolved
    for (const [key, value] of Object.entries(this.abnormalSensed.ewdAbnormalSensed)) {
      if (!value.simVarIsActive.get() || value.flightPhaseInhib.some((e) => e === flightPhase)) {
        failureKeys = failureKeys.filter((e) => e !== key);
        recallFailureKeys = recallFailureKeys.filter((e) => e !== key);
      }
    }

    this.recallFailures.length = 0;
    this.recallFailures.push(...recallFailureKeys);
    this.nonCancellableWarningCount = 0;
    let stateWasChanged = false;

    // Abnormal sensed procedures
    const ewdAbnormalEntries: [string, EwdAbnormalItem][] = Object.entries(this.abnormalSensed.ewdAbnormalSensed);
    for (const [key, value] of ewdAbnormalEntries) {
      if (value.flightPhaseInhib.some((e) => e === flightPhase)) {
        continue;
      }

      // new warning?
      const newWarning = !this.presentedFailures.includes(key) && !recallFailureKeys.includes(key);
      const proc = EcamAbnormalSensedProcedures[key] as AbnormalProcedure;

      if (value.simVarIsActive.get()) {
        // Skip if other fault overrides this one
        let overridden = false;
        value.notActiveWhenFaults.forEach((val) => {
          if (val && this.abnormalSensed.ewdAbnormalSensed[val]) {
            const otherFault = this.abnormalSensed.ewdAbnormalSensed[val] as EwdAbnormalItem;
            if (otherFault.simVarIsActive.get()) {
              overridden = true;
            }
          }
        });
        if (overridden) {
          continue;
        }

        const itemsCompleted = value.whichItemsCompleted();
        const itemsToShow = value.whichItemsToShow ? value.whichItemsToShow() : Array(itemsCompleted.length).fill(true);
        const itemsActive = value.whichItemsActive ? value.whichItemsActive() : Array(itemsCompleted.length).fill(true);

        if (newWarning) {
          failureKeys.push(key);

          if (value.failure === 3) {
            this.masterWarning.set(true);
          }
          if (value.failure === 2) {
            this.masterCaution.set(true);
          }
        }

        if (!this.activeAbnormalSensedList.has(key)) {
          // Insert into internal map
          if (value.whichItemsActive) {
            if (proc.items.length !== value.whichItemsActive().length) {
              console.warn(
                proc.title,
                'ECAM alert definition error: whichItemsActive() not the same size as number of procedure items',
              );
            }
          }
          if (value.whichItemsToShow) {
            if (proc.items.length !== value.whichItemsToShow().length) {
              console.warn(
                proc.title,
                'ECAM alert definition error: whichItemsToShow() not the same size as number of procedure items',
              );
            }
          }
          if (proc.items.length !== value.whichItemsCompleted().length) {
            console.warn(
              proc.title,
              'ECAM alert definition error: whichItemsCompleted() not the same size as number of procedure items',
            );
          }
          this.activeAbnormalSensedList.set(key, {
            id: key,
            itemsActive: itemsActive,
            itemsCompleted: itemsCompleted,
            itemsToShow: itemsToShow,
          });
          stateWasChanged = true;
        } else {
          // Update internal map
          const prevEl = this.activeAbnormalSensedList.get(key);
          // Update only sensed items
          proc.items.forEach((item, idx) => {
            if (item.sensed === true) {
              if (
                prevEl.itemsToShow[idx] !== itemsToShow[idx] ||
                prevEl.itemsActive[idx] !== itemsActive[idx] ||
                prevEl.itemsCompleted[idx] !== itemsCompleted[idx]
              ) {
                stateWasChanged = true;
              }
              prevEl.itemsToShow[idx] = itemsToShow[idx];
              prevEl.itemsActive[idx] = itemsActive[idx];
              prevEl.itemsCompleted[idx] = itemsCompleted[idx];
            }
          });
        }

        if (value.cancel === false && value.failure === 3) {
          this.nonCancellableWarningCount++;
        }

        // if the warning is the same as the aural
        if (value.auralWarning === undefined && value.failure === 3) {
          if (newWarning) {
            this.auralCrcActive.set(true);
          }
          auralCrcKeys.push(key);
        }
        if (value.auralWarning === undefined && value.failure === 2) {
          if (newWarning) {
            this.auralSingleChimePending = true;
            console.log('single chime pending');
          }
          auralScKeys.push(key);
        }

        allFailureKeys.push(key);

        // Add keys for STS page
        FwsCore.pushKeyUnique(value.info, stsInfoKeys);
        FwsCore.pushKeyUnique(value.inopSysAllPhases, stsInopAllPhasesKeys);
        FwsCore.pushKeyUnique(value.inopSysApprLdg, stsInopApprLdgKeys);
        FwsCore.pushKeyUnique(value.limitationsAllPhases, ewdLimitationsAllPhasesKeys);
        FwsCore.pushKeyUnique(value.limitationsApprLdg, ewdLimitationsApprLdgKeys);
        FwsCore.pushKeyUnique(value.limitationsPfd, pfdLimitationsKeys);

        if (!recallFailureKeys.includes(key)) {
          if (value.sysPage > -1) {
            failureSystemCount++;
          }
        }

        if (value.sysPage > -1) {
          SimVar.SetSimVarValue('L:A32NX_ECAM_SFAIL', 'number', value.sysPage);
        }
      }

      if (value.auralWarning?.get() === FwcAuralWarning.Crc) {
        if (!this.auralCrcKeys.includes(key)) {
          this.auralCrcActive.set(true);
        }
        auralCrcKeys.push(key);
      }

      if (value.auralWarning?.get() === FwcAuralWarning.SingleChime) {
        if (!this.auralScKeys.includes(key)) {
          this.auralSingleChimePending = true;
        }
        auralScKeys.push(key);
      }
    }

    // Delete inactive failures from internal map
    this.activeAbnormalSensedList.forEach((_, key) => {
      if (!allFailureKeys.includes(key)) {
        this.activeAbnormalSensedList.delete(key);
        stateWasChanged = true;
      }
    });

    this.auralCrcKeys = auralCrcKeys;
    this.auralScKeys = auralScKeys;

    if (this.auralCrcKeys.length === 0) {
      this.auralCrcActive.set(false);
    }

    if (this.auralScKeys.length === 0) {
      this.auralSingleChimePending = false;
    }

    const failOrder: string[] = [];

    for (const [key] of Object.entries(this.abnormalSensed.ewdAbnormalSensed)) {
      failOrder.push(...key);
    }

    this.allCurrentFailures.length = 0;
    this.allCurrentFailures.push(...allFailureKeys);

    this.presentedFailures.length = 0;
    this.presentedFailures.push(...failureKeys);

    if (stateWasChanged) {
      console.log('%c------- ABN SENSED PROCEDURES -------', 'font-family:monospace; font-weight: bold');
      // Debug output for ABN sensed procedures
      this.activeAbnormalSensedList.forEach((val, key) => {
        const proc = EcamAbnormalSensedProcedures[key] as AbnormalProcedure;
        console.log('%c' + proc.title, 'font-family:monospace; font-weight: bold');
        proc.items.forEach((it, itemIdx) => {
          if (val.itemsToShow[itemIdx]) {
            const cpl = isChecklistAction(it)
              ? val.itemsCompleted[itemIdx]
                ? it.labelNotCompleted
                : ` .......... ${it.labelNotCompleted}`
              : '';
            console.log(
              `%c${'  '.repeat(it.level ?? 0)} ${it.sensed ? (val.itemsCompleted[itemIdx] ? 'X' : 'O') : ' '} ${it.name} ${cpl} ${it.color ? `(${it.color})` : ''}`,
              'font-family:monospace; font-weight: bold',
            );
          }
        });
      });

      console.log('%c------- END -------', 'font-family:monospace; font-weight: bold');
    }

    const fwsCdsProcedures: FwsCdsAbnormalSensedList = Array.from(this.activeAbnormalSensedList.keys()).map((abn) => {
      return { id: abn, itemsActive: [], itemsCompleted: [], itemsToShow: [] };
    });
    this.bus.getPublisher<FwsCdsEvents>().pub('fws_abnormal_sensed_procedures', fwsCdsProcedures);
    SimVar.SetSimVarValue('L:A32NX_EWD_DEBUG_ABNORMAL', 'string', fwsCdsProcedures[0] ? fwsCdsProcedures[0].id : '');

    // MEMOs (except T.O and LDG)
    for (const [, value] of Object.entries(this.memos.ewdMemos)) {
      if (
        value.simVarIsActive.get() &&
        !value.memoInhibit() &&
        !value.flightPhaseInhib.some((e) => e === flightPhase)
      ) {
        const newCode: string[] = [];

        const codeIndex = value.whichCodeToReturn().filter((e) => e !== null);
        codeIndex.forEach((e: number) => {
          newCode.push(value.codesToReturn[e]);
        });
        const tempArrayRight = tempMemoArrayRight.filter((e) => !value.codesToReturn.includes(e));
        tempMemoArrayRight = tempArrayRight.concat(newCode);

        if (value.sysPage > -1) {
          SimVar.SetSimVarValue('L:A32NX_ECAM_SFAIL', 'number', value.sysPage);
        }
      }
    }

    // T.O and LDG MEMOs
    for (const [, value] of Object.entries(this.memos.ewdToLdgMemos)) {
      if (
        value.simVarIsActive.get() &&
        !value.memoInhibit() &&
        !value.flightPhaseInhib.some((e) => e === flightPhase)
      ) {
        const newCode: string[] = [];

        const codeIndex = value.whichCodeToReturn().filter((e) => e !== null);
        codeIndex.forEach((e: number) => {
          newCode.push(value.codesToReturn[e]);
        });

        tempMemoArrayLeft = tempMemoArrayLeft.concat(newCode);
        if (value.sysPage > -1) {
          SimVar.SetSimVarValue('L:A32NX_ECAM_SFAIL', 'number', value.sysPage);
        }
      }
    }

    const memoOrderLeft: string[] = [];
    const memoOrderRight: string[] = [];

    for (const [, value] of Object.entries(this.memos.ewdToLdgMemos)) {
      if (value.side === 'LEFT') {
        memoOrderLeft.push(...value.codesToReturn);
      } else {
        memoOrderRight.push(...value.codesToReturn);
      }
    }

    const orderedMemoArrayLeft = this.mapOrder(tempMemoArrayLeft, memoOrderLeft);
    const orderedMemoArrayRight: string[] = this.mapOrder(tempMemoArrayRight, memoOrderRight);

    if (this.allCurrentFailures.length === 0) {
      this.masterCaution.set(false);
      if (this.nonCancellableWarningCount === 0) {
        this.masterWarning.set(false);
      }
    }

    if (failureSystemCount + rightFailureSystemCount === 0) {
      SimVar.SetSimVarValue('L:A32NX_ECAM_SFAIL', 'number', -1);
    }

    this.ewdMessageLinesLeft.forEach((l, i) => l.set(orderedMemoArrayLeft[i]));
    this.ewdMessageLinesRight.forEach((l, i) => l.set(orderedMemoArrayRight[i]));

    // TODO order by decreasing importance
    this.pfdMemoLines.forEach((l, i) => l.set(orderedMemoArrayRight.filter((it) => pfdMemoDisplay.includes(it))[i]));

    // TODO order by decreasing importance
    this.sdStatusInfoLines.forEach((l, i) => l.set(stsInfoKeys[i]));
    this.sdStatusInopAllPhasesLines.forEach((l, i) => l.set(stsInopAllPhasesKeys[i]));
    this.sdStatusInopApprLdgLines.forEach((l, i) => l.set(stsInopApprLdgKeys[i]));

    // TODO order by decreasing importance
    this.ewdLimitationsAllPhasesLines.forEach((l, i) => l.set(ewdLimitationsAllPhasesKeys[i]));
    this.ewdLimitationsApprLdgLines.forEach((l, i) => l.set(ewdLimitationsApprLdgKeys[i]));

    // For now, also push EWD limitations to PFD, until EWD limitations are implemented
    const pfdLimitationsCombined = [
      ...new Set(pfdLimitationsKeys.concat(ewdLimitationsAllPhasesKeys).concat(ewdLimitationsApprLdgKeys)),
    ];
    this.pfdLimitationsLines.forEach((l, i) => l.set(pfdLimitationsCombined[i]));

    // This does not consider interrupting c-chord, priority of synthetic voice etc.
    // We shall wait for the rust FWC for those nice things!
    if (this.auralSingleChimePending && !this.auralCrcActive.get() && !this.auralSingleChimeInhibitTimer.isPending()) {
      this.auralSingleChimePending = false;
      SimVar.SetSimVarValue('L:A32NX_FWC_SC', 'bool', true);
      // there can only be one SC per 2 seconds, non-cumulative, so clear any pending ones at the end of that inhibit period
      this.auralSingleChimeInhibitTimer.schedule(
        () => (this.auralSingleChimePending = false),
        FwsCore.AURAL_SC_INHIBIT_TIME,
      );
      this.auralSingleChimePlayingTimer.schedule(
        () => SimVar.SetSimVarValue('L:A32NX_FWC_SC', 'bool', false),
        FwsCore.AURAL_SC_PLAY_TIME,
      );
    }

    this.updateRowRopWarnings();
  }

  updateRowRopWarnings() {
    const w = Arinc429Word.fromSimVarValue('L:A32NX_ROW_ROP_WORD_1');

    // ROW
    SimVar.SetSimVarValue('L:A32NX_AUDIO_ROW_RWY_TOO_SHORT', 'bool', w.getBitValueOr(15, false));

    // ROP
    // MAX BRAKING, only for manual braking, if maximum pedal braking is not applied
    const maxBrakingSet =
      SimVar.GetSimVarValue('L:A32NX_LEFT_BRAKE_PEDAL_INPUT', 'number') > 90 ||
      SimVar.GetSimVarValue('L:A32NX_RIGHT_BRAKE_PEDAL_INPUT', 'number') > 90;
    const maxBraking = w.getBitValueOr(13, false) && !maxBrakingSet;
    SimVar.SetSimVarValue('L:A32NX_AUDIO_ROP_MAX_BRAKING', 'bool', maxBraking);

    // SET MAX REVERSE, if not already max. reverse set and !MAX_BRAKING
    const maxReverseSet =
      SimVar.GetSimVarValue('L:XMLVAR_Throttle1Position', 'number') < 0.1 &&
      SimVar.GetSimVarValue('L:XMLVAR_Throttle2Position', 'number') < 0.1;
    const maxReverse = (w.getBitValueOr(12, false) || w.getBitValueOr(13, false)) && !maxReverseSet;
    SimVar.SetSimVarValue('L:A32NX_AUDIO_ROW_SET_MAX_REVERSE', 'bool', !maxBraking && maxReverse);

    // At 80kt, KEEP MAX REVERSE once, if max. reversers deployed
    const ias = SimVar.GetSimVarValue('AIRSPEED INDICATED', 'knots');
    SimVar.SetSimVarValue(
      'L:A32NX_AUDIO_ROP_KEEP_MAX_REVERSE',
      'bool',
      ias <= 80 && ias > 4 && (w.getBitValueOr(12, false) || w.getBitValueOr(13, false)),
    );
  }
}
