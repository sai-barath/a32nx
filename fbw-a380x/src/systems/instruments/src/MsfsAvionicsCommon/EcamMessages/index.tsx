// Copyright (c) 2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

import { EcamAbnormalSensedAta212223 } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata21-22-23';
import { EcamAbnormalSensedAta24 } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata24';
import { EcamAbnormalSensedAta26 } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata26';
import { EcamAbnormalSensedAta27 } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata27';
import { EcamAbnormalSensedAta28 } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata28';
import { EcamAbnormalSensedAta2930 } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata29-30';
import { EcamAbnormalSensedAta313233 } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata31-32-33';
import { EcamAbnormalSensedAta34 } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata34';
import { EcamAbnormalSensedAta353642 } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata35-36-42';
import { EcamAbnormalSensedAta46495256 } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata46-49-52-56';
import { EcamAbnormalSensedAta70 } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata70';
import { EcamAbnormalSensedAta80Rest } from 'instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/ata80-rest';

// Convention for IDs:
// First two digits: ATA chapter
// Third digit: Sub chapter, if needed
// Fourth digit:
//    0 for MEMOs,
//    1 for normal checklists,
//    2 for infos,
//    3 for INOP SYS,
//    4 for limitations,
//    8 for ABN sensed procedures,
//    9 for ABN non-sensed procedures

/** All MEMOs should be here, EWD and PFD. */
export const EcamMemos: { [n: string]: string } = {
  '000000001': '              \x1b<3mNORMAL',
  '000001001': ' \x1b<7m\x1b4mT.O\x1bm',
  '000001002': '   \x1b<5m-SEAT BELTS ....ON',
  '000001003': '   \x1b<3m-SEAT BELTS ON',
  '000001006': '   \x1b<5m-GND SPLRs ....ARM',
  '000001007': '   \x1b<3m-GND SPLRs ARM',
  '000001008': '   \x1b<5m-FLAPS ........T.O',
  '000001009': '   \x1b<3m-FLAPS : T.O',
  '000001010': '   \x1b<5m-AUTO BRAKE ...RTO',
  '000001011': '   \x1b<3m-AUTO BRAKE RTO',
  '000001012': '   \x1b<5m-T.O CONFIG ..TEST',
  '000001013': '   \x1b<3m-T.O CONFIG NORMAL',
  '000002001': ' \x1b<7m\x1b4mLDG\x1bm',
  '000002002': '   \x1b<5m-SEAT BELTS ....ON',
  '000002003': '   \x1b<3m-SEAT BELTS ON',
  '000002006': '   \x1b<5m-LDG GEAR ....DOWN',
  '000002007': '   \x1b<3m-LDG GEAR DOWN',
  '000002008': '   \x1b<5m-GND SPLRs ....ARM',
  '000002009': '   \x1b<3m-GND SPLRs ARM',
  '000002010': '   \x1b<5m-FLAPS ........LDG',
  '000002011': '   \x1b<3m-FLAPS : LDG',
  '320000001': '\x1b<4mAUTO BRK OFF',
  '320000002': '\x1b<3mPARK BRK ON',
  '321000001': '\x1b<3mFLT L/G DOWN',
  '321000002': '\x1b<3mL/G GRVTY EXTN',
  '322000001': '\x1b<4mN/W STEER DISC',
  '322000002': '\x1b<3mN/W STEER DISC',
  '000005001': '\x1b<3mREFUELG',
  '000005501': '\x1b<3mGND SPLRs ARMED',
  '000056101': '\x1b<3mCOMPANY ALERT',
  '000056102': '\x1b<3m\x1b)mCOMPANY ALERT',
  '000006001': '\x1b<3mSPEED BRK',
  '000006002': '\x1b<4mSPEED BRK',
  '000010501': '\x1b<3mOUTR TK FUEL XFRD',
  '000011001': '\x1b<3mFOB BELOW 3 T',
  '000011002': '\x1b<3mFOB BELOW 6600 LBS',
  '000013501': '\x1b<3mACARS STBY',
  '000030501': '\x1b<3mGPWS FLAP MODE OFF',
  '000066001': '\x1b<3mGSM DISC < 4MN',
  '290000001': '\x1b<3mG ELEC PMP A CTL',
  '290000002': '\x1b<3mG ELEC PMP B CTL',
  '290000003': '\x1b<3mY ELEC PMP A CTL',
  '290000004': '\x1b<3mY ELEC PMP B CTL',
  '000017001': '\x1b<3mAPU AVAIL',
  '000018001': '\x1b<3mAPU BLEED',
  '000019001': '\x1b<3mLDG LT',
  '240000001': '\x1b<3mCOMMERCIAL PART SHED',
  '241000001': '\x1b<4mELEC EXT PWR',
  '241000002': '\x1b<3mELEC EXT PWR',
  '242000001': '\x1b<4mRAT OUT',
  '242000002': '\x1b<3mRAT OUT',
  '243000001': '\x1b<3mREMOTE C/B CTL ON',
  '000023001': '\x1b<3mMAN LDG ELEV',
  '000025001': '\x1b<3mFUEL X FEED',
  '000025002': '\x1b<4mFUEL X FEED',
  '000026001': '\x1b<3mENG A. ICE',
  '000027001': '\x1b<3mWING A. ICE',
  '000027501': '\x1b<3mICE NOT DET',
  '000029001': '\x1b<3mSWITCHG PNL',
  '000030001': '\x1b<3mGPWS FLAP 3',
  '000032001': '\x1b<3mTCAS STBY',
  '000032501': '\x1b<4mTCAS STBY',
  '000035001': '\x1b<2mLAND ASAP',
  '000036001': '\x1b<4mLAND ASAP',
  '000054001': '\x1b<3mPRED W/S OFF',
  '000054002': '\x1b<4mPRED W/S OFF',
  '000054501': '\x1b<3mTERR OFF',
  '000054502': '\x1b<4mTERR OFF',
  '000055201': '\x1b<3mCOMPANY MSG',
  '000056001': '\x1b<3mHI ALT SET',
  '220000001': '\x1b<2mAP OFF',
  '220000002': '\x1b<4mA/THR OFF',
  '221000001': '\x1b<3mFMS SWTG',
  '230000001': '\x1b<3mCAPT ON RMP 3',
  '230000002': '\x1b<3mF/O ON RMP 3',
  '230000003': '\x1b<3mCAPT+F/O ON RMP 3',
  '230000004': '\x1b<3mCABIN READY',
  '230000005': '\x1b<3mCPNY DTLNK NOT AVAIL',
  '230000006': '\x1b<3mGND HF DATALINK OVRD',
  '230000007': '\x1b<3mHF VOICE',
  '230000008': '\x1b<3mPA IN USE',
  '230000009': '\x1b<3mRMP 1+2+3 OFF',
  '230000010': '\x1b<3mRMP 1+3 OFF',
  '230000011': '\x1b<3mRMP 2+3 OFF',
  '230000012': '\x1b<3mRMP 3 OFF',
  '230000013': '\x1b<3mSATCOM ALERT',
  '230000014': '\x1b<3mVHF DTLNK MAN SCAN',
  '230000015': '\x1b<3mVHF VOICE',
  '271000001': '\x1b<3mGND SPLRs ARMED',
  '280000001': '\x1b<3mCROSSFEED OPEN',
  '280000002': '\x1b<3mCOLDFUEL OUTR TK XFR',
  '280000003': '\x1b<3mDEFUEL IN PROGRESS',
  '280000004': '\x1b<3mFWD XFR IN PROGRESS',
  '280000005': '\x1b<3mGND XFR IN PROGRESS',
  '280000006': '\x1b<3mJETTISON IN PROGRESS',
  '280000007': '\x1b<3mOUTR TK XFR IN PROG',
  '280000008': '\x1b<3mOUTR TKS XFRD',
  '280000009': '\x1b<3mREFUEL IN PROGRESS',
  '280000010': '\x1b<3mREFUEL PNL DOOR OPEN',
  '280000011': '\x1b<3mREFUEL PNL DOOR OPEN',
  '280000012': '\x1b<3mTRIM TK XFRD',
  '308118601': '\x1b<4m\x1b4mSEVERE ICE\x1bm DETECTED',
  '310000001': '\x1b<4mMEMO NOT AVAIL',
  '314000001': '\x1b<6mT.O INHIBIT',
  '314000002': '\x1b<6mLDG INHIBIT',
  '317000001': '\x1b<3mCLOCK INT',
  '340000001': '\x1b<3mTRUE NORTH REF',
  '340002701': '\x1b<3mIR 1 IN ATT ALIGN',
  '340002702': '\x1b<3mIR 2 IN ATT ALIGN',
  '340002703': '\x1b<3mIR 3 IN ATT ALIGN',
  '340002704': '\x1b<3mIR 1+2 IN ATT ALIGN',
  '340002705': '\x1b<3mIR 1+3 IN ATT ALIGN',
  '340002706': '\x1b<3mIR 2+3 IN ATT ALIGN',
  '340002707': '\x1b<3mIR 1+2+3 IN ATT ALIGN',
  '340003001': '\x1b<3mIR IN ALIGN > 7 MN',
  '340003002': '\x1b<4mIR IN ALIGN > 7 MN',
  '340003003': '\x1b<3mIR IN ALIGN 6 MN',
  '340003004': '\x1b<4mIR IN ALIGN 6 MN',
  '340003005': '\x1b<3mIR IN ALIGN 5 MN',
  '340003006': '\x1b<4mIR IN ALIGN 5 MN',
  '333000001': '\x1b<3mSTROBE LT OFF',
  '335000001': '\x1b<3mSEAT BELTS',
  '335000002': '\x1b<3mNO SMOKING',
  '335000003': '\x1b<3mNO MOBILE',
  '340003007': '\x1b<3mIR IN ALIGN 4 MN',
  '340003008': '\x1b<4mIR IN ALIGN 4 MN',
  '340003101': '\x1b<3mIR IN ALIGN 3 MN',
  '340003102': '\x1b<4mIR IN ALIGN 3 MN',
  '340003103': '\x1b<3mIR IN ALIGN 2 MN',
  '340003104': '\x1b<4mIR IN ALIGN 2 MN',
  '340003105': '\x1b<3mIR IN ALIGN 1 MN',
  '340003106': '\x1b<4mIR IN ALIGN 1 MN',
  '340003107': '\x1b<3mIR IN ALIGN',
  '340003108': '\x1b<4mIR IN ALIGN',
  '340003109': '\x1b<3mIR ALIGNED',
  '340068001': '\x1b<3mADIRS SWTG',
  '709000001': '\x1b<3mIGNITION',
};

/** Only these IDs will be shown in the PFD MEMO section */
export const pfdMemoDisplay: string[] = [
  '322000001',
  '000006002',
  '000026001',
  '000027001',
  '000032501',
  '000035001',
  '000036001',
  '000054502',
  '220000001',
  '220000002',
  '320000001',
];

/** All possible INFOs (e.g. CAT 3 SINGLE ONLY), with special formatting characters. */
export const EcamInfos: { [n: number]: string } = {
  220200001: '\x1b<3mFMS 1 ON FMC-C',
  220200002: '\x1b<3mFMS 2 ON FMC-C',
  220200003: '\x1b<3mSTBY INSTRUMENTS NAV AVAIL',
  220200004: '\x1b<3mCAT 2 ONLY',
  220200005: '\x1b<3mCAT 3 SINGLE ONLY',
  220200006: '\x1b<3mFOR AUTOLAND: MAN ROLL OUT ONLY',
  220200007: '\x1b<3mAPPR MODE NOT AVAIL',
  220200008: '\x1b<3mLOC MODE AVAIL ONLY',
  220200009: '\x1b<3mWHEN L/G DOWN AND AP OFF: USE MAN PITCH TRIM',
  220200010: '\x1b<3mCAT 1 ONLY',
  230200001: '\x1b<3mSATCOM DATALINK AVAIL',
  340200002: '\x1b<3mALTN LAW : PROT LOST',
  340200003: '\x1b<3mFLS LIMITED TO F-APP + RAW',
  340200004: '\x1b<3mDIRECT LAW : PROT LOST',
  340200005: '\x1b<3mPFD BKUP SPEED & ALT AVAIL',
  340200006: '\x1b<3mFPV / VV AVAIL',
  340200007: '\x1b<3mCABIN ALT TRGT: SEE FCOM', // TODO add table
  340200008: '\x1b<3mSTANDBY NAV IN TRUE GPS TRK',
};

/** All possible LIMITATIONs, with special formatting characters. */
export const EcamLimitations: { [n: number]: string } = {
  230400001: '\x1b<5mNO COM AVAIL',
  240400001: '\x1b<5mGA THR : TOGA ONLY',
  240400002: '\x1b<5mMAX SPEED: 310/.86',
  240400003: '\x1b<5mSPD BRK: DO NOT USE',
  240400004: '\x1b<5mMANEUVER WITH CARE',
  300400001: '\x1b<5mAVOID ICING CONDs',
};

/** All possible INOP sys, with special formatting characters. */
export const EcamInopSys: { [n: number]: string } = {
  210300001: '\x1b<4mCAB PRESS AUTO CTL',
  220300001: '\x1b<4mA/THR',
  220300002: '\x1b<4mCAT 3',
  220300004: '\x1b<4mAFS CTL PNL',
  220300005: '\x1b<4mAP 1',
  220300006: '\x1b<4mAP 2',
  220300007: '\x1b<4mAP 1+2',
  220300008: '\x1b<4mCAT 3 DUAL',
  220300009: '\x1b<4mCAT 2',
  220300010: '\x1b<4mGLS AUTOLAND',
  220300012: '\x1b<4mCAPT AFS BKUP CTL',
  220300013: '\x1b<4mF/O AFS BKUP CTL',
  220300014: '\x1b<4mENG 1 A/THR',
  220300015: '\x1b<4mENG 2 A/THR',
  220300016: '\x1b<4mENG 3 A/THR',
  220300017: '\x1b<4mENG 4 A/THR',
  220300018: '\x1b<4mROLL OUT',
  220300020: '\x1b<4mAP/FD TCAS MODE',
  220300021: '\x1b<4mREACTIVE W/S DET',
  220300022: '\x1b<4mFD 1',
  220300023: '\x1b<4mFD 2',
  220300024: '\x1b<4mFD 1+2',
  220300025: '\x1b<4mGA SOFT',
  220300026: '\x1b<4mAUTOLAND',
  221300001: '\x1b<4mFMC-A',
  221300002: '\x1b<4mFMC-B',
  221300003: '\x1b<4mFMC-C',
  221300004: '\x1b<4mFMS 1',
  221300005: '\x1b<4mFMS 2',
  221300006: '\x1b<4mFMS 1+2',
  230300001: '\x1b<4mCIDS 1+2+3',
  230300002: '\x1b<4mUPPER DECK PA',
  230300003: '\x1b<4mMAIN DECK PA',
  230300004: '\x1b<4mLOWER DECK PA',
  230300005: '\x1b<4mCABIN INTERPHONE',
  230300006: '\x1b<4mDATALINK',
  230300007: '\x1b<4mHF 1 DATALINK',
  230300008: '\x1b<4mHF 2 DATALINK',
  230300009: '\x1b<4mRMP 1',
  230300010: '\x1b<4mRMP 2',
  230300011: '\x1b<4mRMP 3',
  230300012: '\x1b<4mRMP 1+2',
  230300013: '\x1b<4mRMP 1+3',
  230300014: '\x1b<4mRMP 2+3',
  230300015: '\x1b<4mSTBY RAD NAV',
  230300016: '\x1b<4mRMP 1+2+3',
  230300017: '\x1b<4mVHF 1+2+3',
  230300018: '\x1b<4mHF 1+2',
  230300019: '\x1b<4mSATCOM',
  230300020: '\x1b<4mSATCOM DATALINK',
  230300021: '\x1b<4mVHF 3 DATALINK',
  240300001: '\x1b<4mAPU BAT',
  240300002: '\x1b<4mAPU GEN A ',
  240300003: '\x1b<4mAPU GEN B',
  240300004: '\x1b<4mAPU TR',
  240300005: '\x1b<4mBAT 1',
  240300006: '\x1b<4mBAT 2',
  240300007: '\x1b<4mBAT ESS',
  240300008: '\x1b<4mBAT 1+2',
  240300009: '\x1b<4mC/B MONITORING',
  240300010: '\x1b<4mGEN 1',
  240300011: '\x1b<4mGEN 2',
  240300012: '\x1b<4mGEN 3',
  240300013: '\x1b<4mGEN 4',
  240300014: '\x1b<4mAPU GEN A',
  240300015: '\x1b<4mAPU GEN B',
  240300016: '\x1b<4mEXT PWR 1',
  240300017: '\x1b<4mEXT PWR 2',
  240300018: '\x1b<4mEXT PWR 3',
  240300019: '\x1b<4mEXT PWR 4',
  240300020: '\x1b<4mENMU 1',
  240300021: '\x1b<4mENMU 2',
  240300022: '\x1b<4mPART GALLEY',
  240300023: '\x1b<4mEMER C/B MONITORING',
  240300024: '\x1b<4mELEC LOAD MANAGT',
  240300025: '\x1b<4mELEC PRIMARY CTR 1',
  240300026: '\x1b<4mELEC PRIMARY CTR 2',
  240300027: '\x1b<4mCOMMERCIAL',
  240300028: '\x1b<4mPART COMMERCIAL',
  240300029: '\x1b<4mRAT',
  240300030: '\x1b<4mELEC SECONDARY CTR 1',
  240300031: '\x1b<4mELEC SECONDARY CTR 2',
  240300032: '\x1b<4mPART ELEC SEC CTR 1',
  240300033: '\x1b<4mPART ELEC SEC CTR 2',
  240300034: '\x1b<4mTR 1',
  240300035: '\x1b<4mTR 2',
  240300036: '\x1b<4mTR ESS',
  290300001: '\x1b<4m G ELEC PMP A',
  290300002: '\x1b<4m G ELEC PMP B',
  290300003: '\x1b<4m Y ELEC PMP A',
  290300004: '\x1b<4m Y ELEC PMP B',
  290300005: '\x1b<4m G ENG 1 PMP A',
  290300006: '\x1b<4m G ENG 1 PMP B',
  290300007: '\x1b<4m G ENG 2 PMP A',
  290300008: '\x1b<4m G ENG 2 PMP B',
  290300009: '\x1b<4m Y ENG 3 PMP A',
  290300010: '\x1b<4m Y ENG 3 PMP B',
  290300011: '\x1b<4m Y ENG 4 PMP A',
  290300012: '\x1b<4m Y ENG 4 PMP B',
  290300013: '\x1b<4m G SYS CHAN A OVHT DET',
  290300014: '\x1b<4m G SYS CHAN B OVHT DET',
  290300015: '\x1b<4m Y SYS CHAN A OVHT DET',
  290300016: '\x1b<4m Y SYS CHAN B OVHT DET',
  290300017: '\x1b<4m G HSMU',
  290300018: '\x1b<4m Y HSMU',
  290300019: '\x1b<4m G SYS OVHT DET',
  290300020: '\x1b<4m Y SYS OVHT DET',
  290300021: '\x1b<4m G HYD SYS',
  290300022: '\x1b<4m Y HYD SYS',
  310300001: '\x1b<4mAUTO CALLOUT',
  320300001: '\x1b<4mA-SKID',
  320300002: '\x1b<4mAUTO BRK',
  320300003: '\x1b<4mPART A-SKID',
  320300004: '\x1b<4mBRK ACCU',
  320300005: '\x1b<4mALTN BRK',
  320300006: '\x1b<4mEMER BRK',
  320300007: '\x1b<4mBTV',
  320300008: '\x1b<4mNORM BRK',
  320300009: '\x1b<4mPARK BRK',
  320300010: '\x1b<4mPPEDAL BRAKING',
  320300011: '\x1b<4mL/G CTL 1+2',
  320300012: '\x1b<4mL/G DOORS',
  320300013: '\x1b<4mL/G RETRACTION',
  320300014: '\x1b<4mB/W STEER',
  320300015: '\x1b<4mCAPT STEER TILLER',
  320300016: '\x1b<4mFO STEER TILLER',
  320300017: '\x1b<4mN/W + B/W STEER',
  320300018: '\x1b<4mN/W STEER DISC',
  320300019: '\x1b<4mN/W STEER',
  320300020: '\x1b<4mNORM N/W STEER',
  320300021: '\x1b<4mPEDAL STEER CTL',
  320300022: '\x1b<4mROW/ROP',
  340300001: '\x1b<4mGPWS 1',
  340300002: '\x1b<4mGPWS 2',
  340300003: '\x1b<4mGPWS 1+2',
  340300004: '\x1b<4mADR 1',
  340300005: '\x1b<4mADR 2',
  340300006: '\x1b<4mADR 3',
  340300007: '\x1b<4mADR 1+2',
  340300008: '\x1b<4mADR 2+3',
  340300009: '\x1b<4mADR 1+3',
  340300010: '\x1b<4mADR 1+2+3',
  340300011: '\x1b<4mTCAS 1',
  340300012: '\x1b<4mTCAS 2',
  340300013: '\x1b<4mF/CTL PROT',
  340300014: '\x1b<4mLOAD ALLEVIATION',
  340300021: '\x1b<4mGUST LOAD PROT',
  340300022: '\x1b<4mRA SYS A',
  340300023: '\x1b<4mRA SYS B',
  340300024: '\x1b<4mRA SYS C',
  340300025: '\x1b<4mRA SYS A+B',
  340300026: '\x1b<4mRA SYS A+C',
  340300027: '\x1b<4mRA SYS B+C',
  340300028: '\x1b<4mRA SYS A+B+C',
  340300029: '\x1b<4mTCAS 1+2',
  340300030: '\x1b<4mIR 1',
  340300031: '\x1b<4mIR 2',
  340300032: '\x1b<4mIR 3',
  340300033: '\x1b<4mIR 1+2',
  340300034: '\x1b<4mIR 1+3',
  340300035: '\x1b<4mIR 2+3',
  340300036: '\x1b<4mIR 1+2+3',
  340300037: '\x1b<4mWXR 1',
  340300038: '\x1b<4mWXR 2',
  340300043: '\x1b<4mWXR 1+2',
  340300039: '\x1b<4mTERR SYS 1',
  340300040: '\x1b<4mTERR SYS 2',
  340300044: '\x1b<4mTERR SYS 1+2',
  340300041: '\x1b<4mADS-B RPTG 1',
  340300042: '\x1b<4mADS-B RPTG 2',
  340300045: '\x1b<4mADS-B RPTG 1+2',
  341300001: '\x1b<4mPRED W/S 1',
  341300002: '\x1b<4mPRED W/S 2',
  341300003: '\x1b<4mPRED W/S 1+2',
};

interface AbstractChecklistItem {
  /** The name of the item, displayed at the beginning of the line. Does not accept special formatting tokens. No leading dot. */
  name: string;
  /** sensed or not sensed item. Sensed items are automatically checked. */
  sensed: boolean;
  /** On which level of indentation to print the item. 0 equals the first level. Optional, not set means first level. */
  level?: number;
  /** Manually define color. standard (cyan when not completed, white/green when completed), or always cyan/green/amber. Standard, if not set. */
  color?: 'standard' | 'cyan' | 'green' | 'amber' | 'white_underlined' | 'separation_line';
}
interface ChecklistAction extends AbstractChecklistItem {
  /** Label at the end of the line if action is not completed. */
  labelNotCompleted: string;
  /** Label after "name" if action is completed. Optional, only fill if different from "labelNotCompleted". */
  labelCompleted?: string;
}

interface ChecklistCondition extends AbstractChecklistItem {}

export interface AbnormalProcedure {
  /** Title of the fault, e.g. "_HYD_ G SYS PRESS LO". \n produces second line. Accepts special formatting tokens  */
  title: string;
  /** sensed or not sensed abnormal procedure */
  sensed: boolean;
  /** An array of possible checklist items. */
  items: (ChecklistAction | ChecklistCondition)[];
  /** LAND ASAP or LAND ANSA displayed below title? Optional, don't fill if no recommendation */
  recommendation?: 'LAND ASAP' | 'LAND ANSA';
}

export interface NormalProcedure {
  /** Title of the checklist, e.g. "BEFORE START".  */
  title: string;
  /** An array of possible checklist items.. */
  items: ChecklistAction[];
}

export function isChecklistAction(element: ChecklistAction | ChecklistCondition): element is ChecklistAction {
  return 'labelNotCompleted' in element;
}

/** All normal procedures (checklists, via ECL) should be here. */
export const EcamNormalProcedures: { [n: number]: void } = {};

/** All abnormal sensed procedures (alerts, via ECL) should be here. */
export const EcamAbnormalSensedProcedures: { [n: number]: AbnormalProcedure } = {
  ...EcamAbnormalSensedAta212223,
  ...EcamAbnormalSensedAta24,
  ...EcamAbnormalSensedAta26,
  ...EcamAbnormalSensedAta27,
  ...EcamAbnormalSensedAta28,
  ...EcamAbnormalSensedAta2930,
  ...EcamAbnormalSensedAta313233,
  ...EcamAbnormalSensedAta34,
  ...EcamAbnormalSensedAta353642,
  ...EcamAbnormalSensedAta46495256,
  ...EcamAbnormalSensedAta70,
  ...EcamAbnormalSensedAta80Rest,
};

/** All abnormal non-sensed procedures (via ECL) should be here. Don't start for now, format needs to be defined. */
export const EcamAbnormalNonSensedProcedures: { [n: number]: AbnormalProcedure } = {};