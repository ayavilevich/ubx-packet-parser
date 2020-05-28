/* eslint-disable no-bitwise,default-case */
import { gnssIdentifiersInversed, gnssSignalIdentifiersInversed } from './ubx';

/**
 * @typedef {object} protocolMessage
 * @property {number} messageClass
 * @property {number} messageId
 * @property {Buffer} payload
 */

const weekSeconds = (60 * 60 * 24 * 7);

function bitToBool(byte, bit) {
  return ((byte >> bit) % 2 !== 0);
}

function itowToDate(itow) {
  const gpsEpochSeconds = 315964800; // delta between unix epoch, January 1, 1970, UTC, (used by Date()) and gps baseline at January 6, 1980, UTC
  const week = Math.floor(((new Date()).getTime() - (new Date('1980-01-06')).getTime()) / 1000 / 60 / 60 / 24 / 7); //  calculate current week number since gps baseline using local time
  // TODO: this is problematic, because local time (where parsing happens) might not be in sync with true time, so at week change points local we will have discontinuity.
  // either use NAV-PVT with UTC time coming from GNSS alone or calc local Tow and week and try to match to next or prev local week based on match with receiver itow (i.e. which end it is close to).
  // additionally, if doing the calculation ourselves, there will be inaccuracies due to leap seconds. See Integration Manual section "3.7 Clocks and time".
  // best solution for relative deltas is to subtract itow while taking care of overflow.
  // best solution for UTC absolute values is to wait for receiver to properly sync with time. Use flags and fields in NAV-PVT.

  return new Date((gpsEpochSeconds + (weekSeconds * week) + (itow / 1000)) * 1000);
}

// returns difference (in ms) between two itow values
// handles overflow
// can't correctly diff two values more than one week apart
function itowDiff(itowStart, itowEnd) {
  if (itowEnd < itowStart) { // if overflow
    return itowEnd + weekSeconds * 1000 - itowStart;
  }
  return itowEnd - itowStart; // simple case
}

/**
 * @param {protocolMessage} packet
 */
function status(packet) {
  const gpsFixRaw = {
    value: packet.payload.readUInt8(4),
    string: '',
  };

  let gpsFix = '';
  switch (gpsFixRaw.value) {
    case 0x00:
      gpsFixRaw.string = 'no fix';
      gpsFix = 'no-fix';
      break;
    case 0x01:
      gpsFixRaw.string = 'dead reckoning only';
      gpsFix = 'dead-reckoning';
      break;
    case 0x02:
      gpsFixRaw.string = '2D-fix';
      gpsFix = '2d-fix';
      break;
    case 0x03:
      gpsFixRaw.string = '3D-fix';
      gpsFix = '3d-fix';
      break;
    case 0x04:
      gpsFixRaw.string = 'GPS + dead reckoning combined';
      gpsFix = 'gps+dead-reckoning';
      break;
    case 0x05:
      gpsFixRaw.string = 'Time only fix';
      gpsFix = 'time-only';
      break;
    default:
      gpsFixRaw.string = 'reserved';
      gpsFix = 'reserved';
      break;
  }

  const flags = {
    gpsFixOk: bitToBool(packet.payload.readUInt8(5), 0),
    diffSoln: bitToBool(packet.payload.readUInt8(5), 1),
    wknSet: bitToBool(packet.payload.readUInt8(5), 2),
    towSet: bitToBool(packet.payload.readUInt8(5), 3),
    psmStateRaw: {},
    spoofDetStateRaw: {},
  };

  // Power Save Mode state
  flags.psmStateRaw.bits = `${(packet.payload.readUInt8(7) >> 1) % 2}${(packet.payload.readUInt8(7) >> 0) % 2}`;
  switch (flags.psmStateRaw.bits) {
    case '00':
      flags.psmState = 'acquisition';
      flags.psmStateRaw.string = 'ACQUISITION';
      break;
    case '01':
      flags.psmState = 'tracking';
      flags.psmStateRaw.string = 'TRACKING';
      break;
    case '10':
      flags.psmState = 'power-optimized-tracking';
      flags.psmStateRaw.string = 'POWER OPTIMIZED TRACKING';
      break;
    case '11':
      flags.psmState = 'inactive';
      flags.psmStateRaw.string = 'INACTIVE';
      break;
  }

  // Spoofing detection state
  flags.spoofDetStateRaw.bits = `${(packet.payload.readUInt8(7) >> 4) % 2}${(packet.payload.readUInt8(7) >> 3) % 2}`;
  switch (flags.spoofDetStateRaw.bits) {
    case '00':
      flags.spoofDetState = 'unknown';
      flags.spoofDetStateRaw.string = 'Unknown or deactivated';
      break;
    case '01':
      flags.spoofDetState = 'no-spoofing';
      flags.spoofDetStateRaw.string = 'No spoofing indicated';
      break;
    case '10':
      flags.spoofDetState = 'spoofing';
      flags.spoofDetStateRaw.string = 'Spoofing indicated';
      break;
    case '11':
      flags.spoofDetState = 'multiple-spoofing';
      flags.spoofDetStateRaw.string = 'Multiple spoofing indications';
      break;
  }

  const fixStat = {
    diffCorr: bitToBool(packet.payload.readUInt8(6), 0),
    // mapMatching: `${(packet.payload.readUInt8(6) >> 7) % 2}${(packet.payload.readUInt8(6) >> 6) % 2}`, // add parsing of bit options if going to expose this field
  };

  return {
    type: 'NAV-STATUS',
    iTOW: packet.payload.readUInt32LE(0), // GPS time of week of the navigation epoch. [ms]
    timeStamp: itowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
      gpsFix,
      gpsFixRaw,
      flags,
      fixStat,
      ttff: packet.payload.readUInt32LE(8), // Time to first fix (millisecond time tag) [ms]
      msss: packet.payload.readUInt32LE(12), // Milliseconds since Startup / Reset [ms]
    },
  };
}

/**
 * @param {protocolMessage} packet
 */
function posllh(packet) {
  return {
    type: 'NAV-POSLLH',
    iTOW: packet.payload.readUInt32LE(0),
    timeStamp: itowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
      lon: (packet.payload.readInt32LE(4) / 1e7), // [deg]
      lat: (packet.payload.readInt32LE(8) / 1e7), // [deg]
      height: packet.payload.readInt32LE(12), // [mm]
      hMSL: packet.payload.readInt32LE(16), // [mm]
      hAcc: packet.payload.readUInt32LE(20), // [mm]
      vAcc: packet.payload.readUInt32LE(24), // [mm]
    },
  };
}

/**
 * @param {protocolMessage} packet
 */
function velned(packet) {
  return {
    type: 'NAV-VELNED',
    iTOW: packet.payload.readUInt32LE(0),
    timeStamp: itowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
      velN: packet.payload.readInt32LE(4), // [cm/s]
      velE: packet.payload.readInt32LE(8), // [cm/s]
      velD: packet.payload.readInt32LE(12), // [cm/s]
      speed: packet.payload.readUInt32LE(16), // [cm/s]
      gSpeed: packet.payload.readUInt32LE(20), // [cm/s]
      heading: (packet.payload.readInt32LE(24) / 1e5), // [deg]
      sAcc: packet.payload.readUInt32LE(28), // [cm/s]
      cAcc: (packet.payload.readInt32LE(32) / 1e5), // [deg]
    },
  };
}

/**
 * @param {protocolMessage} packet
 */
function sat(packet) {
  const satCount = packet.payload.readUInt8(5); // Number of satellites
  const sats = [];

  for (let i = 0; i < satCount; i += 1) {
    const flags = {
      qualityIndRaw: { // Signal quality indicator:
        bits: `${(packet.payload.readUInt8(16 + (12 * i)) >> 2) % 2}${(packet.payload.readUInt8(16 + (12 * i)) >> 1) % 2}${(packet.payload.readUInt8(16 + (12 * i)) >> 0) % 2}`,
      },
      svUsed: bitToBool(packet.payload.readUInt8(16 + (12 * i)), 3), // Signal in the subset specified in Signal Identifiers is currently being used for navigation
      healthRaw: { // Signal health flag
        bits: `${(packet.payload.readUInt8(16 + (12 * i)) >> 5) % 2}${(packet.payload.readUInt8(16 + (12 * i)) >> 4) % 2}`,
      },
      diffCorr: bitToBool(packet.payload.readUInt8(16 + (12 * i)), 6), // differential correction data is available for this SV
      smoothed: bitToBool(packet.payload.readUInt8(16 + (12 * i)), 7), // carrier smoothed pseudorange used
      orbitSourceRaw: { // Orbit source
        bits: `${(packet.payload.readUInt8(17 + (12 * i)) >> 2) % 2}${(packet.payload.readUInt8(17 + (12 * i)) >> 1) % 2}${(packet.payload.readUInt8(17 + (12 * i)) >> 0) % 2}`,
      },
      ephAvail: bitToBool(packet.payload.readUInt8(17 + (12 * i)), 3), // ephemeris is available for this SV
      almAvail: bitToBool(packet.payload.readUInt8(17 + (12 * i)), 4), // almanac is available for this SV
      anoAvail: bitToBool(packet.payload.readUInt8(17 + (12 * i)), 5), // AssistNow Offline data is available for this SV
      aopAvail: bitToBool(packet.payload.readUInt8(17 + (12 * i)), 6), // AssistNow Autonomous data is available for this SV
      sbasCorrUsed: bitToBool(packet.payload.readUInt8(18 + (12 * i)), 0), // SBAS corrections have been used for a signal in the subset specified in Signal Identifier
      rtcmCorrUsed: bitToBool(packet.payload.readUInt8(18 + (12 * i)), 1), // RTCM corrections have been used for a signal...
      slasCorrUsed: bitToBool(packet.payload.readUInt8(18 + (12 * i)), 2), //  QZSS SLAS corrections have been used for a signal...
      prCorrUsed: bitToBool(packet.payload.readUInt8(18 + (12 * i)), 4), // Pseudorange corrections have been used for a signal...
      crCorrUsed: bitToBool(packet.payload.readUInt8(18 + (12 * i)), 5), // Carrier range corrections have been used for a signal...
      doCorrUsed: bitToBool(packet.payload.readUInt8(18 + (12 * i)), 6), // Range rate (Doppler) corrections have been used for a signal...
    };

    switch (flags.qualityIndRaw.bits) {
      case '000':
        flags.qualityIndRaw.raw = 0;
        flags.qualityIndRaw.string = 'no signal';
        flags.qualityInd = 'no-signal';
        break;
      case '001':
        flags.qualityIndRaw.raw = 1;
        flags.qualityIndRaw.string = 'searching signal';
        flags.qualityInd = 'searching-signal';
        break;
      case '010':
        flags.qualityIndRaw.raw = 2;
        flags.qualityIndRaw.string = 'signal acquired';
        flags.qualityInd = 'signal-acquired';
        break;
      case '011':
        flags.qualityIndRaw.raw = 3;
        flags.qualityIndRaw.string = 'signal detected but unusable';
        flags.qualityInd = 'signal-detected';
        break;
      case '100':
        flags.qualityIndRaw.raw = 4;
        flags.qualityIndRaw.string = 'code locked and time synchronized';
        flags.qualityInd = 'code-locked';
        break;
      case '101':
        flags.qualityIndRaw.raw = 5;
        flags.qualityIndRaw.string = 'code and carrier locked and time synchronized';
        flags.qualityInd = 'code-carrier-locked';
        break;
      case '110':
        flags.qualityIndRaw.raw = 6;
        flags.qualityIndRaw.string = 'code and carrier locked and time synchronized';
        flags.qualityInd = 'code-carrier-locked';
        break;
      case '111':
        flags.qualityIndRaw.raw = 7;
        flags.qualityIndRaw.string = 'code and carrier locked and time synchronized';
        flags.qualityInd = 'code-carrier-locked';
        break;
    }

    switch (flags.healthRaw.bits) {
      case '00':
        flags.healthRaw.raw = 0;
        flags.healthRaw.string = 'unknown';
        break;
      case '01':
        flags.healthRaw.raw = 1;
        flags.healthRaw.string = 'healthy';
        break;
      case '10':
        flags.healthRaw.raw = 2;
        flags.healthRaw.string = 'unhealthy';
        break;
    }
    flags.health = flags.healthRaw.string; // all values fit convention

    switch (flags.orbitSourceRaw.bits) {
      case '000':
        flags.orbitSourceRaw.raw = 0;
        flags.orbitSourceRaw.string = 'no orbit information is available for this SV';
        flags.orbitSource = 'no-info';
        break;
      case '001':
        flags.orbitSourceRaw.raw = 1;
        flags.orbitSourceRaw.string = 'ephemeris is used';
        flags.orbitSource = 'ephemeris';
        break;
      case '010':
        flags.orbitSourceRaw.raw = 2;
        flags.orbitSourceRaw.string = 'almanac is used';
        flags.orbitSource = 'almanac';
        break;
      case '011':
        flags.orbitSourceRaw.raw = 3;
        flags.orbitSourceRaw.string = 'AssistNow Offline orbit is used';
        flags.orbitSource = 'assistnow-offline';
        break;
      case '100':
        flags.orbitSourceRaw.raw = 4;
        flags.orbitSourceRaw.string = 'AssistNow Autonomous orbit is used';
        flags.orbitSource = 'assistnow-autonomous';
        break;
      case '101':
        flags.orbitSourceRaw.raw = 5;
        flags.orbitSourceRaw.string = 'other orbit information is used';
        flags.orbitSource = 'other';
        break;
      case '110':
        flags.orbitSourceRaw.raw = 6;
        flags.orbitSourceRaw.string = 'other orbit information is used';
        flags.orbitSource = 'other';
        break;
      case '111':
        flags.orbitSourceRaw.raw = 7;
        flags.orbitSourceRaw.string = 'other orbit information is used';
        flags.orbitSource = 'other';
        break;
    }

    const gnssId = packet.payload.readUInt8(8 + (12 * i));
    sats.push({
      gnssRaw: {
        raw: gnssId, // GNSS identifier
        string: gnssIdentifiersInversed[gnssId],
      },
      gnss: gnssIdentifiersInversed[gnssId],
      svId: packet.payload.readUInt8(9 + (12 * i)), // Satellite identifier
      cno: packet.payload.readUInt8(10 + (12 * i)), // Carrier to noise ratio (signal strength) [dBHz]
      elev: packet.payload.readInt8(11 + (12 * i)), // Elevation (range: +/-90), unknown if out of range [deg]
      azim: packet.payload.readInt16LE(12 + (12 * i)), // Azimuth (range 0-360), unknown if elevation is out of range [deg]
      prRes: (packet.payload.readInt16LE(14 + (12 * i)) / 1e1), // Pseudorange residual [m]
      flags,
    });
  }

  return {
    type: 'NAV-SAT',
    iTOW: packet.payload.readUInt32LE(0),
    timeStamp: itowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
      version: packet.payload.readUInt8(4),
      numSvs: packet.payload.readUInt8(5),
      sats,
    },
  };
}

function sig(packet) {
  const sigCount = packet.payload.readUInt8(5); // Number of satellites
  const sigs = [];

  for (let i = 0; i < sigCount; i += 1) {
    const flagsLowByte = packet.payload.readUInt8(18 + (16 * i));
    const flags = {
      healthRaw: { // Signal health flag
        bits: `${(flagsLowByte >> 1) % 2}${(flagsLowByte >> 0) % 2}`,
      },
      prSmoothed: bitToBool(flagsLowByte, 2), // Pseudorange has been smoothed
      prUsed: bitToBool(flagsLowByte, 3), // Pseudorange has been used for a signal...
      crUsed: bitToBool(flagsLowByte, 4), // Carrier range has been used for a signal...
      doUsed: bitToBool(flagsLowByte, 5), // Range rate (Doppler) has been used for a signal...
      prCorrUsed: bitToBool(flagsLowByte, 6), // Pseudorange corrections have been used for a signal...
      crCorrUsed: bitToBool(flagsLowByte, 7), // Carrier range corrections have been used for a signal...
      doCorrUsed: bitToBool(packet.payload.readUInt8(19 + (16 * i)), 0), // Range rate (Doppler) corrections have been used for a signal...
    };

    switch (flags.healthRaw.bits) {
      case '00':
        flags.healthRaw.raw = 0;
        flags.healthRaw.string = 'unknown';
        break;
      case '01':
        flags.healthRaw.raw = 1;
        flags.healthRaw.string = 'healthy';
        break;
      case '10':
        flags.healthRaw.raw = 2;
        flags.healthRaw.string = 'unhealthy';
        break;
    }
    flags.health = flags.healthRaw.string; // all values fit convention

    // enums that are not a flag
    const qualityIndRaw = { raw: packet.payload.readUInt8(15 + (16 * i)) }; // Signal quality indicator.
    let qualityInd = false;

    switch (qualityIndRaw.raw) {
      case 0:
        qualityIndRaw.string = 'no signal';
        qualityInd = 'no-signal';
        break;
      case 1:
        qualityIndRaw.string = 'searching signal';
        qualityInd = 'searching-signal';
        break;
      case 2:
        qualityIndRaw.string = 'signal acquired';
        qualityInd = 'signal-acquired';
        break;
      case 3:
        qualityIndRaw.string = 'signal detected but unusable';
        qualityInd = 'signal-detected';
        break;
      case 4:
        qualityIndRaw.string = 'code locked and time synchronized';
        qualityInd = 'code-locked';
        break;
      case 5:
      case 6:
      case 7:
        qualityIndRaw.string = 'code and carrier locked and time synchronized';
        qualityInd = 'code-carrier-locked';
        break;
    }

    const ionoModelRaw = { raw: packet.payload.readUInt8(17 + (16 * i)) }; // Ionospheric model used
    let ionoModel = false;

    switch (ionoModelRaw.raw) {
      case 0:
        ionoModelRaw.string = 'No model';
        ionoModel = 'no-model';
        break;
      case 1:
        ionoModelRaw.string = 'Klobuchar model transmitted by GPS';
        ionoModel = 'klobuchar-gps';
        break;
      case 2:
        ionoModelRaw.string = 'SBAS model';
        ionoModel = 'sbas';
        break;
      case 3:
        ionoModelRaw.string = 'Klobuchar model transmitted by BeiDou';
        ionoModel = 'klobuchar-beidou';
        break;
      case 8:
        ionoModelRaw.string = 'Iono delay derived from dual frequency observations';
        ionoModel = 'dual-frequency';
        break;
    }

    const gnssId = packet.payload.readUInt8(8 + (16 * i)); // GNSS identifier
    const sigId = packet.payload.readUInt8(10 + (16 * i)); // New style signal identifier
    sigs.push({
      gnssId, // GNSS identifier
      gnss: gnssIdentifiersInversed[gnssId],
      svId: packet.payload.readUInt8(9 + (16 * i)), // Satellite identifier
      sigId, // New style signal identifier
      sig: gnssSignalIdentifiersInversed[gnssId] ? gnssSignalIdentifiersInversed[gnssId][sigId] : false,
      freqId: packet.payload.readUInt8(11 + (16 * i)), // Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)
      prRes: (packet.payload.readInt16LE(12 + (16 * i)) / 1e1), // Pseudorange residual [m]
      cno: packet.payload.readUInt8(14 + (16 * i)), // Carrier to noise ratio (signal strength) [dBHz]
      qualityInd, // Signal quality indicator.
      qualityIndRaw,
      ionoModel, // Ionospheric model used
      ionoModelRaw,
      flags,
    });
  }

  return {
    type: 'NAV-SIG',
    iTOW: packet.payload.readUInt32LE(0),
    timeStamp: itowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
      version: packet.payload.readUInt8(4),
      numSigs: sigCount,
      sigs,
    },
  };
}

/*
This message combines position, velocity and time solution, including accuracy
figures.
Note that during a leap second there may be more or less than 60 seconds in a
minute. See the section Leap seconds in Integration manual for details.
*/
function pvt(packet) {
  const gpsFixRaw = {
    value: packet.payload.readUInt8(20),
    string: '',
  };

  let gpsFix = '';
  switch (gpsFixRaw.value) {
    case 0x00:
      gpsFixRaw.string = 'no fix';
      gpsFix = 'no-fix';
      break;
    case 0x01:
      gpsFixRaw.string = 'dead reckoning only';
      gpsFix = 'dead-reckoning';
      break;
    case 0x02:
      gpsFixRaw.string = '2D-fix';
      gpsFix = '2d-fix';
      break;
    case 0x03:
      gpsFixRaw.string = '3D-fix';
      gpsFix = '3d-fix';
      break;
    case 0x04:
      gpsFixRaw.string = 'GPS + dead reckoning combined';
      gpsFix = 'gps+dead-reckoning';
      break;
    case 0x05:
      gpsFixRaw.string = 'Time only fix';
      gpsFix = 'time-only';
      break;
    default:
      gpsFixRaw.string = 'reserved';
      gpsFix = 'reserved';
      break;
  }

  const flags = {
    gnssFixOk: bitToBool(packet.payload.readUInt8(21), 0), // valid fix (i.e within DOP & accuracy masks)
    diffSoln: bitToBool(packet.payload.readUInt8(21), 1), // differential corrections were applied
    psmState: { // Power Save Mode state
      raw: 0,
      string: `${(packet.payload.readUInt8(21) >> 4) % 2}${(packet.payload.readUInt8(21) >> 3) % 2}${(packet.payload.readUInt8(21) >> 2) % 2}`,
    },
    headVehValid: bitToBool(packet.payload.readUInt8(21), 5), // heading of vehicle is valid
    carrSolnRaw: { // Carrier phase range solution status
      bits: `${(packet.payload.readUInt8(21) >> 7) % 2}${(packet.payload.readUInt8(21) >> 6) % 2}`,
    },
    // flags2
    confirmedAvai: bitToBool(packet.payload.readUInt8(22), 5), // information about UTC Date and Time of Day validity confirmation is available
    confirmedDate: bitToBool(packet.payload.readUInt8(22), 6), // UTC Date validity could be confirmed
    confirmedTime: bitToBool(packet.payload.readUInt8(22), 7), // UTC Time of Day could be confirmed
    // flags3
    invalidLlh: bitToBool(packet.payload.readUInt8(78), 0), //  Invalid lon, lat, height and hMSL
  };

  switch (flags.carrSolnRaw.bits) {
    case '00':
      flags.carrSoln = 'none';
      flags.carrSolnRaw.string = 'no carrier phase range solution';
      break;
    case '01':
      flags.carrSoln = 'float';
      flags.carrSolnRaw.string = 'carrier phase range solution with floating ambiguities';
      break;
    case '10':
      flags.carrSoln = 'fix';
      flags.carrSolnRaw.string = 'carrier phase range solution with fixed ambiguities';
      break;
  }

  const valid = {
    validDate: bitToBool(packet.payload.readUInt8(11), 0), // valid UTC Date
    validTime: bitToBool(packet.payload.readUInt8(11), 1), // valid UTC Time of Day
    fullyResolved: bitToBool(packet.payload.readUInt8(11), 2), // UTC Time of Day has been fully resolved (no seconds uncertainty). Cannot be used to check if time is completely solved
    validMag: bitToBool(packet.payload.readUInt8(11), 3), // valid Magnetic declination
  };

  return {
    type: 'NAV-PVT',
    iTOW: packet.payload.readUInt32LE(0),
    timeStamp: itowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
      year: packet.payload.readUInt16LE(4), // Year (UTC) [y]
      month: packet.payload.readUInt8(6), // Month, range 1..12 (UTC) [month]
      day: packet.payload.readUInt8(7), // Day of month, range 1..31 (UTC) [day]
      hour: packet.payload.readUInt8(8), // Hour of day, range 0..23 (UTC) [h]
      minute: packet.payload.readUInt8(9), // Minute of hour, range 0..59 (UTC) [min]
      second: packet.payload.readUInt8(10), // Seconds of minute, range 0..60 (UTC) [s]
      valid, // Validity flags
      tAcc: packet.payload.readUInt32LE(12), // Time accuracy estimate (UTC) [ns]
      nano: packet.payload.readInt32LE(16), // Fraction of second, range -1e9 .. 1e9 (UTC) [ns]
      fixType: gpsFix,
      fixTypeRaw: gpsFixRaw,
      flags,
      numSV: packet.payload.readUInt8(23), // Number of satellites used in Nav Solution
      lon: (packet.payload.readInt32LE(24) / 1e7), // [deg]
      lat: (packet.payload.readInt32LE(28) / 1e7), // [deg]
      height: packet.payload.readInt32LE(32), // Height above ellipsoid [mm]
      hMSL: packet.payload.readInt32LE(36), // Height above mean sea level [mm]
      hAcc: packet.payload.readUInt32LE(40), // [mm]
      vAcc: packet.payload.readUInt32LE(44), // [mm]
      velN: packet.payload.readInt32LE(48), // [mm/s]
      velE: packet.payload.readInt32LE(52), // [mm/s]
      velD: packet.payload.readInt32LE(56), // [mm/s]
      gSpeed: packet.payload.readInt32LE(60), // Ground Speed (2-D) [mm/s]
      headMot: (packet.payload.readInt32LE(64) / 1e5), // Heading of motion (2-D) [deg]
      sAcc: packet.payload.readUInt32LE(68), // Speed accuracy estimate [mm/s]
      headAcc: (packet.payload.readUInt32LE(72) / 1e5), // Heading accuracy estimate (both motion and vehicle) [deg]
      pDOP: packet.payload.readUInt16LE(76), // Position DOP
      headVeh: (packet.payload.readInt32LE(84) / 1e5), // Heading of vehicle (2-D) [deg]
      magDec: (packet.payload.readInt16LE(88) / 1e2), // Magnetic declination [deg]
      magAcc: (packet.payload.readInt16LE(90) / 1e2), // Magnetic declination accuracy[deg]
    },
  };
}

/*
This message outputs the Geodetic position in the currently selected ellipsoid.
The default is the WGS84 Ellipsoid, but can be changed with the message
CFG-NAVSPG-USE_USRDAT.
*/
/**
 * @param {protocolMessage} packet
 */
function hpposllh(packet) {
  const flags = {
    invalidLlh: bitToBool(packet.payload.readUInt8(3), 0), //  Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp
  };

  return {
    type: 'NAV-HPPOSLLH',
    iTOW: packet.payload.readUInt32LE(4),
    timeStamp: itowToDate(packet.payload.readUInt32LE(4)),
    data: {
      flags,
      iTOW: packet.payload.readUInt32LE(4),
      lon: ((packet.payload.readInt32LE(8) * 1e2 + packet.payload.readInt8(24)) / 1e9), // [deg]
      lat: ((packet.payload.readInt32LE(12) * 1e2 + packet.payload.readInt8(25)) / 1e9), // [deg]
      height: (packet.payload.readInt32LE(16) + packet.payload.readInt8(26) / 1e1), // [mm]
      hMSL: (packet.payload.readInt32LE(20) + packet.payload.readInt8(27) / 1e1), // [mm]
      hAcc: (packet.payload.readUInt32LE(28) / 1e1), // [mm]
      vAcc: (packet.payload.readUInt32LE(32) / 1e1), // [mm]
    },
  };
}

/*
The NED frame is defined as the local topological system at the reference
station. The relative position vector components in this message, along with
their associated accuracies, are given in that local topological system
This message contains the relative position vector from the Reference Station
to the Rover, including accuracy figures, in the local topological system defined
at the reference station
*/
function relposned(packet) {
  const flags = {
    gnssFixOK: bitToBool(packet.payload.readUInt8(60), 0), // A valid fix (i.e within DOP & accuracy masks)
    diffSoln: bitToBool(packet.payload.readUInt8(60), 1), // 1 if differential corrections were applied
    relPosValid: bitToBool(packet.payload.readUInt8(60), 2), // 1 if relative position components and accuracies are valid and, in moving base mode only, if baseline is valid
    carrSolnRaw: { // Carrier phase range solution status
      bits: `${(packet.payload.readUInt8(60) >> 4) % 2}${(packet.payload.readUInt8(60) >> 3) % 2}`,
    },
    isMoving: bitToBool(packet.payload.readUInt8(60), 5), // 1 if the receiver is operating in moving base mode
    refPosMiss: bitToBool(packet.payload.readUInt8(60), 6), // 1 if extrapolated reference position was used to compute moving base solution this epoch
    refObsMiss: bitToBool(packet.payload.readUInt8(60), 7), // 1 if extrapolated reference observations were used to compute moving base solution this epoch
    relPosHeadingValid: bitToBool(packet.payload.readUInt8(61), 0), // 1 if relPosHeading is valid
    relPosNormalized: bitToBool(packet.payload.readUInt8(61), 1), // 1 if the components of the relative position vector (including the high-precision parts) are normalized
  };

  switch (flags.carrSolnRaw.bits) {
    case '00':
      flags.carrSoln = 'none';
      flags.carrSolnRaw.string = 'no carrier phase range solution';
      break;
    case '01':
      flags.carrSoln = 'float';
      flags.carrSolnRaw.string = 'carrier phase range solution with floating ambiguities';
      break;
    case '10':
      flags.carrSoln = 'fix';
      flags.carrSolnRaw.string = 'carrier phase range solution with fixed ambiguities';
      break;
  }

  return {
    type: 'NAV-RELPOSNED',
    iTOW: packet.payload.readUInt32LE(4),
    timeStamp: itowToDate(packet.payload.readUInt32LE(4)),
    data: {
      refStationId: packet.payload.readUInt16LE(2), // Reference Station ID. Must be in the range 0..4095
      iTOW: packet.payload.readUInt32LE(4),
      relPosN: (packet.payload.readInt32LE(8) * 10 + packet.payload.readInt8(32) / 1e1), // [mm]
      relPosE: (packet.payload.readInt32LE(12) * 10 + packet.payload.readInt8(33) / 1e1), // [mm]
      relPosD: (packet.payload.readInt32LE(16) * 10 + packet.payload.readInt8(34) / 1e1), // [mm]
      relPosLength: (packet.payload.readInt32LE(20) * 10 + packet.payload.readInt8(35) / 1e1), // [mm]
      relPosHeading: (packet.payload.readInt32LE(24) / 1e5), // [deg]
      accN: (packet.payload.readUInt32LE(36) / 1e1), // Accuracy of relative position North component [mm]
      accE: (packet.payload.readUInt32LE(40) / 1e1), // Accuracy of relative position East component [mm]
      accD: (packet.payload.readUInt32LE(44) / 1e1), // Accuracy of relative position Down component [mm]
      accLength: (packet.payload.readUInt32LE(48) / 1e1), // Accuracy of length of the relative position vector [mm]
      accHeading: (packet.payload.readUInt32LE(52) / 1e5), // [deg]
      flags,
    },
  };
}

/*
This message is intended to be used as a marker to collect all navigation
messages of an epoch. It is output after all enabled NAV class messages (except
UBX-NAV-HNR) and after all enabled NMEA messages.
*/
/**
 * @param {protocolMessage} packet
 */
function eoe(packet) {
  return {
    type: 'NAV-EOE',
    iTOW: packet.payload.readUInt32LE(0),
    timeStamp: itowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
    },
  };
}

export default {
  status,
  posllh,
  velned,
  sat,
  sig,
  pvt,
  hpposllh,
  relposned,
  eoe,
  itowDiff,
};
