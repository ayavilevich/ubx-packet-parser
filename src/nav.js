/* eslint-disable no-bitwise,default-case */
import { gnssIdentifiersInversed } from './ubx';

/**
 * @typedef {object} protocolMessage
 * @property {number} messageClass
 * @property {number} messageId
 * @property {Buffer} payload
 */

function bitToBool(byte, bit) {
  return ((byte >> bit) % 2 !== 0);
}

function iTowToDate(itow) {
  const gpsEpochSeconds = 315964800; // delta between unix epoch, January 1, 1970, UTC, (used by Date()) and gps baseline at January 6, 1980, UTC
  const weekSeconds = (60 * 60 * 24 * 7);
  const week = Math.floor(((new Date()).getTime() - (new Date('1980-01-06')).getTime()) / 1000 / 60 / 60 / 24 / 7); //  calculate current week number since gps baseline using local time
  // TODO: this is problematic, because local time might not be in sync with true time, so at week change points local we will have discontinuity.
  // either use NAV-PVT with time coming from GNSS alone or calc local Tow and try to match to next or prev local week based on match with itow.

  return new Date((gpsEpochSeconds + (weekSeconds * week) + (itow / 1000)) * 1000);
}

/**
 * @param {protocolMessage} packet
 */
function status(packet) {
  const gpsFix = {
    raw: packet.payload.readUInt8(4),
    string: '',
  };

  switch (gpsFix.raw) {
    case 0x00:
      gpsFix.string = 'no fix';
      break;
    case 0x01:
      gpsFix.string = 'dead reckoning only';
      break;
    case 0x02:
      gpsFix.string = '2D-fix';
      break;
    case 0x03:
      gpsFix.string = '3D-fix';
      break;
    case 0x04:
      gpsFix.string = 'GPS + dead reckoning combined';
      break;
    case 0x05:
      gpsFix.string = 'Time only fix';
      break;
    default:
      gpsFix.string = 'reserved';
      break;
  }

  const flags = {
    gpsFixOk: bitToBool(packet.payload.readUInt8(5), 0),
    diffSoln: bitToBool(packet.payload.readUInt8(5), 1),
    wknSet: bitToBool(packet.payload.readUInt8(5), 2),
    towSet: bitToBool(packet.payload.readUInt8(5), 3),
  };

  const flags2 = {
    psmState: {
      raw: 0,
      string: '',
    },
    spoofDetState: {
      raw: 0,
      string: '',
    },
  };

  switch (`${(packet.payload.readUInt8(7) >> 1) % 2}${(packet.payload.readUInt8(7) >> 0) % 2}`) {
    case '00':
      flags2.psmState.raw = 0;
      flags2.psmState.string = 'ACQUISITION';
      break;
    case '01':
      flags2.psmState.raw = 1;
      flags2.psmState.string = 'TRACKING';
      break;
    case '10':
      flags2.psmState.raw = 2;
      flags2.psmState.string = 'POWER OPTIMIZED TRACKING';
      break;
    case '11':
      flags2.psmState.raw = 3;
      flags2.psmState.string = 'INACTIVE';
      break;
  }

  switch (`${(packet.payload.readUInt8(7) >> 4) % 2}${(packet.payload.readUInt8(7) >> 3) % 2}`) {
    case '00':
      flags2.spoofDetState.raw = 0;
      flags2.spoofDetState.string = 'Unknown or deactivated';
      break;
    case '01':
      flags2.spoofDetState.raw = 1;
      flags2.spoofDetState.string = 'No spoofing indicated';
      break;
    case '10':
      flags2.spoofDetState.raw = 2;
      flags2.spoofDetState.string = 'Spoofing indicated';
      break;
    case '11':
      flags2.spoofDetState.raw = 3;
      flags2.spoofDetState.string = 'Multiple spoofing indications';
      break;
  }

  const fixStat = {
    diffCorr: bitToBool(packet.payload.readUInt8(6), 0),
    mapMatching: `${(packet.payload.readUInt8(6) >> 7) % 2}${(packet.payload.readUInt8(6) >> 6) % 2}`,
  };

  return {
    type: 'NAV-STATUS',
    iTOW: packet.payload.readUInt32LE(0), // GPS time of week of the navigation epoch. [ms]
    timeStamp: iTowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
      gpsFix,
      flags,
      fixStat,
      flags2,
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
    timeStamp: iTowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
      lon: (packet.payload.readInt32LE(4) * 1e-7), // [deg]
      lat: (packet.payload.readInt32LE(8) * 1e-7), // [deg]
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
    timeStamp: iTowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
      velN: packet.payload.readInt32LE(4), // [cm/s]
      velE: packet.payload.readInt32LE(8), // [cm/s]
      velD: packet.payload.readInt32LE(12), // [cm/s]
      speed: packet.payload.readUInt32LE(16), // [cm/s]
      gSpeed: packet.payload.readUInt32LE(20), // [cm/s]
      heading: (packet.payload.readInt32LE(24) * 1e-5), // [deg]
      sAcc: packet.payload.readUInt32LE(28), // [cm/s]
      cAcc: (packet.payload.readInt32LE(32) * 1e-5), // [deg]
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
      qualityInd: {
        raw: 0,
        string: `${(packet.payload.readUInt8(16 + (12 * i)) >> 2) % 2}${(packet.payload.readUInt8(16 + (12 * i)) >> 1) % 2}${(packet.payload.readUInt8(16 + (12 * i)) >> 0) % 2}`,
      },
      svUsed: bitToBool(packet.payload.readUInt8(16 + (12 * i)), 3), // Signal in the subset specified in Signal Identifiers is currently being used for navigation
      health: {
        raw: 0,
        string: `${(packet.payload.readUInt8(16 + (12 * i)) >> 5) % 2}${(packet.payload.readUInt8(16 + (12 * i)) >> 4) % 2}`,
      },
      diffCorr: bitToBool(packet.payload.readUInt8(16 + (12 * i)), 6), // differential correction data is available for this SV
      smoothed: bitToBool(packet.payload.readUInt8(16 + (12 * i)), 7), // carrier smoothed pseudorange used
      orbitSource: {
        raw: 0,
        string: `${(packet.payload.readUInt8(17 + (12 * i)) >> 2) % 2}${(packet.payload.readUInt8(17 + (12 * i)) >> 1) % 2}${(packet.payload.readUInt8(17 + (12 * i)) >> 0) % 2}`,
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

    switch (flags.qualityInd.string) {
      case '000':
        flags.qualityInd.raw = 0;
        flags.qualityInd.string = 'no signal';
        break;
      case '001':
        flags.qualityInd.raw = 1;
        flags.qualityInd.string = 'searching signal';
        break;
      case '010':
        flags.qualityInd.raw = 2;
        flags.qualityInd.string = 'signal acquired';
        break;
      case '011':
        flags.qualityInd.raw = 3;
        flags.qualityInd.string = 'signal detected but unusable';
        break;
      case '100':
        flags.qualityInd.raw = 4;
        flags.qualityInd.string = 'code locked and time synchronized';
        break;
      case '101':
        flags.qualityInd.raw = 5;
        flags.qualityInd.string = 'code and carrier locked and time synchronized';
        break;
      case '110':
        flags.qualityInd.raw = 6;
        flags.qualityInd.string = 'code and carrier locked and time synchronized';
        break;
      case '111':
        flags.qualityInd.raw = 7;
        flags.qualityInd.string = 'code and carrier locked and time synchronized';
        break;
    }

    switch (flags.health.string) {
      case '00':
        flags.health.raw = 0;
        flags.health.string = 'unknown';
        break;
      case '01':
        flags.health.raw = 1;
        flags.health.string = 'healthy';
        break;
      case '10':
        flags.health.raw = 2;
        flags.health.string = 'unhealthy';
        break;
    }

    switch (flags.orbitSource.string) {
      case '000':
        flags.orbitSource.raw = 0;
        flags.orbitSource.string = 'no orbit information is available for this SV';
        break;
      case '001':
        flags.orbitSource.raw = 1;
        flags.orbitSource.string = 'ephemeris is used';
        break;
      case '010':
        flags.orbitSource.raw = 2;
        flags.orbitSource.string = 'almanac is used';
        break;
      case '011':
        flags.orbitSource.raw = 3;
        flags.orbitSource.string = 'AssistNow Offline orbit is used';
        break;
      case '100':
        flags.orbitSource.raw = 4;
        flags.orbitSource.string = 'AssistNow Autonomous orbit is used';
        break;
      case '101':
        flags.orbitSource.raw = 5;
        flags.orbitSource.string = 'other orbit information is used';
        break;
      case '110':
        flags.orbitSource.raw = 6;
        flags.orbitSource.string = 'other orbit information is used';
        break;
      case '111':
        flags.orbitSource.raw = 7;
        flags.orbitSource.string = 'other orbit information is used';
        break;
    }

    sats.push({
      gnss: {
        raw: packet.payload.readUInt8(8 + (12 * i)), // GNSS identifier
        string: gnssIdentifiersInversed[packet.payload.readUInt8(8 + (12 * i))],
      },
      svId: packet.payload.readUInt8(9 + (12 * i)), // Satellite identifier
      cno: packet.payload.readUInt8(10 + (12 * i)), // Carrier to noise ratio (signal strength) [dBHz]
      elev: packet.payload.readInt8(11 + (12 * i)), // Elevation (range: +/-90), unknown if out of range [deg]
      azim: packet.payload.readInt16LE(12 + (12 * i)), // Azimuth (range 0-360), unknown if elevation is out of range [deg]
      prRes: (packet.payload.readInt16LE(14 + (12 * i)) * 0.1), // Pseudorange residual [m]
      flags,
    });
  }

  return {
    type: 'NAV-SAT',
    iTOW: packet.payload.readUInt32LE(0),
    timeStamp: iTowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
      version: packet.payload.readUInt8(4),
      numSvs: packet.payload.readUInt8(5),
      sats,
    },
  };
}

function pvt(packet) {
  const gpsFix = {
    raw: packet.payload.readUInt8(20),
    string: '',
  };

  switch (gpsFix.raw) {
    case 0x00:
      gpsFix.string = 'no fix';
      break;
    case 0x01:
      gpsFix.string = 'dead reckoning only';
      break;
    case 0x02:
      gpsFix.string = '2D-fix';
      break;
    case 0x03:
      gpsFix.string = '3D-fix';
      break;
    case 0x04:
      gpsFix.string = 'GNSS + dead reckoning combined';
      break;
    case 0x05:
      gpsFix.string = 'Time only fix';
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
    carrSoln: { // Carrier phase range solution status
      raw: 0,
      string: `${(packet.payload.readUInt8(21) >> 7) % 2}${(packet.payload.readUInt8(21) >> 6) % 2}`,
    },
  };

  switch (flags.carrSoln.string) {
    case '00':
      flags.carrSoln.raw = 0;
      flags.carrSoln.string = 'no carrier phase range solution';
      break;
    case '01':
      flags.carrSoln.raw = 1;
      flags.carrSoln.string = 'carrier phase range solution with floating ambiguities';
      break;
    case '10':
      flags.carrSoln.raw = 2;
      flags.carrSoln.string = 'carrier phase range solution with fixed ambiguities';
      break;
  }

  const valid = {
    validDate: bitToBool(packet.payload.readUInt8(11), 0), // valid UTC Date
    validTime: bitToBool(packet.payload.readUInt8(11), 1), // valid UTC Time of Day
    fullyResolved: bitToBool(packet.payload.readUInt8(11), 2), // UTC Time of Day has been fully resolved (no seconds uncertainty). Cannot be used to check if time is completely solved
    validMag: bitToBool(packet.payload.readUInt8(11), 3), // valid Magnetic declination
  };

  const flags2 = {
    confirmedAvai: bitToBool(packet.payload.readUInt8(22), 5), // information about UTC Date and Time of Day validity confirmation is available
    confirmedDate: bitToBool(packet.payload.readUInt8(22), 6), // UTC Date validity could be confirmed
    confirmedTime: bitToBool(packet.payload.readUInt8(22), 7), // UTC Time of Day could be confirmed
  };

  const flags3 = {
    invalidLlh: bitToBool(packet.payload.readUInt8(78), 0), //  Invalid lon, lat, height and hMSL
  };

  return {
    type: 'NAV-PVT',
    iTOW: packet.payload.readUInt32LE(0),
    timeStamp: iTowToDate(packet.payload.readUInt32LE(0)),
    data: {
      iTOW: packet.payload.readUInt32LE(0),
      year: packet.payload.readUInt16LE(4), // Year (UTC) [y]
      month: packet.payload.readUInt8(6), // Month, range 1..12 (UTC) [month]
      day: packet.payload.readUInt8(7), // Day of month, range 1..31 (UTC) [day]
      hour: packet.payload.readUInt8(8), // Hour of day, range 0..23 (UTC) [h]
      minute: packet.payload.readUInt8(9), // Minute of hour, range 0..59 (UTC) [min]
      second: packet.payload.readUInt8(10), // Seconds of minute, range 0..60 (UTC) [s]
      valid,
      tAcc: packet.payload.readUInt32LE(12), // Time accuracy estimate (UTC) [ns]
      nano: packet.payload.readInt32LE(16), // Fraction of second, range -1e9 .. 1e9 (UTC) [ns]
      fixType: gpsFix,
      flags,
      flags2,
      numSV: packet.payload.readUInt8(23), // Number of satellites used in Nav Solution
      lon: (packet.payload.readInt32LE(24) * 1e-7), // [deg]
      lat: (packet.payload.readInt32LE(28) * 1e-7), // [deg]
      height: packet.payload.readInt32LE(32), // [mm]
      hMSL: packet.payload.readInt32LE(36), // [mm]
      hAcc: packet.payload.readUInt32LE(40), // [mm]
      vAcc: packet.payload.readUInt32LE(44), // [mm]
      velN: packet.payload.readInt32LE(48), // [mm/s]
      velE: packet.payload.readInt32LE(52), // [mm/s]
      velD: packet.payload.readInt32LE(56), // [mm/s]
      gSpeed: packet.payload.readInt32LE(60), // [mm/s]
      headMot: (packet.payload.readInt32LE(64) * 1e-5), // [deg]
      sAcc: packet.payload.readUInt32LE(68), // [mm/s]
      headAcc: (packet.payload.readUInt32LE(72) * 1e-5), // [deg]
      pDOP: packet.payload.readUInt16LE(76), // Position DOP
      flags3,
      headVeh: (packet.payload.readInt32LE(84) * 1e-5), // Heading of vehicle (2-D) [deg]
      magDec: (packet.payload.readInt16LE(88) * 1e-2), // Magnetic declination [deg]
      magAcc: (packet.payload.readInt16LE(90) * 1e-2), // Magnetic declination accuracy[deg]
    },
  };
}

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
    timeStamp: iTowToDate(packet.payload.readUInt32LE(0)),
    data: {
      flags,
      iTOW: packet.payload.readUInt32LE(4),
      lon: (packet.payload.readInt32LE(8) * 1e-7 + packet.payload.readInt8(24) * 1e-9), // [deg]
      lat: (packet.payload.readInt32LE(12) * 1e-7 + packet.payload.readInt8(25) * 1e-9), // [deg]
      height: (packet.payload.readInt32LE(16) + packet.payload.readInt8(26) * 0.1), // [mm]
      hMSL: (packet.payload.readInt32LE(20) + packet.payload.readInt8(27) * 0.1), // [mm]
      hAcc: (packet.payload.readUInt32LE(28) * 0.1), // [mm]
      vAcc: (packet.payload.readUInt32LE(32) * 0.1), // [mm]
    },
  };
}

function relposned(packet) {
  const flags = {
    gnssFixOK: bitToBool(packet.payload.readUInt8(60), 0), // A valid fix (i.e within DOP & accuracy masks)
    diffSoln: bitToBool(packet.payload.readUInt8(60), 1), // 1 if differential corrections were applied
    relPosValid: bitToBool(packet.payload.readUInt8(60), 2), // 1 if relative position components and accuracies are valid and, in moving base mode only, if baseline is valid
    carrSoln: { // Carrier phase range solution status
      raw: 0,
      string: `${(packet.payload.readUInt8(60) >> 4) % 2}${(packet.payload.readUInt8(60) >> 3) % 2}`,
    },
    isMoving: bitToBool(packet.payload.readUInt8(60), 5), // 1 if the receiver is operating in moving base mode
    refPosMiss: bitToBool(packet.payload.readUInt8(60), 6), // 1 if extrapolated reference position was used to compute moving base solution this epoch
    refObsMiss: bitToBool(packet.payload.readUInt8(60), 7), // 1 if extrapolated reference observations were used to compute moving base solution this epoch
    relPosHeadingValid: bitToBool(packet.payload.readUInt8(60), 8), // 1 if relPosHeading is valid
    relPosNormalized: bitToBool(packet.payload.readUInt8(61), 0), // 1 if the components of the relative position vector (including the high-precision parts) are normalized
  };

  switch (flags.carrSoln.string) {
    case '00':
      flags.carrSoln.raw = 0;
      flags.carrSoln.string = 'no carrier phase range solution';
      break;
    case '01':
      flags.carrSoln.raw = 1;
      flags.carrSoln.string = 'carrier phase range solution with floating ambiguities';
      break;
    case '10':
      flags.carrSoln.raw = 2;
      flags.carrSoln.string = 'carrier phase range solution with fixed ambiguities';
      break;
  }

  return {
    type: 'NAV-RELPOSNED',
    iTOW: packet.payload.readUInt32LE(4),
    timeStamp: iTowToDate(packet.payload.readUInt32LE(0)),
    data: {
      refStationId: packet.payload.readUInt16LE(2), // Reference Station ID. Must be in the range 0..4095
      iTOW: packet.payload.readUInt32LE(4),
      relPosN: (packet.payload.readInt32LE(8) * 10 + packet.payload.readInt8(32) * 0.1), // [mm]
      relPosE: (packet.payload.readInt32LE(12) * 10 + packet.payload.readInt8(33) * 0.1), // [mm]
      relPosD: (packet.payload.readInt32LE(16) * 10 + packet.payload.readInt8(34) * 0.1), // [mm]
      relPosLength: (packet.payload.readInt32LE(20) * 10 + packet.payload.readInt8(35) * 0.1), // [mm]
      relPosHeading: (packet.payload.readInt32LE(24) * 1e-5), // [deg]
      accN: (packet.payload.readUInt32LE(36) * 0.1), // Accuracy of relative position North component [mm]
      accE: (packet.payload.readUInt32LE(40) * 0.1), // Accuracy of relative position East component [mm]
      accD: (packet.payload.readUInt32LE(44) * 0.1), // Accuracy of relative position Down component [mm]
      accLength: (packet.payload.readUInt32LE(48) * 0.1), // Accuracy of length of the relative position vector [mm]
      accHeading: (packet.payload.readUInt32LE(52) * 1e-5), // [deg]
      flags,
    },
  };
}

export default {
  status,
  posllh,
  velned,
  sat,
  pvt,
  hpposllh,
  relposned,
};
