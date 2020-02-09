/* eslint no-bitwise: "off" */
import Debug from 'debug';
import { Transform } from 'stream';
import { packetTypes, packetTypesInversed } from './ubx';
import navFunctions from './nav';

const debug = Debug('ubx:packet:parser');

export default class UBXPacketParser extends Transform {
  constructor(options) {
    super({
      ...options,
      objectMode: true,
    });
  }

  /**
   * @typedef {object} protocolMessage
   * @property {number} messageClass
   * @property {number} messageId
   * @property {Buffer} payload
   */

  /**
   * @param {protocolMessage} chunk
   * @private
   */
  // eslint-disable-next-line no-underscore-dangle
  _transform(chunk, encoding, cb) {
    const packetType = `${chunk.messageClass}_${chunk.messageId}`;
    const packetTypeString = packetTypesInversed[packetType];

    let result;
    switch (packetType) {
      case packetTypes['NAV-STATUS']:
        result = navFunctions.status(chunk);
        break;
      case packetTypes['NAV-POSLLH']:
        result = navFunctions.posllh(chunk);
        break;

      case packetTypes['NAV-VELNED']:
        result = navFunctions.velned(chunk);
        break;

      case packetTypes['NAV-SAT']:
        result = navFunctions.sat(chunk);
        break;

      case packetTypes['NAV-PVT']:
        result = navFunctions.pvt(chunk);
        break;

      case packetTypes['NAV-HPPOSLLH']:
        result = navFunctions.hpposllh(chunk);
        break;

      case packetTypes['NAV-RELPOSNED']:
        result = navFunctions.relposned(chunk);
        break;

      default:
        debug(`Unknown packet type: "${packetTypeString}" "${packetType}"`);
        // emit an event about the unhandled packet
        this.emit('unknown_packet', { packet: chunk, packetType, packetTypeString });
        cb();

        return;
    }

    this.push(result);

    cb();
  }

  static getPacketName(packetClass, packetId) {
    const packetType = `${packetClass}_${packetId}`;
    const packetTypeString = packetTypesInversed[packetType];
    return packetTypeString || 'UNKNOWN';
  }
}
