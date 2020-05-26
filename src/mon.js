function trimEndNull(s) {
  // eslint-disable-next-line no-control-regex
  return s.replace(/[\u0000]+$/g, '');
}

// payload length is expected to be 30 + 10 + 30N
function ver(packet) {
  const extensions = [];
  for (let i = 40; i < packet.payload.length; i += 30) {
    extensions.push(trimEndNull(packet.payload.toString('ascii', i, i + 30)));
  }
  return {
    type: 'MON-VER',
    data: {
      swVersion: trimEndNull(packet.payload.toString('ascii', 0, 30)),
      hwVersion: trimEndNull(packet.payload.toString('ascii', 30, 40)),
      extensions,
    },
  };
}

// payload length is expected to be 4 + 24N
function rf(packet) {
  const blocks = [];
  const version = packet.payload.readInt8(0);
  const nBlocks = packet.payload.readInt8(1);
  // check length
  if (version === 0 && packet.payload.length === 4 + 24 * nBlocks) {
    // parse blocks
    for (let i = 0; i < nBlocks; i += 1) {
      const flags = packet.payload.readInt8(5 + 24 * i); // lower two bits is "jammingState" - output from Jamming/Interference Monitor
      // jammingState - (0 = unknown or feature disabled, 1 = ok - no significant jamming, 2 = warning - interference visible but fix OK, 3 = critical - interference visible and no fix)

      blocks.push({
        blockId: packet.payload.readInt8(4 + 24 * i),
        jammingState: flags,
        antStatus: packet.payload.readUInt8(6 + 24 * i), // Status of the antenna supervisor state machine (0x00=INIT,0x01=DONTKNOW, 0x02=OK,0x03=SHORT,0x04=OPEN)
        antPower: packet.payload.readUInt8(7 + 24 * i), // Current power status of antenna (0x00=OFF,0x01=ON,0x02=DONTKNOW)
        postStatus: packet.payload.readUInt32LE(8 + 24 * i), // POST status word
        noisePerMS: packet.payload.readUInt16LE(16 + 24 * i), // Noise level as measured by the GPS core
        agcCnt: packet.payload.readUInt16LE(18 + 24 * i), // AGC Monitor (counts SIGHI xor SIGLO, range 0 to 8191)
        jamInd: packet.payload.readUInt8(20 + 24 * i), // CW jamming indicator, scaled (0=no CW jamming, 255 = strong CW jamming)
        ofsI: packet.payload.readInt8(21 + 24 * i), // Imbalance of I-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance)
        magI: packet.payload.readUInt8(22 + 24 * i), // Magnitude of I-part of complex signal, scaled (0= no signal, 255 = max. magnitude)
        ofsQ: packet.payload.readInt8(23 + 24 * i), // Imbalance of Q-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance)
        magQ: packet.payload.readUInt8(24 + 24 * i), // Magnitude of Q-part of complex signal, scaled (0= no signal, 255 = max. magnitude)
      });
    }
  }
  return {
    type: 'MON-RF',
    version,
    nBlocks,
    blocks,
  };
}


export default {
  ver,
  rf,
};
