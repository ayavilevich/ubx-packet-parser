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

export default {
  ver,
};
