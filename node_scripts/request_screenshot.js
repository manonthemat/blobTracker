var net = require('net');
var client = new net.Socket();

client.connect(8888, '127.0.0.1', function() {
  client.write('drawCams;screenshot;hideCams;');
  client.end();
});
