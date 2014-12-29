var env = process.env.NODE_ENV || 'development';
var sendgrid_config = require(__dirname + '/config/sendgrid.js')[env];
var sendgrid = require('sendgrid')(sendgrid_config.user, sendgrid_config.password);

var net = require('net');
var client = new net.Socket();

function mailPhoto(file) {
  var email = new sendgrid.Email();
  if(!process.argv[2]) {
    recipient = sendgrid_config.default_recipient;
  } else {
    recipient = process.argv[2];
  }
  email.addTo(recipient);
  email.setFrom(sendgrid_config.from);
  email.setSubject(sendgrid_config.subject);
  email.setText(sendgrid_config.text);
  email.addFile({
    filename: sendgrid_config.filename,
    path: file
  });
  sendgrid.send(email, function(err, json) {
    if (err) {
      return console.error(err);
    }
    console.log(json);
  });
}

client.connect(8888, '127.0.0.1', function() {
  client.write('drawCams;screenshot;hideCams;');
  client.end();
});

file = __dirname + '/../bin/data/screenshot.jpg';
console.log(file);
mailPhoto(file);
