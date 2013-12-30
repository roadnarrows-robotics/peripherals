// 
// rosbridge xbo360 connection
// 
// required js imports:
//    http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js
//    http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js
//    rosbridge_global.js
//

// xbox360 controller state subscription
var xbox360_state_sub = new ROSLIB.Topic({
  ros: ros,
  name: "/xbox_360/controller_state",
  messageType: "hid/Controller360State"
})


// xbox360 rumble command publisher and message
var xbox360_rumble_pub = new ROSLIB.Topic({
  ros: ros,
  name: "/xbox_360/rumble_command",
  messageType: "hid/RumbleCmd"
})

var rumble_cmd = new ROSLIB.Message({
  left_rumble:0,
  right_rumble:0
});

// xbox360 set_rumble service and service request
var xbox360_set_rumble_srv = new ROSLIB.Service({
  ros: ros,
  name: "/xbox_360/set_rumble",
  messageType: "hid/SetRumble"
})

var set_rumble_req = new ROSLIB.ServiceRequest({
  left_rumble:0,
  right_rumble:0
});


// xbox360 set_led service and service request
var xbox360_set_led_srv = new ROSLIB.Service({
  ros: ros,
  name: "/xbox_360/set_led",
  messageType: "hid/SetLED"
})

var set_led_req = new ROSLIB.ServiceRequest({
  val:0 // one of 0-14. TODO: define enum for LED settings
});


// xbox360 ping service
var xbox360_ping_srv = new ROSLIB.Service({
  ros: ros,
  name: "/xbox_360/ping_controller",
  messageType: "hid/SetLED"
})

var ping_req = new ROSLIB.ServiceRequest({});


// convenience functions
function xbox360_set_rumble(left, right) {
  console.info(set_rumble_req);
  set_rumble_req.left_rumble = left;
  set_rumble_req.right_rumble = right;

  console.info(set_rumble_req);
  xbox360_set_rumble_srv.callService(set_rumble_req, function(res,err){});
}

function xbox360_set_led(val) {
  console.info(set_led_req);
  set_led_req.left_rumble = left;

  console.info(set_led_req);
  xbox360_set_led_srv.callService(set_led_req, function(res,err){});
}
