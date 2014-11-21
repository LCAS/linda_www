/**
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 *
 * adapted from keyboardteleop.js by Russell Toris (rctoris@wpi.edu)
 */

var PTU_KEYBOARDTELEOP = PTU_KEYBOARDTELEOP || {
    REVISION : '2'
};

/**
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 */

/**
 * Manages connection to the server and all interactions with ROS.
 *
 * Emits the following events:
 *   * 'change' - emitted with a change in speed occurs
 *
 * @constructor
 * @param options - possible keys include:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the Vector3 topic to publish to, like '/ptu_jointstate'
 */
PTU_KEYBOARDTELEOP.Teleop = function(options) {
    var that = this;
    options = options || {};
    var ros = options.ros;
    var topic = options.topic || '/ptu_jointstate';
    
    var y = 0;
    var z = 0;

    var ptu_topic = new ROSLIB.Topic({
        ros : ros,
        name : topic,
        messageType : 'sensor_msgs/JointState'
    });

    // sets up a key listener on the page used for keyboard teleoperation
    var handleKey = function(keyCode, keyDown) {
        // used to check for changes in speed
        var oldY = y;
        var oldZ = z;
        
        var speed = 0;
        // throttle the speed by the slider and throttle constant
        if (keyDown === true) {
            speed = 1.0;
        }

        // check which key was pressed
        switch (keyCode) {
        case 72: // H
            // turn left
            y = oldY + 0.1 * speed;
            break;
        case 75: // K
            // up
            z = oldZ - 0.1 * speed;
            break;
        case 76: // L
            // turn right
            y = oldY - 0.1 * speed;
            break;
        case 74: // J
            // down
            z = oldZ + 0.1 * speed;
            break;
        case 79: // O
            // set default pose
            y = 0.0
            z = 0.0
            break;
        }

        // publish the command
        var jointstate = new ROSLIB.Message({
            header: {
                seq: 0,
                stamp: "",
                frame_id: ""
            },
            name:  ['pan', 'tilt'],
            position: [z,  y],
            velocity: [0.0,  0.0],
            effort:  [0.0,  0.0]
        });
        console.log('publishing');
        ptu_topic.publish(jointstate);

        // check for changes
        if (oldY !== y || oldZ !== z) {
            that.emit('change', jointstate);
        }
    };

    // handle the key
    var body = document.getElementsByTagName('body')[0];
    body.addEventListener('keydown', function(e) {
        console.log('handling keydown', e);
        handleKey(e.keyCode, true);
    }, false);
    body.addEventListener('keyup', function(e) {
        console.log('handling keyup', e);
        handleKey(e.keyCode, false);
    }, false);
};
PTU_KEYBOARDTELEOP.Teleop.prototype.__proto__ = EventEmitter2.prototype;
