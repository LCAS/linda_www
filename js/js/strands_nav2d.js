/**
 * @author Russell Toris - rctoris@wpi.edu
 * 
 * adapted by Lars Kunze - l.kunze@cs.bham.ac.uk
 * - 2D navigator with orientation 
 */

var NAV2D = NAV2D || {
  REVISION : '1'
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A navigator can be used to add click-to-navigate options to an object.
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 */
NAV2D.Navigator = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var serverName = options.serverName || '/move_base';
  var actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  this.rootObject = options.rootObject || new createjs.Container();

  // setup the actionlib client
  var actionClient = new ROSLIB.ActionClient({
    ros : ros,
    actionName : actionName,
    serverName : serverName
  });

  /**
   * Send a goal to the navigation stack with the given pose.
   *
   * @param pose - the goal pose
   */
  function sendGoal(pose) {
    // create a goal
    var goal = new ROSLIB.Goal({
      actionClient : actionClient,
      goalMessage : {
        target_pose : {
          header : {
            frame_id : '/map'
          },
          pose : pose
        }
      }
    });
    goal.send();

    // create a marker for the goal
    var goalMarker = new ROS2D.NavigationArrow({
      size : 8,
      strokeSize : 1,
      fillColor : createjs.Graphics.getRGB(255, 64, 128, 0.66),
      pulse : true
    });
    goalMarker.x = pose.position.x;
    goalMarker.y = -pose.position.y;
    goalMarker.rotation = stage.rosQuaternionToGlobalTheta(pose.orientation);
    goalMarker.scaleX = 1.0 / stage.scaleX;
    goalMarker.scaleY = 1.0 / stage.scaleY;
    that.rootObject.addChild(goalMarker);

    goal.on('result', function() {
      that.rootObject.removeChild(goalMarker);
    });
  }

  // get a handle to the stage
  var stage;
  if (that.rootObject instanceof createjs.Stage) {
    stage = that.rootObject;
  } else {
    stage = that.rootObject.getStage();
  }

  // marker for the robot
  var robotMarker = new ROS2D.NavigationArrow({
    size : 12,
    strokeSize : 1,
    fillColor : createjs.Graphics.getRGB(255, 128, 0, 0.66),
    pulse : true
  });
  // wait for a pose to come in first
  robotMarker.visible = false;
  this.rootObject.addChild(robotMarker);
  var initScaleSet = false;

  // setup a listener for the robot pose
  var poseListener = new ROSLIB.Topic({
    ros : ros,
    name : '/robot_pose',
    messageType : 'geometry_msgs/Pose',
    throttle_rate : 100
  });
  poseListener.subscribe(function(pose) {
    // update the robots position on the map
    robotMarker.x = pose.position.x;
    robotMarker.y = -pose.position.y;
    if (!initScaleSet) {
      robotMarker.scaleX = 1.0 / stage.scaleX;
      robotMarker.scaleY = 1.0 / stage.scaleY;
      initScaleSet = true;
    }

    // change the angle
    robotMarker.rotation = stage.rosQuaternionToGlobalTheta(pose.orientation);

    robotMarker.visible = true;
  });

    var position = null;
    var directionMarker = null;
    var that = this;
    var down = false;
    var deg = 0;

    var handle_mouse = function(event, mouse_state) {
        //console.log('mouse_state: ', mouse_state);

        if (mouse_state == 'down'){
            position = stage.globalToRos(event.stageX, event.stageY);
            down = true;
        }
        else if (mouse_state == 'move'){

            that.rootObject.removeChild(directionMarker);
            
            if ( down == true) {
                direction_pos = stage.globalToRos(event.stageX, event.stageY);
                dir_pos = new ROSLIB.Vector3(direction_pos)

                var size = 30;

                directionMarker = new ROS2D.NavigationArrow({
                    size : size,
                    strokeSize : 1,
                    fillColor : createjs.Graphics.getRGB(0, 255, 0, 0.66),
                    pulse : false
                });
                var pose = new ROSLIB.Pose({
                    position : new ROSLIB.Vector3(position),
                    orientation : new ROSLIB.Quaternion()
                });

                var x_diff =  dir_pos.x - pose.position.x;
                var y_diff =  dir_pos.y - pose.position.y;
                
                var theta  = Math.atan2(x_diff,y_diff) + Math.PI/2;

                deg = theta * (180.0 / Math.PI);
                if (deg >= 0 && deg <= 180) {
                    deg += 270;
                } else {
                    deg -= 90;
                }

                directionMarker.x = pose.position.x;
                directionMarker.y = -pose.position.y;
                directionMarker.rotation = deg // stage.rosQuaternionToGlobalTheta(pose.orientation);
                directionMarker.scaleX = 1.0 / stage.scaleX;
                directionMarker.scaleY = 1.0 / stage.scaleY;
                
                that.rootObject.addChild(directionMarker);
            }
        } else { // mouse_state == 'up'
            down = false;

            that.rootObject.addChild(directionMarker);

            var goal_direction_pos = stage.globalToRos(event.stageX, event.stageY);

            var goal_dir_pos = new ROSLIB.Vector3(goal_direction_pos)
            
            var pose = new ROSLIB.Pose({
                position : new ROSLIB.Vector3(position),
                orientation : new ROSLIB.Quaternion()
            });


            var x_diff =  goal_dir_pos.x - pose.position.x;
            var y_diff =  goal_dir_pos.y - pose.position.y;
            
            var theta  = Math.atan2(x_diff,y_diff);
            
            if (theta >= 0 && theta <= Math.PI) {
                theta += (3 * Math.PI / 2);
            } else {
                theta -= (Math.PI/2);
            }
            var qz =  Math.sin(-theta/2.0);
            var qw =  Math.cos(-theta/2.0);
                
            var orientation = new ROSLIB.Quaternion({x:0, y:0, z:qz, w:qw});
            
            var pose = new ROSLIB.Pose({
                position : new ROSLIB.Vector3(position),
                orientation : orientation
            });
            // send the goal
            sendGoal(pose);
        }
    };

    this.rootObject.addEventListener('stagemousedown', function(event) {
        handle_mouse(event,'down');
    }, false);

    this.rootObject.addEventListener('stagemousemove', function(event) {
        handle_mouse(event,'move');
    }, false);

    this.rootObject.addEventListener('stagemouseup', function(event) {
        handle_mouse(event,'up');
    }, false);
    

};

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A OccupancyGridClientNav uses an OccupancyGridClient to create a map for use with a Navigator.
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map topic to listen to
 *   * rootObject (optional) - the root object to add this marker to
 *   * continuous (optional) - if the map should be continuously loaded (e.g., for SLAM)
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * viewer - the main viewer to render to
 */
NAV2D.OccupancyGridClientNav = function(options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  var topic = options.topic || '/map';
  var continuous = options.continuous;
  this.serverName = options.serverName || '/move_base';
  this.actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  this.rootObject = options.rootObject || new createjs.Container();
  this.viewer = options.viewer;

  this.navigator = null;

  // setup a client to get the map
  var client = new ROS2D.OccupancyGridClient({
    ros : this.ros,
    rootObject : this.rootObject,
    continuous : continuous
  });
  client.on('change', function() {
    that.navigator = new NAV2D.Navigator({
      ros : that.ros,
      serverName : that.serverName,
      actionName : that.actionName,
      rootObject : that.rootObject
    });
    
    // scale the viewer to fit the map
    that.viewer.scaleToDimensions(client.currentGrid.width, client.currentGrid.height);
    that.viewer.shift(client.currentGrid.x,client.currentGrid.y)
  });
};