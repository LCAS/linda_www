/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * The main marker control object for an interactive marker.
 *
 * @constructor
 * @param options - object with following keys:
 *  * parent - the parent of this control
 *  * message - the interactive marker control message
 *  * camera - the main camera associated with the viewer for this marker client
 *  * path (optional) - the base path to any meshes that will be loaded
 */
ROS3D.InteractiveMarkerControl = function(options) {
  var that = this;
  THREE.Object3D.call(this);
  THREE.EventDispatcher.call(this);

  options = options || {};
  this.parent = options.parent;
  var message = options.message;
  this.name = message.name;
  this.camera = options.camera;
  this.path = options.path || '/';
  this.dragging = false;

  // orientation for the control
  var controlOri = new THREE.Quaternion(message.orientation.x, message.orientation.y,
      message.orientation.z, message.orientation.w);
  controlOri.normalize();

  // transform x axis into local frame
  var controlAxis = new THREE.Vector3(1, 0, 0);
  controlAxis.applyQuaternion(controlOri);

  this.currentControlOri = new THREE.Quaternion();

  // determine mouse interaction
  switch (message.interaction_mode) {
    case ROS3D.INTERACTIVE_MARKER_MOVE_AXIS:
      this.addEventListener('mousemove', this.parent.moveAxis.bind(this.parent, this, controlAxis));
      this.addEventListener('touchmove', this.parent.moveAxis.bind(this.parent, this, controlAxis));
      break;
    case ROS3D.INTERACTIVE_MARKER_ROTATE_AXIS:
      this
          .addEventListener('mousemove', this.parent.rotateAxis.bind(this.parent, this, controlOri));
      break;
    case ROS3D.INTERACTIVE_MARKER_MOVE_PLANE:
      this
          .addEventListener('mousemove', this.parent.movePlane.bind(this.parent, this, controlAxis));
      break;
    case ROS3D.INTERACTIVE_MARKER_BUTTON:
      this.addEventListener('click', this.parent.buttonClick.bind(this.parent, this));
      break;
    default:
      break;
  }

  /**
   * Install default listeners for highlighting / dragging.
   *
   * @param event - the event to stop
   */
  function stopPropagation(event) {
    event.stopPropagation();
  }

  // check the mode
  if (message.interaction_mode !== ROS3D.INTERACTIVE_MARKER_NONE) {
    this.addEventListener('mousedown', this.parent.startDrag.bind(this.parent, this));
    this.addEventListener('mouseup', this.parent.stopDrag.bind(this.parent, this));
    this.addEventListener('contextmenu', this.parent.showMenu.bind(this.parent, this));
    this.addEventListener('mouseover', stopPropagation);
    this.addEventListener('mouseout', stopPropagation);
    this.addEventListener('click', stopPropagation);

    // touch support
    this.addEventListener('touchstart', function(event3d) {
      console.log(event3d.domEvent);
      if (event3d.domEvent.touches.length === 1) {
        event3d.type = 'mousedown';
        event3d.domEvent.button = 0;
        that.dispatchEvent(event3d);
      }
    });
    this.addEventListener('touchmove', function(event3d) {
      if (event3d.domEvent.touches.length === 1) {
        console.log(event3d.domEvent);
        event3d.type = 'mousemove';
        event3d.domEvent.button = 0;
        that.dispatchEvent(event3d);
      }
    });
    this.addEventListener('touchend', function(event3d) {
      if (event3d.domEvent.touches.length === 0) {
        event3d.domEvent.button = 0;
        event3d.type = 'mouseup';
        that.dispatchEvent(event3d);
        event3d.type = 'click';
        that.dispatchEvent(event3d);
      }
    });
  }

  // rotation behavior
  var rotInv = new THREE.Quaternion();
  var posInv = this.parent.position.clone().multiplyScalar(-1);
  switch (message.orientation_mode) {
    case ROS3D.INTERACTIVE_MARKER_INHERIT:
      rotInv = this.parent.quaternion.clone().inverse();
      this.updateMatrixWorld = function(force) {
        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
        that.currentControlOri.copy(that.quaternion);
        that.currentControlOri.normalize();
      };
      break;
    case ROS3D.INTERACTIVE_MARKER_FIXED:
      this.updateMatrixWorld = function(force) {
        that.useQuaternion = true;
        that.quaternion = that.parent.quaternion.clone().inverse();
        that.updateMatrix();
        that.matrixWorldNeedsUpdate = true;
        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
        that.currentControlOri.copy(that.quaternion);
      };
      break;
    case ROS3D.INTERACTIVE_MARKER_VIEW_FACING:
      var independentMarkerOrientation = message.independentMarkerOrientation;
      this.updateMatrixWorld = function(force) {
        that.camera.updateMatrixWorld();
        var cameraRot = new THREE.Matrix4().extractRotation(that.camera.matrixWorld);

        var ros2Gl = new THREE.Matrix4();
        var r90 = Math.PI * 0.5;
        var rv = new THREE.Vector3(-r90, 0, r90);
        ros2Gl.setRotationFromEuler(rv);

        var worldToLocal = new THREE.Matrix4();
        worldToLocal.getInverse(that.parent.matrixWorld);

        cameraRot.multiplyMatrices(cameraRot, ros2Gl);
        cameraRot.multiplyMatrices(worldToLocal, cameraRot);

        that.currentControlOri.setFromRotationMatrix(cameraRot);

        // check the orientation
        if (!independentMarkerOrientation) {
          that.useQuaternion = true;
          that.quaternion.copy(that.currentControlOri);
          that.updateMatrix();
          that.matrixWorldNeedsUpdate = true;
        }
        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
      };
      break;
    default:
      console.error('Unkown orientation mode: ' + message.orientation_mode);
      break;
  }

  // create visuals (markers)
  message.markers.forEach(function(markerMsg) {
    var markerHelper = new ROS3D.Marker({
      message : markerMsg,
      path : that.path
    });

    if (markerMsg.header.frame_id !== '') {
      // if the marker lives in its own coordinate frame, convert position into IM's local frame
      markerHelper.position.add(posInv);
      markerHelper.position.applyQuaternion(rotInv);
      markerHelper.quaternion.multiplyQuaternions(rotInv, markerHelper.quaternion);
      markerHelper.updateMatrixWorld();
    }

    // add the marker
    that.add(markerHelper);
  });
};
ROS3D.InteractiveMarkerControl.prototype.__proto__ = THREE.Object3D.prototype;
