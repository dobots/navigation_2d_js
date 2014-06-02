/**
 * @author Bart van Vliet - bart@dobots.nl
 */

var NAVIGATION2D = NAVIGATION2D || {
  REVISION : '1'
};

/**
 * @author Bart van Vliet - bart@dobots.nl
 */

/**
 * Send and draw a goal pose
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * rootObject (optional) - the root object to render to
 *   * actionTopic (optional) - the action server topic to use for navigation, like '/move_base'
 *   * actionMsgType (optional) - the navigation action message type, like 'move_base_msgs/MoveBaseAction'
 *   * mapFrame (optional) - the frame of the map to use when sending a goal, like '/map'
 */
NAVIGATION2D.NavGoal = function(options) {
	var that = this;
	options = options || {};
	var ros = options.ros;
	this.rootObject = options.rootObject || new createjs.Container();
	var actionTopic = options.actionTopic || '/move_base';
	var actionMsgType = options.actionMsgType || 'move_base_msgs/MoveBaseAction';
	this.mapFrame = options.mapFrame || '/map';
	
	// setup the actionlib client
	this.actionClient = new ROSLIB.ActionClient({
		ros : ros,
		actionName : actionMsgType,
		serverName : actionTopic
	});
	
	// get a handle to the stage
	if (this.rootObject instanceof createjs.Stage) {
		this.stage = this.rootObject;
	} else {
		this.stage = this.rootObject.getStage();
	}
	
	this.container = new createjs.Container();
	this.rootObject.addChild(this.container);
	
	// marker for goal orientation
	this.goalOrientationMarker = new ROS2D.ArrowShape({
		size : 30,
		strokeSize : 1,
		fillColor : createjs.Graphics.getRGB(0, 255, 0, 0.66),
		pulse : false
	});
	this.goalOrientationMarker.visible = false;
	this.container.addChild(this.goalOrientationMarker);
	
	// Used to set the goal marker
	this.goalStartPos = null;
	
	this.initScaleSet = false;
};


/**
 * Initialize scale, current scale will be used for the goal markers
 */
NAVIGATION2D.NavGoal.prototype.initScale = function() {
	if (this.initScaleSet) {
		console.log('Warning: scale has already been initialized!');
		// TODO: reinit
	}
	this.initScaleSet = true;
	this.initScaleX = 1.0 / this.stage.scaleX;
	this.initScaleY = 1.0 / this.stage.scaleY;
};


/**
 * Start goal selection, given position will be the goal position, draw the orientation marker
 *
 * @param pos - current selection position on the map in meters (ROSLIB.Vector3)
 */
NAVIGATION2D.NavGoal.prototype.startGoalSelection = function(pos) {
	this.goalStartPos = pos;
	this.goalOrientationMarker.visible = true;
	this.goalOrientationMarker.scaleX = 1.0 / this.stage.scaleX;
	this.goalOrientationMarker.scaleY = 1.0 / this.stage.scaleY;
	this.goalOrientationMarker.x = pos.x;
	this.goalOrientationMarker.y = -pos.y;
};

/**
 * Orient goal, after starting the goal, this function updates the orientation of the goal orientation marker
 *
 * @param pos - current selection position on the map in meters (ROSLIB.Vector3)
 */
NAVIGATION2D.NavGoal.prototype.orientGoalSelection = function(pos) {
	this.goalOrientationMarker.scaleX = 1.0 / this.stage.scaleX;
	this.goalOrientationMarker.scaleY = 1.0 / this.stage.scaleY;
	var dx = pos.x - this.goalStartPos.x;
	var dy = pos.y - this.goalStartPos.y;
	this.goalOrientationMarker.rotation = -Math.atan2(dy, dx) * 180.0 / Math.PI;
};

/**
 * End of selecting a goal, removes the orientation marker
 *
 * @param pos - current selection position on the map in meters (ROSLIB.Vector3)
 *
 * @returns the goal pose (ROSLIB.Pose)
 */
NAVIGATION2D.NavGoal.prototype.endGoalSelection = function() {
	this.goalOrientationMarker.visible = false;

	// Get angle from orientation marker, so that the goal always matches with the marker
	// convert to radians and counter clock wise
	var theta = -this.goalOrientationMarker.rotation * Math.PI / 180.0;
	var qz =  Math.sin(theta/2.0);
	var qw =  Math.cos(theta/2.0);
	var quat = new ROSLIB.Quaternion({
		x : 0,
		y : 0,
		z : qz,
		w : qw
	});
	return new ROSLIB.Pose({
		position : this.goalStartPos,
		orientation : quat
	});
};


/**
 * Send a goal to the navigation stack with the given pose.
 * Draw the goal
 *
 * @param pose - the goal pose (ROSLIB.Pose)
 */
NAVIGATION2D.NavGoal.prototype.sendGoal = function(pose) {
	// create a goal
	var goal = new ROSLIB.Goal({
		actionClient : this.actionClient,
		goalMessage : {
			target_pose : {
				header : {
					frame_id : this.mapFrame
				},
				pose : pose
			}
		}
	});
	goal.send();
	
	// create a marker for the goal
	var goalMarker = new ROS2D.ArrowShape({
		size : 10,
		strokeSize : 1,
		fillColor : createjs.Graphics.getRGB(255, 64, 128, 0.66),
		pulse : true
	});
	goalMarker.x = pose.position.x;
	goalMarker.y = -pose.position.y;
	goalMarker.rotation = this.stage.rosQuaternionToGlobalTheta(pose.orientation);
	goalMarker.scaleX = this.initScaleX;
	goalMarker.scaleY = this.initScaleY;
	this.container.addChild(goalMarker);
	
	var that = this;
	goal.on('result', function() {
		that.container.removeChild(goalMarker);
	});
};

/**
 * @author Bart van Vliet - bart@dobots.nl
 */

/**
 * Listens for path msgs and draws the path
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * rootObject (optional) - the root object to render to
 *   * pathTopic (optional) - the path topic to subscribe to, like '/plan', must be of type: 'nav_msgs/Path'
 *   * color (optional) - color of the marker
 *   * size (optional) - size of the marker
 */
NAVIGATION2D.NavPath = function(options) {
	var that = this;
	options = options || {};
	var ros = options.ros;
	this.rootObject = options.rootObject || new createjs.Container();
	var pathTopic = options.pathTopic || '/plan';
	var color = options.color || createjs.Graphics.getRGB(0, 255, 0, 1);
	var size = options.size || 1;
	
	// get a handle to the stage
	if (this.rootObject instanceof createjs.Stage) {
		this.stage = this.rootObject;
	} else {
		this.stage = this.rootObject.getStage();
	}
	
	// shape for the path
	this.path = new ROS2D.PathShape({
		strokeSize : size,
		strokeColor : color
	});
	this.path.visible = false;
	this.rootObject.addChild(this.path);
	
	this.initScaleSet = false;
	
	// Set up a listener for the planned path
	var pathListener = new ROSLIB.Topic({
		ros : ros,
		name : pathTopic,
		messageType : 'nav_msgs/Path',
		throttle_rate : 100
	});
	pathListener.subscribe(this.updatePath.bind(this));
};

/**
 * Initialize scale, current scale will be used for the goal markers
 */
NAVIGATION2D.NavPath.prototype.initScale = function() {
	if (this.initScaleSet) {
		console.log('Warning: scale has already been initialized!');
		// TODO: reinit
	}
	this.initScaleSet = true;
	this.path.scaleX = 1.0 / this.stage.scaleX;
	this.path.scaleY = 1.0 / this.stage.scaleY;
};

/**
 * Update the robot's path drawing
 *
 * @param path - the path (nav_msgs/Path)
 */
NAVIGATION2D.NavPath.prototype.updatePath = function(path) {
	if (this.initScaleSet) {
		this.path.visible = true;
	}
	this.path.setPath(path);
};

/**
 * @author Bart van Vliet - bart@dobots.nl
 */

/**
 * Listens for pose msgs and draws the robot pose and a trace
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * rootObject (optional) - the root object to render to
 *   * poseTopic (optional) - the pose topic to subscribe to, like '/robot_pose', must be of type: 'geometry_msgs/Pose'
 *   * withTrace (optional) - whether to draw the robot's trace (default: true)
 *   * maxTraceLength (optional) - maximum length of the trace in number of poses (0 for infinite)
 *   * traceColor (optional) - color of the trace shape
 *   * traceSize (optional) - size of the trace shape
 *   * robotColor (optional) - color of the robot shape
 *   * robotSize (optional) - size of the robot shape
 *   * robotShape (optional) - shape of your robot, front should point to the east at 0 rotation
 */
NAVIGATION2D.PoseAndTrace = function(options) {
	var that = this;
	options = options || {};
	var ros = options.ros;
	this.rootObject = options.rootObject || new createjs.Container();
	var poseTopic = options.poseTopic || '/robot_pose';
	this.withTrace = options.withTrace || true;
	this.maxTraceLength = options.maxTraceLength || 100;
	var traceColor = options.traceColor || createjs.Graphics.getRGB(0, 150, 0, 0.66);
	var traceSize = options.traceSize || 1.5;
	var robotColor = options.robotColor || createjs.Graphics.getRGB(255, 0, 0, 0.66);
	var robotSize = options.robotSize || 15;
	this.robotMarker = options.robotShape || null;
	
	// get a handle to the stage
	if (this.rootObject instanceof createjs.Stage) {
		this.stage = this.rootObject;
	} else {
		this.stage = this.rootObject.getStage();
	}
	
	// shape for the trace
	this.trace = new ROS2D.TraceShape({
		strokeSize : traceSize,
		strokeColor : traceColor,
		maxPoses : this.maxTraceLength
	});
	this.trace.visible = false;
	this.rootObject.addChild(this.trace);
	
	// marker for the robot
	if (!this.robotMarker) {
		this.robotMarker = new ROS2D.ArrowShape({
			size : robotSize,
			strokeSize : 1,
			strokeColor : robotColor,
			fillColor : robotColor,
			pulse : true
		});
	}
	this.robotMarker.visible = false;
	this.rootObject.addChild(this.robotMarker);
	
	this.initScaleSet = false;
	
	// setup a listener for the robot pose
	var poseListener = new ROSLIB.Topic({
		ros : ros,
		name : poseTopic,
		messageType : 'geometry_msgs/Pose',
		throttle_rate : 100
	});
	poseListener.subscribe(this.updatePose.bind(this));
};

/**
 * Initialize scale, current scale will be used for the goal markers
 */
NAVIGATION2D.PoseAndTrace.prototype.initScale = function() {
	if (this.initScaleSet) {
		console.log('Warning: scale has already been initialized!');
		// TODO: reinit
	}
	this.initScaleSet = true;
	this.robotMarker.scaleX = 1.0 / this.stage.scaleX;
	this.robotMarker.scaleY = 1.0 / this.stage.scaleY;
	this.trace.scaleX = 1.0 / this.stage.scaleX;
	this.trace.scaleY = 1.0 / this.stage.scaleY;
};

/**
 * Update the robot's pose: move the robot marker and add to trace
 *
 * @param pose - the robot's pose (geometry_msgs/Pose)
 */
NAVIGATION2D.PoseAndTrace.prototype.updatePose = function(pose) {
	// update the robot's position and rotation on the map
	this.robotMarker.x = pose.position.x;
	this.robotMarker.y = -pose.position.y;
	this.robotMarker.rotation = this.stage.rosQuaternionToGlobalTheta(pose.orientation);
	if (this.initScaleSet) {
		this.robotMarker.visible = true;
	}
	
	// Draw trace
	if (this.withTrace === true && this.initScaleSet === true) {
		this.trace.addPose(pose);
		this.trace.visible = true;
	}
};
