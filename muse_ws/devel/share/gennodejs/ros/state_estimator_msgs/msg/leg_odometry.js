// Auto-generated. Do not edit!

// (in-package state_estimator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class leg_odometry {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lin_vel_lf = null;
      this.lin_vel_rf = null;
      this.lin_vel_lh = null;
      this.lin_vel_rh = null;
      this.base_velocity = null;
      this.stance_lf = null;
      this.stance_rf = null;
      this.stance_lh = null;
      this.stance_rh = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lin_vel_lf')) {
        this.lin_vel_lf = initObj.lin_vel_lf
      }
      else {
        this.lin_vel_lf = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('lin_vel_rf')) {
        this.lin_vel_rf = initObj.lin_vel_rf
      }
      else {
        this.lin_vel_rf = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('lin_vel_lh')) {
        this.lin_vel_lh = initObj.lin_vel_lh
      }
      else {
        this.lin_vel_lh = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('lin_vel_rh')) {
        this.lin_vel_rh = initObj.lin_vel_rh
      }
      else {
        this.lin_vel_rh = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('base_velocity')) {
        this.base_velocity = initObj.base_velocity
      }
      else {
        this.base_velocity = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('stance_lf')) {
        this.stance_lf = initObj.stance_lf
      }
      else {
        this.stance_lf = false;
      }
      if (initObj.hasOwnProperty('stance_rf')) {
        this.stance_rf = initObj.stance_rf
      }
      else {
        this.stance_rf = false;
      }
      if (initObj.hasOwnProperty('stance_lh')) {
        this.stance_lh = initObj.stance_lh
      }
      else {
        this.stance_lh = false;
      }
      if (initObj.hasOwnProperty('stance_rh')) {
        this.stance_rh = initObj.stance_rh
      }
      else {
        this.stance_rh = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type leg_odometry
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [lin_vel_lf] has the right length
    if (obj.lin_vel_lf.length !== 3) {
      throw new Error('Unable to serialize array field lin_vel_lf - length must be 3')
    }
    // Serialize message field [lin_vel_lf]
    bufferOffset = _arraySerializer.float64(obj.lin_vel_lf, buffer, bufferOffset, 3);
    // Check that the constant length array field [lin_vel_rf] has the right length
    if (obj.lin_vel_rf.length !== 3) {
      throw new Error('Unable to serialize array field lin_vel_rf - length must be 3')
    }
    // Serialize message field [lin_vel_rf]
    bufferOffset = _arraySerializer.float64(obj.lin_vel_rf, buffer, bufferOffset, 3);
    // Check that the constant length array field [lin_vel_lh] has the right length
    if (obj.lin_vel_lh.length !== 3) {
      throw new Error('Unable to serialize array field lin_vel_lh - length must be 3')
    }
    // Serialize message field [lin_vel_lh]
    bufferOffset = _arraySerializer.float64(obj.lin_vel_lh, buffer, bufferOffset, 3);
    // Check that the constant length array field [lin_vel_rh] has the right length
    if (obj.lin_vel_rh.length !== 3) {
      throw new Error('Unable to serialize array field lin_vel_rh - length must be 3')
    }
    // Serialize message field [lin_vel_rh]
    bufferOffset = _arraySerializer.float64(obj.lin_vel_rh, buffer, bufferOffset, 3);
    // Check that the constant length array field [base_velocity] has the right length
    if (obj.base_velocity.length !== 3) {
      throw new Error('Unable to serialize array field base_velocity - length must be 3')
    }
    // Serialize message field [base_velocity]
    bufferOffset = _arraySerializer.float64(obj.base_velocity, buffer, bufferOffset, 3);
    // Serialize message field [stance_lf]
    bufferOffset = _serializer.bool(obj.stance_lf, buffer, bufferOffset);
    // Serialize message field [stance_rf]
    bufferOffset = _serializer.bool(obj.stance_rf, buffer, bufferOffset);
    // Serialize message field [stance_lh]
    bufferOffset = _serializer.bool(obj.stance_lh, buffer, bufferOffset);
    // Serialize message field [stance_rh]
    bufferOffset = _serializer.bool(obj.stance_rh, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type leg_odometry
    let len;
    let data = new leg_odometry(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lin_vel_lf]
    data.lin_vel_lf = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [lin_vel_rf]
    data.lin_vel_rf = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [lin_vel_lh]
    data.lin_vel_lh = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [lin_vel_rh]
    data.lin_vel_rh = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [base_velocity]
    data.base_velocity = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [stance_lf]
    data.stance_lf = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stance_rf]
    data.stance_rf = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stance_lh]
    data.stance_lh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stance_rh]
    data.stance_rh = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 124;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimator_msgs/leg_odometry';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d1be3f0837dc21a71a6cc54730caa1c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float64[3] lin_vel_lf
    float64[3] lin_vel_rf
    float64[3] lin_vel_lh
    float64[3] lin_vel_rh
    
    float64[3] base_velocity
    
    bool stance_lf
    bool stance_rf
    bool stance_lh
    bool stance_rh
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new leg_odometry(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lin_vel_lf !== undefined) {
      resolved.lin_vel_lf = msg.lin_vel_lf;
    }
    else {
      resolved.lin_vel_lf = new Array(3).fill(0)
    }

    if (msg.lin_vel_rf !== undefined) {
      resolved.lin_vel_rf = msg.lin_vel_rf;
    }
    else {
      resolved.lin_vel_rf = new Array(3).fill(0)
    }

    if (msg.lin_vel_lh !== undefined) {
      resolved.lin_vel_lh = msg.lin_vel_lh;
    }
    else {
      resolved.lin_vel_lh = new Array(3).fill(0)
    }

    if (msg.lin_vel_rh !== undefined) {
      resolved.lin_vel_rh = msg.lin_vel_rh;
    }
    else {
      resolved.lin_vel_rh = new Array(3).fill(0)
    }

    if (msg.base_velocity !== undefined) {
      resolved.base_velocity = msg.base_velocity;
    }
    else {
      resolved.base_velocity = new Array(3).fill(0)
    }

    if (msg.stance_lf !== undefined) {
      resolved.stance_lf = msg.stance_lf;
    }
    else {
      resolved.stance_lf = false
    }

    if (msg.stance_rf !== undefined) {
      resolved.stance_rf = msg.stance_rf;
    }
    else {
      resolved.stance_rf = false
    }

    if (msg.stance_lh !== undefined) {
      resolved.stance_lh = msg.stance_lh;
    }
    else {
      resolved.stance_lh = false
    }

    if (msg.stance_rh !== undefined) {
      resolved.stance_rh = msg.stance_rh;
    }
    else {
      resolved.stance_rh = false
    }

    return resolved;
    }
};

module.exports = leg_odometry;
