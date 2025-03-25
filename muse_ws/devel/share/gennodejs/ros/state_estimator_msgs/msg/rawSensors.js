// Auto-generated. Do not edit!

// (in-package state_estimator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let actuatorForces = require('./actuatorForces.js');
let imu = require('./imu.js');
let timeSync = require('./timeSync.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class rawSensors {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.tau = null;
      this.imu = null;
      this.imu_secondary = null;
      this.time_sync = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('tau')) {
        this.tau = initObj.tau
      }
      else {
        this.tau = new actuatorForces();
      }
      if (initObj.hasOwnProperty('imu')) {
        this.imu = initObj.imu
      }
      else {
        this.imu = new imu();
      }
      if (initObj.hasOwnProperty('imu_secondary')) {
        this.imu_secondary = initObj.imu_secondary
      }
      else {
        this.imu_secondary = new imu();
      }
      if (initObj.hasOwnProperty('time_sync')) {
        this.time_sync = initObj.time_sync
      }
      else {
        this.time_sync = new timeSync();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rawSensors
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [tau]
    bufferOffset = actuatorForces.serialize(obj.tau, buffer, bufferOffset);
    // Serialize message field [imu]
    bufferOffset = imu.serialize(obj.imu, buffer, bufferOffset);
    // Serialize message field [imu_secondary]
    bufferOffset = imu.serialize(obj.imu_secondary, buffer, bufferOffset);
    // Serialize message field [time_sync]
    bufferOffset = timeSync.serialize(obj.time_sync, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rawSensors
    let len;
    let data = new rawSensors(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [tau]
    data.tau = actuatorForces.deserialize(buffer, bufferOffset);
    // Deserialize message field [imu]
    data.imu = imu.deserialize(buffer, bufferOffset);
    // Deserialize message field [imu_secondary]
    data.imu_secondary = imu.deserialize(buffer, bufferOffset);
    // Deserialize message field [time_sync]
    data.time_sync = timeSync.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 192;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimator_msgs/rawSensors';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1bf8247f04179d165127b1c3762c3128';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    actuatorForces tau
    imu imu
    imu imu_secondary
    timeSync time_sync
    
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
    
    ================================================================================
    MSG: state_estimator_msgs/actuatorForces
    float64[12] tau
    
    ================================================================================
    MSG: state_estimator_msgs/imu
    float64[3] w
    float64[3] a
    
    ================================================================================
    MSG: state_estimator_msgs/timeSync
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rawSensors(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.tau !== undefined) {
      resolved.tau = actuatorForces.Resolve(msg.tau)
    }
    else {
      resolved.tau = new actuatorForces()
    }

    if (msg.imu !== undefined) {
      resolved.imu = imu.Resolve(msg.imu)
    }
    else {
      resolved.imu = new imu()
    }

    if (msg.imu_secondary !== undefined) {
      resolved.imu_secondary = imu.Resolve(msg.imu_secondary)
    }
    else {
      resolved.imu_secondary = new imu()
    }

    if (msg.time_sync !== undefined) {
      resolved.time_sync = timeSync.Resolve(msg.time_sync)
    }
    else {
      resolved.time_sync = new timeSync()
    }

    return resolved;
    }
};

module.exports = rawSensors;
