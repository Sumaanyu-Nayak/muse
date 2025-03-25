// Auto-generated. Do not edit!

// (in-package state_estimator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class imu {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.w = null;
      this.a = null;
    }
    else {
      if (initObj.hasOwnProperty('w')) {
        this.w = initObj.w
      }
      else {
        this.w = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('a')) {
        this.a = initObj.a
      }
      else {
        this.a = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type imu
    // Check that the constant length array field [w] has the right length
    if (obj.w.length !== 3) {
      throw new Error('Unable to serialize array field w - length must be 3')
    }
    // Serialize message field [w]
    bufferOffset = _arraySerializer.float64(obj.w, buffer, bufferOffset, 3);
    // Check that the constant length array field [a] has the right length
    if (obj.a.length !== 3) {
      throw new Error('Unable to serialize array field a - length must be 3')
    }
    // Serialize message field [a]
    bufferOffset = _arraySerializer.float64(obj.a, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type imu
    let len;
    let data = new imu(null);
    // Deserialize message field [w]
    data.w = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [a]
    data.a = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimator_msgs/imu';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3eb7a1d5e42bc0c9297c14a9acbb436b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[3] w
    float64[3] a
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new imu(null);
    if (msg.w !== undefined) {
      resolved.w = msg.w;
    }
    else {
      resolved.w = new Array(3).fill(0)
    }

    if (msg.a !== undefined) {
      resolved.a = msg.a;
    }
    else {
      resolved.a = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = imu;
