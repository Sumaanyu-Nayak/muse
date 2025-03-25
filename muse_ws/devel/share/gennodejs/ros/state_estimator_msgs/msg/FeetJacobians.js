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

class FeetJacobians {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lf = null;
      this.rf = null;
      this.lh = null;
      this.rh = null;
    }
    else {
      if (initObj.hasOwnProperty('lf')) {
        this.lf = initObj.lf
      }
      else {
        this.lf = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('rf')) {
        this.rf = initObj.rf
      }
      else {
        this.rf = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('lh')) {
        this.lh = initObj.lh
      }
      else {
        this.lh = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('rh')) {
        this.rh = initObj.rh
      }
      else {
        this.rh = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FeetJacobians
    // Check that the constant length array field [lf] has the right length
    if (obj.lf.length !== 9) {
      throw new Error('Unable to serialize array field lf - length must be 9')
    }
    // Serialize message field [lf]
    bufferOffset = _arraySerializer.float64(obj.lf, buffer, bufferOffset, 9);
    // Check that the constant length array field [rf] has the right length
    if (obj.rf.length !== 9) {
      throw new Error('Unable to serialize array field rf - length must be 9')
    }
    // Serialize message field [rf]
    bufferOffset = _arraySerializer.float64(obj.rf, buffer, bufferOffset, 9);
    // Check that the constant length array field [lh] has the right length
    if (obj.lh.length !== 9) {
      throw new Error('Unable to serialize array field lh - length must be 9')
    }
    // Serialize message field [lh]
    bufferOffset = _arraySerializer.float64(obj.lh, buffer, bufferOffset, 9);
    // Check that the constant length array field [rh] has the right length
    if (obj.rh.length !== 9) {
      throw new Error('Unable to serialize array field rh - length must be 9')
    }
    // Serialize message field [rh]
    bufferOffset = _arraySerializer.float64(obj.rh, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FeetJacobians
    let len;
    let data = new FeetJacobians(null);
    // Deserialize message field [lf]
    data.lf = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [rf]
    data.rf = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [lh]
    data.lh = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [rh]
    data.rh = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    return 288;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimator_msgs/FeetJacobians';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c109ea601aeb1745edc38521e6c4252c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[9] lf
    float64[9] rf
    float64[9] lh
    float64[9] rh
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FeetJacobians(null);
    if (msg.lf !== undefined) {
      resolved.lf = msg.lf;
    }
    else {
      resolved.lf = new Array(9).fill(0)
    }

    if (msg.rf !== undefined) {
      resolved.rf = msg.rf;
    }
    else {
      resolved.rf = new Array(9).fill(0)
    }

    if (msg.lh !== undefined) {
      resolved.lh = msg.lh;
    }
    else {
      resolved.lh = new Array(9).fill(0)
    }

    if (msg.rh !== undefined) {
      resolved.rh = msg.rh;
    }
    else {
      resolved.rh = new Array(9).fill(0)
    }

    return resolved;
    }
};

module.exports = FeetJacobians;
