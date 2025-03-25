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

class slip_detection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.legs_slipping = null;
      this.slip_lf = null;
      this.slip_rf = null;
      this.slip_lh = null;
      this.slip_rh = null;
      this.slippage = null;
    }
    else {
      if (initObj.hasOwnProperty('legs_slipping')) {
        this.legs_slipping = initObj.legs_slipping
      }
      else {
        this.legs_slipping = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('slip_lf')) {
        this.slip_lf = initObj.slip_lf
      }
      else {
        this.slip_lf = false;
      }
      if (initObj.hasOwnProperty('slip_rf')) {
        this.slip_rf = initObj.slip_rf
      }
      else {
        this.slip_rf = false;
      }
      if (initObj.hasOwnProperty('slip_lh')) {
        this.slip_lh = initObj.slip_lh
      }
      else {
        this.slip_lh = false;
      }
      if (initObj.hasOwnProperty('slip_rh')) {
        this.slip_rh = initObj.slip_rh
      }
      else {
        this.slip_rh = false;
      }
      if (initObj.hasOwnProperty('slippage')) {
        this.slippage = initObj.slippage
      }
      else {
        this.slippage = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type slip_detection
    // Check that the constant length array field [legs_slipping] has the right length
    if (obj.legs_slipping.length !== 4) {
      throw new Error('Unable to serialize array field legs_slipping - length must be 4')
    }
    // Serialize message field [legs_slipping]
    bufferOffset = _arraySerializer.float64(obj.legs_slipping, buffer, bufferOffset, 4);
    // Serialize message field [slip_lf]
    bufferOffset = _serializer.bool(obj.slip_lf, buffer, bufferOffset);
    // Serialize message field [slip_rf]
    bufferOffset = _serializer.bool(obj.slip_rf, buffer, bufferOffset);
    // Serialize message field [slip_lh]
    bufferOffset = _serializer.bool(obj.slip_lh, buffer, bufferOffset);
    // Serialize message field [slip_rh]
    bufferOffset = _serializer.bool(obj.slip_rh, buffer, bufferOffset);
    // Serialize message field [slippage]
    bufferOffset = _serializer.bool(obj.slippage, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type slip_detection
    let len;
    let data = new slip_detection(null);
    // Deserialize message field [legs_slipping]
    data.legs_slipping = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [slip_lf]
    data.slip_lf = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [slip_rf]
    data.slip_rf = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [slip_lh]
    data.slip_lh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [slip_rh]
    data.slip_rh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [slippage]
    data.slippage = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 37;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimator_msgs/slip_detection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87252336384194759607e2e036859110';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[4] legs_slipping
    bool slip_lf
    bool slip_rf
    bool slip_lh
    bool slip_rh
    bool slippage
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new slip_detection(null);
    if (msg.legs_slipping !== undefined) {
      resolved.legs_slipping = msg.legs_slipping;
    }
    else {
      resolved.legs_slipping = new Array(4).fill(0)
    }

    if (msg.slip_lf !== undefined) {
      resolved.slip_lf = msg.slip_lf;
    }
    else {
      resolved.slip_lf = false
    }

    if (msg.slip_rf !== undefined) {
      resolved.slip_rf = msg.slip_rf;
    }
    else {
      resolved.slip_rf = false
    }

    if (msg.slip_lh !== undefined) {
      resolved.slip_lh = msg.slip_lh;
    }
    else {
      resolved.slip_lh = false
    }

    if (msg.slip_rh !== undefined) {
      resolved.slip_rh = msg.slip_rh;
    }
    else {
      resolved.slip_rh = false
    }

    if (msg.slippage !== undefined) {
      resolved.slippage = msg.slippage;
    }
    else {
      resolved.slippage = false
    }

    return resolved;
    }
};

module.exports = slip_detection;
