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

class ForwardKinematics {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lf_haa = null;
      this.rf_haa = null;
      this.lh_haa = null;
      this.rh_haa = null;
      this.lf_hfe = null;
      this.rf_hfe = null;
      this.lh_hfe = null;
      this.rh_hfe = null;
      this.lf_kfe = null;
      this.rf_kfe = null;
      this.lh_kfe = null;
      this.rh_kfe = null;
      this.lf_foot = null;
      this.rf_foot = null;
      this.lh_foot = null;
      this.rh_foot = null;
    }
    else {
      if (initObj.hasOwnProperty('lf_haa')) {
        this.lf_haa = initObj.lf_haa
      }
      else {
        this.lf_haa = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('rf_haa')) {
        this.rf_haa = initObj.rf_haa
      }
      else {
        this.rf_haa = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('lh_haa')) {
        this.lh_haa = initObj.lh_haa
      }
      else {
        this.lh_haa = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('rh_haa')) {
        this.rh_haa = initObj.rh_haa
      }
      else {
        this.rh_haa = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('lf_hfe')) {
        this.lf_hfe = initObj.lf_hfe
      }
      else {
        this.lf_hfe = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('rf_hfe')) {
        this.rf_hfe = initObj.rf_hfe
      }
      else {
        this.rf_hfe = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('lh_hfe')) {
        this.lh_hfe = initObj.lh_hfe
      }
      else {
        this.lh_hfe = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('rh_hfe')) {
        this.rh_hfe = initObj.rh_hfe
      }
      else {
        this.rh_hfe = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('lf_kfe')) {
        this.lf_kfe = initObj.lf_kfe
      }
      else {
        this.lf_kfe = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('rf_kfe')) {
        this.rf_kfe = initObj.rf_kfe
      }
      else {
        this.rf_kfe = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('lh_kfe')) {
        this.lh_kfe = initObj.lh_kfe
      }
      else {
        this.lh_kfe = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('rh_kfe')) {
        this.rh_kfe = initObj.rh_kfe
      }
      else {
        this.rh_kfe = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('lf_foot')) {
        this.lf_foot = initObj.lf_foot
      }
      else {
        this.lf_foot = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('rf_foot')) {
        this.rf_foot = initObj.rf_foot
      }
      else {
        this.rf_foot = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('lh_foot')) {
        this.lh_foot = initObj.lh_foot
      }
      else {
        this.lh_foot = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('rh_foot')) {
        this.rh_foot = initObj.rh_foot
      }
      else {
        this.rh_foot = new Array(16).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ForwardKinematics
    // Check that the constant length array field [lf_haa] has the right length
    if (obj.lf_haa.length !== 16) {
      throw new Error('Unable to serialize array field lf_haa - length must be 16')
    }
    // Serialize message field [lf_haa]
    bufferOffset = _arraySerializer.float64(obj.lf_haa, buffer, bufferOffset, 16);
    // Check that the constant length array field [rf_haa] has the right length
    if (obj.rf_haa.length !== 16) {
      throw new Error('Unable to serialize array field rf_haa - length must be 16')
    }
    // Serialize message field [rf_haa]
    bufferOffset = _arraySerializer.float64(obj.rf_haa, buffer, bufferOffset, 16);
    // Check that the constant length array field [lh_haa] has the right length
    if (obj.lh_haa.length !== 16) {
      throw new Error('Unable to serialize array field lh_haa - length must be 16')
    }
    // Serialize message field [lh_haa]
    bufferOffset = _arraySerializer.float64(obj.lh_haa, buffer, bufferOffset, 16);
    // Check that the constant length array field [rh_haa] has the right length
    if (obj.rh_haa.length !== 16) {
      throw new Error('Unable to serialize array field rh_haa - length must be 16')
    }
    // Serialize message field [rh_haa]
    bufferOffset = _arraySerializer.float64(obj.rh_haa, buffer, bufferOffset, 16);
    // Check that the constant length array field [lf_hfe] has the right length
    if (obj.lf_hfe.length !== 16) {
      throw new Error('Unable to serialize array field lf_hfe - length must be 16')
    }
    // Serialize message field [lf_hfe]
    bufferOffset = _arraySerializer.float64(obj.lf_hfe, buffer, bufferOffset, 16);
    // Check that the constant length array field [rf_hfe] has the right length
    if (obj.rf_hfe.length !== 16) {
      throw new Error('Unable to serialize array field rf_hfe - length must be 16')
    }
    // Serialize message field [rf_hfe]
    bufferOffset = _arraySerializer.float64(obj.rf_hfe, buffer, bufferOffset, 16);
    // Check that the constant length array field [lh_hfe] has the right length
    if (obj.lh_hfe.length !== 16) {
      throw new Error('Unable to serialize array field lh_hfe - length must be 16')
    }
    // Serialize message field [lh_hfe]
    bufferOffset = _arraySerializer.float64(obj.lh_hfe, buffer, bufferOffset, 16);
    // Check that the constant length array field [rh_hfe] has the right length
    if (obj.rh_hfe.length !== 16) {
      throw new Error('Unable to serialize array field rh_hfe - length must be 16')
    }
    // Serialize message field [rh_hfe]
    bufferOffset = _arraySerializer.float64(obj.rh_hfe, buffer, bufferOffset, 16);
    // Check that the constant length array field [lf_kfe] has the right length
    if (obj.lf_kfe.length !== 16) {
      throw new Error('Unable to serialize array field lf_kfe - length must be 16')
    }
    // Serialize message field [lf_kfe]
    bufferOffset = _arraySerializer.float64(obj.lf_kfe, buffer, bufferOffset, 16);
    // Check that the constant length array field [rf_kfe] has the right length
    if (obj.rf_kfe.length !== 16) {
      throw new Error('Unable to serialize array field rf_kfe - length must be 16')
    }
    // Serialize message field [rf_kfe]
    bufferOffset = _arraySerializer.float64(obj.rf_kfe, buffer, bufferOffset, 16);
    // Check that the constant length array field [lh_kfe] has the right length
    if (obj.lh_kfe.length !== 16) {
      throw new Error('Unable to serialize array field lh_kfe - length must be 16')
    }
    // Serialize message field [lh_kfe]
    bufferOffset = _arraySerializer.float64(obj.lh_kfe, buffer, bufferOffset, 16);
    // Check that the constant length array field [rh_kfe] has the right length
    if (obj.rh_kfe.length !== 16) {
      throw new Error('Unable to serialize array field rh_kfe - length must be 16')
    }
    // Serialize message field [rh_kfe]
    bufferOffset = _arraySerializer.float64(obj.rh_kfe, buffer, bufferOffset, 16);
    // Check that the constant length array field [lf_foot] has the right length
    if (obj.lf_foot.length !== 16) {
      throw new Error('Unable to serialize array field lf_foot - length must be 16')
    }
    // Serialize message field [lf_foot]
    bufferOffset = _arraySerializer.float64(obj.lf_foot, buffer, bufferOffset, 16);
    // Check that the constant length array field [rf_foot] has the right length
    if (obj.rf_foot.length !== 16) {
      throw new Error('Unable to serialize array field rf_foot - length must be 16')
    }
    // Serialize message field [rf_foot]
    bufferOffset = _arraySerializer.float64(obj.rf_foot, buffer, bufferOffset, 16);
    // Check that the constant length array field [lh_foot] has the right length
    if (obj.lh_foot.length !== 16) {
      throw new Error('Unable to serialize array field lh_foot - length must be 16')
    }
    // Serialize message field [lh_foot]
    bufferOffset = _arraySerializer.float64(obj.lh_foot, buffer, bufferOffset, 16);
    // Check that the constant length array field [rh_foot] has the right length
    if (obj.rh_foot.length !== 16) {
      throw new Error('Unable to serialize array field rh_foot - length must be 16')
    }
    // Serialize message field [rh_foot]
    bufferOffset = _arraySerializer.float64(obj.rh_foot, buffer, bufferOffset, 16);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ForwardKinematics
    let len;
    let data = new ForwardKinematics(null);
    // Deserialize message field [lf_haa]
    data.lf_haa = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [rf_haa]
    data.rf_haa = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [lh_haa]
    data.lh_haa = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [rh_haa]
    data.rh_haa = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [lf_hfe]
    data.lf_hfe = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [rf_hfe]
    data.rf_hfe = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [lh_hfe]
    data.lh_hfe = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [rh_hfe]
    data.rh_hfe = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [lf_kfe]
    data.lf_kfe = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [rf_kfe]
    data.rf_kfe = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [lh_kfe]
    data.lh_kfe = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [rh_kfe]
    data.rh_kfe = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [lf_foot]
    data.lf_foot = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [rf_foot]
    data.rf_foot = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [lh_foot]
    data.lh_foot = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [rh_foot]
    data.rh_foot = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    return data;
  }

  static getMessageSize(object) {
    return 2048;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimator_msgs/ForwardKinematics';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ae408d1e170b8d8b6fa4c83a5b6670c0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[16] lf_haa
    float64[16] rf_haa
    float64[16] lh_haa
    float64[16] rh_haa
    
    float64[16] lf_hfe
    float64[16] rf_hfe
    float64[16] lh_hfe
    float64[16] rh_hfe
    
    float64[16] lf_kfe
    float64[16] rf_kfe
    float64[16] lh_kfe
    float64[16] rh_kfe
    
    float64[16] lf_foot
    float64[16] rf_foot
    float64[16] lh_foot
    float64[16] rh_foot
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ForwardKinematics(null);
    if (msg.lf_haa !== undefined) {
      resolved.lf_haa = msg.lf_haa;
    }
    else {
      resolved.lf_haa = new Array(16).fill(0)
    }

    if (msg.rf_haa !== undefined) {
      resolved.rf_haa = msg.rf_haa;
    }
    else {
      resolved.rf_haa = new Array(16).fill(0)
    }

    if (msg.lh_haa !== undefined) {
      resolved.lh_haa = msg.lh_haa;
    }
    else {
      resolved.lh_haa = new Array(16).fill(0)
    }

    if (msg.rh_haa !== undefined) {
      resolved.rh_haa = msg.rh_haa;
    }
    else {
      resolved.rh_haa = new Array(16).fill(0)
    }

    if (msg.lf_hfe !== undefined) {
      resolved.lf_hfe = msg.lf_hfe;
    }
    else {
      resolved.lf_hfe = new Array(16).fill(0)
    }

    if (msg.rf_hfe !== undefined) {
      resolved.rf_hfe = msg.rf_hfe;
    }
    else {
      resolved.rf_hfe = new Array(16).fill(0)
    }

    if (msg.lh_hfe !== undefined) {
      resolved.lh_hfe = msg.lh_hfe;
    }
    else {
      resolved.lh_hfe = new Array(16).fill(0)
    }

    if (msg.rh_hfe !== undefined) {
      resolved.rh_hfe = msg.rh_hfe;
    }
    else {
      resolved.rh_hfe = new Array(16).fill(0)
    }

    if (msg.lf_kfe !== undefined) {
      resolved.lf_kfe = msg.lf_kfe;
    }
    else {
      resolved.lf_kfe = new Array(16).fill(0)
    }

    if (msg.rf_kfe !== undefined) {
      resolved.rf_kfe = msg.rf_kfe;
    }
    else {
      resolved.rf_kfe = new Array(16).fill(0)
    }

    if (msg.lh_kfe !== undefined) {
      resolved.lh_kfe = msg.lh_kfe;
    }
    else {
      resolved.lh_kfe = new Array(16).fill(0)
    }

    if (msg.rh_kfe !== undefined) {
      resolved.rh_kfe = msg.rh_kfe;
    }
    else {
      resolved.rh_kfe = new Array(16).fill(0)
    }

    if (msg.lf_foot !== undefined) {
      resolved.lf_foot = msg.lf_foot;
    }
    else {
      resolved.lf_foot = new Array(16).fill(0)
    }

    if (msg.rf_foot !== undefined) {
      resolved.rf_foot = msg.rf_foot;
    }
    else {
      resolved.rf_foot = new Array(16).fill(0)
    }

    if (msg.lh_foot !== undefined) {
      resolved.lh_foot = msg.lh_foot;
    }
    else {
      resolved.lh_foot = new Array(16).fill(0)
    }

    if (msg.rh_foot !== undefined) {
      resolved.rh_foot = msg.rh_foot;
    }
    else {
      resolved.rh_foot = new Array(16).fill(0)
    }

    return resolved;
    }
};

module.exports = ForwardKinematics;
