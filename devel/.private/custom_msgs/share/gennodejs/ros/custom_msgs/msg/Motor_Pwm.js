// Auto-generated. Do not edit!

// (in-package custom_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Motor_Pwm {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.L = null;
      this.R = null;
    }
    else {
      if (initObj.hasOwnProperty('L')) {
        this.L = initObj.L
      }
      else {
        this.L = 0;
      }
      if (initObj.hasOwnProperty('R')) {
        this.R = initObj.R
      }
      else {
        this.R = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Motor_Pwm
    // Serialize message field [L]
    bufferOffset = _serializer.int64(obj.L, buffer, bufferOffset);
    // Serialize message field [R]
    bufferOffset = _serializer.int64(obj.R, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Motor_Pwm
    let len;
    let data = new Motor_Pwm(null);
    // Deserialize message field [L]
    data.L = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [R]
    data.R = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msgs/Motor_Pwm';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4184f594ee6fa4706c2c2eca40be03fe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 L
    int64 R
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Motor_Pwm(null);
    if (msg.L !== undefined) {
      resolved.L = msg.L;
    }
    else {
      resolved.L = 0
    }

    if (msg.R !== undefined) {
      resolved.R = msg.R;
    }
    else {
      resolved.R = 0
    }

    return resolved;
    }
};

module.exports = Motor_Pwm;
