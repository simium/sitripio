function decodeUplink(input) {
    var data = {};
    var warnings = {};
    var events = {
      1: "setup",
      2: "interval",
      3: "motion",
      4: "button"
    };
    data.event = events[input.fPort];
    data.latitude = ((input.bytes[3] << 24) + (input.bytes[2] << 16) + (input.bytes[1] << 8) + (input.bytes[0]))/1000000.0;
    data.longitude = ((input.bytes[7] << 24) + (input.bytes[6] << 16) + (input.bytes[5] << 8) + (input.bytes[4]))/1000000.0;
    data.altitude = (input.bytes[11] << 24) + (input.bytes[10] << 16) + (input.bytes[9] << 8) + (input.bytes[8]);
    data.temperature = (input.bytes[12]);
   
    return {
      data: data,
      warnings: []
    };
  }