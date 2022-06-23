function init() {
    console.log("Init");
    // Connect to ROS.
    var ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    // Create the main viewer.
    var viewer = new ROS2D.Viewer({
      divID : 'nav',
      width : 750,
      height : 800
    });

    // Setup the nav client.
    var nav = NAV2D.OccupancyGridClientNav({
      ros : ros,
      rootObject : viewer.scene,
      viewer : viewer,
      serverName : '/move_base_node'
    });

  }