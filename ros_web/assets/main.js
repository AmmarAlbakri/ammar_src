let vueApp = new Vue({
    el: "#vueApp",
    data: {
        // ros connection
        ros: null,
        rosbridge_address: 'ws://127.0.0.1:9090',
        connected: false,
        // page content
        menu_title: 'Connection',
        viewer_map: null,
        viewer_nav: null,
        gridClient: null,
        nav: null
    },
    methods: {
        connect: function () {
            // define ROSBridge connection object
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            // define callbacks
            this.ros.on('connection', () => {
                this.connected = true
                console.log('Connection to ROSBridge established!')
            })
            this.ros.on('error', (error) => {
                console.log('Something went wrong when trying to connect')
                console.log(error)
            })
            this.ros.on('close', () => {
                this.connected = false
                console.log('Connection to ROSBridge was closed!')
            })

            viewer_map = new ROS2D.Viewer({
                divID: 'map',
                width: 600,
                height: 500
            });
            gridClient = new ROS2D.OccupancyGridClient({
                ros: this.ros,
                rootObject: viewer_map.scene,
                // continuous: true
            });

            viewer_nav = new ROS2D.Viewer({
                divID: 'nav',
                width: 600,
                height: 500
            });

            // Setup the nav client.
            nav = NAV2D.OccupancyGridClientNav({
                ros : this.ros,
                rootObject : viewer_nav.scene,
                viewer : viewer_nav,   
                serverName : '/move_base_node'
            });

            // Setup the map client.
           
            // Scale the canvas to fit to the map
            gridClient.on('change', function () {
                viewer_map.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
            });
            // nav.on('change', function () {
            //     viewer_nav.scaleToDimensions(nav.currentGrid.width, nav.currentGrid.height);
            // });
        },
        draw: function () {
            var canvas = document.querySelector('#map canvas')
            var ctx = canvas.getContext("2d");
            ctx.moveTo(0, 0);
            ctx.lineTo(200, 100);
            ctx.stroke();
        },
        disconnect: function () {
            this.ros.close()
        },
        sendCommand: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.joystick.horizontal, },
            })
            topic.publish(message)
            console.log("World!");
        },
    },
    mounted() {
        // page is ready
        window.addEventListener('mouseup', this.stopDrag)
    },
})
