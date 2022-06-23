let vueApp = new Vue({
    el: "#vueApp",
    data: {
        // ros connection
        ros: null,
        rosbridge_address: 'ws://192.168.3.25:9090',
        connected: false,
        // page content
        menu_title: 'Connection',
        // dragging data
        dragging: false,
        x: 'no',
        y: 'no',
        old_x: 0,
        old_y: 0,
        dragCircleStyle: {
            margin: '0px',
            top: '0px',
            left: '0px',
            display: 'none',
            width: '75px',
            height: '75px',
        },
        // joystick valules
        joystick: {
            vertical: 0,
            horizontal: 0,
        }
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


        startDrag() {
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            console.log("Stopped Dragging")
            this.resetJoystickVals()
        },
        doDrag(event) {
            if (this.dragging) {
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let maxTop = minTop + 200
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let maxLeft = minLeft + 200
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }
        },
        setJoystickVals() {
            this.joystick.vertical = -1 * ((this.y / 200) - 0.5)
            this.joystick.horizontal = +1 * ((this.x / 200) - 0.5)
            this.joystick.vertical *= 2
            this.joystick.horizontal *= -2
            this.joystick.horizontal = Math.round((this.joystick.horizontal + Number.EPSILON) * 100) / 100
            this.joystick.vertical = Math.round((this.joystick.vertical + Number.EPSILON) * 100) / 100
            if (Math.abs(this.old_x - this.joystick.horizontal) > 0.1 || Math.abs(this.old_y - this.joystick.vertical) > 0.1) {
                this.old_x = this.joystick.horizontal
                this.old_y = this.joystick.vertical
                this.sendCommand()
            }

        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
            this.sendCommand()
        },
    },
    mounted() {
        // page is ready
        window.addEventListener('mouseup', this.stopDrag)
    },
})
