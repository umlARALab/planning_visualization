// ws://192.168.10.5:9090

var app = new Vue({
    el: '#app',
    // computed values
    computed: {
        ws_address: function() {
            return `${this.rosbridge_address}`
        },
    },
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        topic: '/stretch/joint_states',
        msg: null,
        rosbridge_address: 'ws://192.168.10.5:9090',
        port: '9090',
        // subscriber for joint states
        joint_state_data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        step: [0.05, 0.1, 0.5, 0.5, 0.5, 0.25, 0.4],
        // action goal
        // goal: null
    },
    // helper methods to connect to ROS
    methods: {
        connect: function() {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.ws_address
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false
                var joint_sub = new ROSLIB.Topic({
                    ros: this.ros,
                    name: this.topic,
                    messageType: 'sensor_msgs/msg/JointState'
                })
                joint_sub.subscribe(function(msg) {
                    joint_state_data = msg.position
                })
                this.setCamera()
                // this.setCamera2()
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                document.getElementById('divCamera').innerHTML = ''
                // document.getElementById('divCamera2').innerHTML = ''
            })
        },
        disconnect: function() {
            this.ros.close()
            this.goal = null
        },
        sendGoal: function(name, new_pos) {
            let actionClient = new ROSLIB.Action({
                ros: this.ros,
                name: '/stretch_controller/follow_joint_trajectory',
                actionType: 'control_msgs/action/FollowJointTrajectory'
            })

            let goal =  new ROSLIB.ActionGoal ({
                trajectory: {joint_names: [name], points: [{position: [new_pos], time_from_start: {sec: 2}}]}
            })
            
            // this.goal = new ROSLIB.goal({
            //     actionClient: actionClient,
            //     goalMessage: this.action.goal.trajectory
            // })

            // this.goal.on('feedback', (feedback) => {
            //     this.action.feedback = feedback
            // })
            // this.goal.on('result', (result) => {
            //     this.action.result = result
            // })

            let goal_id = actionClient.sendGoal(goal)
        },
        cancelGoal: function() {
            this.goal.cancel()
        },
        getIndex: function(joint_name) {
            switch (joint_name) {
                case 'wrist_extension':
                    return 0
                case 'joint_lift':
                    return 1
                case 'joint_head_pan':
                    return 6
                case 'joint_head_tilt':
                    return 7
                case 'joint_wrist_yaw':
                    return 8
                case 'joint_wrist_pitch':
                    return 9
                case 'joint_wrist_roll':
                    return 10
                case 'joint_gripper_finger_left':
                case 'joint_gripper_finger_right':
                    return 11
            }
        },
        // navigation
        setTopic: function() {
            this.callNavSrv()
            this.topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/stretch/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
        },
        forward: function() {
            this.callNavSrv()
            this.message = new ROSLIB.Message({
                linear: { x: 1, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        stop: function() {
            this.callNavSrv()
            this.message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        backward: function() {
            this.callNavSrv()
            this.message = new ROSLIB.Message({
                linear: { x: -1, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        turnLeft: function() {
            this.callNavSrv()
            this.message = new ROSLIB.Message({
                linear: { x: 0.0, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0.5, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        turnRight: function() {
            this.callNavSrv()
            this.message = new ROSLIB.Message({
                linear: { x: 0.0, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: -0.5, },
            })
            this.setTopic()
            this.topic.publish(this.message)
        },
        // manipulation
        // joint state names [wrist_extension, joint_lift, joint_arm_l3, joint_arm_l2, joint_arm_l1, joint_arm_l0, 
        //                    joint_head_pan, joint_head_tilt, joint_wrist_yaw, joint_wrist_pitch, joint_wrist_roll,
        //                    joint_gripper_finger_left, joint_gripper_finger_right]
        // step [wrist_extension, joint_lift, joint_head_pan, joint_head_tilt, joint_wrist_yaw, joint_wrist_pitch, joint_wrist_rol, joint_gripper
        // ]
        armLiftUp: function() {
            this.callManSrv()
            let name = this.getIndex('joint_lift')
            let new_pos = joint_state_data[name] + (this.step[1])
            this.sendGoal('joint_lift', new_pos)
        },
        armLiftDown: function() {
            this.callManSrv()
            let name = this.getIndex('joint_lift')
            let new_pos = joint_state_data[name] + (this.step[1] * -1)
            this.sendGoal('joint_lift', new_pos)
        },
        setCamera: function() {
            let host = '192.168.10.5'
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                port: 8080,
                width: 640,
                height: 480,
                topic: '/camera/color/image_raw',
                ssl: false,
            })
        },
        callNavSrv: function() {
            let nav_srv = new ROSLIB.Service({
                ros: this.ros,
                name: '/switch_to_navigation_mode',
                serviceType: 'std_srvs/srv/Trigger',
            })
            var req = {};
            nav_srv.callService(req, (result) => {
                    console.log('nav mode')
                    console.log(result)
                }, (error) => {
                    console.error(error)
                }
            )
        },
        callManSrv: function() {
            let nav_srv = new ROSLIB.Service({
                ros: this.ros,
                name: '/switch_to_position_mode',
                serviceType: 'std_srvs/srv/Trigger',
            })
            var req = {};
            nav_srv.callService(req, (result) => {
                    console.log('pos mode')
                    console.log(result)
                }, (error) => {
                    console.error(error)
                }
            )
        }
        // setCamera2: function() {
        //     // let without_wss = this.rosbridge_address.split('wss://')[1]
        //     // console.log(without_wss)
        //     // let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
        //     // console.log(domain)
        //     let host = '192.168.10.5:7000'
        //     let viewer = new MJPEGCANVAS.Viewer({
        //         divID: 'divCamera2',
        //         host: host,
        //         width: 480,
        //         height: 640,
        //         topic: '/camera/depth/image_rect_raw',
        //         ssl: true,
        //     })
        // },
    },
    mounted() {
    },
})

