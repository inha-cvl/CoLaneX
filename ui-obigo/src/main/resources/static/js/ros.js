

const ros = new ROSLIB.Ros();


const hlvSystemTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/hlv_system',
    messageType: "std_msgs/Float32MultiArray",
});


const tlvSystemTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/tlv_system',
    messageType: "std_msgs/Float32MultiArray",
});

const hlvGeojsonTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/planning/hlv_geojson',
});

const tlvGeojsonTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/planning/tlv_geojson',
});

const hlvPoseTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/car/hlv_pose",
});


const tlvPoseTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/car/tlv_pose",
});



const hlvModeTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/mode",
    messageType: "std_msgs/Int8"
});


const hlvSignalTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/hlv_signal",
    // messageType: "std_msgs/Int8"
});

const tlvSignalTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/tlv_signal",
    messageType: "std_msgs/Int8"
});



window.addEventListener('DOMContentLoaded', function() {
    initRos();
});


const initRos = function() {
    ros.connect('ws://localhost:9090');


    hlvPoseTopic.subscribe(function (message) {
        updateVehicle('hlv' , message.position);
    });

    tlvPoseTopic.subscribe(function (message) {
        updateVehicle('tlv' , message.position);
    });


    hlvGeojsonTopic.subscribe(function (message) {
        const lineObj = JSON.parse(message.data);
        updateLine('hlv-line',lineObj);
    });

    tlvGeojsonTopic.subscribe(function (message) {
        const lineObj = JSON.parse(message.data);
        updateLine('tlv-line',lineObj);
        
    });


    hlvSystemTopic.subscribe(function (message) {
        updateSystem('hlvMode' , message.data);
    });

    tlvSystemTopic.subscribe(function (message) {
        updateSystem('hlvMode' , message.data);
    });


}



ros.on('error', function (error) {
    console.log(error);
});

// 정상 연결
ros.on('connection', function () {
    console.log('Connection made!');
});

// 연결 닫힘
ros.on('close', function () {
    console.log('Connection closed.');
});

// 수신
const rosSub = function(data) {
    console.log(data);
}