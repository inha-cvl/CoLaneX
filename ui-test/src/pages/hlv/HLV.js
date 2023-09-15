import React, { useState } from "react";
import { Grid, Button, Card, CardContent } from "@material-ui/core";
import ROSLIB from "roslib";

import HLVStyles from "./HLVStyles"
import DeckMap from "../../components/DeckMap/DeckMap";
import Sensor from "../../components/Layout/Sensor";

const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
const hlvSystemTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/hlv_system",
  messageType: "std_msgs/Float32MultiArray",
});
const hlvSignalTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/hlv_signal",
  messageType: "std_msgs/Int8"
});
const hlvPoseTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/car/hlv_pose',
  messageType: 'geometry_msgs/Pose'
});
const hlvPathTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/planning/hlv_geojson',
  messageType: 'std_msgs/String',
});

const tlvPoseTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/v2x/tlv_pose',
  messageType: 'geometry_msgs/Pose'
});
const tlvPathTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/planning/tlv_geojson',
  messageType: 'std_msgs/String',
});


const HLV = () => {
  const classes = HLVStyles();
  const [signalClasses, setSignalClasses] = useState([classes.basic50, classes.basic50]);
  const [system, setSystem] = useState([0, 0, 0, 0]);
  const [messages, setMessages] = useState([
    "Wait",
    "Attempt to Merge\ninto the Left Lane",
    "Attempt to Merge\ninto the Right Lane",
    "TLV Accepts Request\nto Join Lane",
    "TLV Rejects Request\nto Join Lane",
    "Over",
  ]);
  const [messageIdx, setMessageIdx] = useState(0);
  const [HLVPose, setHLVPose] = useState({
    latitude: 37.384553,
    longitude: 126.657895,
    heading: 0,
    speed: 0,
  });
  const [TLVPose, setTLVPose] = useState({
    latitude: 37.384563,
    longitude: 126.657900,
    heading: 0,
    speed: 0,
  });
  const [TLVPath, setTLVPath] = useState('')
  const [HLVPath, setHLVPath] = useState('')

  hlvSystemTopic.subscribe(function (message) {
    setMessageIdx(message.data[0]);
    setSystem([message.data[1], message.data[2], message.data[3], message.data[4]]);
    if (message.data[0] === 3) {
      hlvSignalTopic.publish({ data: 0 });
      setSignalClasses([classes.basic50, classes.basic50]);
    }
  });

  hlvPoseTopic.subscribe(function (message) {
    setHLVPose({ latitude: message.position.x, longitude: message.position.y, heading: message.position.z+180, speed:message.orientation.x });
  });

  hlvPathTopic.subscribe(function (message) {
    const geoJsonObject = JSON.parse(message.data);
    setHLVPath(geoJsonObject)
  });

  tlvPoseTopic.subscribe(function (message) {
    setTLVPose({ latitude: message.position.x, longitude: message.position.y, heading: message.position.z+180, speed:message.orientation.x });
  });

  tlvPathTopic.subscribe(function (message) {
    const geoJsonObject = JSON.parse(message.data);
    setTLVPath(geoJsonObject)
  });


  const handleClick = (signalData) => {
    setSignalClasses(signalData === 1 ? [classes.purple50, classes.basic50] : [classes.basic50, classes.purple50]);
  
    const interval = setInterval(() => {
      hlvSignalTopic.publish({ data: signalData });
    }, 200);
  
    setTimeout(() => {
      clearInterval(interval);
    }, 10000);
  };

  const leftClick = () => {
    handleClick(1);
  };
  const rightClick = () => {
    handleClick(2);
  };

  return (
    <div>
      <Grid container  className={classes.map}>
        <Grid item xs={6}  >
          <DeckMap main='hlv' HLVPose={HLVPose} HLVPath={HLVPath} TLVPose={TLVPose} TLVPath={TLVPath} />
        </Grid>
        <Grid item xs={6}>
          <Grid container>
            <Grid item xs={6} >
              <Button
                className={signalClasses[0]}
                onClick={leftClick}
              >⇦</Button>
            </Grid>
            <Grid item xs={6}>
              <Button
                className={signalClasses[1]}
                onClick={rightClick}
              >⇨</Button>
            </Grid>
            <Grid item xs={12}>
              <Card className={classes.basic_card}>
                <CardContent>
                  {messages[messageIdx]}
                </CardContent>
              </Card>
            </Grid>
            <Sensor system={system} /> 
        </Grid>
      </Grid>
    </Grid>
    </div>
    
  );
};

export default HLV;