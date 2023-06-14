import React, { useState } from "react";
import { Grid, Button, Card, CardContent } from "@material-ui/core";
import ROSLIB from "roslib";
import TLVStyles from "./TLVStyles"
import DeckMap from "../../components/DeckMap/DeckMap";
import Sensor from "../../components/Layout/Sensor";

const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
const tlvSystemTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/tlv_system",
  messageType: "std_msgs/Float32MultiArray",
});

const TLV = () => {
  const classes = TLVStyles();
  const [signalClasses, setSignalClasses] = useState([classes.basic50, classes.basic50]);
  const [system, setSystem] = useState([0, 0, 0, 0]);
  const [messages, setMessages] = useState([
    "State",
    "Safe",
    "Dangerous",
  ]);
  const [messageIdx, setMessageIdx] = useState(1);
  tlvSystemTopic.subscribe(function (message) {
    setSystem([message.data[0], message.data[1],message.data[2], message.data[3]]);
  });

  const okClick = () => {
    setSignalClasses([classes.purple50, classes.basic50]);
  };
  const noClick = () => {
    setSignalClasses([classes.basic50, classes.purple50]);
  };
  return (
    <div>
      <Grid container  className={classes.map}>
      <Grid item xs={6}  >
        <DeckMap />
      </Grid>
      <Grid item xs={6}>
        <Grid container >
            <Grid item xs={6} >
            <Button
                className={signalClasses[0]}
                onClick={okClick}
              >○</Button>
          </Grid>
            <Grid item xs={6}>
              <Button
                className={signalClasses[1]}
                onClick={noClick}
              >×</Button>
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

export default TLV;