import React, {useEffect, useState} from "react";
import { Grid, Card } from "@material-ui/core";
import SensorStyles from "./SensorStyles";

export default function Sensor(props) {
    const classes = SensorStyles();
    const [sensorClasses, setSensorClasses] = useState([classes.basic30, classes.basic30]);
    const [latency, setLatency] = useState(0);
    const [speed, setSpeed] = useState(0);

    useEffect(() => {
        let temp = sensorClasses;
        temp[0] = props.system[0] === 1 ? classes.basic30 : classes.red30;
        temp[1] = props.system[1] === 1 ? classes.basic30 : classes.red30
        setSensorClasses(temp);
        setLatency(props.system[2]);
        setSpeed(props.system[3]);
    }, [props.system])

    return (
        <Grid container>
            <Grid item xs={3}>
              <Card
                className={sensorClasses[0]}
              >GPS</Card>
            </Grid>
            <Grid item xs={3}>
              <Card
                className={sensorClasses[1]}
              >V2X</Card>
            </Grid>
            <Grid item xs={6}>
              <Grid container>
                <Grid item xs={6} className={classes.label}>
                  <p>Latency</p>
                </Grid>
                <Grid item xs={6} className={classes.label}>
                  <p>Speed</p>
                </Grid>
                <Grid item xs={6}>
                  <Card
                    className={classes.small_card}
                  >{latency}ms</Card>
                </Grid>
                <Grid item xs={6}>
                  <Card
                    className={classes.small_card}
                  >{speed}Mbps</Card>
                </Grid>
              </Grid>
            </Grid>            
        </Grid>
    )
}