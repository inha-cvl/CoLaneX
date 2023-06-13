import React, { useEffect, useState } from "react";
import { Grid, Button, Card, CardContent, Typography } from "@material-ui/core";

import HLVStyles from "./HLVStyles"
import DeckMap from "../../components/DeckMap/DeckMap";

const HLV = () => {
  const classes = HLVStyles();
  const [styleClasses, setStyleClasses] = useState([classes.basic50, classes.basic50]);
  const [messages, setMessages] = useState([
    "State",
    "TLV Accepted",
    "TLV Rejected",
  ]);
  const [messageIdx, setMessageIdx] = useState(1);
  // useEffect(() => {
  //   let temp = styleClasses;
  //   temp[index] = state ? "purple50" : "basic50";
  //   setStyleClasses(temp);
  // }, [element.state]);


  const leftClick = () => {
    setStyleClasses([classes.purple50, classes.basic50]);
  };
  const rightClick = () => {
    setStyleClasses([classes.basic50, classes.purple50]);
  };
  return (
    <div>
      <Grid container spacing={2} className={classes.map}>
      <Grid item xs={6}  >
        <DeckMap />
      </Grid>
      <Grid item xs={6}>
        <Grid container spacing={2} className={classes.set_value}>
            <Grid item xs={6} >
            <Button
                className={styleClasses[0]}
                onClick={leftClick}
              >⇦</Button>
          </Grid>
            <Grid item xs={6}>
              <Button
                className={styleClasses[1]}
                onClick={rightClick}
              >⇨</Button>
          </Grid>
            <Grid item xs={12}>
              <Card className={classes.basic_card}>
                <CardContent sx={{align:"center"}}>
                  {messages[messageIdx]}
                </CardContent>
              </Card>
          </Grid>
        </Grid>
      </Grid>
    </Grid>
    </div>
    
  );
};

export default HLV;