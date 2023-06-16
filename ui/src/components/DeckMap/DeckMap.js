import mapboxgl from 'mapbox-gl';
import Map, { GeolocateControl, Marker, NavigationControl, ScaleControl } from "react-map-gl";
import "mapbox-gl/dist/mapbox-gl.css";
import { makeStyles } from "@material-ui/core";
import { useState } from "react";

const MAPBOX_ACCESS_TOKEN =
  "pk.eyJ1Ijoia2thLW5hIiwiYSI6ImNsaXR0aW43cDFxNngza29jaXgyZWZsa3EifQ.jT-5SMVtHtPD4FuBfuvBpg";
const MY_MAP_STYLE = "mapbox://styles/kka-na/ckrbxk1fd0z5z18ljxdswgu1s";
mapboxgl.accessToken = MAPBOX_ACCESS_TOKEN

// const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
// const gpsTopic = new ROSLIB.Topic({
//   ros: ros,
//   name: "/gps",
//   messageType: "geometry_msgs/Vector3",
// });

const Styles = makeStyles((theme) => ({
  marker1: {
    backgroundColor: "#ff75ba",
    width: "1rem",
    height: "1rem",
    borderRadius: "50%",
  },
  marker2: {
    backgroundColor: "#5cdcff",
    width: "1rem",
    height: "1rem",
    borderRadius: "50%",
  },
}));

export default function DeckMap() {
  const classes = Styles();

  const [position1, setPosition1] = useState({
    latitude: 37.384553,
    longitude: 126.657895,
  });
  const [position2, setPosition2] = useState({
    latitude: 37.384563,
    longitude: 126.657900,
  });
  const [viewport, setViewport] = useState({
    latitude: 37.384553,
    longitude: 126.657895,
    pitch: 45,
    bearing: 120,
    zoom: 20,
  });

  // if (!isSub && props.sub) {
  //   setIsSub(true);
  //   gpsTopic.subscribe(function (message) {
  //     setPosition({ latitude: message.x, longitude: message.y });
  //     setViewport({
  //       latitude: message.x,
  //       longitude: message.y,
  //       pitch: viewport.pitch,
  //       bearing: -65,
  //       zoom: viewport.zoom,
  //     });
  //   });
  // }
  // if (isSub && !props.sub) {
  //   setIsSub(false);
  //   gpsTopic.unsubscribe();
  // }


  return (
      <Map
        initialViewState={viewport}
        mapStyle={MY_MAP_STYLE}
        mapboxApiAccessToken={MAPBOX_ACCESS_TOKEN}
        onViewportChange={(viewport) => {
          setViewport(viewport);
        }}
        attributionControl={false}
      >
        <GeolocateControl />
        <NavigationControl  />
        <ScaleControl />
        <Marker {...position1}>
          <div className={classes.marker1}></div>
        </Marker>
        <Marker {...position2}>
          <div className={classes.marker2}></div>
        </Marker>
       
      </Map>
  );
}