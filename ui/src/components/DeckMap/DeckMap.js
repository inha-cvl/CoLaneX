import mapboxgl from 'mapbox-gl';
import Map, { GeolocateControl, Marker, NavigationControl, ScaleControl,Source, Layer } from "react-map-gl";
import type { LineLayer } from 'react-map-gl';
import ROSLIB from "roslib";

import "mapbox-gl/dist/mapbox-gl.css";
import { makeStyles } from "@material-ui/core";
import { useEffect, useRef, useState } from "react";

const MAPBOX_ACCESS_TOKEN =
  "pk.eyJ1Ijoia2thLW5hIiwiYSI6ImNsazd3cXB6ajBjbW0zZXByYmRzd2FtYXYifQ.F-q5aMxHrdlt6o49OcRtrw";
const MY_MAP_STYLE = "mapbox://styles/kka-na/ckrbxk1fd0z5z18ljxdswgu1s";
mapboxgl.accessToken = MAPBOX_ACCESS_TOKEN

const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
const hlvPositionTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/novatel/oem7/hlv_inspva',
  messageType: 'novatel_oem7_msgs/INSPVA'
});
const hlvPathTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/planning/hlv_geojson',
  messageType: 'std_msgs/String',
});


const tlvPositionTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/novatel/oem7/tlv_inspva',
  messageType: 'novatel_oem7_msgs/INSPVA'
});
const tlvPathTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/planning/tlv_geojson',
  messageType: 'std_msgs/String',
});

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

const tlvPathStyle: LineLayer = {
  id: 'TLV Path',
  type: 'line',
  "paint": {
    "line-color": "#5cdcff",
    "line-width": 7,
    "line-opacity": 0.5
  },
}
const hlvPathStyle: LineLayer = {
  id: 'THLV Path',
  type: 'line',
  "paint": {
    "line-color": "#ff75ba",
    "line-width": 7,
    "line-opacity": 0.5
  },
}

const DeckMap = (props) => {
  const classes = Styles();

  const [HLVPosition, setHLVPosition] = useState({
    latitude: 37.384553,
    longitude: 126.657895,
  });
  const [TLVPosition, setTLVPosition] = useState({
    latitude: 37.384563,
    longitude: 126.657900,
  });
  const [TLVPath, setTLVPath] = useState('')
  const [HLVPath, setHLVPath] = useState('')
  const [viewport, setViewport] = useState({
    latitude: 37.384553,
    longitude: 126.657895,
    pitch: 45,
    bearing: -65,
    zoom: 20,
  });

  hlvPositionTopic.subscribe(function (message) {
    setHLVPosition({ latitude: message.latitude, longitude: message.longitude });
    if (props.main === 'hlv') {
      setViewport({ latitude: message.latitude, longitude: message.longitude, bearing: message.azimuth })
    }
  });

  hlvPathTopic.subscribe(function (message) {
    const geoJsonObject = JSON.parse(message.data);
    setHLVPath(geoJsonObject)
  });


  tlvPositionTopic.subscribe(function (message) {
    setTLVPosition({ latitude: message.latitude, longitude: message.longitude });
    if (props.main === 'tlv') {
      setViewport({ latitude: message.latitude, longitude: message.longitude, bearing: message.azimuth })
    }
  });

  tlvPathTopic.subscribe(function (message) {
    const geoJsonObject = JSON.parse(message.data);
    setTLVPath(geoJsonObject)
  });

  return (
    <Map
        {...viewport}
        //initialViewState={viewport}
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
        <Marker {...HLVPosition}>
          <div className={classes.marker1}></div>
        </Marker>
        <Marker {...TLVPosition}>
          <div className={classes.marker2}></div>
        </Marker>
        <Source id="tlv_path" type="geojson" data={TLVPath}>
          <Layer {...tlvPathStyle}/>
        </Source>
        <Source id="hlv_path" type="geojson" data={HLVPath}>
          <Layer {...hlvPathStyle}/>
        </Source>
       
      </Map>
  );
};

export default DeckMap;