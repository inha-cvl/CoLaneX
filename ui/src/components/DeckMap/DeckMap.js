import mapboxgl from 'mapbox-gl';
import Map, { GeolocateControl, Marker, NavigationControl, ScaleControl,Source, Layer } from "react-map-gl";
import type { LineLayer } from 'react-map-gl';

import "mapbox-gl/dist/mapbox-gl.css";
import { makeStyles } from "@material-ui/core";
import { useEffect, useRef, useState } from "react";

const MAPBOX_ACCESS_TOKEN =
  "pk.eyJ1Ijoia2thLW5hIiwiYSI6ImNsazd3cXB6ajBjbW0zZXByYmRzd2FtYXYifQ.F-q5aMxHrdlt6o49OcRtrw";
const MY_MAP_STYLE = "mapbox://styles/kka-na/ckrbxk1fd0z5z18ljxdswgu1s";
mapboxgl.accessToken = MAPBOX_ACCESS_TOKEN


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
  const [TLVPath, setTLVPath] = useState(null)
  const [HLVPath, setHLVPath] = useState(null)
  const [viewport, setViewport] = useState({
    latitude: 37.384553,
    longitude: 126.657895,
    pitch: 60,
    bearing: -65,
    zoom: 25,
  });

  useEffect(() => {
    if (props.HLVPose) {
      setHLVPosition({ latitude: props.HLVPose.latitude, longitude: props.HLVPose.longitude });
      if (props.main == 'hlv') {
        setViewport({ latitude: props.HLVPose.latitude, longitude: props.HLVPose.longitude, bearing: props.HLVPose.heading });
      }
    }
  }, [props.HLVPose])

  useEffect(() => {
    if(props.TLVPose){
      setTLVPosition({ latitude: props.TLVPose.latitude, longitude: props.TLVPose.longitude });
      if (props.main == 'tlv') {
        setViewport({ latitude: props.TLVPose.latitude, longitude: props.TLVPose.longitude, bearing: props.TLVPose.heading });
      }
    }
  }, [props.TLVPose])

  useEffect(() => {
    if (props.HLVPath) {
      setHLVPath(props.HLVPath);
    }
  }, [props.HLVPath]);

  useEffect(() => {
    if (props.TLVPath) {
      setTLVPath(props.TLVPath);
    }

  }, [props.TLVPath]);

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