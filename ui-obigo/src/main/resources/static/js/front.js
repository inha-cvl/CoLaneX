mapboxgl.accessToken = "pk.eyJ1Ijoia2thLW5hIiwiYSI6ImNsazd3cXB6ajBjbW0zZXByYmRzd2FtYXYifQ.F-q5aMxHrdlt6o49OcRtrw";
const styleUrl = "mapbox://styles/kka-na/ckrbxk1fd0z5z18ljxdswgu1s";

const MAP_LNG = 127.11058607387321;
const MAP_LAT = 37.39984828837143;
const MAP_LNG2 = 127.11067253000084;
const MAP_LAT2 = 37.39980936847563;

const hlv_text = ['대기', '왼쪽으로 차선 변경', '오른쪽으로 차선 변경', '  TLV가 차선 변경 수락', 'TLV가 차선 변경 거절', '종료'];
const tlv_text = ['대기', '오른쪽 차 합류 요청', '왼쪽 차 합류 요청', '  HLV가 합류 시 안전함', 'HLV가 합류 시 위험함', '종료']

let api = {
    buildings: true
};

let minZoom = 15;
let mapConfig = {
    map: {
        center: [MAP_LNG, MAP_LAT],
        zoom: 20,
        pitch: 60,
        bearing: -65
    },
    hlv: {
        origin: [MAP_LNG, MAP_LAT, 0],
        type: 'mtl',
        model: '/3d/3d-model',
        rotation: {
            x: 90,
            y: 0,
            z: 0
        },
        scale: 0.03,
        startRotation: {
            x: 0,
            y: 0,
            z: 270
        },
    },
    tlv: {
        // origin: [MAP_LNG, MAP_LAT, 0],
        origin: [MAP_LNG2, MAP_LAT2, 0],
        type: 'mtl',
        model: '/3d/3d-model',
        rotation: {
            x: 90,
            y: 0,
            z: 0
        },
        scale: 0.03,
        startRotation: {
            x: 0,
            y: 0,
            z: 270
        },
    },
    names: {
        compositeSource: "composite",
        compositeSourceLayer: "building",
        compositeLayer: "3d-buildings"
    }
}


let map = new mapboxgl.Map({
    container: 'map',
    style: styleUrl,
    zoom: mapConfig.map.zoom,
    center: mapConfig.map.center,
    pitch: mapConfig.map.pitch,
    bearing: mapConfig.map.bearing,
    antialias: true // create the gl context with MSAA antialiasing, so custom layers are antialiased
});

window.tb = new Threebox(
    map,
    map.getCanvas().getContext('webgl'), {
        realSunlight: true,
        enableSelectingObjects: true,
        enableDraggingObjects: true,
        enableRotatingObjects: true,
        enableTooltips: true
    }
);

let vehicle,vehicle2;

let vehicleArr = [];

let modeId = 'hlvMode';

//HLV , TLV 클릭시  처리
const selectMode = function(e) {
    const id = e.id;
    modeId = id;
    if(id == 'hlvMode') {
        e.className = 'title-select select';
        getID('tlvMode').className = 'title-select';
        getID('tlvBody').style.display = 'none';
        getID('hlvBody').style.display = 'block';
    }
    else if(id =='tlvMode') {
        e.className = 'title-select select';
        getID('hlvMode').className = 'title-select';
        getID('hlvBody').style.display = 'none';
        getID('tlvBody').style.display = 'block';
    }
}


//3D 차량 렌더링 함수
function createCustomLayer(layerName) {
    initLine();
    let customLayer3D = {
        id: layerName,
        type: 'custom',
        renderingMode: '3d',
        onAdd: function(map, gl) {
            let options = {
                type: mapConfig.hlv.type, //model type
                obj: mapConfig.hlv.model + '.obj', //model .obj url
                mtl: mapConfig.hlv.model + '.mtl', //model .mtl url
                units: 'meters', // in meters
                scale: mapConfig.hlv.scale, //x3 times is real size for this model
                rotation: mapConfig.hlv.rotation, //default rotation
                anchor: 'auto'
            }
            let options2 = {
                type: mapConfig.tlv.type, //model type
                obj: mapConfig.tlv.model + '.obj', //model .obj url
                mtl: mapConfig.tlv.model + '.mtl', //model .mtl url
                units: 'meters', // in meters
                scale: mapConfig.tlv.scale, //x3 times is real size for this model
                rotation: mapConfig.tlv.rotation, //default rotation
                anchor: 'auto'
            }
            tb.loadObj(options, function(model) {
                vehicleArr['hlv'] = model.setCoords(mapConfig.hlv.origin);
                vehicleArr['hlv'].setRotation(mapConfig.hlv.startRotation);

                vehicleArr['hlv'].addTooltip("HLV", true, mapConfig.hlv.anchor, true, 2);

                vehicleArr['hlv'].renderOrder = 999;
                vehicleArr['hlv'].onBeforeRender = function( renderer ) { renderer.clearDepth(); };
                tb.add(vehicleArr['hlv']);
            });
            tb.loadObj(options2, function(model) {
                vehicleArr['tlv'] = model.setCoords(mapConfig.tlv.origin);
                vehicleArr['tlv'].setRotation(mapConfig.tlv.startRotation);
                vehicleArr['tlv'].addTooltip("TLV", true, mapConfig.tlv.anchor, true, 2);
                tb.add(vehicleArr['tlv']);
            });
        },
        render: function(gl, matrix) {
            tb.update();
        }
    };
    return customLayer3D;

};

function easing(t) {
    return t * (20 - t);
}

map.on('style.load', function() {

    let l = mapConfig.names.compositeLayer;
    if (api.buildings) {
        if (!map.getLayer(l)) {
            map.addLayer(createCompositeLayer(l));
        }
    }

    map.addLayer(createCustomLayer('vehicle-layer'), 'waterway-label');

    map.getCanvas().focus();
    printLocation();

    map.moveLayer( 'hlv-line', 'vehicle-layer');
    map.moveLayer('tlv-line', 'vehicle-layer');
});

//맵 정보
function createCompositeLayer(layerId) {
    let layer = {
        'id': layerId,
        'source': mapConfig.names.compositeSource,
        'source-layer': mapConfig.names.compositeSourceLayer,
        'filter': ['==', 'extrude', 'true'],
        'type': 'fill-extrusion',
        'minzoom': minZoom,
        'paint': {
            'fill-extrusion-color': [
                'case',
                ['boolean', ['feature-state', 'select'], false],
                "red",
                ['boolean', ['feature-state', 'hover'], false],
                "lightblue",
                '#aaa'
            ],
            'fill-extrusion-height': [
                'interpolate',
                ['linear'],
                ['zoom'],
                minZoom,
                0,
                minZoom + 0.05,
                ['get', 'height']
            ],
            'fill-extrusion-base': [
                'interpolate',
                ['linear'],
                ['zoom'],
                minZoom,
                0,
                minZoom + 0.05,
                ['get', 'min_height']
            ],
            'fill-extrusion-opacity': 1
        }
    };
    return layer;
}


const getBearing = function(currentLng, currentLat, destinationLng, destinationLat) {
    const x = Math.cos(destinationLat) * Math.sin(destinationLng - currentLng);
    const y = Math.cos(currentLat) * Math.sin(destinationLat) - Math.sin(currentLat) * Math.cos(destinationLat) * Math.cos(destinationLng - currentLng);
    const bearing = Math.atan2(x, y);
    return (bearing * 180 / Math.PI + 360) % 360;
}




//라인 초기 시작
const initLine = function() {
    map.addSource('hlv-line', {
        'type': 'geojson',
        'data': {
            'type': 'Feature',
            'properties': {},
            'geometry': {
                'type': 'LineString',
                'coordinates': [
                    [127.10237920911715, 37.39987196235322],
                    [127.11058607387321, 37.39984828837143]
                ]
            }
        }
    });
    map.addLayer({
        'id': 'hlv-line',
        'type': 'line',
        'source': 'hlv-line',
        'layout': {
            'line-join': 'round',
            'line-cap': 'round'
        },
        'paint': {
            'line-color': '#ff75ba',
            'line-width': 10,
            'line-opacity':0.5
        }
    });
    map.addSource('tlv-line', {
        'type': 'geojson',
        'data': {
            'type': 'Feature',
            'properties': {},
            'geometry': {
                'type': 'LineString',
                'coordinates': [
                    [127.11067253000084, 37.39980936847563],
                    [127.10235520557052 , 37.39984557140154]
                ]
            }
        }
    });

    map.addLayer({
        'id': 'tlv-line',
        'type': 'line',
        'source': 'tlv-line',
        'layout': {
            'line-join': 'round',
            'line-cap': 'round'
        },
        'paint': {
            'line-color': '#5cdcff',
            'line-width': 10,
            'line-opacity':0.5
        }
    });

}

let vehicleData = {};

//ROS 차량 위치 변경 함수
const updateVehicle = function(id, data) {

    vehicleArr[id].setCoords([data.y, data.x]);
    vehicleArr[id].setRotation(data.z-90);
    if (modeId == 'hlvMode' && id =='hlv') {
        let options = {
            center: [data.y, data.x],
            bearing: data.z + 90,
            easing: easing
        };
        map.jumpTo(options);
    }else if (modeId == 'tlvMode' && id =='tlv') {
        let options = {
            center: [data.y, data.x],
            bearing: data.z + 90,
            easing: easing
        };
        map.jumpTo(options);
    }
}

//ROS 업데이트 라인 함수
const updateLine = function(id, data) {
    map.getSource(id).setData(data);
}


//ROS 시스템 메시지 함수
const updateSystem = function(id,data) {
    const state = data[0];
    const gps = data[1];
    const v2x = data[2];
    const latency = data[3];
    const speed = data[4];
    if(modeId == id) {
        getID('latency').innerText = latency.toFixed(2);
        getID('speed').innerText = speed.toFixed(3);
        updateIcon('gpsStatus', gps, 'gps');
        updateIcon('v2xStatus', v2x, 'v2x');
        getID('hlvMessageTxt').innerText = hlv_text[state];
    }
    else {
        getID('latency').innerText = latency.toFixed(2);
        getID('speed').innerText = speed.toFixed(3);
        updateIcon('gpsStatus', gps, 'gps');
        updateIcon('v2xStatus', v2x, 'v2x');
        getID('tlvMessageTxt').innerText = tlv_text[state];
    }
}

const updateIcon = function(id, status, icon) {

    const prefix = '/images/';
    if(status == 1) {
        getID(id).src = prefix+icon+'.png';
    }
    else {
        getID(id).src = prefix+icon+'_off.png';
    }
}


//hlv 클릭 이벤트
const setHlvClick = function(signalData) {
    const interval = setInterval(() => {
        hlvSignalTopic.publish({ data: signalData });
    }, 200);

    setTimeout(() => {
        clearInterval(interval);
    }, 10000);
}


//tlv 클릭 이벤트
const setTlvClick = function(signalData) {
    const interval = setInterval(() => {
        tlvSignalTopic.publish({ data: signalData });
    }, 200);
    setTimeout(() => {
        clearInterval(interval);
    }, 10000);
}






















//test
let idx = 0;
let idx2 = 0;
let codeArr = [
    [ 127.11058607387321, 37.39984828837143],
    [127.10237920911715, 37.39987196235322]
];


let codeArr2 = [
    [127.11067253000084, 37.39980936847563],
    [127.10235520557052 , 37.39984557140154]
]

const moveEvent = function(id,data) {
    let vehicleObj = vehicleArr[id];

    if(typeof(vehicleObj) == "undefined" || typeof data == "undefined") return;
    const lat = vehicleObj.coordinates[1];
    const lng = vehicleObj.coordinates[0];
    const arrLat = data.lat;
    const arrLng = data.lng;
    let deg = getBearing(lng,lat,arrLng,arrLat);
    let delay = 16.6;
    let result = [];
    vehicleObj.setRotation(-deg);
    const distance = turf.distance([lng,lat], [arrLng, arrLat], { units: 'meters' });
    let steps = Math.round(distance / 10 );
    for (let j = 1; j <= steps; j++) {
        const interpolatedPoint = turf.along(turf.lineString([[lng,lat],[arrLng, arrLat]]), distance /  steps * j, { units: 'meters' });
        result.push(interpolatedPoint.geometry.coordinates)
    }
    for(let i=0; i < result.length-10; i++){
        delay += 16.6;
        setTimeout(async () => {
            vehicleObj.setCoords(result[i]);
            let options = {
                center: vehicleObj.coordinates,
                bearing: deg,
                easing: easing
            };
            if(id =='vehicle') {
                // if(idx2 == 1) {
                //     map.setPaintProperty('route', 'line-color', 'red');
                //     idx2 = 0;
                // }
                // else if(idx2 == 0){
                //     map.setPaintProperty('route', 'line-color', 'blue');
                //     idx2 = 1;
                // }

                map.jumpTo(options);
            }
        }, delay);
    }
}



const test = function() {
    let data = {};
    let data2 = {};
    if(idx == 1) {
        data['lng'] = codeArr[0][0];
        data['lat'] = codeArr[0][1];

        data2['lng'] = codeArr2[0][0];
        data2['lat'] = codeArr2[0][1];
        idx = 0;
    }
    else if(idx == 0) {
        data['lng'] = codeArr[1][0];
        data['lat'] = codeArr[1][1];

        data2['lng'] = codeArr2[1][0];
        data2['lat'] = codeArr2[1][1];
        idx = 1;
    }``

    moveEvent('vehicle', data);
    moveEvent('vehicle2', data2);
}



function printLocation() {
    map.on('contextmenu', (e) => {
        console.log('Clicked location:', e.lngLat.lng ,',',e.lngLat.lat);
    });
}


