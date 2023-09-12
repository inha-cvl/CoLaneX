<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8" %>
<%@ page trimDirectiveWhitespaces="true" %>
<%@ taglib prefix="c" uri="http://java.sun.com/jsp/jstl/core" %>
<%@ taglib prefix="spring" uri="http://www.springframework.org/tags" %>
<!DOCTYPE html>
<html>
<head>
	<meta charset="UTF-8" />
	<title>OBIGO Map</title>
	<!-- favicon -->
	<link href="/images/obigo_Favicon.png" rel="shortcut icon" type="image/x-icon" />
	<!-- Mapbox -->
	<meta name="viewport" content="initial-scale=1,maximum-scale=1,user-scalable=no">
<%--	<link href="https://api.mapbox.com/mapbox-gl-js/v2.15.0/mapbox-gl.css" rel="stylesheet">--%>
<%--	<script src="https://api.mapbox.com/mapbox-gl-js/v2.15.0/mapbox-gl.js"></script>--%>
	<link href="https://api.mapbox.com/mapbox-gl-js/v2.12.0/mapbox-gl.css" rel="stylesheet">
	<script src="https://api.mapbox.com/mapbox-gl-js/v2.12.0/mapbox-gl.js"></script>
	<script src="/js/roslib.min.js" defer></script>
	<script src="/js/ros.js" defer></script>
	<script src="/js/front.js" defer></script>
	<link href="/css/front.css" rel="stylesheet">
	<script src="/js/util.js" defer></script>
	<script type="module" src="/js/string.js"></script>
	<script src="https://unpkg.com/@turf/turf@6/turf.min.js" ></script> <%-- Turf : to smoothly animate a point along the distance of a line --%>
<%--	<script src="https://unpkg.com/three@0.126.0/build/three.min.js"></script>--%>
	<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
</head>

    </script>
	<script src="https://unpkg.com/three@0.126.0/examples/js/loaders/GLTFLoader.js"></script>
	<script src="https://cdn.jsdelivr.net/gh/jscastro76/threebox@v.2.2.2/dist/threebox.min.js" type="text/javascript"></script>
<%--	<link href="https://cdn.jsdelivr.net/gh/jscastro76/threebox@v.2.2.2/dist/threebox.css" rel="stylesheet">--%>
</head>
	<script>

	</script>
<body>
<%--	<h1>OBIGO Map</h1>--%>
	<div class="main">
		<div class="header"></div>
		<div id="map" class="map"></div>
		<div class="info">
			<div class="info-detail">
				<div class="info-title">
					<div id="hlvMode" class="title-select select" onclick="selectMode(this)">HLV</div>
					<div class="v-line"></div>
					<div id="tlvMode" class="title-select" onclick="selectMode(this)">TLV</div>
				</div>
				<div id="hlvBody">
					<div class="info-body">
						<div class="info-btn" onclick="setHlvClick(1)">
							<img src="/images/left-btn.png">
							Left
						</div>
						<div class="info-btn" onclick="setHlvClick(3)">
							Right
							<img src="/images/right-btn.png">
						</div>
					</div>
					<div class="info-footer">
						<div class="footer-title">
<%--							<img src="/images/icon_warning.png">--%>
							<span id="hlvMessageTxt">
							</span>
						</div>
						<div  class="footer-message">
						</div>
					</div>
				</div>
				<div id="tlvBody" style="display: none;">
					<div class="info-body">
						<div class="info-btn" onclick="setTlvClick(1)">
							Accept
						</div>
						<div class="info-btn" onclick="setTlvClick(2)">
							Reject
						</div>
					</div>
					<div class="info-footer">
						<div class="footer-title">
							<%--							<img src="/images/icon_warning.png">--%>
							<span id="tlvMessageTxt">
							</span>
						</div>
						<div  class="footer-message">
						</div>
					</div>
				</div>
			</div>
			<div class="info-detail-gps">
				<div class="iconArea">
					<div class="icon-body">
						<img id="gpsStatus" src="/images/gps.png">
						<span>
							GPS
						</span>
					</div>
					<div class="mt32"></div>
					<div class="icon-body">
						<img id="v2xStatus" src="/images/v2x.png">
						<span>
							V2X
						</span>
					</div>
				</div>
				<div class="gps-data">
					<div class="gps-title">
						Latency
					</div>
					<div class="gps-body">
						<div id="latency" class="gps-value">0.00000</div>
						<div class="gps-unit">ms</div>
					</div>

				</div>
				<div class="gps-data2">
					<div class="gps-title">
						Speed
					</div>
					<div class="gps-body">
						<div id="speed" class="gps-value">100.000</div>
						<div class="gps-unit">Mbps</div>
					</div>
				</div>

			</div>
		</div>
	</div>
<%--	<div class="overlay">--%>
<%--		<button id="replay">Replay</button>--%>
<%--	</div>--%>

</body>
</html>