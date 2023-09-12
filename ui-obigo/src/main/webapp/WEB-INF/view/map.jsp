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
<%--	<meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate" />--%>
	<link href="https://api.mapbox.com/mapbox-gl-js/v2.12.0/mapbox-gl.css" rel="stylesheet">
	<script src="https://api.mapbox.com/mapbox-gl-js/v2.12.0/mapbox-gl.js"></script>
	<script src="/js/apexcharts.js" defer></script>
	<script src="/js/roslib.min.js" defer></script>
	<script src="/js/ros.js" defer></script>
	<script src="https://unpkg.com/@turf/turf@6/turf.min.js" ></script> <%-- Turf : to smoothly animate a point along the distance of a line --%>
	<style>
		html, body {
			margin: 0;
			height: 100%;
			overflow: hidden;
		}
		body { margin: 0; padding: 0;}
		#map { width: 80%; height: 100%; float: left}
		.carInfo { width: 20%; height: 100%; float: left}
		.infoControl {
			width: 100%;
			height: 10%;
			text-align: center;
			border: 1px solid #000000;
		}
		.infoBody {
			width: 100%;
			height: 90%;
			border: 1px solid #000000;
		}
		.detailInfo {
			width: 100%;
			height: 50%;
			border: 1px solid #000000;
			float: left;
		}
		.detailLabel {
			width: 20%;
			float: left;
		}
		.detailP {
			width: 80%;
			float: left;
		}
		.logDetail {
			width: 100%;
			height: 204px;
		}



		.find-effect {
			width: 100px;
			height: 100px;
			border-radius: 50%;
			background-color: #000000;
			text-align: center;
			margin: -35px;
			opacity: 0.2;
		}


		.pulse-effect {
			width: 100px;
			height: 100px;
			margin-top: -35px;
			margin-left: -35px;
			border-radius: 50%;
			background-color: #ff7b7b;
			animation: pulse 1s infinite;
		}

		@keyframes pulse {
			0% {
				transform: scale(0.5);
				opacity: 1;
			}
			100% {
				transform: scale(2);
				opacity: 0.3;
			}
		}
	</style>
</head>
<body>
<%--	<h1>OBIGO Map</h1>--%>
	<div id="map"></div>
	<div class="carInfo" >
		<div class="infoControl">
			<input id ='startCarAll' type="button" value="start"/>
			<input id ='stopCarAll' type="button" value="stop" disabled/>
			<input id ='changeLine' type="button" value="changeLine" disabled/>
		</div>
		<div class="infoBody">
			<div class="detailInfo">
				<div>
					<label class="detailLabel">ID : </label>
					<p id="car1Id" class="detailP" >CAR1</p>
				</div>
				<div>
					<label class="detailLabel">SPEED</label>
					<p id="car-marker-1_speed" class="detailP" >10km/h</p>
					<input id="car-marker-1_speedLimit" type="number" min="1" value="6"/>
					<input id="car-marker-1_speedSet" type="button" value="setSpeed">
				</div>
				<div>
					<label class="detailLabel" >Stuats</label>
					<select id ="car-marker-1_carType">
						<option value="1">일반</option>
						<option value="2">긴급</option>
					</select>
				</div>
				<div>
					<label class="detailLabel" >log</label>
					<textarea id ="car-marker-1_log" class="logDetail" value="">
					</textarea>
				</div>
			</div>
			<div id="car2" class="detailInfo">
				<div>
					<label class="detailLabel">ID : </label>
					<p id="car2Id" class="detailP" >CAR2</p>
				</div>
				<div>
					<label class="detailLabel">SPEED</label>
					<p id="car-marker-2_speed" class="detailP" >10km/h</p>
					<input id="car-marker-2_speedLimit" type="number" min="1" value="5"/>
					<input id="car-marker-2_speedSet" type="button" value="setSpeed">
				</div>
				<div>
					<label class="detailLabel" >Stuats</label>
					<select id ="car-marker-2_carType">
						<option value="1">일반</option>
						<option value="2">긴급</option>
					</select>
				</div>
				<div>
					<label class="detailLabel" >log</label>
					<textarea id ="car-marker-2_log" class="logDetail" value="">
					</textarea>
				</div>
			</div>

		</div>
	</div>
<%--	<div class="overlay">--%>
<%--		<button id="replay">Replay</button>--%>
<%--	</div>--%>
	<script type="module" src="/js/map.js"></script>
	<script type="module" src="/js/string.js"></script>
</body>
</html>