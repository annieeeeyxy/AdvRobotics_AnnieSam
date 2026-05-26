#include "web_dashboard.h"

#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "gps_module.h"
#include "motor_control.h"
#include "imu_module.h"
#include "robot_behavior.h"
#include "state.h"

static WiFiServer server(WEB_PORT);

static String getValue(String req, String key) {
  int start = req.indexOf(key + "=");
  if (start == -1) return "";

  start += key.length() + 1;

  int end = req.indexOf("&", start);
  if (end == -1) end = req.indexOf(" ", start);
  if (end == -1) return "";

  return req.substring(start, end);
}

static void closeClient(WiFiClient& client) {
  client.flush();
  delay(5);
  client.stop();
}

static void sendBody(WiFiClient& client, const char* contentType, const String& body) {
  client.println("HTTP/1.1 200 OK");
  client.print("Content-Type: ");
  client.println(contentType);
  client.println("Cache-Control: no-store");
  client.println("Pragma: no-cache");
  client.println("Connection: close");
  client.print("Content-Length: ");
  client.println(body.length());
  client.println();
  client.print(body);
}

static void sendText(WiFiClient& client, const char* text) {
  String body = text;
  body += "\n";
  sendBody(client, "text/plain", body);
}

static void sendNoContent(WiFiClient& client) {
  client.println("HTTP/1.1 204 No Content");
  client.println("Connection: close");
  client.println("Content-Length: 0");
  client.println();
}

static void sendBadRequest(WiFiClient& client, const char* text) {
  String body = text;
  body += "\n";
  client.println("HTTP/1.1 400 Bad Request");
  client.println("Content-Type: text/plain");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.print("Content-Length: ");
  client.println(body.length());
  client.println();
  client.print(body);
}

static void sendDataJson(WiFiClient& client) {
  GpsLocation location = gps();
  String body;
  body.reserve(800);
  body += "{\"mode\":\"";
  body += modeName();
  body += "\",\"enabled\":";
  body += emergencyStop ? "false" : "true";
  body += ",\"yellowVisible\":";
  body += yellowLineVisible ? "true" : "false";
  body += ",\"yellowId\":";
  body += String(yellowLineId);
  body += ",\"xCenter\":";
  body += String(yellowLineXCenter);
  body += ",\"yCenter\":";
  body += String(yellowLineYCenter);
  body += ",\"width\":";
  body += String(yellowLineWidth);
  body += ",\"height\":";
  body += String(yellowLineHeight);
  body += ",\"area\":";
  body += String(yellowLineArea);
  body += ",\"errorX\":";
  body += String(yellowLineErrorX);
  body += ",\"lastSeenAgeMs\":";
  body += String(yellowLineLastSeenMs == 0 ? -1 : (long)(millis() - yellowLineLastSeenMs));
  body += ",\"servo\":";
  body += String(currentServoPosition);
  body += ",\"speed\":";
  body += String(motorSpeed);
  body += ",\"escOutput\":";
  body += String(currentEscOutput);
  body += ",\"kp\":";
  body += String(kp, 3);
  body += ",\"yellowLineMaxTurn\":";
  body += String(yellowLineMaxTurn);
  body += ",\"imuReady\":";
  body += imuReadOk ? "true" : "false";
  body += ",\"yaw\":";
  body += String(getYaw(), 2);
  body += ",\"pitch\":";
  body += String(getPitch(), 2);
  body += ",\"roll\":";
  body += String(getRoll(), 2);
  body += ",\"frontBlocked\":";
  body += frontBlocked ? "true" : "false";
  body += ",\"frontDistance\":";
  body += String(frontDistance, 1);
  body += ",\"frontAngle\":";
  body += String(frontAngle, 1);
  body += ",\"gpsHasFix\":";
  body += location.hasFix ? "true" : "false";
  body += ",\"gpsHasData\":";
  body += location.hasReceivedData ? "true" : "false";
  body += ",\"gpsLat\":";
  body += String(location.latitude, 6);
  body += ",\"gpsLon\":";
  body += String(location.longitude, 6);
  body += ",\"gpsSat\":";
  body += String(location.satellites);
  body += ",\"gpsHdop\":";
  body += String(location.hdop, 2);
  body += ",\"gpsHdopValid\":";
  body += location.hdopValid ? "true" : "false";
  body += ",\"gpsSpeedKmph\":";
  body += String(location.speedKmph, 2);
  body += ",\"gpsCourseDeg\":";
  body += String(location.courseDeg, 2);
  body += ",\"gpsDistanceTargetM\":";
  body += String(location.distanceToTargetMeters, 2);
  body += ",\"gpsBearingTargetDeg\":";
  body += String(location.bearingToTargetDeg, 2);
  body += ",\"gpsAgeMs\":";
  body += String(location.lastFixMs == 0 ? -1 : (long)location.ageMs);
  body += ",\"gpsChars\":";
  body += String(location.charsProcessed);
  body += ",\"gpsBaud\":";
  body += String(location.baud);
  body += ",\"gpsWaypointIndex\":";
  body += String(gpsWaypointIndex + 1);
  body += ",\"gpsDistanceWaypointM\":";
  body += String(gpsDistanceToWaypoint, 2);
  body += ",\"gpsBearingWaypointDeg\":";
  body += String(gpsBearingToWaypoint, 2);
  body += ",\"gpsHeadingErrorDeg\":";
  body += String(gpsHeadingError, 2);
  body += ",\"gpsCompletedLoops\":";
  body += String(gpsCompletedLoops);
  body += "}";
  sendBody(client, "application/json", body);
}

static void sendPage(WiFiClient& client) {
#if ENABLE_GPS
  GpsLocation location = gps();
#endif

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Cache-Control: no-store");
  client.println("Pragma: no-cache");
  client.println("Connection: close");
  client.println();

  client.print("<!DOCTYPE html><html><head>");
  client.print("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  client.print("<title>GPS Monitor</title>");
  client.print("<style>");
  client.print("body{font-family:Arial,sans-serif;margin:0;background:#f4f6f8;color:#1f2933;}");
  client.print(".wrap{max-width:920px;margin:auto;padding:18px;}");
  client.print(".box{background:white;border:1px solid #d9e0e6;border-radius:10px;padding:18px;margin-bottom:16px;}");
  client.print(".sectionTitle{font-size:16px;font-weight:bold;margin:4px 0 12px;color:#1f2933;}");
  client.print(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:10px;}");
  client.print(".row{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:12px;}");
  client.print(".card{background:white;border:1px solid #d9e0e6;border-radius:8px;padding:14px;}");
  client.print(".label{font-size:12px;color:#5b6875;text-transform:uppercase;}");
  client.print(".value{font-size:24px;font-weight:bold;margin-top:7px;}");
  client.print(".btn{width:100%;padding:14px;border:none;border-radius:8px;font-weight:bold;cursor:pointer;margin-top:12px;}");
  client.print(".blue{background:#2f6ea3;color:white;}");
  client.print(".red{background:#c73b3b;color:white;}");
  client.print(".green{background:#2f7d4f;color:white;}");
  client.print("input[type=text],input[type=number],input[type=range]{width:100%;padding:10px;border:1px solid #d9e0e6;border-radius:8px;box-sizing:border-box;}");
  client.print("</style>");

  client.print("<script>");
  client.print("function setText(id,v){var e=document.getElementById(id);if(e)e.innerHTML=v;}");
  client.print("function setStatus(v){setText('statusText',v);}");
  client.print("function showSpeed(v){setText('speedText',v);}");
  client.print("function showDeadband(v){setText('deadbandText',v);}");
  client.print("function showMaxTurn(v){setText('maxTurnText',v);}");
  client.print("function valueOf(id,fallback){let e=document.getElementById(id);return e?e.value:fallback;}");
  client.print("function showReaction(v){let k=(v/100).toFixed(2);setText('reactionText',k);let e=document.getElementById('kpHidden');if(e)e.value=k;}");
  client.print("function params(){");
  client.print("let speed=valueOf('speedSlider','100');");
  client.print("let kp=valueOf('kpHidden','");
  client.print(kp, 3);
  client.print("');");
  client.print("let ki=valueOf('kiInput','");
  client.print(ki, 3);
  client.print("');");
  client.print("let kd=valueOf('kdInput','");
  client.print(kd, 3);
  client.print("');");
  client.print("let deadband=valueOf('deadbandSlider','");
  client.print(yellowLineDeadbandPixels);
  client.print("');");
  client.print("let maxturn=valueOf('maxTurnSlider','");
  client.print(yellowLineMaxTurn);
  client.print("');");
  client.print("return 'speed='+speed+'&kp='+kp+'&ki='+ki+'&kd='+kd+'&deadband='+deadband+'&maxturn='+maxturn;");
  client.print("}");
  client.print("let liveTimer=null,requestBusy=false,setBusy=false,pollBusy=false,pollFails=0;");
  client.print("function textRequest(url,busyMessage,failMessage,after){if(requestBusy)return;requestBusy=true;setStatus(busyMessage);return fetch(url,{cache:'no-store'}).then(r=>{if(!r.ok)throw new Error(r.status);return r.text();}).then(t=>{setStatus(t);if(after)after();}).catch(()=>setStatus(failMessage)).finally(()=>{requestBusy=false;});}");
  client.print("function liveTune(){clearTimeout(liveTimer);liveTimer=setTimeout(function(){");
  client.print("if(requestBusy||setBusy)return;");
  client.print("textRequest('/live?'+params(),'updating live settings...','live update failed');");
  client.print("},120);}");
  client.print("function sendParams(){if(requestBusy||setBusy)return;clearTimeout(liveTimer);setBusy=true;textRequest('/set?'+params(),'applying settings...','settings failed',poll).finally(()=>{setBusy=false;});}");
  client.print("function startTracking(){textRequest('/follow','starting line tracking...','start failed',poll);}");
  client.print("function startGpsNav(){textRequest('/gpsstart?'+params(),'starting gps oval...','gps start failed',poll);}");
  client.print("function estop(){clearTimeout(liveTimer);textRequest('/stop','stopping...','stop failed',poll);}");
  client.print("function schedulePoll(){setTimeout(poll,pollFails?1000:250);}");
  client.print("function poll(){if(pollBusy||requestBusy||setBusy){schedulePoll();return;}pollBusy=true;fetch('/data',{cache:'no-store'}).then(r=>{if(!r.ok)throw new Error(r.status);return r.json();}).then(d=>{pollFails=0;");
  client.print("setText('enabledLive',d.enabled?'ENABLED':'STOPPED');setText('modeLive',d.mode);");
  client.print("setText('visibleLive',d.yellowVisible?'YES':'NO');setText('idLive',d.yellowId);");
  client.print("setText('xLive',d.xCenter);setText('yLive',d.yCenter);setText('wLive',d.width);setText('hLive',d.height);");
  client.print("setText('areaLive',d.area);setText('errorLive',d.errorX);setText('ageLive',d.lastSeenAgeMs<0?'never':d.lastSeenAgeMs);");
  client.print("setText('servoLive',d.servo);");
  client.print("setText('escLive',d.escOutput);");
  client.print("setText('imuReadyLive',d.imuReady?'YES':'NO');setText('yawLive',Number(d.yaw).toFixed(2));");
  client.print("setText('pitchLive',Number(d.pitch).toFixed(2));setText('rollLive',Number(d.roll).toFixed(2));");
  client.print("setText('blockedLive',d.frontBlocked?'YES':'NO');setText('frontDistanceLive',d.frontDistance>=9999?'none':Number(d.frontDistance).toFixed(0));");
  client.print("setText('frontAngleLive',d.frontAngle<0?'none':Number(d.frontAngle).toFixed(1));");
  client.print("setText('gpsFixLive',d.gpsHasFix?'YES':'NO');setText('gpsLatLive',d.gpsHasFix?Number(d.gpsLat).toFixed(6):'waiting');");
  client.print("setText('gpsLonLive',d.gpsHasFix?Number(d.gpsLon).toFixed(6):'waiting');setText('gpsSatLive',d.gpsSat);");
  client.print("setText('gpsDataLive',d.gpsHasData?'YES':'NO');setText('gpsHdopLive',d.gpsHdopValid?Number(d.gpsHdop).toFixed(2):'n/a');");
  client.print("setText('gpsSpeedLive',Number(d.gpsSpeedKmph).toFixed(2));setText('gpsCourseLive',Number(d.gpsCourseDeg).toFixed(2));");
  client.print("setText('gpsTargetDistanceLive',d.gpsHasFix?Number(d.gpsDistanceTargetM).toFixed(2):'waiting');");
  client.print("setText('gpsTargetBearingLive',d.gpsHasFix?Number(d.gpsBearingTargetDeg).toFixed(2):'waiting');");
  client.print("setText('gpsAgeLive',d.gpsAgeMs<0?'never':d.gpsAgeMs);");
  client.print("setText('gpsCharsLive',d.gpsChars);setText('gpsBaudLive',d.gpsBaud);");
  client.print("setText('gpsWaypointLive',d.gpsWaypointIndex);setText('gpsWaypointDistanceLive',Number(d.gpsDistanceWaypointM).toFixed(2));");
  client.print("setText('gpsWaypointBearingLive',Number(d.gpsBearingWaypointDeg).toFixed(2));setText('gpsHeadingErrorLive',Number(d.gpsHeadingErrorDeg).toFixed(2));");
  client.print("setText('gpsLoopsLive',d.gpsCompletedLoops);");
  client.print("}).catch(()=>{pollFails++;}).finally(()=>{pollBusy=false;schedulePoll();});}");
  client.print("document.addEventListener('DOMContentLoaded',poll);");
  client.print("document.addEventListener('keydown',function(e){if(e.code==='Space'){e.preventDefault();estop();}});");
  client.print("</script>");

  client.print("</head><body><div class='wrap'>");
  client.print("<div class='box'><h2>GPS Monitor</h2><div class='sectionTitle'>Robot</div><div class='grid'>");

  client.print("<div class='card'><div class='label'>Robot</div><div class='value'><span id='enabledLive'>");
  client.print(emergencyStop ? "STOPPED" : "ENABLED");
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Mode</div><div class='value'><span id='modeLive'>");
  client.print(modeName());
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Servo</div><div class='value'><span id='servoLive'>");
  client.print(currentServoPosition);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>ESC Output</div><div class='value'><span id='escLive'>");
  client.print(currentEscOutput);
  client.print("</span></div></div>");

  client.print("</div></div>");

#if ENABLE_HUSKYLENS
  client.print("<div class='box'><div class='sectionTitle'>HUSKYLENS Yellow Line</div><div class='grid'>");

  client.print("<div class='card'><div class='label'>Yellow Visible</div><div class='value'><span id='visibleLive'>");
  client.print(yellowLineVisible ? "YES" : "NO");
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>ID</div><div class='value'><span id='idLive'>");
  client.print(yellowLineId);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>xCenter</div><div class='value'><span id='xLive'>");
  client.print(yellowLineXCenter);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>yCenter</div><div class='value'><span id='yLive'>");
  client.print(yellowLineYCenter);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Width</div><div class='value'><span id='wLive'>");
  client.print(yellowLineWidth);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Height</div><div class='value'><span id='hLive'>");
  client.print(yellowLineHeight);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Area</div><div class='value'><span id='areaLive'>");
  client.print(yellowLineArea);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>errorX</div><div class='value'><span id='errorLive'>");
  client.print(yellowLineErrorX);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Last Seen ms</div><div class='value'><span id='ageLive'>");
  if (yellowLineLastSeenMs == 0) client.print("never");
  else client.print(millis() - yellowLineLastSeenMs);
  client.print("</span></div></div>");

  client.print("</div></div>");
#endif

#if ENABLE_IMU
  client.print("<div class='box'><div class='sectionTitle'>IMU Heading</div><div class='grid'>");

  client.print("<div class='card'><div class='label'>IMU Ready</div><div class='value'><span id='imuReadyLive'>");
  client.print(imuReadOk ? "YES" : "NO");
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Yaw</div><div class='value'><span id='yawLive'>");
  client.print(getYaw(), 2);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Pitch</div><div class='value'><span id='pitchLive'>");
  client.print(getPitch(), 2);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Roll</div><div class='value'><span id='rollLive'>");
  client.print(getRoll(), 2);
  client.print("</span></div></div>");

  client.print("</div></div>");
#endif

#if ENABLE_LIDAR
  client.print("<div class='box'><div class='sectionTitle'>LiDAR Obstacle</div><div class='grid'>");

  client.print("<div class='card'><div class='label'>LiDAR Blocked</div><div class='value'><span id='blockedLive'>");
  client.print(frontBlocked ? "YES" : "NO");
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Front mm</div><div class='value'><span id='frontDistanceLive'>");
  if (frontDistance < 9999) client.print(frontDistance, 0);
  else client.print("none");
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Front Angle</div><div class='value'><span id='frontAngleLive'>");
  if (frontAngle >= 0) client.print(frontAngle, 1);
  else client.print("none");
  client.print("</span></div></div>");

  client.print("</div></div>");
#endif

#if ENABLE_GPS
  client.print("<div class='box'><div class='sectionTitle'>GPS Location</div><div class='grid'>");

  client.print("<div class='card'><div class='label'>GPS Fix</div><div class='value'><span id='gpsFixLive'>");
  client.print(location.hasFix ? "YES" : "NO");
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Data Received</div><div class='value'><span id='gpsDataLive'>");
  client.print(location.hasReceivedData ? "YES" : "NO");
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Latitude</div><div class='value'><span id='gpsLatLive'>");
  if (location.hasFix) client.print(location.latitude, 6);
  else client.print("waiting");
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Longitude</div><div class='value'><span id='gpsLonLive'>");
  if (location.hasFix) client.print(location.longitude, 6);
  else client.print("waiting");
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Satellites</div><div class='value'><span id='gpsSatLive'>");
  client.print(location.satellites);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>GPS HDOP</div><div class='value'><span id='gpsHdopLive'>");
  if (location.hdopValid) client.print(location.hdop, 2);
  else client.print("n/a");
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Speed km/h</div><div class='value'><span id='gpsSpeedLive'>");
  client.print(location.speedKmph, 2);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Course deg</div><div class='value'><span id='gpsCourseLive'>");
  client.print(location.courseDeg, 2);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>GPS Age ms</div><div class='value'><span id='gpsAgeLive'>");
  if (location.lastFixMs == 0) client.print("never");
  else client.print(location.ageMs);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>GPS Chars</div><div class='value'><span id='gpsCharsLive'>");
  client.print(location.charsProcessed);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>GPS Baud</div><div class='value'><span id='gpsBaudLive'>");
  client.print(location.baud);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Waypoint</div><div class='value'><span id='gpsWaypointLive'>");
  client.print(gpsWaypointIndex + 1);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Waypoint m</div><div class='value'><span id='gpsWaypointDistanceLive'>");
  client.print(gpsDistanceToWaypoint, 2);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Target Bearing</div><div class='value'><span id='gpsWaypointBearingLive'>");
  client.print(gpsBearingToWaypoint, 2);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Heading Error</div><div class='value'><span id='gpsHeadingErrorLive'>");
  client.print(gpsHeadingError, 2);
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Loops</div><div class='value'><span id='gpsLoopsLive'>");
  client.print(gpsCompletedLoops);
  client.print("</span></div></div>");

  client.print("</div></div>");

  client.print("<div class='box'><div class='sectionTitle'>GPS Oval Driving</div><div class='row'>");
  client.print("<div><div class='label'>Motor Speed: <span id='speedText'>");
  client.print(motorSpeed);
  client.print("</span></div><input id='speedSlider' type='range' name='speed' min='0' max='180' value='");
  client.print(motorSpeed);
  client.print("' oninput='showSpeed(this.value);liveTune()'></div>");
  client.print("<button class='btn green' onclick='startGpsNav()'>Start GPS Oval</button>");
  client.print("<button class='btn red' onclick='estop()'>Stop Everything</button>");
  client.print("</div></div>");
#endif

#if ENABLE_HUSKYLENS
  client.print("<div class='box'><h3>Line Tracking Settings</h3>");
  client.print("<div class='row'>");

#if !ENABLE_GPS
  client.print("<div><div class='label'>Motor Speed: <span id='speedText'>");
  client.print(motorSpeed);
  client.print("</span></div><input id='speedSlider' type='range' name='speed' min='0' max='180' value='");
  client.print(motorSpeed);
  client.print("' oninput='showSpeed(this.value);liveTune()'></div>");
#endif

  client.print("<div><div class='label'>Line Reaction: <span id='reactionText'>");
  client.print(kp, 2);
  client.print("</span></div><input id='reactionSlider' type='range' name='reaction' min='1' max='80' value='");
  client.print((int)(kp * 100));
  client.print("' oninput='showReaction(this.value);liveTune()'></div>");

  client.print("<input id='kpHidden' type='hidden' name='kp' value='");
  client.print(kp, 3);
  client.print("'>");

  client.print("<div><div class='label'>Ki</div><input id='kiInput' type='text' name='ki' value='");
  client.print(ki, 3);
  client.print("' oninput='liveTune()'></div>");

  client.print("<div><div class='label'>Kd</div><input id='kdInput' type='text' name='kd' value='");
  client.print(kd, 3);
  client.print("' oninput='liveTune()'></div>");

  client.print("<div><div class='label'>Ignore Center Wiggle: <span id='deadbandText'>");
  client.print(yellowLineDeadbandPixels);
  client.print("</span> px</div><input id='deadbandSlider' type='range' name='deadband' min='0' max='60' value='");
  client.print(yellowLineDeadbandPixels);
  client.print("' oninput='showDeadband(this.value);liveTune()'></div>");

  client.print("<div><div class='label'>Turn Limit: <span id='maxTurnText'>");
  client.print(yellowLineMaxTurn);
  client.print("</span> deg</div><input id='maxTurnSlider' type='range' name='maxturn' min='0' max='");
  client.print(YELLOW_LINE_MAX_TURN_LIMIT);
  client.print("' value='");
  client.print(yellowLineMaxTurn);
  client.print("' oninput='showMaxTurn(this.value);liveTune()'></div>");

  client.print("</div>");
  client.print("<button class='btn blue' onclick='sendParams()'>Apply Settings</button>");

  client.print("<div class='row'>");
  client.print("<button class='btn green' onclick='startTracking()'>Enable Line Tracking</button>");
  client.print("<button class='btn red' onclick='estop()'>Stop Everything</button>");
  client.print("</div></div>");

  client.print("<div class='box'><h3 id='statusText'>Status: ready</h3></div>");
#else
  client.print("<div class='box'><h3 id='statusText'>Status: GPS dashboard ready</h3></div>");
#endif

  client.print("</div></body></html>");
}

static void handleClient(WiFiClient& client) {
  client.setTimeout(500);
  unsigned long start = millis();
  while (client.connected() && !client.available()) {
    if (millis() - start > 1000) {
      sendBadRequest(client, "Request timed out");
      closeClient(client);
      return;
    }
  }

  String req = client.readStringUntil('\r');
  if (req.length() == 0) {
    sendBadRequest(client, "Empty request");
    closeClient(client);
    return;
  }

  while (client.available()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }

  if (req.indexOf("/favicon.ico") != -1) {
    sendNoContent(client);
    closeClient(client);
    return;
  }

  if (req.indexOf("/data") != -1) {
    sendDataJson(client);
    closeClient(client);
    return;
  }

  if (req.indexOf("/live") != -1) {
    String sSpeed = getValue(req, "speed");
    String sKp = getValue(req, "kp");
    String sKi = getValue(req, "ki");
    String sKd = getValue(req, "kd");
    String sDeadband = getValue(req, "deadband");
    String sMaxTurn = getValue(req, "maxturn");

    if (sSpeed != "") motorSpeed = constrain(sSpeed.toInt(), 0, 180);
    if (sKp != "") kp = constrain(sKp.toFloat(), 0.01f, 0.80f);
    if (sKi != "") ki = sKi.toFloat();
    if (sKd != "") kd = sKd.toFloat();
    if (sDeadband != "") yellowLineDeadbandPixels = constrain(sDeadband.toInt(), 0, 60);
    if (sMaxTurn != "") yellowLineMaxTurn = constrain(sMaxTurn.toInt(), 0, YELLOW_LINE_MAX_TURN_LIMIT);

    if (!emergencyStop && sSpeed != "") {
      setEscSpeed(motorSpeed);
    }

    sendText(client, "Live settings updated");
    closeClient(client);
    return;
  }

  if (req.indexOf("/stop") != -1) {
    emergencyStop = true;
    setSteeringServo(servoCenter);
    stopMotor();
    Serial.println("[WEB] Emergency stop activated");
    sendText(client, "Emergency stop activated");
    closeClient(client);
    return;
  } else if (req.indexOf("/gpsstart") != -1) {
    String sSpeed = getValue(req, "speed");
    if (sSpeed != "") motorSpeed = constrain(sSpeed.toInt(), 0, 180);
    startGpsOvalNavigation();
    Serial.println("[WEB] GPS oval navigation enabled");
    GpsLocation location = gps();
    if (!location.hasFix || location.lastFixMs == 0 || location.ageMs > 3000) {
      sendText(client, "GPS oval armed, waiting for fresh GPS fix");
    } else {
      sendText(client, "GPS oval navigation enabled");
    }
    closeClient(client);
    return;
  } else if (req.indexOf("/follow") != -1) {
    emergencyStop = false;
    mode = FOLLOW_COLOR;
    resetPidState();
    Serial.println("[WEB] Line tracking enabled");
    sendText(client, "Line tracking enabled");
    closeClient(client);
    return;
  } else if (req.indexOf("/set") != -1) {
    String sSpeed = getValue(req, "speed");
    String sKp = getValue(req, "kp");
    String sKi = getValue(req, "ki");
    String sKd = getValue(req, "kd");
    String sDeadband = getValue(req, "deadband");
    String sMaxTurn = getValue(req, "maxturn");

    Serial.print("[SET] req=");
    Serial.println(req);
    Serial.print("[SET] speed=");
    Serial.println(sSpeed);
    Serial.print("[SET] kp=");
    Serial.println(sKp);

    if (sSpeed != "") motorSpeed = constrain(sSpeed.toInt(), 0, 180);
    if (sKp != "") kp = constrain(sKp.toFloat(), 0.01f, 0.80f);
    if (sKi != "") ki = sKi.toFloat();
    if (sKd != "") kd = sKd.toFloat();
    if (sDeadband != "") yellowLineDeadbandPixels = constrain(sDeadband.toInt(), 0, 60);
    if (sMaxTurn != "") yellowLineMaxTurn = constrain(sMaxTurn.toInt(), 0, YELLOW_LINE_MAX_TURN_LIMIT);

    emergencyStop = false;
    mode = FOLLOW_COLOR;
    Serial.println("[SET] before resetPidState()");
    resetPidState();
    Serial.println("[SET] after resetPidState()");
    Serial.println("[WEB] Settings applied, tracking started");

    Serial.println("[SET] before sendText()");
    sendText(client, "Settings applied, tracking started");
    Serial.println("[SET] after sendText()");
    closeClient(client);

    Serial.println("[SET] before saveCurrentSettings()");
    saveCurrentSettings();
    Serial.println("[SET] after saveCurrentSettings()");

    if (!emergencyStop && sSpeed != "") {
      Serial.println("[SET] before setEscSpeed()");
      setEscSpeed(motorSpeed);
      Serial.println("[SET] after setEscSpeed()");
    } else {
      Serial.println("[SET] setEscSpeed() skipped");
    }
    return;
  }

  sendPage(client);
  closeClient(client);
}

void setupWebDashboard() {
#if ENABLE_WEB
  WiFi.beginAP(WIFI_SSID, WIFI_PASS);
  server.begin();
#endif
}

void serviceWebDashboard() {
#if ENABLE_WEB
  WiFiClient client = server.accept();
  if (client) {
    handleClient(client);
  }
#endif
}
