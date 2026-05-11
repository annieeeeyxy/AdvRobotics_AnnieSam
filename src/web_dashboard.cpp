#include "web_dashboard.h"

#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "motor_control.h"
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

static void sendText(WiFiClient& client, const char* text) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();
  client.println(text);
}

static void sendDataJson(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Cache-Control: no-store");
  client.println("Pragma: no-cache");
  client.println("Connection: close");
  client.println();

  client.print("{\"mode\":\"");
  client.print(modeName());
  client.print("\",\"enabled\":");
  client.print(emergencyStop ? "false" : "true");
  client.print(",\"yellowVisible\":");
  client.print(yellowLineVisible ? "true" : "false");
  client.print(",\"yellowId\":");
  client.print(yellowLineId);
  client.print(",\"xCenter\":");
  client.print(yellowLineXCenter);
  client.print(",\"yCenter\":");
  client.print(yellowLineYCenter);
  client.print(",\"width\":");
  client.print(yellowLineWidth);
  client.print(",\"height\":");
  client.print(yellowLineHeight);
  client.print(",\"area\":");
  client.print(yellowLineArea);
  client.print(",\"errorX\":");
  client.print(yellowLineErrorX);
  client.print(",\"lastSeenAgeMs\":");
  client.print(yellowLineLastSeenMs == 0 ? -1 : (long)(millis() - yellowLineLastSeenMs));
  client.print(",\"servo\":");
  client.print(currentServoPosition);
  client.print(",\"speed\":");
  client.print(motorSpeed);
  client.print(",\"kp\":");
  client.print(kp, 3);
  client.print(",\"yellowLineMaxTurn\":");
  client.print(yellowLineMaxTurn);
  client.print("}");
}

static void sendPage(WiFiClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Cache-Control: no-store");
  client.println("Pragma: no-cache");
  client.println("Connection: close");
  client.println();

  client.print("<!DOCTYPE html><html><head>");
  client.print("<meta name='viewport' content='width=device-width, initial-scale=1'>");
  client.print("<title>HuskyLens Line Tracker Test</title>");
  client.print("<style>");
  client.print("body{font-family:Arial,sans-serif;margin:0;background:#f4f6f8;color:#1f2933;}");
  client.print(".wrap{max-width:920px;margin:auto;padding:18px;}");
  client.print(".box{background:white;border:1px solid #d9e0e6;border-radius:10px;padding:18px;margin-bottom:16px;}");
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
  client.print("function showReaction(v){let k=(v/100).toFixed(2);setText('reactionText',k);document.getElementById('kpHidden').value=k;}");
  client.print("function params(){");
  client.print("let speed=document.getElementById('speedSlider').value;");
  client.print("let kp=document.getElementById('kpHidden').value;");
  client.print("let ki=document.getElementById('kiInput').value;");
  client.print("let kd=document.getElementById('kdInput').value;");
  client.print("let deadband=document.getElementById('deadbandSlider').value;");
  client.print("let maxturn=document.getElementById('maxTurnSlider').value;");
  client.print("return 'speed='+speed+'&kp='+kp+'&ki='+ki+'&kd='+kd+'&deadband='+deadband+'&maxturn='+maxturn;");
  client.print("}");
  client.print("let liveTimer=null;");
  client.print("function liveTune(){clearTimeout(liveTimer);liveTimer=setTimeout(function(){");
  client.print("fetch('/live?'+params(),{cache:'no-store'}).then(r=>r.text()).then(t=>setStatus(t)).catch(()=>setStatus('live update failed'));");
  client.print("},120);}");
  client.print("function sendParams(){fetch('/set?'+params(),{cache:'no-store'}).then(r=>r.text()).then(t=>{setStatus(t);poll();}).catch(()=>setStatus('settings failed'));}");
  client.print("function startTracking(){fetch('/follow',{cache:'no-store'}).then(r=>r.text()).then(t=>{setStatus(t);poll();}).catch(()=>setStatus('start failed'));}");
  client.print("function estop(){fetch('/stop',{cache:'no-store'}).then(r=>r.text()).then(t=>{setStatus(t);poll();}).catch(()=>setStatus('stop failed'));}");
  client.print("function poll(){fetch('/data',{cache:'no-store'}).then(r=>r.json()).then(d=>{");
  client.print("setText('enabledLive',d.enabled?'ENABLED':'STOPPED');setText('modeLive',d.mode);");
  client.print("setText('visibleLive',d.yellowVisible?'YES':'NO');setText('idLive',d.yellowId);");
  client.print("setText('xLive',d.xCenter);setText('yLive',d.yCenter);setText('wLive',d.width);setText('hLive',d.height);");
  client.print("setText('areaLive',d.area);setText('errorLive',d.errorX);setText('ageLive',d.lastSeenAgeMs<0?'never':d.lastSeenAgeMs);");
  client.print("setText('servoLive',d.servo);");
  client.print("}).catch(()=>{});}setInterval(poll,500);");
  client.print("document.addEventListener('keydown',function(e){if(e.code==='Space'){e.preventDefault();estop();}});");
  client.print("</script>");

  client.print("</head><body><div class='wrap'>");
  client.print("<div class='box'><h2>HuskyLens Line Tracker Test</h2><div class='grid'>");

  client.print("<div class='card'><div class='label'>Robot</div><div class='value'><span id='enabledLive'>");
  client.print(emergencyStop ? "STOPPED" : "ENABLED");
  client.print("</span></div></div>");

  client.print("<div class='card'><div class='label'>Mode</div><div class='value'><span id='modeLive'>");
  client.print(modeName());
  client.print("</span></div></div>");

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

  client.print("<div class='card'><div class='label'>Servo</div><div class='value'><span id='servoLive'>");
  client.print(currentServoPosition);
  client.print("</span></div></div>");

  client.print("</div></div>");

  client.print("<div class='box'><h3>Line Tracking Settings</h3>");
  client.print("<div class='row'>");

  client.print("<div><div class='label'>Motor Speed: <span id='speedText'>");
  client.print(motorSpeed);
  client.print("</span></div><input id='speedSlider' type='range' name='speed' min='0' max='180' value='");
  client.print(motorSpeed);
  client.print("' oninput='showSpeed(this.value);liveTune()'></div>");

  client.print("<div><div class='label'>Line Reaction: <span id='reactionText'>");
  client.print(kp, 2);
  client.print("</span></div><input id='reactionSlider' type='range' name='reaction' min='1' max='150' value='");
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
  client.print("</span> deg</div><input id='maxTurnSlider' type='range' name='maxturn' min='0' max='35' value='");
  client.print(yellowLineMaxTurn);
  client.print("' oninput='showMaxTurn(this.value);liveTune()'></div>");

  client.print("</div>");
  client.print("<button class='btn blue' onclick='sendParams()'>Apply Settings</button>");

  client.print("<div class='row'>");
  client.print("<button class='btn green' onclick='startTracking()'>Enable Line Tracking</button>");
  client.print("<button class='btn red' onclick='estop()'>Stop Everything</button>");
  client.print("</div></div>");

  client.print("<div class='box'><h3 id='statusText'>Status: ready</h3></div>");

  client.print("</div></body></html>");
}

static void handleClient(WiFiClient& client) {
  unsigned long start = millis();
  while (client.connected() && !client.available()) {
    if (millis() - start > 1000) {
      client.stop();
      return;
    }
  }

  String req = client.readStringUntil('\r');

  while (client.available()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }

  if (req.indexOf("/data") != -1) {
    sendDataJson(client);
    client.stop();
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
    if (sKp != "") kp = constrain(sKp.toFloat(), 0.01f, 1.50f);
    if (sKi != "") ki = sKi.toFloat();
    if (sKd != "") kd = sKd.toFloat();
    if (sDeadband != "") yellowLineDeadbandPixels = constrain(sDeadband.toInt(), 0, 60);
    if (sMaxTurn != "") yellowLineMaxTurn = constrain(sMaxTurn.toInt(), 0, 35);

    if (!emergencyStop && sSpeed != "") {
      setEscSpeed(motorSpeed);
    }

    sendText(client, "Live settings updated");
    client.stop();
    return;
  }

  if (req.indexOf("/stop") != -1) {
    emergencyStop = true;
    setSteeringServo(servoCenter);
    stopMotor();
    Serial.println("[WEB] Emergency stop activated");
    sendText(client, "Emergency stop activated");
    client.stop();
    return;
  } else if (req.indexOf("/follow") != -1) {
    emergencyStop = false;
    mode = FOLLOW_COLOR;
    resetPidState();
    Serial.println("[WEB] Line tracking enabled");
    sendText(client, "Line tracking enabled");
    client.stop();
    return;
  } else if (req.indexOf("/set") != -1) {
    String sSpeed = getValue(req, "speed");
    String sKp = getValue(req, "kp");
    String sKi = getValue(req, "ki");
    String sKd = getValue(req, "kd");
    String sDeadband = getValue(req, "deadband");
    String sMaxTurn = getValue(req, "maxturn");

    Serial.print("[SET] speed=");
    Serial.println(sSpeed);
    Serial.print("[SET] kp=");
    Serial.println(sKp);

    if (sSpeed != "") motorSpeed = constrain(sSpeed.toInt(), 0, 180);
    if (sKp != "") kp = sKp.toFloat();
    if (sKi != "") ki = sKi.toFloat();
    if (sKd != "") kd = sKd.toFloat();
    if (sDeadband != "") yellowLineDeadbandPixels = constrain(sDeadband.toInt(), 0, 60);
    if (sMaxTurn != "") yellowLineMaxTurn = constrain(sMaxTurn.toInt(), 0, 35);

    emergencyStop = false;
    mode = FOLLOW_COLOR;
    resetPidState();
    saveCurrentSettings();
    Serial.println("[WEB] Settings applied, tracking started");

    if (!emergencyStop && sSpeed != "") {
      setEscSpeed(motorSpeed);
    }

    sendText(client, "Settings applied, tracking started");
    client.stop();
    return;
  }

  sendPage(client);
  client.stop();
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
