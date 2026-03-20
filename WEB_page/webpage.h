// The webpage, stored in program memory
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Arduino GIGA PID Controller</title>
    <script>
        function getSensor() {
            fetch('/data')
            .then(response => response.text())
            .then(data => {
                document.getElementById("sensorValue").innerText = data;
            });
        }
        setInterval(getSensor, 500);

        function sendData() {
            let kp = document.getElementById("kp").value;
            let ki = document.getElementById("ki").value;
            let kd = document.getElementById("kd").value;
            let sp = document.getElementById("sp").value;

            let query = "/update?kp=" + encodeURIComponent(kp) +
                        "&ki=" + encodeURIComponent(ki) +
                        "&kd=" + encodeURIComponent(kd) +
                        "&sp=" + encodeURIComponent(sp);

            fetch(query)
            .then(response => response.text())
            .then(data => {
                document.getElementById("status").innerText = data;
            });
        }
    </script>
</head>
<style>
.content {
  max-width: 500px;
  margin: auto;
  text-align: center;
  font-family: courier;
}
input {
  width: 120px;
  margin: 5px;
  padding: 5px;
  font-family: courier;
}
button {
  margin-top: 10px;
  padding: 8px 16px;
  font-family: courier;
}
</style>
<body>
  <div class="content">
      <h2>Arduino GIGA R1 WiFi</h2>
      <h3>Simple PID Web Server</h3>

      <p>Kp: <input id="kp" type="number" step="0.01" value="1.0"></p>
      <p>Ki: <input id="ki" type="number" step="0.01" value="0.0"></p>
      <p>Kd: <input id="kd" type="number" step="0.01" value="0.0"></p>
      <p>Setpoint: <input id="sp" type="number" step="0.01" value="512.0"></p>

      <button onclick="sendData()">Send Updates</button>

      <p>Live sensor value on A0: <span id="sensorValue">0</span></p>
      <p id="status">Waiting...</p>
  </div>
</body>
</html>
)rawliteral";
