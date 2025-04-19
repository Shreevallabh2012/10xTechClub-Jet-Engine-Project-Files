let socket;
let currentMode = "WEB"; // Default mode
let speedInterval = null;
let lastSpeed = 0;

function connectWebSocket() {
  socket = new WebSocket("ws://" + window.location.hostname + ":81");

  socket.onopen = () => {
    console.log("WebSocket connected.");
  };

  socket.onmessage = (event) => {
    const data = parseSensorData(event.data);

    setText("temperature", data.Temp);
    setText("humidity", data.Humidity);
    setText("aqi", data.AQI);
    setText("co2", data.CO2);
    setText("nh3", data.NH3);
    setText("nox", data.NOx);
    setText("alcohol", data.Alcohol);
    setText("benzene", data.Benzene);
    setText("smoke", data.Smoke);

    // Flame detection
    const flameElement = document.getElementById("flame");
    const flameWarning = document.getElementById("flame-warning");

    if (data.Flame === "1") {
      setText("flame", "Detected");
      flameElement.style.color = "red";
      flameWarning.style.display = "block";
    } else {
      setText("flame", "Fire Not Detected");
      flameElement.style.color = "green";
      flameWarning.style.display = "none";
    }

    setText("last-update", new Date().toLocaleTimeString());
  };

  socket.onclose = () => {
    console.log("WebSocket disconnected. Retrying in 3s...");
    setTimeout(connectWebSocket, 3000);
  };

  socket.onerror = (err) => {
    console.error("WebSocket error:", err);
    socket.close();
  };
}

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function parseSensorData(data) {
  return data.split(",").reduce((obj, item) => {
    const [key, val] = item.split(":");
    if (key && val !== undefined) obj[key.trim()] = val.trim();
    return obj;
  }, {});
}

// Motor Speed Handling with easing (quadratic-like curve)
function applyThrottleCurve(value) {
  const percentage = parseInt(value, 10) / 100;
  return Math.round(100 * percentage * percentage); // Quadratic curve
}

document.addEventListener("DOMContentLoaded", () => {
  connectWebSocket();

  const speedSlider = document.getElementById("motorSpeed");
  const speedValue = document.getElementById("motorSpeedValue");
  const emergencyStopBtn = document.getElementById("emergencyStop");
  const modeToggleBtn = document.getElementById("modeToggle");
  const currentModeText = document.getElementById("currentMode");

  const updateModeUI = () => {
    currentModeText.textContent = currentMode;
    modeToggleBtn.textContent =
      currentMode === "WEB" ? "Switch to MANUAL Mode" : "Switch to WEB Mode";
  };

  updateModeUI();

  if (speedSlider && speedValue) {
    speedSlider.addEventListener("input", () => {
      const rawSpeed = speedSlider.value;
      const curvedSpeed = applyThrottleCurve(rawSpeed);
      speedValue.textContent = curvedSpeed;
      lastSpeed = curvedSpeed;

      if (currentMode === "WEB" && socket && socket.readyState === WebSocket.OPEN) {
        if (!speedInterval) {
          speedInterval = setInterval(() => {
            socket.send("MOTOR_SPEED:" + lastSpeed);
          }, 200);
        }
      }
    });

    speedSlider.addEventListener("change", () => {
      clearInterval(speedInterval);
      speedInterval = null;
    });
  }

  // Emergency Stop
  emergencyStopBtn.addEventListener("click", () => {
    speedSlider.value = 0;
    speedValue.textContent = "0";
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send("MOTOR_SPEED:0");
    }
  });

  // Mode toggle
  modeToggleBtn.addEventListener("click", () => {
    currentMode = currentMode === "WEB" ? "MANUAL" : "WEB";
    updateModeUI();

    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send("MODE:" + currentMode);
    }

    if (currentMode === "MANUAL") {
      clearInterval(speedInterval);
      speedInterval = null;
    }
  });
});