/**
 * DECLIVOLT: Downhill Energy Recovery & Smart Vehicle Control System
 * Pure Vanilla JavaScript Web Serial Interface & Dashboard Logic
 */

// -----------------------------------------------------------------------------
// State Management
// -----------------------------------------------------------------------------
let port = null;
let reader = null;
let readableStreamClosed = null;
let keepReading = false;
let currentMotorCommand = 'S'; // 'S' = Stopped, 'F' = Forward, 'B' = Backward

// -----------------------------------------------------------------------------
// DOM Elements
// -----------------------------------------------------------------------------
const connectBtn = document.getElementById('connectBtn');
const connectBtnText = document.getElementById('connectBtnText');
const connectionStatusText = document.getElementById('connectionStatusText');
const connectionDot = document.getElementById('connectionDot');
const serialWarningBanner = document.getElementById('serialWarningBanner');

// Energy Recovery Card
const voltageValue = document.getElementById('voltageValue');
const adcValue = document.getElementById('adcValue');
const rpmValue = document.getElementById('rpmValue');
const chargingBadge = document.getElementById('chargingBadge');
const batteryFill = document.getElementById('batteryFill');
const chargingBoltIcon = document.getElementById('chargingBoltIcon');

// Slope Status Card
const pitchValue = document.getElementById('pitchValue');
const slopeBadge = document.getElementById('slopeBadge');
const slopeIndicatorPill = document.getElementById('slopeIndicatorPill');
const slopeIndicatorText = document.getElementById('slopeIndicatorText');
const recoveryBanner = document.getElementById('recoveryBanner');
const accelX = document.getElementById('accelX');
const accelY = document.getElementById('accelY');
const accelZ = document.getElementById('accelZ');

// Vehicle Controls
const motorStateBadge = document.getElementById('motorStateBadge');
const btnForward = document.getElementById('btnForward');
const btnStop = document.getElementById('btnStop');
const btnBackward = document.getElementById('btnBackward');

// -----------------------------------------------------------------------------
// Initialization & Compatibility Verification
// -----------------------------------------------------------------------------
document.addEventListener('DOMContentLoaded', () => {
  // Check for Web Serial API support
  if (!('serial' in navigator)) {
    serialWarningBanner.classList.remove('hidden');
    connectBtn.disabled = true;
    connectBtn.title = 'Web Serial API is not supported in this browser';
    console.warn('Web Serial API is not supported in this browser. Use Chrome or Edge.');
  }

  // Connect / Disconnect button listener
  connectBtn.addEventListener('click', toggleConnection);

  // Motor Control Button Listeners
  btnForward.addEventListener('click', () => sendMotorCommand('F'));
  btnStop.addEventListener('click', () => sendMotorCommand('S'));
  btnBackward.addEventListener('click', () => sendMotorCommand('B'));

  // Keyboard navigation & motor controls
  window.addEventListener('keydown', handleKeyboardControl);

  // Set initial disconnected state
  resetDashboardToDisconnected();
});

// -----------------------------------------------------------------------------
// Web Serial Connection Logic
// -----------------------------------------------------------------------------
async function toggleConnection() {
  if (port) {
    await disconnectFromPort();
  } else {
    await connectToPort();
  }
}

async function connectToPort() {
  try {
    // 1. Request port selection from user
    port = await navigator.serial.requestPort();

    // 2. Open serial connection at 115200 baud
    await port.open({ baudRate: 115200 });

    // Handle unexpected physical unplugging of USB cable
    port.addEventListener('disconnect', () => {
      console.warn('ESP32 device physically disconnected.');
      disconnectFromPort();
    });

    // 3. Update connection UI
    updateConnectionUI(true);
    keepReading = true;

    // 4. Start listening for incoming JSON packets
    readSerialStream();

  } catch (err) {
    console.error('Serial connection error:', err);
    port = null;
    updateConnectionUI(false);
  }
}

async function disconnectFromPort() {
  keepReading = false;

  try {
    if (reader) {
      await reader.cancel();
      reader = null;
    }
    if (readableStreamClosed) {
      await readableStreamClosed.catch(() => {});
      readableStreamClosed = null;
    }
    if (port) {
      await port.close();
      port = null;
    }
  } catch (err) {
    console.warn('Error while closing port:', err);
  } finally {
    port = null;
    updateConnectionUI(false);
    resetDashboardToDisconnected();
  }
}

// -----------------------------------------------------------------------------
// Continuous Serial Stream Reading & JSON Parsing
// -----------------------------------------------------------------------------
async function readSerialStream() {
  let partialLine = '';

  while (port && port.readable && keepReading) {
    const textDecoder = new TextDecoderStream();
    readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
    reader = textDecoder.readable.getReader();

    try {
      while (true) {
        const { value, done } = await reader.read();
        if (done) {
          break; // Stream cancelled or disconnected
        }
        if (value) {
          // Accumulate chunks and split by newline
          partialLine += value;
          const lines = partialLine.split('\n');
          // The last element is the incomplete remainder
          partialLine = lines.pop();

          for (const line of lines) {
            const cleanLine = line.trim();
            if (cleanLine.startsWith('{') && cleanLine.endsWith('}')) {
              try {
                const telemetry = JSON.parse(cleanLine);
                updateDashboard(telemetry);
              } catch (parseError) {
                // Ignore incomplete or transient corrupted JSON fragments
                console.debug('JSON parse ignored line:', cleanLine);
              }
            }
          }
        }
      }
    } catch (err) {
      console.warn('Serial read stream interrupted:', err);
    } finally {
      if (reader) {
        reader.releaseLock();
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Dashboard UI Updates
// -----------------------------------------------------------------------------
function updateDashboard(data) {
  // Expected JSON Schema:
  // {
  //   "voltage": 2.45,
  //   "pitch": 4.2,
  //   "slope": "DOWNHILL",
  //   "x": 0.12,
  //   "y": 0.08,
  //   "z": 0.98,
  //   "adc": 400,
  //   "charging": "FAST CHARGING"
  // }

  // 1. Voltage & Energy Recovery Updates
  const voltage = typeof data.voltage === 'number' ? data.voltage : 0.0;
  voltageValue.textContent = voltage.toFixed(2);
  adcValue.textContent = data.adc !== undefined ? data.adc : '--';

  // Live Wheel RPM from LM393 Speed Sensor
  if (rpmValue) {
    if (typeof data.rpm === 'number') {
      rpmValue.textContent = Math.round(data.rpm);
    } else {
      rpmValue.textContent = '--';
    }
  }

  // Charging status logic:
  // <= 0.1V: IDLE
  // > 0.1V and <= 1.5V: CHARGING
  // > 1.5V: FAST CHARGING
  let chargingStatus = 'IDLE';
  if (voltage > 1.5) {
    chargingStatus = 'FAST CHARGING';
  } else if (voltage > 0.1) {
    chargingStatus = 'CHARGING';
  }

  // If ESP32 explicitly provides charging status in JSON, sync with it
  if (data.charging) {
    chargingStatus = data.charging.toUpperCase();
  }

  updateChargingUI(voltage, chargingStatus);

  // 2. Slope Status Updates
  const pitch = typeof data.pitch === 'number' ? data.pitch : 0.0;
  const sign = pitch > 0 ? '+' : '';
  pitchValue.textContent = `${sign}${pitch.toFixed(1)}`;

  const slope = data.slope ? data.slope.toUpperCase() : 'FLAT';
  updateSlopeUI(pitch, slope, voltage);

  // 3. Accelerometer Telemetry
  accelX.textContent = data.x !== undefined ? data.x.toFixed(2) : '--';
  accelY.textContent = data.y !== undefined ? data.y.toFixed(2) : '--';
  accelZ.textContent = data.z !== undefined ? data.z.toFixed(2) : '--';
}

function updateChargingUI(voltage, status) {
  chargingBadge.textContent = status;
  chargingBadge.className = 'badge badge-charging';

  // Scale battery fill visually (0V = 0%, 3.0V+ = 100%)
  const maxScaleVoltage = 3.0;
  const percentage = Math.min(100, Math.max(0, (voltage / maxScaleVoltage) * 100));
  batteryFill.style.width = `${percentage}%`;

  if (status === 'FAST CHARGING') {
    chargingBadge.classList.add('fast');
    chargingBoltIcon.classList.add('active');
  } else if (status === 'CHARGING') {
    chargingBadge.classList.add('charging');
    chargingBoltIcon.classList.add('active');
  } else {
    chargingBadge.classList.add('idle');
    chargingBoltIcon.classList.remove('active');
  }
}

function updateSlopeUI(pitch, slope, voltage) {
  slopeBadge.textContent = slope;
  slopeBadge.className = 'badge badge-slope ' + slope.toLowerCase();

  // Update small visual status indicator (showing only UPHILL, DOWNHILL or FLAT)
  if (slopeIndicatorText) {
    slopeIndicatorText.textContent = slope;
  }
  if (slopeIndicatorPill) {
    slopeIndicatorPill.className = 'slope-indicator-pill ' + slope.toLowerCase();
  }

  // Trigger Energy Recovery status banner when going downhill and voltage is generated
  if (recoveryBanner) {
    if (slope === 'DOWNHILL' && voltage > 0.1) {
      recoveryBanner.classList.remove('hidden');
    } else {
      recoveryBanner.classList.add('hidden');
    }
  }
}

// -----------------------------------------------------------------------------
// Motor Control & Serial Transmission
// -----------------------------------------------------------------------------
async function sendMotorCommand(command) {
  if (!port || !port.writable) {
    console.warn('Cannot send command. Hardware is not connected.');
    return;
  }

  try {
    const encoder = new TextEncoder();
    const writer = port.writable.getWriter();
    // Send single character command terminated by newline
    await writer.write(encoder.encode(command + '\n'));
    writer.releaseLock();

    currentMotorCommand = command;
    updateMotorStateUI(command);
  } catch (err) {
    console.error(`Failed to send motor command '${command}':`, err);
  }
}

function updateMotorStateUI(command) {
  motorStateBadge.className = 'badge badge-motor';

  btnForward.classList.remove('active-pressed');
  btnStop.classList.remove('active-pressed');
  btnBackward.classList.remove('active-pressed');

  if (command === 'F') {
    motorStateBadge.textContent = 'FORWARD (F)';
    motorStateBadge.classList.add('active');
    btnForward.classList.add('active-pressed');
  } else if (command === 'B') {
    motorStateBadge.textContent = 'BACKWARD (B)';
    motorStateBadge.classList.add('active');
    btnBackward.classList.add('active-pressed');
  } else {
    motorStateBadge.textContent = 'STOPPED (S)';
    motorStateBadge.classList.add('stopped');
    btnStop.classList.add('active-pressed');
  }
}

// Keyboard shortcuts for driving controls
function handleKeyboardControl(e) {
  // Only handle keys if connected and not typing into an input
  if (!port) return;
  if (['input', 'textarea'].includes(document.activeElement.tagName.toLowerCase())) return;

  const key = e.key;

  if (key === 'ArrowUp' || key === 'w' || key === 'W') {
    e.preventDefault();
    sendMotorCommand('F');
  } else if (key === 'ArrowDown' || key === 's' || key === 'S') {
    e.preventDefault();
    sendMotorCommand('B');
  } else if (key === ' ' || key === 'x' || key === 'X' || key === 'Escape') {
    e.preventDefault();
    sendMotorCommand('S');
  }
}

// -----------------------------------------------------------------------------
// Connection State Management & Reset
// -----------------------------------------------------------------------------
function updateConnectionUI(isConnected) {
  if (isConnected) {
    connectionStatusText.textContent = 'CONNECTED';
    connectionDot.className = 'status-dot connected';
    connectBtnText.textContent = 'DISCONNECT';
    connectBtn.classList.add('connected-mode');

    // Enable motor controls
    btnForward.disabled = false;
    btnStop.disabled = false;
    btnBackward.disabled = false;
  } else {
    connectionStatusText.textContent = 'DISCONNECTED';
    connectionDot.className = 'status-dot disconnected';
    connectBtnText.textContent = 'CONNECT';
    connectBtn.classList.remove('connected-mode');

    // Disable motor controls
    btnForward.disabled = true;
    btnStop.disabled = true;
    btnBackward.disabled = true;
  }
}

function resetDashboardToDisconnected() {
  updateConnectionUI(false);

  // Strictly adhere to: Do not use simulated sensor values when hardware is disconnected
  voltageValue.textContent = '--';
  adcValue.textContent = '--';
  if (rpmValue) rpmValue.textContent = '--';
  batteryFill.style.width = '0%';
  chargingBadge.textContent = '--';
  chargingBadge.className = 'badge badge-charging idle';
  chargingBoltIcon.classList.remove('active');

  pitchValue.textContent = '--';
  slopeBadge.textContent = '--';
  slopeBadge.className = 'badge badge-slope neutral';
  if (slopeIndicatorText) slopeIndicatorText.textContent = '--';
  if (slopeIndicatorPill) slopeIndicatorPill.className = 'slope-indicator-pill disconnected';
  if (recoveryBanner) recoveryBanner.classList.add('hidden');

  accelX.textContent = '--';
  accelY.textContent = '--';
  accelZ.textContent = '--';

  motorStateBadge.textContent = 'STOPPED';
  motorStateBadge.className = 'badge badge-motor stopped';

  btnForward.classList.remove('active-pressed');
  btnStop.classList.remove('active-pressed');
  btnBackward.classList.remove('active-pressed');
}
