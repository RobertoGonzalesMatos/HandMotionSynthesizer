import { test, expect } from "@playwright/test";
import { before, beforeEach } from "node:test";

// FSM state enums
const state_enums = {
  s_INIT : 0,
  s_REG_CALC : 1,
  s_REG_WAIT : 2,
  s_GESTURE_WAIT : 3,
  s_GESTURE_CALC : 4,
} 

//define initial FSM state and inputs
const fsm_state = {noteFrequency: 0, vibratoLevel: 0, gestureMode : false, savedClock : 0, state: state_enums.s_INIT, harmonies: [false,false,false], wasRecording: false};
const inputs = { x: 0, y: 0, z: 0, drumMode: false, clock: 0, isRecording: false};

// Mock FSM update function
function mockUpdateFSM(currState, inputs) {
  const ret = { ...currState };
  ret.serialMessages = [];
  switch (ret.state) {
    case state_enums.s_INIT:
      // Input during initial state is registered as a note
      if (inputs.y !== 0) {
        ret.state = state_enums.s_REG_CALC;
        ret.noteFrequency = inputs.y;
        ret.serialMessages.push("NOTE:C");
      }
      break;
    case state_enums.s_REG_CALC:
      if (ret.savedClock - clock > 5) {
        ret.state = state_enums.s_REG_WAIT;
        ret.savedClock = inputs.clock;
      }
      break;
    case state_enums.s_REG_WAIT:
      // Check for silence
      if (inputs.z > 25) {
        ret.serialMessages.push("NOTE:0");
        ret.savedClock = inputs.clock;
        ret.state = state_enums.s_REG_CALC;
      }
      // Check for new note
      if (inputs.y !== 0 && inputs.z <= 25) {
        ret.noteFrequency = inputs.y;
        ret.serialMessages.push("NOTE:C");
        ret.state = state_enums.s_REG_CALC;
      }
      // Check for recording state change
      if (inputs.isRecording && !ret.wasRecording) {
        ret.serialMessages.push("REC:[");
        for (const note of ["A", "B", "C", "D"]) {
          ret.serialMessages.push(note);
        }
      }
      // Ending recording
      if (!inputs.isRecording && ret.wasRecording) {
        ret.serialMessages.push("]");
      }
      ret.wasRecording = inputs.isRecording;
      // Check for drum mode activation
      if (inputs.drumMode) {
        ret.gestureMode = true;
        ret.state = state_enums.s_GESTURE_WAIT;
      } 
      break;
    case state_enums.s_GESTURE_WAIT:
      // Check for drum mode deactivation
      if (!inputs.drumMode) {
        ret.gestureMode = false;
        ret.state = state_enums.s_REG_WAIT
      }
    default:
      break;
  }

  // Handle harmonies
  if (ret.harmonies[0]) ret.serialMessages.push("1_HARM:G");
  if (ret.harmonies[1]) ret.serialMessages.push("2_HARM:E");
  if (ret.harmonies[2]) ret.serialMessages.push("3_HARM:C");

  return ret;
}

test.describe("Arduino-UI Integration", () => {
  test.beforeEach(async ({ page }) => {
    
     await page.goto(`file://${__dirname}/../UI.html`);
     await page.waitForLoadState("load");
     await page.waitForFunction(
       () => typeof window.handleArduinoMessage === "function" && typeof window.handleRecordingLine === "function"
     );

     await page.evaluate(() => {
       document.querySelector("#notes-layer").innerHTML = "";
       document.querySelector("#current-note").textContent = "-";
       window.drumMode = false;
       window.isTesting = true;
     });

  });

  test("Initial state shows no note", async ({ page }) => {
    const result = mockUpdateFSM(fsm_state, { ...inputs });

    for (const msg of result.serialMessages) {
      await page.evaluate((msg) => window.handleArduinoMessage(msg), msg);
    }

    const noteElement = page.locator("#current-note");
    const scoreElement = page.locator("#notes-layer .note-head");
    const state = result.state;

    await expect(noteElement).toHaveText("-", { timeout: 2000 });
    await expect(scoreElement).toHaveCount(0, { timeout: 2000 });
    expect(state).toBe(state_enums.s_INIT);
  });

  test("Initial note is shown visually in UI", async ({ page }) => {
    const result = mockUpdateFSM(fsm_state, { ...inputs, y: 5 });

    for (const msg of result.serialMessages) {
      await page.evaluate((msg) => window.handleArduinoMessage(msg), msg);
    }

    const noteElement = page.locator("#current-note");
    const scoreElement = page.locator("#notes-layer .note-head");
    const state = result.state;

    await expect(noteElement).toHaveText("C", { timeout: 2000 });
    await expect(scoreElement).toHaveCount(1, { timeout: 2000 });
    expect(state).toBe(state_enums.s_REG_CALC);
  });

  test("Silence (NOTE:0) is shown visually in UI", async ({ page }) => {
    const result = mockUpdateFSM(
      { ...fsm_state, state: state_enums.s_REG_WAIT },
      { ...inputs, z: 30, y: 5 }
    );

    for (const msg of result.serialMessages) {
      await page.evaluate((msg) => window.handleArduinoMessage(msg), msg);
    }

    const noteElement = page.locator("#current-note");
    const state = result.state;
    await expect(noteElement).toHaveText("-", { timeout: 2000 });
    expect(state).toBe(state_enums.s_REG_CALC);
  });

  test("Regular note is shown visually in UI", async ({ page }) => {
    const result = mockUpdateFSM({...fsm_state, state: state_enums.s_REG_WAIT}, { ...inputs, y: 5 });

    for (const msg of result.serialMessages) {
      await page.evaluate((msg) => window.handleArduinoMessage(msg), msg);
    }

    const noteElement = page.locator("#current-note");
    const scoreElement = page.locator("#notes-layer .note-head");
    const state = result.state;

    await expect(noteElement).toHaveText("C", { timeout: 2000 });
    await expect(scoreElement).toHaveCount(1, { timeout: 2000 });
    expect(state).toBe(state_enums.s_REG_CALC);
  });

  test("Harmony note is added to score", async ({ page }) => {
    const result = mockUpdateFSM({...fsm_state, state: state_enums.s_REG_WAIT, harmonies: [true, true, true]}, { ...inputs, y: 5 });

    for (const msg of result.serialMessages) {
      await page.evaluate((msg) => window.handleArduinoMessage(msg), msg);
    }

    const noteElement = page.locator("#current-note");
    const scoreElement = page.locator("#notes-layer .note-head");
    const state = result.state;

    await expect(noteElement).toHaveText("C", { timeout: 2000 });
    await expect(scoreElement).toHaveCount(4, { timeout: 2000 });
    expect(state).toBe(state_enums.s_REG_CALC);
  });

  test("Drum mode activation sends correct message", async ({ page }) => {
    const modeButton = page.locator("#drum");
    const log = page.locator("#log");
    await expect(modeButton).toHaveText("Drum Mode");

    await modeButton.click();
    await expect(modeButton).toHaveText("Regular Mode");
    await expect(log).toContainText("Switched to drum mode.");

     const gesture = mockUpdateFSM(
       { ...fsm_state, state: state_enums.s_REG_WAIT },
       { ...inputs, drumMode: true }
     );

     if (gesture.gestureMode) {
      await page.evaluate(() => {
        window.drumMode = true;
      });
       
     }
    const state = gesture.state;
    expect(state).toBe(state_enums.s_GESTURE_WAIT);

     const regular = mockUpdateFSM(
       { ...fsm_state, state: state_enums.s_GESTURE_WAIT },
       { ...inputs, drumMode: false }
     );

     if (!regular.gestureMode) {
       await page.evaluate(() => {
         window.drumMode = false;
       });
     }

    const regularState = regular.state;
    expect(regularState).toBe(state_enums.s_REG_WAIT);
  });

  test("Recording start and stop", async ({ page }) => {
    let current_fsm = {...fsm_state, state: state_enums.s_REG_WAIT};
    const recordButton = page.locator("#record");
    const log = page.locator("#log");
    await expect(recordButton).toContainText("Record");

    await recordButton.click();
    current_fsm = mockUpdateFSM(current_fsm, { ...inputs, isRecording: true });
    for (const msg of current_fsm.serialMessages) {await page.evaluate((msg) => 
      window.handleRecordingLine(msg), msg
      );
    }

    await expect(recordButton).toContainText("Stop Recording");
    await expect(log).toContainText("Recording started...");

    await recordButton.click();
    current_fsm = mockUpdateFSM(current_fsm, { ...inputs, isRecording: false });
    for (const msg of current_fsm.serialMessages) {await page.evaluate((msg) => 
        window.handleRecordingLine(msg), msg
      );
    }

    await expect(log).toContainText("Recording stopped. Waiting for data...");
    await expect(log).toContainText("Full recording received.");

    const save = page.locator("#save");
    const discard = page.locator("#discard");
    await expect(save).toContainText("Save");
    await expect(discard).toContainText("Discard");
  });
});