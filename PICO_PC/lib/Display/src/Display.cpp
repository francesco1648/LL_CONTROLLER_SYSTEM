#include "Display.h"

Display::Display() {
  
}

/*
 * Initialization of the display.
 * Sets basic graphic settings and shows the team's logo.
 */
void Display::begin() {
  display.begin(DISPLAY_ADDR, true);
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();
  showLogo(); // Show the logo on startup
}

/**
 * Displays the team's logo.
 */
void Display::showLogo() {
  display.clearDisplay();
  display.drawBitmap(44, 4,  bitmap_logo_isaac, 41, 58, 1);
  display.display();
}

/**
 * Displays WiFi status and properties.
 */
void Display::showWifi() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_wifi, 26, 21, 1);
  display.setCursor(0, 32);
  display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  display.display();
}

/**
 * Displays battery status.
 */
void Display::showBattery() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_bat, 23, 11, 1);
  display.setCursor(0, 24);
  display.printf("Voltage:  %.2fV\n\n", battery.readVoltage());
  display.printf("Charge:   %.1f%%\n", battery.chargePercent());
  display.display();
}

/**
 * Display software version, and module address.
 */
void Display::showVersion() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_upd, 24, 24, 1);
  display.setCursor(0, 32);
  display.printf("Version: %s\n\n", VERSION);
  display.printf("Can ID:  %#04X",CAN_ID);
  display.display();
}

/**
 * Handles display via recorded interrupts. 
 * This function needs to be called as often as possible.
 * If no button is pressed for more than #MENUTIMEOUT automatically returns to the logo.
 */
void Display::handleGUI() {
  bool change = false;

  if(nav > 0) {
    change = true;
    nav--;
    menupos++;
    if (menupos >= NMENUS) menupos = 0;
    else menutime = millis();
  } else if(menupos != 0 && millis() - menutime > (MENUTIMEOUT * 1000)) {
    change = true;
    menupos = 0;
  }

  switch (menupos) {
    case 0:
      if(change) showLogo();
      break;
    case 1:
      if (change) showBattery();
      break;
    case 2:
      if(change) showWifi();
      break;
    case 3:
      if (change) showVersion();
      break;
    case 4:
      // Error menu
      if (change) showCurrentError(idx);
      break;
    case 5:
      if (change) showMotorStatus(idx_motorStatus);
      break;
  }
}

/**
 * NAV button ISR.
 */
void Display::navInterrupt() {
  int now = millis();
  if (now - lastnav > DEBOUNCE) {
    nav++;
    lastnav = now;
  }
}


/**
 * OK button ISR.
 * Added errors to the list are shown one by one with each button press.
 * If no error is present, the button press is counted as a menu change.
 */
void Display::okInterrupt() {
  int now = millis();
  if (now - lastok > DEBOUNCE) {
    if (menupos == 4 && errorCount > 0) {     // in the error menu and there are errors
      idx = (idx + 1) % errorCount;       // cycle through errors with each press
      showCurrentError(idx);                // Show the next error message 
    } else {
    ok++;
    }

        if (menupos == 5 && StatusCount > 0) {     // in the error menu and there are errors
      idx_motorStatus = (idx_motorStatus + 1) % StatusCount;       // cycle through errors with each press
      showMotorStatus(idx_motorStatus);                // Show the next error message 
    } else {
    ok++;
    }
    lastok = now;
  }
}

/*
 * Displays the current error message based on the index.
 * Shows two errors at once: the current and the next one below.
 * Writes to both sides of the screen 
 */
void Display::showCurrentError(int idx) {
    // Check if there is error
    if (errorCount == 0) {
        display.clearDisplay();
        display.setCursor(0, 16);
        display.printf("No errors.");
        display.display();
        return;
    // If both screen sider are occupied
    // cleart he screen
    } else if (errorTopPrinted && errorBottomPrinted) {
        errorTopPrinted = false;
        errorBottomPrinted = false;
        display.clearDisplay();
    } 


    // First, try to print the error on the upper side of the screen
    if (!errorTopPrinted) {
        display.clearDisplay();
        if (errorCount > 1) {
          // Draw right arrow to indicate more errors are available
          display.drawBitmap(display.width() - 10, display.height() - 10, bitmap_arrow_right, 8, 8, 1);
        }
        display.setCursor(0, 0);
        display.printf("[%d] %s", idx, errorList[idx].Msg);
        errorTopPrinted = true;
        display.display();
        return;
    }

    // If the upper side of the screen is occupied
    // try writing error to the bottom
    if (!errorBottomPrinted && (idx < errorCount)) {
        display.setCursor(0, 30);
        display.printf("[%d] %s", idx, errorList[idx].Msg);
        errorBottomPrinted = true;
        display.display();
        return;
    }
    
}

void Display::showMotorStatus(int idx_motorStatus) {
    // Check if there is error
    if (errorCount == 0) {
        display.clearDisplay();
        display.setCursor(0, 16);
        display.printf("No errors.");
        display.display();
        return;
    // If both screen sider are occupied
    // cleart he screen
    } else if (errorTopPrintedMotor && errorBottomPrintedMotor) {
        errorTopPrintedMotor = false;
        errorBottomPrinted = false;
        display.clearDisplay();
    } 


    // First, try to print the error on the upper side of the screen
    if (!errorTopPrintedMotor) {
        display.clearDisplay();
        if (errorCount > 1) {
          // Draw right arrow to indicate more errors are available
          display.drawBitmap(display.width() - 10, display.height() - 10, bitmap_arrow_right, 8, 8, 1);
        }
        display.setCursor(0, 0);
        display.printf("[%d] %s", idx, motorStatus[idx_motorStatus].Msg);
        errorTopPrintedMotor = true;
        display.display();
        return;
    }

    // If the upper side of the screen is occupied
    // try writing error to the bottom
    if (!errorBottomPrintedMotor && (idx < errorCount)) {
        display.setCursor(0, 30);
        display.printf("[%d] %s", idx, errorList[idx].Msg);
        errorBottomPrintedMotor = true;
        display.display();
        return;
    }
    
}

/*
 * Adds an error message to the list of errors.
 */
void Display::addError(const char* Msg, int cursorY) {
  // Check if the error message is a CAN bus error
  bool isCanBusError = (strstr(Msg, "CAN") != nullptr);

  // If it's a CAN bus error, check if one already exists
  if (isCanBusError) {
    for (int i = 0; i < errorCount; ++i) {
      if (strstr(errorList[i].Msg, "CAN") != nullptr) {
        // Already have a CAN bus error, do not add another
        return;
      }
    }
  }

  if (errorCount < 10) { // Check if there's space for a new error
    errorList[errorCount].Msg = Msg;
    errorList[errorCount].cursorY = cursorY;
    errorCount++;
  } else {
    // Error list is full, ignore the new error
    Serial.println("Error list is full. Cannot add new error.");
  }
}


void Display::addStatus(const char* Msg, int cursorY) {


  if (errorCount < 10) { // Check if there's space for a new error
    motorStatus[StatusCount].Msg = Msg;
    motorStatus[StatusCount].cursorY = cursorY;
    StatusCount++;
  } else {
    // Error list is full, ignore the new error
    Serial.println("StatusCount list is full. Cannot add new error.");
  }
}