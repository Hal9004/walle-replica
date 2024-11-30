int offset = 55;
int width = 205;
int y = 45;
int t = 8;
int g = 8;

// ------------------------------------------------
// Init screen + boot animation
// ------------------------------------------------

void bootScreen(){
  tft.fillScreen(ST77XX_BLACK);
  // text
  tft.setCursor(offset-5, 15);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setTextWrap(false);
  tft.print("SOLAR CHARGE LEVEL");

  delay(1000);

  // SUN
  tft.fillCircle(offset+30, y+30, 15, ST77XX_YELLOW);

  tft.fillRect(offset+30-2, y, 4, 60, ST77XX_YELLOW);
  tft.fillRect(offset, y+28, 60, 4, ST77XX_YELLOW);

  tft.drawLine(offset+15, y+5, offset+45, y+55, ST77XX_YELLOW);
  tft.drawLine(offset+15-1, y+5, offset+45-1, y+55, ST77XX_YELLOW);
  tft.drawLine(offset+15+1, y+5, offset+45+1, y+55, ST77XX_YELLOW);
  tft.drawLine(offset+15-1, y+5+1, offset+45-1, y+55+1, ST77XX_YELLOW);
  tft.drawLine(offset+15+1, y+5-1, offset+45+1, y+55-1, ST77XX_YELLOW);

  tft.drawLine(offset+5, y+15, offset+55, y+45, ST77XX_YELLOW);
  tft.drawLine(offset+5, y+15-1, offset+55, y+45-1, ST77XX_YELLOW);
  tft.drawLine(offset+5, y+15+1, offset+55, y+45+1, ST77XX_YELLOW);
  tft.drawLine(offset+5+1, y+15-1, offset+55+1, y+45-1, ST77XX_YELLOW);
  tft.drawLine(offset+5-1, y+15+1, offset+55-1, y+45+1, ST77XX_YELLOW);

  tft.drawLine(offset+5, y+45, offset+55, y+15, ST77XX_YELLOW);
  tft.drawLine(offset+5, y+45-1, offset+55, y+15-1, ST77XX_YELLOW);
  tft.drawLine(offset+5, y+45+1, offset+55, y+15+1, ST77XX_YELLOW);
  tft.drawLine(offset+5-1, y+45-1, offset+55-1, y+15-1, ST77XX_YELLOW);
  tft.drawLine(offset+5+1, y+45+1, offset+55+1, y+15+1, ST77XX_YELLOW);

  tft.drawLine(offset+15, y+55, offset+45, y+5, ST77XX_YELLOW);
  tft.drawLine(offset+15-1, y+55, offset+45-1, y+5, ST77XX_YELLOW);
  tft.drawLine(offset+15+1, y+55, offset+45+1, y+5, ST77XX_YELLOW);
  tft.drawLine(offset+15+1, y+55+1, offset+45+1, y+5+1, ST77XX_YELLOW);
  tft.drawLine(offset+15-1, y+55-1, offset+45-1, y+5-1, ST77XX_YELLOW);

  tft.fillCircle(offset+30, y+30, 18, ST77XX_BLACK);
  tft.fillCircle(offset+30, y+30, 15, ST77XX_YELLOW);
  tft.fillCircle(offset+30, y+30, 11, ST77XX_BLACK);
  
  delay(1000);

  // Battery animate
  tft.fillRect(offset+width*0.45, y+9*(t+g), width*0.55, 25, ST77XX_YELLOW);
  delay(1000);
  tft.fillRect(offset+width*0.45, y+8*(t+g), width*0.55, t, ST77XX_YELLOW);
  delay(800);
  tft.fillRect(offset+width*0.45, y+7*(t+g), width*0.55, t, ST77XX_YELLOW);
  delay(100);
  tft.fillRect(offset+width*0.45, y+6*(t+g), width*0.55, t, ST77XX_YELLOW);
  delay(100);
  tft.fillRect(offset+width*0.45, y+5*(t+g), width*0.55, t, ST77XX_YELLOW);
  delay(100);
  tft.fillRect(offset+width*0.45, y+4*(t+g), width*0.55, t, ST77XX_YELLOW);
  delay(100);
  tft.fillRect(offset+width*0.45, y+3*(t+g), width*0.55, t, ST77XX_YELLOW);
  delay(100);
  tft.fillRect(offset+width*0.45, y+2*(t+g), width*0.55, t, ST77XX_YELLOW);
  delay(100);
  tft.fillRect(offset+width*0.45, y+1*(t+g), width*0.55, t, ST77XX_YELLOW);
  delay(100);
  tft.fillRect(offset+width*0.45, y, width*0.55, t, ST77XX_YELLOW);
  delay(2000);
}

// -------------------------------------------------------------------
/// Battery level detection
// -------------------------------------------------------------------

void displayBatteryLevel(int batt){

  if (booted) {
    if (batt > 10) {
      tft.fillRect(offset+width*0.45, y+9*(t+g), width*0.55, 25, ST77XX_YELLOW);
    } else {
      tft.fillRect(offset+width*0.45, y+9*(t+g), width*0.55, 25, ST77XX_RED);
      tft.setCursor(offset+width*0.45+15, y+9*(t+g)+5);
      tft.setTextColor(ST77XX_BLACK);
      tft.print("WARNING");
    }
    if (batt > 20) {
      tft.fillRect(offset+width*0.45, y+8*(t+g), width*0.55, t, ST77XX_YELLOW);
    } else {
      tft.fillRect(offset+width*0.45, y+8*(t+g), width*0.55, t, ST77XX_BLACK);
    }
    if (batt > 30) {
      tft.fillRect(offset+width*0.45, y+7*(t+g), width*0.55, t, ST77XX_YELLOW);
    } else {
      tft.fillRect(offset+width*0.45, y+7*(t+g), width*0.55, t, ST77XX_BLACK);
    }
    if (batt > 40) {
      tft.fillRect(offset+width*0.45, y+6*(t+g), width*0.55, t, ST77XX_YELLOW);
    } else {
      tft.fillRect(offset+width*0.45, y+6*(t+g), width*0.55, t, ST77XX_BLACK);
    }
    if (batt > 50) {
      tft.fillRect(offset+width*0.45, y+5*(t+g), width*0.55, t, ST77XX_YELLOW);
    } else {
      tft.fillRect(offset+width*0.45, y+5*(t+g), width*0.55, t, ST77XX_BLACK);
    }
    if (batt > 60) {
      tft.fillRect(offset+width*0.45, y+4*(t+g), width*0.55, t, ST77XX_YELLOW);
    } else {
      tft.fillRect(offset+width*0.45, y+4*(t+g), width*0.55, t, ST77XX_BLACK);
    }
    if (batt > 70) {
      tft.fillRect(offset+width*0.45, y+3*(t+g), width*0.55, t, ST77XX_YELLOW);
    } else {
      tft.fillRect(offset+width*0.45, y+3*(t+g), width*0.55, t, ST77XX_BLACK);
    }
    if (batt > 80) {
      tft.fillRect(offset+width*0.45, y+2*(t+g), width*0.55, t, ST77XX_YELLOW);
    } else {
      tft.fillRect(offset+width*0.45, y+2*(t+g), width*0.55, t, ST77XX_BLACK);
    }
    if (batt > 90) {
      tft.fillRect(offset+width*0.45, y+1*(t+g), width*0.55, t, ST77XX_YELLOW);
    } else {
      tft.fillRect(offset+width*0.45, y+1*(t+g), width*0.55, t, ST77XX_BLACK);
    }
    if (batt > 95) {
      tft.fillRect(offset+width*0.45, y, width*0.55, t, ST77XX_YELLOW);
    } else {
      tft.fillRect(offset+width*0.45, y, width*0.55, t, ST77XX_BLACK);
    }
  } else {
    tft.fillRect(offset, y+60, 100, 20, ST77XX_BLACK); // clear previous
    tft.setCursor(offset, y+60);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setTextWrap(false);
    tft.print("Batt: " + String(batt) + "%"); 
  }
}

int l;
void showDebugInfo(String debugInfo){
  if (l >= 5) {
    l = 0;
    tft.fillRect(10, 120, 100, 100, ST77XX_BLACK); // clear debug
  }
  tft.setCursor(10, 120+l*20);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setTextWrap(false);
  tft.print(debugInfo);
  l++;
}

void initDisplay(){
  tft.init(240, 280);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(offset, y);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setTextWrap(false);
  tft.print("Connect to WALL-E");
  tft.setCursor(offset, y+20);
  tft.print("to reboot him");
}

void showWarning(bool show){
  if (show){
    tft.fillRect(offset+width*0.45, y+9*(t+g), width*0.55, 25, ST77XX_RED);
    tft.setCursor(offset+width*0.45+15, y+9*(t+g)+5);
    tft.setTextColor(ST77XX_BLACK);
    tft.print("WARNING");
  } else {
    tft.fillRect(offset+width*0.45, y+9*(t+g), width*0.55, 25, ST77XX_BLACK);
  }
}