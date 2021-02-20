/********************************************************
    send data to display
******************************************************/
void printScreen() {
  int weightRate = 0;
  if (!sensorError) {
    u8g2.clearBuffer();

    // Draw weight progress bar
    u8g2.drawFrame(15, 58, 102, 4);
    weightRate = (weightBrew / weightSetpoint) * 100;
    if (weightRate < 0) {
      weightRate = 0;
    } else if (weightRate > 1) {
      weightRate = 100;
    }
    u8g2.drawLine(16, 59, weightRate + 16, 59);
    u8g2.drawLine(16, 60, weightRate + 16, 60);


    //draw (blinking) temp
    if (fabs(Input - setPoint) < 0.3) {
      if (isrCounter < 500) {
        u8g2.setCursor(2, 2);
        u8g2.setFont(u8g2_font_profont22_tf);
        u8g2.print(Input, 1);
        u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);
        u8g2.print(char(78));
        u8g2.setCursor(78, 2);
        u8g2.setFont(u8g2_font_profont22_tf);
        u8g2.print(setPoint, 1);
      }
    } else {
      u8g2.setCursor(2, 2);
      u8g2.setFont(u8g2_font_profont22_tf);
      u8g2.print(Input, 1);
      u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);
      u8g2.setCursor(56, 6);
      if (pidMode == 1) {
        u8g2.print(char(74));
      } else {
        u8g2.print(char(70));
      }
      u8g2.setCursor(79, 2);
      u8g2.setFont(u8g2_font_profont22_tf);
      u8g2.print(setPoint, 1);
    }

    u8g2.setFont(u8g2_font_profont10_tf);

    // Print time
    u8g2.setCursor(24, 30);
    u8g2.print("T: ");
    u8g2.print(bezugsZeit / 1000, 1);
    u8g2.print("/");
    if (OnlyPID == 1) {
      u8g2.print(brewtimersoftware, 0);             // deaktivieren wenn Preinfusion ( // voransetzen )
    }
    else
    {
      u8g2.print(totalbrewtime / 1000, 0);            // aktivieren wenn Preinfusion
    }
    u8g2.print(" (");
    u8g2.print(float(bezugsZeitAlt / 100) / 10, 1);
    u8g2.print(")");

    // print weight
    u8g2.setCursor(24, 38);
    u8g2.print("W: ");
    if (scaleFailure) {
      u8g2.print("fault");
    } else {
      if ( brewswitch == LOW) {
        u8g2.print(weight, 0);
      } else {
        u8g2.print(weightBrew, 0);
      }
      u8g2.print("/");
      u8g2.print(weightSetpoint, 0);
      u8g2.print(" (");
      u8g2.print(weightBrew, 1);
      u8g2.print(")");
    }

    // print pressure
    u8g2.setCursor(24, 46);
    u8g2.print("P: ");
    //u8g2.print(inputPressure);
    u8g2.setCursor(98, 48);
    if (pidMode == 1) {
      if (Output < 99) {
        u8g2.print(Output / 10, 2);
      } else if (Output < 999) {
        u8g2.print(Output / 10, 1);
      } else {
        u8g2.print(Output / 10, 0);
      }
      u8g2.print("%");
    } else {
      u8g2.drawBox(97, 48, 28, 9);   //Draw Box
      u8g2.setDrawColor(0);
      if (Output < 99) {
        u8g2.print(Output / 10, 2);
      } else if (Output < 999) {
        u8g2.print(Output / 10, 1);
      } else {
        u8g2.print(Output / 10, 0);
      }
      u8g2.print("%");
      u8g2.setDrawColor(1);
    }

    // print status information
    if (Offlinemodus == 0) {
      getSignalStrength();
      if (WiFi.status() != WL_CONNECTED) {
        u8g2.drawFrame(116, 28, 12, 12);
        u8g2.drawXBMP(118, 30, 8, 8, antenna_NOK_u8g2);
      } else {
        if (!Blynk.connected()) {
          u8g2.drawFrame(116, 28, 12, 12);
          u8g2.drawXBMP(118, 30, 8, 8, blynk_NOK_u8g2);
        }
      }
    } else {
      u8g2.drawFrame(116, 28, 12, 12);
      u8g2.setCursor(120, 30);
      u8g2.print("O");
    }
    u8g2.sendBuffer();    
  }
}
