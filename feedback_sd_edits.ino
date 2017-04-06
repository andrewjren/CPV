void loop()
{
  // Serial commands to start/stop optimization, measure pyranometer voltage, and get the current position of the stages 
  if(Serial.available() > 0)
  {
    serialComm = Serial.readStringUntil('\n');    
    if(serialComm == "stopcpv")
    {
      enableCPV = false;
    }
    else if(serialComm == "startcpv")
    {
      enableCPV = true;
    }
    else if(serialComm == "dataon")
    {
      logData = true;
      SD.begin(chipSelect);
      
      // create a new file
      char filename[] = "CPVRun00.CSV";
      
      for (uint8_t i = 0; i < 100; i++)
      {
        filename[6] = i/10 + '0';
        filename[7] = i%10 + '0';
        if (! SD.exists(filename)) 
        {
          // only open a new file if it doesn't exist
          logSD = SD.open(filename, FILE_WRITE); 
          break;  // leave the loop!
        }
      }
      
      String timestamp;
      
      timestamp = String(now.month());
      timestamp += '/' + String(now.day()) + '/' + String(now.year()) + '\t' + String(now.hour()) + ':' + String(now.minute()) + ':' + String(now.second());
      logSD.println(timestamp);
      logSD.println(headerSD);
    }
    else if(serialComm == "dataoff")
    {
      logData = false;
      logSD.close();
    }
    else if(serialComm == "measpyr")
    {      
      Serial.println(readAnalog(pinPyro, iter8));
    }
    else if(serialComm == "measpv")
    {
      Serial.println(readAnalog(pinPV, iter8));
    }
    else if(serialComm == "meascpv")
    {
      Serial.println(readAnalog(pinCPV, iter8));
    }
    else if(serialComm == "getposx")
    {
      posX = sendCommand(portA, axisX, getPos, 0);
      posY = sendCommand(portA, axisY, getPos, 0);
      Serial.print(posX);
      Serial.print(',');
      Serial.println(posY);
    }
    else if(serialComm == "cpvsmu")
    {
      digitalWrite(cpvSMU, HIGH);
      delay(5);
      digitalWrite(cpvSMU, LOW);
    }
    else if(serialComm == "cpvtia")
    {
      digitalWrite(cpvTIA, HIGH);
      delay(5);
      digitalWrite(cpvTIA, LOW);
    }
    else if(serialComm == "pvsmu")
    {
      digitalWrite(pvSMU, HIGH);
      delay(5);
      digitalWrite(pvSMU, LOW);
    }
    else if(serialComm == "pvtia")
    {
      digitalWrite(pvTIA, HIGH);
      delay(5);
      digitalWrite(pvTIA, LOW);
    }
    
    if(serialComm.substring(0, 5) == "setdt")
    {
      comm1 = serialComm.substring(6);
      intervalCPV = comm1.toInt();
    }
    if(serialComm.substring(0, 5) == "setdz")
    {
      comm1 = serialComm.substring(6);
      increment = comm1.toInt();
    }
    if(serialComm.substring(0, 5) == "setav")
    {
      comm1 = serialComm.substring(6);
      iter8 = comm1.toInt();
    }
    if(serialComm.substring(0, 5) == "setpv")
    {
      comm1 = serialComm.substring(6);
      dpData = comm1.toInt();

      // Generating two bytes to be sent to the digipot shift register, MSByte first
      dpCommand[0] = (1024 + dpData) >> 8;
      dpCommand[1] = dpData & 255;
  
      Wire.beginTransmission(0x2C);
      Wire.write(dpCommand, 2);
      Wire.endTransmission();
    }
    if(serialComm.substring(0, 6) == "setcpv")
    {
      comm1 = serialComm.substring(7);
      dpData = comm1.toInt();

      // Generating two bytes to be sent to the digipot shift register, MSByte first
      dpCommand[0] = (1024 + dpData) >> 8;
      dpCommand[1] = dpData & 255;

      Wire.beginTransmission(0x2F);
      Wire.write(dpCommand, 2);
      Wire.endTransmission(); 
    }
  }

  // Running optimization function along X and Y
  currentMillis = millis();
  if((currentMillis - millisCPV >= intervalCPV) && (enableCPV == true))
  {   
    millisCPV = currentMillis;     
    
    posX = sendCommand(portA, axisX, getPos, 0);
    posY = sendCommand(portA, axisY, getPos, 0);     
    
    dx = 0;
    dy = 0;
    
    optimize(um(increment));

    if(logData == true)
    {
      String dataSD;

      // Find distance between starting and ending positions in microns
      int x1 = posX;
      int y1 = posY;      
      posX = sendCommand(portA, axisX, getPos, 0);
      posY = sendCommand(portA, axisY, getPos, 0);
      int x2 = posX - x1;
      int y2 = posY - y1;
      float d = sqrt(x2*x2 - y2*y2) * umResolution;

      // Assemble string and write to SD card
      dataSD = String(voltage);
      dataSD += " , " + String(dx) + " , " + String(dy) + " , " + String(d);
      logSD.println(dataSD);      
    }
  }
}

void optimize(long increment)
{ 
  int moves = 0; 
  int voltage_x1 = 0;
  int voltage_y1 = 0;
  int dFdx = 0;
  int dFdy = 0;
  long delX;
  long delY;
  double step = 1; // step size for relative movement increment. 
  // Get starting conditions before optimizing
  voltage = readAnalog(pinCPV, iter8); 

  // Print voltage to LCD
  lcdPrint(voltage);

  // Move one increment in +X direction and get new voltage and position
  replyData = sendCommand(portA, axisX, moveRel, increment);
  moves++;
  delay(dLay);
  voltage_x1 = readAnalog(pinCPV, iter8);

  // Move one increment in +Y direction and get new voltage and position
  replyData = sendCommand(portA, axisY, moveRel, increment);
  moves++;
  delay(dLay);
  voltage_y1 = readAnalog(pinCPV, iter8);
  
  // Print voltage to LCD
  lcdPrint(voltage);

  // slopes in each direction
  dFdx = voltage_x1 - voltage;
  dFdy = voltage_y1 - voltage_x1;

  // stop criteria is confusing for steepest descent
  while(dFdx > 5 || dFdy > 5) // where 5 is the tolerance of error - as tracking reaches peak, gradient should decrease to 0 in each direction
  {
    previousVoltage = voltage;

    delX = dFdx * step * increment;
    delY = dFdy * step * increment;
    replyData = sendCommand(portA, axisX, moveRel, delX);
    replyData = sendCommand(portA, axisY, moveRel, delY);
    delay(dLay);
    voltage = readAnalog(pinCPV, iter8);    

    // Print voltage to LCD
    lcdPrint(voltage);

    replyData = sendCommand(portA, axisX, moveRel, increment);
    moves++;
    delay(dLay);
    voltage_x1 = readAnalog(pinCPV, iter8);

    // Move one increment in +Y direction and get new voltage and position
    replyData = sendCommand(portA, axisY, moveRel, increment);
    moves++;
    delay(dLay);
    voltage_y1 = readAnalog(pinCPV, iter8);
    
    // Print voltage to LCD
    lcdPrint(voltage);

    // slopes in each direction
    dFdx = voltage_x1 - voltage;
    dFdy = voltage_y1 - voltage_x1;

  }  
   
   if(axis == axisX)
   {
     dx = increment * moves * umResolution;
   }
   else if(axis == axisY)
   {
     dy = increment * moves * umResolution; 
   }
}

void lcdPrint(int volt_output)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print((float(volt_output)/1023.0) * 5.0);
  lcd.print(" V");
}