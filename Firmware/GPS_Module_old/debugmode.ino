// Empties all bytes from incoming serial buffer.
// Used by Debug mode
void emptySerialBuffer() {
  while (usb.available()) {usb.read();}
}//emptySerialBuffer()



// Called when Debug mode is activated;
// all normal board functions will cease.
// Only purpose is to respond to serial
// commands.
void debugMode() {

  while (true) {

    // Empty buffer
    emptySerialBuffer();

    // Wait for command
    while (!usb.available()) {}
    uint8_t cmd = usb.read();

    if (cmd == 'I') {
      // "Identify" command; return board name
      usb.println(F("[MDE] NASONOV GPS"));
    }//if
    if (cmd == 'F') {
      // "FlashInfo" command; return flash usage stats
      usb.print("[MDE] ");
      usb.print(table.getMaxSize());
      usb.print(F(","));
      usb.println(table.getCurSize());   
    }//if
    if (cmd == 'D') {
      // "DumpFlash" command; dump all flash contents via serial
      table.beginDataDump(&usb); //func overload; dump everything
    }//if
    if (cmd == 'E') {
      // "EraseFlash" command; completely erase contents of flash.
      // Should be restarted afterwards
      usb.println(F("[MDE] Erasing Flash"));
      SerialFlash.eraseAll();
      while (SerialFlash.ready() == false) {}
      usb.println(F("[MDE] Complete"));
    }//if
    if (cmd == 'Q') {
      // QUERY SENSORS
      usb.print(F("[MDE] Null"));
    }//if

  }//while

}//debugMode()
