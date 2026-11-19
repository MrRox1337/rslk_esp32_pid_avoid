#pragma once
// Text command interpreter shared by Serial and the web page.
#include <Arduino.h>

// Everything printed through `con` goes to Serial AND (while capturing) into `buf`,
// so the web server can return command output as the HTTP reply.
class TeePrint : public Print {
public:
  String buf;
  bool capture = false;
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *b, size_t n) override;
};
extern TeePrint con;
extern bool telemetry;

void runCmd(char *line);   // modifies line (strtok)
void printHelp();
void consoleStep();        // call every loop: reads Serial, prints telemetry
