#include "WProgram.h"
#include "CLI.h"

CLI_Class::CLI_Class()
{
    //
    // what to do?
    //
    
    index = 0;
}

uint8_t CLI_Class::parseChar(uint8_t c)
{
    input.append(c);
    
    if (input.indexOf('\n') > 0) {
        return 1;
    }
    
    return 0;
}

String readLine() const
{
    int line_end = input.indexOf('\n');
    
    if (line_end > 0) {
        String line = input.substring(0, line_end);
        if (input.length > (line_end + 1)) {
            input = input.substring(line_end + 1);
        }
        return line;
    }
    else {
        return String();
    }
}
