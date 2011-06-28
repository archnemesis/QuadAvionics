#ifndef CLI_H
#define CLI_H

#include <inttypes.h>

#define CLI_BUFFER_LEN 256

class CLI_Class
{
    public:
        CLI_Class();
        
        uint8_t parseChar(uint8_t c);
        
        String readLine();
        String read(uint16_t len = 0);
        void flush();
    private:
        String input;
        uint8_t line_ready;
}

#endif
