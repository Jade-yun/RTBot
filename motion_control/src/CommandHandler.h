#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include "Parameters/SharedDataType.h"

class Robot;

class CommandHandler
{
public:
//    explicit CommandHandler(Robot* _context);
    explicit CommandHandler(Robot* _context);

    void parseCommand(HighLevelCommand &_cmd);
    uint32_t getSpace();
    void clearFifo();
    void emergencyStop();

private:
    Robot* context;
};

#endif // COMMANDHANDLER_H
