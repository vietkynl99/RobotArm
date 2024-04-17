#ifndef INC_COMMANDLINE_H_
#define INC_COMMANDLINE_H_

#include "main.h"
#include "RobotArm.h"
#include "Vector.h"

#include <string>

using namespace std;

typedef bool (*CommandCallback)(string);
typedef struct
{
    string name;
    CommandCallback callback;
    string description;
} Command;

class CommandLine
{
private:
    static Vector<Command> mCommandList;

public:
    static void init();
    static void onCharacterReceived(char ch);

    static bool install(string name, CommandCallback callback, string description = "");

private:
    static void putstr(const char* str);
    static void putstrln(const char* str);
    static string trim(const string &str);
    static bool isSingleName(const string &name);
    static Vector<string> getCommandList(string prefix = "");
    static string vector2string(const Vector<string> &Vector);
    static string getCommonName(const Vector<string> &nameList);
    static bool hasCommand(const string &name);
    static bool getCommandByName(Command &command, string name);

    static bool onCommandClearScreen(string params);
    static bool onCommandHelp(string commandName);
};

#endif /* INC_COMMANDLINE_H_ */
