#include "CommandLine.h"

#define COMMAND_HEADER "> "

#define INPUT_CODE_CANCEL 0x3
#define INPUT_CODE_BACKSPACE 0x8
#define INPUT_CODE_TAB 0x9
#define INPUT_CODE_ESC 0x1B

#define ESCAPE_CODE_CLEAR "\033c"
#define ESCAPE_CODE_BACKSAPCE "\x7f"

#define DEBUG_SHOW_UNKNOWN_CODE 0

Vector<Command> CommandLine::mCommandList;

void CommandLine::init()
{
    install("clear", onCommandClearScreen, "clear\t: clear screen");
    install("help", onCommandHelp, "help\t: show available commands\r\nhelp [command]\t: show command's description");

    usbPrint(COMMAND_HEADER);
}

bool CommandLine::install(string name, CommandCallback callback, string description)
{
    name = trim(name);
    if (isSingleName(name) && !hasCommand(name))
    {
        trim(description);
        Command command{name, callback, description};
        mCommandList.push_back(command);
        return true;
    }
    return false;
}

void CommandLine::onCharacterReceived(char ch)
{
    static bool handled = 0;
    static string inputStr = "";
    static bool hasPrevTab = false;

    handled = false;
    if (ch == INPUT_CODE_CANCEL)
    {
        handled = true;
        inputStr = "";
        usbPrintln("^C");
    }
    else if (ch == INPUT_CODE_BACKSPACE)
    {
        if (inputStr.length() > 0)
        {
            inputStr.pop_back();
            usbPrint(ESCAPE_CODE_BACKSAPCE);
        }
    }
    else if (ch == INPUT_CODE_TAB)
    {
        string name = inputStr;
        trim(name);
        if (isSingleName(name))
        {
            Vector<string> list = getCommandList(name);
            if (list.size() > 0)
            {
                string substring = getCommonName(list);
                if (substring.length() > 0)
                {
                    unsigned int index = substring.find(name);
                    if (index != string::npos)
                    {
                        string suffix = substring.substr(name.length() + index);
                        if (suffix.length() > 0)
                        {
                            inputStr += suffix;
                            usbPrint(suffix.c_str());
                        }
                        else if (hasPrevTab)
                        {
                            usbPrintln("");
                            usbPrint(" ");
                            usbPrint(vector2string(list).c_str());
                            usbPrintln("");
                            usbPrint(COMMAND_HEADER);
                            usbPrint(inputStr.c_str());
                        }
                    }
                }
            }
        }
        hasPrevTab = true;
    }
    else if (ch == '\r' || ch == '\n')
    {
        handled = true;
        usbPrintln("");
    }
    else if (ch >= ' ' && ch <= '~')
    {
        inputStr += ch;
        usbPrint("%c", ch);
    }
#if DEBUG_SHOW_UNKNOWN_CODE
    else
    {
        usbPrint("{0x%d}", ch);
    }
#endif

    if (!handled)
    {
        return;
    }

    inputStr = trim(inputStr);
    if (inputStr.length() > 0)
    {
        string commandName = "", commandParams = "";
        size_t pos = inputStr.find(' ');
        if (pos != string::npos)
        {
            commandName = inputStr.substr(0, pos);
            commandParams = inputStr.substr(pos + 1);
            trim(commandName);
            trim(commandParams);
        }
        else
        {
            commandName = inputStr;
            commandParams = "";
        }

        bool found = false;
        for (int i = 0; i < mCommandList.size(); i++)
        {
            Command command = mCommandList.at(i);
            if (command.name == commandName)
            {
                found = true;
                if (!command.callback(commandParams))
                {
                    string str = "Invalid usage of command '" + command.name + "'";
                    usbPrintln(str.c_str());
                    str = "Show command's description: help " + command.name;
                    usbPrintln(str.c_str());
                }
            }
        }
        if (!found)
        {
            string str = "Unknown command '" + commandName + "'";
            usbPrintln(str.c_str());
        }
    }

    hasPrevTab = false;
    inputStr = "";
    usbPrint(COMMAND_HEADER);
}

string CommandLine::trim(const string &str)
{
    size_t start = str.find_first_not_of(" \t\n\r");
    size_t end = str.find_last_not_of(" \t\n\r");
    if (start == string::npos || end == string::npos)
    {
        return "";
    }
    return str.substr(start, end - start + 1);
}

bool CommandLine::isSingleName(const string &name)
{
    return name.find(' ') == string::npos;
}

Vector<string> CommandLine::getCommandList(string prefix)
{
    Vector<string> result;
    for (int i = 0; i < mCommandList.size(); i++)
    {
        if (prefix.length() == 0 || mCommandList.at(i).name.compare(0, prefix.size(), prefix) == 0)
        {
            result.push_back(mCommandList.at(i).name);
        }
    }
    return result;
}

string CommandLine::vector2string(const Vector<string> &vector)
{
    string result = "";
    for (int i = 0; i < vector.size(); i++)
    {
        result += vector.at(i);
        if (i != vector.size() - 1)
        {
            result += " ";
        }
    }
    return result;
}

string CommandLine::getCommonName(const Vector<string> &nameList)
{
    if (nameList.size() == 0)
    {
        return "";
    }
    if (nameList.size() == 1)
    {
        return nameList.at(0);
    }

    int keyIndex = 0;
    for (int i = 0; i < nameList.size(); i++)
    {
        if (nameList.at(i).length() < nameList.at(keyIndex).length())
        {
            keyIndex = i;
        }
    }

    string result = "";
    for (unsigned int i = 0; i < nameList.at(keyIndex).length(); i++)
    {
        for (int j = 0; j < nameList.size(); j++)
        {
            if (nameList.at(j)[i] != nameList.at(0)[i])
            {
                return result;
            }
        }
        result += nameList.at(keyIndex)[i];
    }
    return result;
}

bool CommandLine::hasCommand(const string &name)
{
    for (int i = 0; i < mCommandList.size(); i++)
    {
        if (mCommandList.at(i).name == name)
        {
            return true;
        }
    }
    return false;
}

bool CommandLine::getCommandByName(Command &command, string name)
{
    for (int i = 0; i < mCommandList.size(); i++)
    {
        if (mCommandList.at(i).name == name)
        {
            command = mCommandList.at(i);
            return true;
        }
    }
    return false;
}

bool CommandLine::onCommandClearScreen(string params)
{
    if (params.length() > 0)
    {
        return false;
    }
    usbPrint(ESCAPE_CODE_CLEAR);
    return true;
}

bool CommandLine::onCommandHelp(string commandName)
{
    if (!isSingleName(commandName))
    {
        return false;
    }

    if (commandName.length() == 0)
    {
        usbPrint("Available commands: ");
        usbPrintln(vector2string(getCommandList()).c_str());
        usbPrintln("Show command's description: help [command]");
        return true;
    }

    Command command;
    if (!getCommandByName(command, commandName))
    {
        string str = "Unknown command: " + commandName;
        usbPrintln(str.c_str());
        return true;
    }
    if (command.description.length() == 0)
    {
        string str = "Command '" + commandName + "' has no description";
        usbPrintln(str.c_str());
        return true;
    }
    usbPrintln(command.description.c_str());
    return true;
}