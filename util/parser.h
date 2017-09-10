#ifndef PARSER_H
#define PARSER_H

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

class Parser
{
public:
    static const char* parse(const char *filename)
    {
        try
        {
            std::ifstream file(filename);
            file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
            if (file)
            {
                std::stringstream ss_buf;
                ss_buf << file.rdbuf();
                std::string str = ss_buf.str();
                const char *char_str = str.c_str();
                return char_str;
            }
            else
            {
                #ifdef DESKTOP_BUILD
                    throw(errno); // android non-compatible
                #endif
            }
        }
        catch (std::ifstream::failure e)
        {
            std::cout << e.what() << std::endl;
        }
    }
};

#endif // PARSER_H
