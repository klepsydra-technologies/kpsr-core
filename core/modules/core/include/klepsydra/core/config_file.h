/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H

#include <fstream>
#include <map>
#include <string>

#include <klepsydra/core/prop_file_convert.h>

namespace kpsr {
class ConfigFile
{
private:
    std::map<std::string, std::string> contents;

    void removeComment(std::string &line) const
    {
        if (line.find(';') != line.npos)
            line.erase(line.find(';'));
    }

    bool onlyWhitespace(const std::string &line) const
    {
        return (line.find_first_not_of(' ') == line.npos);
    }
    bool validLine(const std::string &line) const
    {
        std::string temp = line;
        temp.erase(0, temp.find_first_not_of("\t "));
        if (temp[0] == '=')
            return false;

        for (size_t i = temp.find('=') + 1; i < temp.length(); i++)
            if (temp[i] != ' ')
                return true;

        return false;
    }

    void extractKey(std::string &key, size_t const &sepPos, const std::string &line) const
    {
        key = line.substr(0, sepPos);
        if (key.find('\t') != line.npos || key.find(' ') != line.npos)
            key.erase(key.find_first_of("\t "));
    }
    void extractValue(std::string &value, size_t const &sepPos, const std::string &line) const
    {
        value = line.substr(sepPos + 1);
        value.erase(0, value.find_first_not_of("\t "));
        value.erase(value.find_last_not_of("\t ") + 1);
    }

    void extractContents(const std::string &line)
    {
        std::string temp = line;
        temp.erase(0, temp.find_first_not_of("\t "));
        size_t sepPos = temp.find('=');

        std::string key, value;
        extractKey(key, sepPos, temp);
        extractValue(value, sepPos, temp);

        if (!keyExists(key))
            contents.insert(std::pair<std::string, std::string>(key, value));
        else
            throw std::invalid_argument("CFG: Can only have unique key names!\n");
        ;
    }

    void parseLine(const std::string &line, size_t const lineNo)
    {
        if (line.find('=') == line.npos)
            throw std::invalid_argument("CFG: Couldn't find separator on line: " +
                                        PropertyFileConvert::T_to_string(lineNo) + "\n");

        if (!validLine(line))
            throw std::invalid_argument(
                "CFG: Bad format for line: " + PropertyFileConvert::T_to_string(lineNo) + "\n");

        extractContents(line);
    }

    void extractKeys(std::istream &file)
    {
        std::string line;
        size_t lineNo = 0;
        while (std::getline(file, line)) {
            lineNo++;
            std::string temp = line;

            if (temp.empty())
                continue;

            removeComment(temp);
            if (onlyWhitespace(temp))
                continue;

            parseLine(temp, lineNo);
        }
    }

public:
    explicit ConfigFile(const std::string &fileName)
    {
        std::ifstream file;
        file.open(fileName.c_str());
        if (!file)
            throw std::invalid_argument("CFG: File " + fileName + " couldn't be found!\n");

        extractKeys(file);

        file.close();
    }

    explicit ConfigFile(std::istream &file) { extractKeys(file); }

    bool keyExists(const std::string &key) const { return contents.find(key) != contents.end(); }

    template<typename ValueType>
    ValueType getValueOfKey(const std::string &key,
                            ValueType const &defaultValue = ValueType()) const
    {
        if (!keyExists(key))
            return defaultValue;

        return PropertyFileConvert::string_to_T<ValueType>(contents.find(key)->second);
    }
};
} // namespace kpsr

#endif
