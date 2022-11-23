// Copyright 2022 Institute of Automatic Control RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and limitations under the License.
//
// Author: Felix Widmayer (f.widmayer@irt.rwth-aachen.de)
//
#ifndef EVOLOGICS_DRIVER_EVOLOGICS_UTILS_HPP
#define EVOLOGICS_DRIVER_EVOLOGICS_UTILS_HPP

// standard library
#include <string>
#include <sstream>
#include <iterator>
#include <iostream>
#include <algorithm>
#include <locale>
#include <vector>

namespace evologics
{
    /**
     * Prints tokens from vector
     * @param name Identifier
     * @param tokens Tokenized line
     */
    inline void print_tokens(const std::string &name, const std::vector<std::string> &tokens)
    {
        std::string s;
        for (const auto &piece : tokens)
            s += piece + ',';
        if (!s.empty())
        {
            s.pop_back();
            ROS_INFO_STREAM("[" << name << "]: " << s);
        }
    }

    /**
     * Tokenizes s by splitting at delim and fills result of type C
     * @tparam C Iterator type
     * @param s String to split
     * @param delim Delimiter to split at
     * @param result Container filled with tokenized string
     */
    template <typename C>
    inline void split(const std::string &s, const char delim, std::back_insert_iterator<C> result)
    {
        // splits string s at each occurence of delim and back-inserts it into container C
        std::istringstream iss(s);
        std::string item;
        while (std::getline(iss, item, delim))
        {
            *result++ = item;
        }
    }

    /**
     * Strip unwanted prefixes from string
     * @param s String
     * @return Postprocessed string
     */
    inline std::string clean_message_type_token(const std::string &s)
    {
        //  strips the +++AT:<number>: prefix from evologics message to return clean message type
        size_t found = s.find_last_of(':');
        if (found != std::string::npos)
            return s.substr(found+1);
        return s;
    }

    /**
     * Returns truth value of s is polling response
     * @param s String
     * @return Truth value of s is polling response
     */
    inline bool is_polling_response(const std::string &s)
    {
        // responses to poll cmds do not have a unique identifier and just return the raw data
        // if the leading token (here string s) contains a digit, the msg is a candidate for response to poll cmd
        return std::any_of(s.begin(), s.end(), ::isdigit);
    }

    /**
     * Fills result with tokenized s converted to type T
     * @tparam T type of tokens in result
     * @param s String
     * @param result Token vector that holds tokens of type T
     */
    template <typename T>
    inline void fill_vector_from_string(const std::string &s, std::vector<T> &result)
    {
        // attempts to convert each word of string s to type T and appends to result if successful
        std::stringstream ss(s);
        std::string word;
        T value;
        while (!ss.eof())
        {
            ss >> word;
            if (std::stringstream(word) >> value)
                result.push_back(value);
            word = "";
        }
    }

} // namespace evologics

#endif // EVOLOGICS_DRIVER_EVOLOGICS_UTILS_HPP