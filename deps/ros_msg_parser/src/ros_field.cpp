/***** MIT License ****
*
*   Copyright (c) 2016-2022 Davide Faconti
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy
*   of this software and associated documentation files (the "Software"), to deal
*   in the Software without restriction, including without limitation the rights
*   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*   copies of the Software, and to permit persons to whom the Software is
*   furnished to do so, subject to the following conditions:
*
*   The above copyright notice and this permission notice shall be included in all
*   copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*   SOFTWARE.
*/

#include "ros_msg_parser/ros_field.hpp"
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/algorithm/string/trim.hpp>

namespace RosMsgParser{

ROSField::ROSField(const ROSType &type, const std::string &name):
  _fieldname(name), _type(type), _is_array(false), _array_size(1)
{

}

ROSField::ROSField(const std::string &definition):
  _is_array(false), _array_size(1)
{
  static const  boost::regex type_regex("[a-zA-Z][a-zA-Z0-9_]*"
                                       "(/[a-zA-Z][a-zA-Z0-9_]*){0,1}"
                                       "(\\[[0-9]*\\]){0,1}");

  static const  boost::regex field_regex("[a-zA-Z][a-zA-Z0-9_]*");

  static const boost::regex array_regex("(.+)(\\[([0-9]*)\\])");

  using boost::regex;
  std::string::const_iterator begin = definition.begin();
  std::string::const_iterator end   = definition.end();
  boost::match_results<std::string::const_iterator> what;

  // Get type and field
  std::string type, value;

  //-------------------------------
  // Find type, field and array size
  if( regex_search(begin, end, what, type_regex)) {
    type = what[0];
    begin = what[0].second;
  }
  else {
    throw std::runtime_error("Bad type when parsing field: " + definition);
  }

  if (regex_search(begin, end, what, field_regex))
  {
    _fieldname = what[0];
    begin = what[0].second;
  }
  else {
    throw std::runtime_error("Bad field when parsing field: " + definition);
  }

  std::string temp_type = type;
  if (regex_search(temp_type, what, array_regex))
  {
    type = what[1];

    if (what.size() == 3) {
      _array_size = -1;
      _is_array = true;
    }
    else if (what.size() == 4) {
      std::string size(what[3].first, what[3].second);
      _array_size = size.empty() ? -1 : atoi(size.c_str());
      _is_array = true;
    }
    else {
      throw std::runtime_error("Bad array size when parsing field:  " + definition);
    }
  }

  //-------------------------------
  // Find if Constant or comment

  // Determine next character
  // if '=' -> constant, if '#' -> done, if nothing -> done, otherwise error
  if (regex_search(begin, end, what, boost::regex("\\S")))
  {
    if (what[0] == "=")
    {
      begin = what[0].second;
      // Copy constant
      if (type == "string") {
        value.assign(begin, end);
      }
      else {
        if (regex_search(begin, end, what, boost::regex("\\s*#")))
        {
          value.assign(begin, what[0].first);
        }
        else {
          value.assign(begin, end);
        }
        // TODO: Raise error if string is not numeric
      }

      boost::algorithm::trim(value);
    } else if (what[0] == "#") {
      // Ignore comment
    } else {
      // Error
      throw std::runtime_error("Unexpected character after type and field:  " +
                               definition);
    }
  }
  _type  = ROSType( type );
  _value = value;
}

}
