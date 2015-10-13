/*
 * Copyright 2015 Ciara Kamahele-Sanfratello
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

#ifndef STRING_REGISTRY_H
#define STRING_REGISTRY_H

#include <map>
#include <memory>
#include <string>
#include <vector>

// Maps strings to integers and back.
class StringRegistry {
 public:
  // Call this once at startup before use.
  static void Init();

  // Returns the singleton instance. Init must have been called before.
  static StringRegistry* Get() { return singleton_.get(); }

  // Retrieve the integer mapping for a string. Adds the string to the
  // registry if not previously added.
  int GetInt(const std::string& str);

  // Returns the string for a previously added mapping.
  const std::string &GetString(const int str_int) const { return *table_[str_int]; }

 private:
  static std::unique_ptr<StringRegistry> singleton_;
  typedef std::map<std::string, int> Map;
  Map map_;
  std::vector<const std::string*> table_;
};

#endif  // STRING_REGISTRY_H

