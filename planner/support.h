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

#ifndef SUPPORT_H
#define SUPPORT_H

#include <atomic>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

template<typename T>
void HashCombine(const T &v, size_t* const seed) {
    using std::hash;  // make sure that overloads are found
    *seed ^= hash<T>()(v) + 0x9e3779b9 + ((*seed) << 6) + ((*seed) >> 2);
}

// Fluents should not be mutated after creation
class Fluent {
 public:
  Fluent(int predicate, const std::vector<int> &args, int value);

  Fluent(int predicate, const std::vector<int> &args, int value, float prob);

  bool operator==(const Fluent &other) const {
    return (predicate_ == other.GetPredicate()) &&
           (value_ == other.GetValue()) &&
           (args_ == other.GetArgs()) &&
           (prob_ == other.GetProb());
  };

  std::string GetString() const;

  int GetPredicate() const;

  const std::vector<int>& GetArgs() const;

  int GetValue() const;

  float GetProb() const;

  float RoundProb(float prob) const;

  size_t Hash() const;

  size_t HashExcludingProb() const;

 private:
  const int predicate_;
  const std::vector<int> args_;
  const int value_;
  const float prob_;
  size_t hash_;
  size_t hash_ex_prob_;
};

inline std::ostream& operator<<(std::ostream &os, const Fluent &fluent) {
  os << fluent.GetString();
  return os;
}

struct FluentHash {
  size_t operator()(const Fluent &fluent) const {
    return fluent.Hash();
  }
};

typedef std::unordered_set<Fluent, FluentHash> FluentSet;

struct FluentExcludingProbHash {
  size_t operator()(const Fluent &fluent) const {
    return fluent.HashExcludingProb();
  }
};

struct FluentExcludingProbEqual {
  bool operator()(const Fluent &a, const Fluent &b) const {
    return (a.GetPredicate() == b.GetPredicate()) &&
           (a.GetValue() == b.GetValue()) &&
           (a.GetArgs() == b.GetArgs());
  }
};

typedef std::unordered_set<Fluent, FluentExcludingProbHash, FluentExcludingProbEqual> FluentExcludingProbSet;

class State {
 public:
  State(const std::vector<Fluent> &fluents);

  bool operator==(const State &other) const {
    return fluents_ == other.fluents_;
  }

  size_t Hash() const;

  std::string GetString() const;

  void Add(const Fluent &f);

  void Remove(const Fluent &f);

  bool SatisfiedBy(const State *state) const;

  int NumSatisfiedBy(const State *state) const;

  float GetProb(const Fluent &f) const;

  bool ApproximatelyEquals(const State *state) const;

 private:
  int hash_;
  FluentSet fluents_;
  FluentExcludingProbSet fluents_ex_prob_;
};

inline std::ostream& operator<<(std::ostream &os, const State &state) {
  os << state.GetString();
  return os;
}

struct StatePtrHash {
  size_t operator()(const State *state) const {
    return state->Hash();
  }
};

struct StatePtrEqual {
  bool operator()(const State *a, const State *b) const {
    return a->ApproximatelyEquals(b);
  }
};

/*struct StatePtrEqual {
  bool operator()(const State *a, const State *b) const {
    return *a == *b;
  }
};*/

typedef std::unordered_set<const State*, StatePtrHash, StatePtrEqual> StateSet;

class Action {
 public:
  Action(int name, float cost, const std::vector<Fluent> &add_list,
         const std::vector<Fluent> &delete_list, const std::vector<int> &info);

  std::string GetString() const;

  std::string GetPlanString() const;

  int GetName() const;

  float GetCost() const;

  const std::vector<int>& GetInfo() const;

  void Successor(State *state) const;

  void AddSuccessor(State *state) const;

 private:
  const int name_;
  const float cost_;
  const std::vector<Fluent> add_list_;
  const std::vector<Fluent> delete_list_;
  const std::vector<int> info_;
};

// Returns a delimiter-separated list of values in the container,
// requires operator<< to be defined for the value types.
template <typename Container>
std::string Stringer(const Container& container, const std::string& delimiter) {
  std::ostringstream ss;
  if (!container.empty()) {
    typename Container::const_iterator iter = container.begin();
    ss << *iter;
    for (++iter; iter != container.end(); ++iter) {
      ss << delimiter << *iter;
    }
  }
  return ss.str();
}

#endif  // SUPPORT_H
