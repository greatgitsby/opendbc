#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "opendbc/can/common.h"


void set_value(std::vector<uint8_t> &msg, const Signal &sig, int64_t ival) {
  int i = sig.lsb / 8;
  int bits = sig.size;
  if (sig.size < 64) {
    ival &= ((1ULL << sig.size) - 1);
  }

  while (i >= 0 && i < msg.size() && bits > 0) {
    int shift = (int)(sig.lsb / 8) == i ? sig.lsb % 8 : 0;
    int size = std::min(bits, 8 - shift);

    msg[i] &= ~(((1ULL << size) - 1) << shift);
    msg[i] |= (ival & ((1ULL << size) - 1)) << shift;

    bits -= size;
    ival >>= size;
    i = sig.is_little_endian ? i+1 : i-1;
  }
}

CANPacker::CANPacker(const std::string& dbc_name) {
  dbc = dbc_lookup(dbc_name);
  assert(dbc);

  for (const auto& msg : dbc->msgs) {
    for (const auto& sig : msg.sigs) {
      signal_lookup[msg.address][sig.name] = sig;
    }
  }
}

std::vector<uint8_t> CANPacker::pack(uint32_t address, const std::vector<SignalPackValue> &signals) {
  auto msg_it = dbc->addr_to_msg.find(address);
  if (msg_it == dbc->addr_to_msg.end()) {
    LOGE("undefined address %d", address);
    return {};
  }

  std::vector<uint8_t> ret(msg_it->second->size, 0);

  // set all values for all given signal/value pairs
  bool counter_set = false;
  for (const auto& sigval : signals) {
    auto sig_it = signal_lookup[address].find(sigval.name);
    if (sig_it == signal_lookup[address].end()) {
      // TODO: do something more here. invalid flag like CANParser?
      LOGE("undefined signal %s - %d\n", sigval.name.c_str(), address);
      continue;
    }
    const auto &sig = sig_it->second;

    int64_t ival = (int64_t)(round((sigval.value - sig.offset) / sig.factor));
    if (ival < 0) {
      ival = (1ULL << sig.size) + ival;
    }
    set_value(ret, sig, ival);

    // FIXME: Type is only assigned if DBC has a ChecksumState
    if (sig.type == COUNTER || sig.name == "COUNTER") {
      counters[address] = sigval.value;
      counter_set = true;
    }
  }

  // set message counter
  auto sig_it_counter = std::find_if(signal_lookup[address].begin(), signal_lookup[address].end(), [](const auto& pair) {
    return pair.second.type == COUNTER || pair.first == "COUNTER";
  });
  if (!counter_set && sig_it_counter != signal_lookup[address].end()) {
    const auto& sig = sig_it_counter->second;

    if (counters.find(address) == counters.end()) {
      counters[address] = 0;
    }
    set_value(ret, sig, counters[address]);
    counters[address] = (counters[address] + 1) % (1 << sig.size);
  }

  // set message checksum
  auto sig_it_checksum = std::find_if(signal_lookup[address].begin(), signal_lookup[address].end(), [](const auto& pair) {
    return pair.second.type > COUNTER;
  });
  if (sig_it_checksum != signal_lookup[address].end()) {
    const auto &sig = sig_it_checksum->second;
    if (sig.calc_checksum != nullptr) {
      unsigned int checksum = sig.calc_checksum(address, sig, ret);
      set_value(ret, sig, checksum);
    }
  }

  return ret;
}

// This function has a definition in common.h and is used in PlotJuggler
const Msg* CANPacker::lookup_message(uint32_t address) {
  return dbc->addr_to_msg.at(address);
}
