#pragma once

#include "Bsp.hpp"

void buzzerSetState(bool on) { pinWrite(buzzerPin, on); }
