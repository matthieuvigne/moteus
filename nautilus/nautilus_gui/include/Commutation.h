/// \file MainWindow.h
/// \brief Main window of the GUI.

#ifndef COMMUTATION_H
#define COMMUTATION_H

#include "NautilusDriver.h"

/// @brief Perform encoder commutation, this is meant to run in a separate thread
/// @param nautilus Nautilus controller
void performCommutation(nautilus::Nautilus *nautilus, double const& targetCurrent);

#endif