/// \file MainWindow.h
/// \brief Main window of the GUI.

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <gtkmm.h>

#include <thread>

#include "NautilusDriver.h"

class NautilusGUI : public Gtk::Window
{
    public:
        NautilusGUI(nautilus::Nautilus *nautilus);


    protected:
        // // Rescan network to find all servos.
        // void rescan();

        // // Action callbacks
        // void resetPosition(int const& servoNumber);
        // void updateEnable(int const& servoNumber);
        // void updateTargetPosition(int const& servoNumber);
        // void updateTargetVelocity(int const& servoNumber);
        // void updateControlMode(int const& servoNumber);
        // void updateId(int const& servoNumber);

        // Update all register readings
        bool updateReadings();
        bool checkAsyncStatus();

        void writeRegister(int const& index);

        void startCommutation();


        nautilus::Nautilus* nautilus_;


        std::vector<Gtk::Label> registerValues_;
        Gtk::Label auxiliaryPosLabel_;

        std::vector<std::pair<nautilus::GUIRegister, Gtk::SpinButton>> updateEntries_;

        std::vector<int> servoIds_;

        std::vector<Gtk::Label> servoPositions_;
        std::vector<Gtk::Label> servoVelocities_;
        std::vector<Gtk::ComboBoxText> controlModes_;
        std::vector<Gtk::SpinButton> targetPositions_;
        std::vector<Gtk::SpinButton> targetVelocities_;
        std::vector<Gtk::CheckButton> torqueEnabled_;
        std::vector<Gtk::Button> resetButtons_;
        std::vector<Gtk::Button> changeIdButtons_;

        Gtk::Grid grid_;
        Gtk::ScrolledWindow scroll_;

        std::vector<Glib::RefPtr<Gtk::SizeGroup>> sizeGroups_;

        Gtk::Label commStats_;

        Gtk::Button commutationButton_;
        Gtk::SpinButton commutationCurrent_;
        bool commutationDone_;
};

#endif