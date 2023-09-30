/// \file MainWindow.h
/// \brief Main window of the GUI.

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <gtkmm.h>

#include <thread>

#include "NautilusDriver.h"


enum class ControlMode{
    POSITION = 0,
    VELOCITY = 1,
    CURRENT = 2
};
static std::vector<std::string> CONTROL_MODES({"Position", "Velocity", "Current"});

enum class SignalType{
    SINUSOID = 0,
    CONSTANT = 1
};
static std::vector<std::string> SIGNAL_TYPES({"Sinus", "Constant"});

struct ThreadStatus
{
    bool terminate = false;
    bool needToPerformCommutation = false;
    bool commutationDone = false;
    bool isRunning = false;
    double commutationCurrent = 0.0;
    ControlMode controlMode;
    SignalType signalType;
    double amplitude = 0.0;
    double frequency = 0.0;
    double offset = 0.0;
};

class NautilusGUI : public Gtk::Window
{
    public:
        NautilusGUI(nautilus::Nautilus *nautilus);
        ~NautilusGUI();

    protected:
        bool updateReadings();
        bool checkAsyncStatus();
        void writeRegister(int const& index);
        void startCommutation();
        void motionClicked();


        nautilus::Nautilus* nautilus_;

        std::vector<Gtk::Label> registerValues_;
        Gtk::Label auxiliaryPosLabel_;
        std::vector<std::pair<nautilus::GUIRegister, Gtk::SpinButton>> updateEntries_;


        Gtk::Grid grid_;
        Gtk::ScrolledWindow scroll_;

        std::vector<Glib::RefPtr<Gtk::SizeGroup>> sizeGroups_;

        Gtk::Label commStats_;

        Gtk::Button commutationButton_;
        Gtk::SpinButton commutationCurrent_;


        Gtk::Button motionButton_;
        Gtk::ComboBoxText motionType_;

        Gtk::ComboBoxText signalType_;
        Gtk::SpinButton motionAmplitude_;
        Gtk::SpinButton motionFrequency_;
        Gtk::SpinButton motionOffset_;

        ThreadStatus status_;
        std::thread bgThread_;
};

#endif