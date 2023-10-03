#include "NautilusGUI.h"
#include "Commutation.h"
#include "Teleplot.h"

#include <iostream>
#include <time.h>

// The background thread, doing motor control
void backgroundThread(ThreadStatus *status, nautilus::Nautilus *nautilus)
{
    while (!status->terminate)
    {
        // Wait for running state
        while (!status->isRunning && !status->needToPerformCommutation && !status->terminate)
            usleep(1000);

        if (status->needToPerformCommutation)
        {
            performCommutation(nautilus, status);
            status->needToPerformCommutation = false;
            status->commutationDone = true;
        }
        else
        {

            struct timespec startTime, currentTime;
            clock_gettime(CLOCK_MONOTONIC, &startTime);
            while (status->isRunning && !status->terminate)
            {
                clock_gettime(CLOCK_MONOTONIC, &currentTime);
                double elapsedTime = currentTime.tv_sec - startTime.tv_sec + (currentTime.tv_nsec - startTime.tv_nsec) / 1.0e9;

                float target = 0.0;
                switch(status->signalType)
                {
                    case SignalType::SINUSOID:
                        target = status->amplitude * std::sin(2 * M_PI * status->frequency * elapsedTime) + status->offset;
                        break;
                    case SignalType::CONSTANT:
                        target = status->offset;
                        break;
                    default: break;
                }

                switch(status->controlMode)
                {
                    case ControlMode::CURRENT: nautilus->writeRegister(nautilus::Register::targetIQ, target); break;
                    case ControlMode::VELOCITY: nautilus->writeRegister(nautilus::Register::targetVelocity, static_cast<float>(target / 2 / M_PI)); break;
                    case ControlMode::POSITION: nautilus->writeRegister(nautilus::Register::targetPosition, static_cast<float>(target / 2 / M_PI)); break;
                    default: break;
                }

                Teleplot::localhost().update("target", target);
                nautilus::NautilusReply rep = nautilus->readRegister(nautilus::Register::measuredIQ);
                if (rep.isValid)
                    Teleplot::localhost().update("current", rep.data);
                rep = nautilus->readRegister(nautilus::Register::measuredPosition);
                if (rep.isValid)
                    Teleplot::localhost().update("position", rep.data);
                rep = nautilus->readRegister(nautilus::Register::measuredVelocity);
                if (rep.isValid)
                    Teleplot::localhost().update("velocity", rep.data);
                rep = nautilus->readRegister(nautilus::Register::measuredIPhaseA);
                if (rep.isValid)
                    Teleplot::localhost().update("currentPhaseA", rep.data);
                rep = nautilus->readRegister(nautilus::Register::measuredIPhaseB);
                if (rep.isValid)
                    Teleplot::localhost().update("currentPhaseB", rep.data);
                rep = nautilus->readRegister(nautilus::Register::measuredIPhaseC);
                if (rep.isValid)
                    Teleplot::localhost().update("currentPhaseC", rep.data);
                usleep(5000);
            }
            nautilus->stop();
        }
    }
}


NautilusGUI::NautilusGUI(nautilus::Nautilus *nautilus):
    nautilus_(nautilus)
{
    Gtk::Grid *topGrid = new Gtk::Grid();

    topGrid->set_margin_top(10);
    topGrid->set_margin_bottom(5);
    topGrid->set_margin_left(10);
    topGrid->set_margin_right(10);
    topGrid->set_row_spacing(10);
    topGrid->set_column_spacing(10);
    this->add(*topGrid);

    Gtk::Label *header;
    header = new Gtk::Label("Register");
    sizeGroups_.push_back(Gtk::SizeGroup::create(Gtk::SIZE_GROUP_HORIZONTAL));
    sizeGroups_.back()->add_widget(*header);
    topGrid->attach(*header, 0, 0, 1, 1);
    header = new Gtk::Label("Value");
    sizeGroups_.push_back(Gtk::SizeGroup::create(Gtk::SIZE_GROUP_HORIZONTAL));
    sizeGroups_.back()->add_widget(*header);
    topGrid->attach(*header, 1, 0, 1, 1);
    header = new Gtk::Label("Write value");
    sizeGroups_.push_back(Gtk::SizeGroup::create(Gtk::SIZE_GROUP_HORIZONTAL));
    sizeGroups_.back()->add_widget(*header);
    topGrid->attach(*header, 2, 0, 1, 1);
    Gtk::Separator *sep = new Gtk::Separator(Gtk::ORIENTATION_VERTICAL);
    topGrid->attach(*sep, 3, 0, 1, 1);
    sep->set_halign(Gtk::ALIGN_START);

    commStats_.set_hexpand(true);
    topGrid->attach(commStats_, 4, 0, 1, 1);

    topGrid->attach(scroll_, 0, 1, 5, 1);

    grid_.set_margin_bottom(10);
    grid_.set_row_spacing(10);
    grid_.set_column_spacing(10);
    grid_.set_size_request(800, 600);
    grid_.set_hexpand(true);
    grid_.set_vexpand(true);

    scroll_.set_size_request(800, 600);
    scroll_.add(grid_);

    // Add all registers

    header = new Gtk::Label("Auxiliary position");
    grid_.attach(*header, 0, 0, 1, 1);
    sizeGroups_[0]->add_widget(*header);
    grid_.attach(auxiliaryPosLabel_, 1, 0, 1, 1);
    sizeGroups_[1]->add_widget(auxiliaryPosLabel_);

    int i = 1;
    for (auto reg: nautilus::registerList)
    {
        Gtk::Label *name = new Gtk::Label(reg.name);
        grid_.attach(*name, 0, i, 1, 1);
        sizeGroups_[0]->add_widget(*name);

        registerValues_.push_back(Gtk::Label("0"));
        grid_.attach(registerValues_.back(), 1, i, 1, 1);
        sizeGroups_[1]->add_widget(registerValues_.back());

        if (reg.isWritable)
        {
            updateEntries_.push_back(std::pair<nautilus::GUIRegister, Gtk::SpinButton>(reg, Gtk::SpinButton()));

            updateEntries_.back().second.set_numeric(true);
            if (reg.isFloat)
            {
                updateEntries_.back().second.set_range(-10000000, 10000000);
                updateEntries_.back().second.set_digits(5);
            }
            else
            {
                updateEntries_.back().second.set_range(-32766, 32766);
                updateEntries_.back().second.set_digits(0);
            }
            updateEntries_.back().second.set_width_chars(7);
            updateEntries_.back().second.set_increments(1, 10);
            updateEntries_.back().second.signal_value_changed().connect(sigc::bind(sigc::mem_fun(this, &NautilusGUI::writeRegister), updateEntries_.size() - 1));
            grid_.attach(updateEntries_.back().second, 2, i, 1, 1);
            sizeGroups_[2]->add_widget(updateEntries_.back().second);
        }
        i++;
    }

    sep = new Gtk::Separator(Gtk::ORIENTATION_VERTICAL);
    grid_.attach(*sep, 3, 0, 1, i);

    Gtk::Box *vBox = new Gtk::Box(Gtk::ORIENTATION_VERTICAL);
    vBox->set_spacing(10);
    vBox->set_valign(Gtk::ALIGN_FILL);
    vBox->set_halign(Gtk::ALIGN_FILL);
    vBox->set_hexpand(true);
    vBox->set_vexpand(true);
    grid_.attach(*vBox, 4, 0, 1, i);

    Gtk::Box *hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    hBox->set_spacing(10);
    header = new Gtk::Label("Commutation current (A):");
    hBox->pack_start(*header, Gtk::PACK_EXPAND_PADDING);
    commutationCurrent_.set_range(0.1, 20.0);
    commutationCurrent_.set_value(1.0);
    commutationCurrent_.set_increments(0.1, 1);
    commutationCurrent_.set_digits(2);
    commutationCurrent_.set_width_chars(4);
    hBox->pack_start(commutationCurrent_, Gtk::PACK_SHRINK);
    hBox->set_halign(Gtk::ALIGN_CENTER);
    vBox->pack_start(*hBox, Gtk::PACK_SHRINK);

    commutationButton_ = Gtk::Button("Perform commutation");
    commutationButton_.set_halign(Gtk::ALIGN_CENTER);
    vBox->pack_start(commutationButton_, Gtk::PACK_SHRINK);
    commutationButton_.signal_clicked().connect(sigc::mem_fun(this, &NautilusGUI::startCommutation));

    Gtk::Button *button = new Gtk::Button("Store settings to persistent memory");
    button->set_halign(Gtk::ALIGN_CENTER);
    vBox->pack_start(*button, Gtk::PACK_SHRINK);
    button->signal_clicked().connect(sigc::mem_fun(nautilus_, &nautilus::Nautilus::storeToPersistentMemory));

    sep = new Gtk::Separator(Gtk::ORIENTATION_HORIZONTAL);
    vBox->pack_start(*sep, Gtk::PACK_SHRINK);


    hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    hBox->set_spacing(10);
    header = new Gtk::Label("Control type:");
    hBox->pack_start(*header);
    for (std::string s : CONTROL_MODES)
        motionType_.append(s);
    motionType_.set_active_text(CONTROL_MODES.at(0));
    hBox->pack_start(motionType_);
    hBox->set_halign(Gtk::ALIGN_CENTER);
    vBox->pack_start(*hBox, Gtk::PACK_SHRINK);


    hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    hBox->set_spacing(10);
    header = new Gtk::Label("Signal type:");
    hBox->pack_start(*header);
    for (std::string s : SIGNAL_TYPES)
        signalType_.append(s);
    signalType_.set_active_text(SIGNAL_TYPES.at(0));
    hBox->pack_start(signalType_);
    hBox->set_halign(Gtk::ALIGN_CENTER);
    vBox->pack_start(*hBox, Gtk::PACK_SHRINK);

    hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    hBox->set_spacing(10);
    header = new Gtk::Label("Signal frequency:");
    hBox->pack_start(*header);
    motionFrequency_.set_range(0.01, 200.0);
    motionFrequency_.set_value(1.0);
    motionFrequency_.set_increments(0.1, 1);
    motionFrequency_.set_digits(2);
    motionFrequency_.set_width_chars(4);
    hBox->pack_start(motionFrequency_);
    hBox->set_halign(Gtk::ALIGN_CENTER);
    vBox->pack_start(*hBox, Gtk::PACK_SHRINK);

    hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    hBox->set_spacing(10);
    header = new Gtk::Label("Signal offset:");
    hBox->pack_start(*header);
    motionOffset_.set_range(-500.0, 500.0);
    motionOffset_.set_value(1.0);
    motionOffset_.set_increments(0.1, 1);
    motionOffset_.set_digits(2);
    motionOffset_.set_width_chars(4);
    hBox->pack_start(motionOffset_);
    hBox->set_halign(Gtk::ALIGN_CENTER);
    vBox->pack_start(*hBox, Gtk::PACK_SHRINK);

    hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    header = new Gtk::Label("Signal amplitude:");
    hBox->pack_start(*header, Gtk::PACK_SHRINK);
    motionAmplitude_.set_range(0.01, 500.0);
    motionAmplitude_.set_value(1.0);
    motionAmplitude_.set_increments(0.1, 1);
    motionAmplitude_.set_digits(2);
    motionAmplitude_.set_width_chars(4);
    hBox->pack_start(motionAmplitude_);
    hBox->set_halign(Gtk::ALIGN_CENTER);
    vBox->pack_start(*hBox, Gtk::PACK_SHRINK);

    motionButton_ = Gtk::Button("Start");
    motionButton_.set_halign(Gtk::ALIGN_CENTER);
    vBox->pack_start(motionButton_, Gtk::PACK_SHRINK);
    motionButton_.signal_clicked().connect(sigc::mem_fun(this, &NautilusGUI::motionClicked));


    sep = new Gtk::Separator(Gtk::ORIENTATION_HORIZONTAL);
    vBox->pack_start(*sep, Gtk::PACK_SHRINK);
    Gtk::ScrolledWindow *scroll = new Gtk::ScrolledWindow();
    vBox->pack_start(*scroll);
    scroll->add(logTextView);


    Glib::signal_timeout().connect(sigc::mem_fun(*this, &NautilusGUI::updateReadings), 100);
    Glib::signal_timeout().connect(sigc::mem_fun(*this, &NautilusGUI::checkAsyncStatus), 50);


    bgThread_ = std::thread(backgroundThread, &status_, nautilus_);

    show_all();
}

NautilusGUI::~NautilusGUI()
{
    status_.terminate = true;
    bgThread_.join();
}

bool NautilusGUI::updateReadings()
{
    for (unsigned int i = 0; i < registerValues_.size(); i++)
    {
        nautilus::NautilusReply rep = nautilus_->readRegister(nautilus::registerList[i].address);

        if (rep.isValid)
            registerValues_[i].set_text(nautilus::registerList[i].isFloat ? std::to_string(rep.data) : std::to_string(reinterpret_cast<uint32_t &>(rep.data)));
        else
            registerValues_[i].set_text("Read fail!");
        if (i == 0)
            auxiliaryPosLabel_.set_text(rep.validEncoder ? std::to_string(rep.encoderPosition) : "Error");
    }
    double const sRate = nautilus_->nSuccess / static_cast<double>(nautilus_->nSuccess + nautilus_->nFailed) * 100.0;

    commStats_.set_text("Communication success rate:" + std::to_string(sRate) + " (" + std::to_string(nautilus_->nSuccess) + "/" + std::to_string(nautilus_->nFailed) + ")");
    return true;
}



void NautilusGUI::startCommutation()
{
    commutationButton_.set_sensitive(false);
    motionButton_.set_sensitive(false);

    status_.isRunning = false;
    status_.needToPerformCommutation = true;
    status_.commutationDone = false;
    status_.commutationCurrent = commutationCurrent_.get_value();
}


bool NautilusGUI::checkAsyncStatus()
{

    status_.mutex.lock();
    if (status_.commutationDone)
    {
        commutationButton_.set_sensitive(true);
        motionButton_.set_sensitive(true);
        status_.commutationDone = false;
    }

    std::vector<std::string> messages = status_.messages;
    status_.messages.clear();
    status_.mutex.unlock();

    for (std::string s : messages)
        logTextView.get_buffer()->insert_at_cursor(s + "\n");
    return true;
}


void NautilusGUI::motionClicked()
{
    // Preprocess arguments
    // Gather parameters from GUI.
    std::string const& sMode = motionType_.get_active_text();
    for (uint i = 0; i < CONTROL_MODES.size(); i++)
        if (CONTROL_MODES.at(i) == sMode)
        {
            status_.controlMode = static_cast<ControlMode>(i);
            break;
        }


    std::string const& sType = signalType_.get_active_text();
    for (uint i = 0; i < SIGNAL_TYPES.size(); i++)
        if (SIGNAL_TYPES.at(i) == sType)
        {
            status_.signalType = static_cast<SignalType>(i);
            break;
        }

    status_.amplitude = motionAmplitude_.get_value();
    status_.frequency = motionFrequency_.get_value();
    status_.offset = motionOffset_.get_value();

    status_.isRunning = !status_.isRunning;
    motionButton_.set_label(status_.isRunning ? "Stop" : "Start");
    commutationButton_.set_sensitive(!status_.isRunning);
}

void NautilusGUI::writeRegister(int const& index)
{
    if (updateEntries_[index].first.isFloat)
        nautilus_->writeRegister(updateEntries_[index].first.address, static_cast<float>(updateEntries_[index].second.get_value()));
    else
        nautilus_->writeRegister(updateEntries_[index].first.address, static_cast<uint32_t>(updateEntries_[index].second.get_value()));
    updateReadings();
}

