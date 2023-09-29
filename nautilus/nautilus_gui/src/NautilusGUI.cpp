#include "NautilusGUI.h"
#include "Commutation.h"
#include "Teleplot.h"

#include <iostream>
#include <time.h>


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
    vBox->set_valign(Gtk::ALIGN_START);
    vBox->set_halign(Gtk::ALIGN_CENTER);
    vBox->set_hexpand(true);
    grid_.attach(*vBox, 4, 0, 1, i);

    Gtk::Box *hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    hBox->set_spacing(10);
    header = new Gtk::Label("Commutation current (A):");
    hBox->pack_start(*header);
    commutationCurrent_.set_range(0.1, 20.0);
    commutationCurrent_.set_value(1.0);
    commutationCurrent_.set_increments(0.1, 1);
    commutationCurrent_.set_digits(2);
    commutationCurrent_.set_width_chars(4);
    hBox->pack_start(commutationCurrent_);
    vBox->pack_start(*hBox);

    commutationButton_ = Gtk::Button("Perform commutation");
    vBox->pack_start(commutationButton_);
    commutationButton_.signal_clicked().connect(sigc::mem_fun(this, &NautilusGUI::startCommutation));

    sep = new Gtk::Separator(Gtk::ORIENTATION_HORIZONTAL);
    vBox->pack_start(*sep);


    hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    hBox->set_spacing(10);
    header = new Gtk::Label("Control type:");
    hBox->pack_start(*header);
    for (std::string s : CONTROL_MODES)
        motionType_.append(s);
    motionType_.set_active_text(CONTROL_MODES.at(0));
    hBox->pack_start(motionType_);
    vBox->pack_start(*hBox);


    hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    hBox->set_spacing(10);
    header = new Gtk::Label("Signal type:");
    hBox->pack_start(*header);
    for (std::string s : SIGNAL_TYPES)
        signalType_.append(s);
    signalType_.set_active_text(SIGNAL_TYPES.at(0));
    hBox->pack_start(signalType_);
    vBox->pack_start(*hBox);

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
    vBox->pack_start(*hBox);

    hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    hBox->set_spacing(10);
    header = new Gtk::Label("Signal offset:");
    hBox->pack_start(*header);
    motionOffset_.set_range(0.01, 200.0);
    motionOffset_.set_value(1.0);
    motionOffset_.set_increments(0.1, 1);
    motionOffset_.set_digits(2);
    motionOffset_.set_width_chars(4);
    hBox->pack_start(motionOffset_);
    vBox->pack_start(*hBox);

    hBox = new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL);
    header = new Gtk::Label("Signal amplitude:");
    hBox->pack_start(*header);
    motionAmplitude_.set_range(0.01, 200.0);
    motionAmplitude_.set_value(1.0);
    motionAmplitude_.set_increments(0.1, 1);
    motionAmplitude_.set_digits(2);
    motionAmplitude_.set_width_chars(4);
    hBox->pack_start(motionAmplitude_);
    vBox->pack_start(*hBox);

    motionButton_ = Gtk::Button("Start");
    isRunning_ = false;
    vBox->pack_start(motionButton_);
    motionButton_.signal_clicked().connect(sigc::mem_fun(this, &NautilusGUI::motionClicked));


    Glib::signal_timeout().connect(sigc::mem_fun(*this, &NautilusGUI::updateReadings), 100);
    Glib::signal_timeout().connect(sigc::mem_fun(*this, &NautilusGUI::checkAsyncStatus), 50);


    std::thread th = std::thread(&NautilusGUI::backgroundThread, this);
    th.detach();

    show_all();
}


bool NautilusGUI::updateReadings()
{
    // Don't query drive while running
    if (isRunning_)
        return true;

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

    isRunning_ = false;
    needToPerformCommutation_ = true;
    commutationDone_ = false;
}

bool NautilusGUI::checkAsyncStatus()
{
    if (commutationDone_)
    {
        std::cout << "Commutation done..." << std::endl;
        commutationButton_.set_sensitive(true);
        motionButton_.set_sensitive(true);
        commutationDone_ = false;
    }
    return true;
}


void NautilusGUI::motionClicked()
{
    isRunning_ = !isRunning_;
    motionButton_.set_label(isRunning_ ? "Stop" : "Start");
    commutationButton_.set_sensitive(!isRunning_);
}

void NautilusGUI::writeRegister(int const& index)
{
    if (updateEntries_[index].first.isFloat)
        nautilus_->writeRegister(updateEntries_[index].first.address, static_cast<float>(updateEntries_[index].second.get_value()));
    else
        nautilus_->writeRegister(updateEntries_[index].first.address, static_cast<uint32_t>(updateEntries_[index].second.get_value()));
    updateReadings();
}


void NautilusGUI::backgroundThread()
{
    while (true)
    {
        // Wait for running state
        while (!isRunning_ && !needToPerformCommutation_)
            usleep(1000);

        if (needToPerformCommutation_)
        {
            performCommutation(nautilus_, commutationCurrent_.get_value());
            needToPerformCommutation_ = false;
            commutationDone_ = true;
        }
        else
        {
            // Gather parameters from GUI.
            std::string const& sMode = motionType_.get_active_text();
            ControlMode controlMode;
            for (uint i = 0; i < CONTROL_MODES.size(); i++)
                if (CONTROL_MODES.at(i) == sMode)
                {
                    controlMode = static_cast<ControlMode>(i);
                    break;
                }


            std::string const& sType = signalType_.get_active_text();
            SignalType signal;
            for (uint i = 0; i < SIGNAL_TYPES.size(); i++)
                if (SIGNAL_TYPES.at(i) == sType)
                {
                    signal = static_cast<SignalType>(i);
                    break;
                }

            double amplitude = motionAmplitude_.get_value();
            double frequency = motionFrequency_.get_value();
            double offset = motionOffset_.get_value();

            struct timespec startTime, currentTime;
            clock_gettime(CLOCK_MONOTONIC, &startTime);
            while (isRunning_)
            {
                clock_gettime(CLOCK_MONOTONIC, &currentTime);
                double elapsedTime = currentTime.tv_sec - startTime.tv_sec + (currentTime.tv_nsec - startTime.tv_nsec) / 1.0e9;

                float target = 0.0;
                switch(signal)
                {
                    case SignalType::SINUSOID:
                        target = amplitude * std::sin(2 * M_PI * frequency * elapsedTime) + offset;
                        break;
                    case SignalType::CONSTANT:
                        target = offset;
                        break;
                    default: break;
                }

                switch(controlMode)
                {
                    case ControlMode::CURRENT: nautilus_->writeRegister(nautilus::Register::targetIQ, target);
                    case ControlMode::VELOCITY: nautilus_->writeRegister(nautilus::Register::targetVelocity, target);
                    case ControlMode::POSITION: nautilus_->writeRegister(nautilus::Register::targetPosition, static_cast<float>(target / 2 / M_PI));
                    default: break;
                }

                Teleplot::localhost().update("target", target);
                nautilus::NautilusReply rep = nautilus_->readRegister(nautilus::Register::measuredIQ);
                if (rep.isValid)
                    Teleplot::localhost().update("current", rep.data);
                rep = nautilus_->readRegister(nautilus::Register::measuredPosition);
                if (rep.isValid)
                    Teleplot::localhost().update("position", rep.data);
                rep = nautilus_->readRegister(nautilus::Register::measuredVelocity);
                if (rep.isValid)
                    Teleplot::localhost().update("velocity", rep.data);
                usleep(5000);
            }
            nautilus_->stop();
        }
    }
}